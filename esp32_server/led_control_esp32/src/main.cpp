// BLE LED dimmer for the ESP32-C3 SuperMini (HW-466AB).
//
// Every LED pin is attached to the same PWM channel, so all LEDs always show
// the same brightness. The phone picks which GPIOs are LEDs (PINS) and sends
// "go to brightness X over Y ms" (BRIGHTNESS).
//
// Protocol: one service, two characteristics, little-endian.
//   RX (write / write-without-response), phone -> ESP:
//     HELLO       [0x01][optional client name, ASCII]  ESP answers with HELLO_REPLY
//     BRIGHTNESS  [0x02][u8 level][u16 fadeMs]         fade all LEDs to level (0..255)
//     PINS        [0x03][u8 count][count x u8 gpio]    replace the set of LED pins
//   TX (notify + read), ESP -> phone:
//     HELLO_REPLY [0x81][u8 version][u32 safeMask][u32 pinMask][u8 level]
//     PINS_OK     [0x83][u32 pinMask]
//     ERROR       [0xEE][u8 cmd][u8 code][u8 detail]   code: 1 malformed,
//                                                      2 unsafe pin (detail = gpio),
//                                                      3 unknown command
//   Masks: bit n set = GPIO n.
//
// Connection handshake: the phone subscribes to TX, then sends HELLO. The ESP
// replies with what it is (version), which pins are allowed, which pins are
// active and the current brightness. TX also holds HELLO_REPLY for plain reads.
//
// If no command arrives for 5 s, all LEDs fade to 0. Any command resets the timer.
// On power-up every safe GPIO is blinked once, in order, so you can find your wiring.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

const uint8_t PROTOCOL_VERSION = 1;

// GPIOs that are safe to drive LEDs on the C3 SuperMini. Excluded:
//   2, 8, 9  strapping pins (8 is also the onboard LED, 9 the BOOT button)
//   11..17   SPI flash / VDD_SPI
//   18, 19   native USB (D-/D+), used for upload and Serial
// 20/21 are UART0; they're free because Serial runs over USB CDC
// (see build_flags), but the ROM boot log will flicker GPIO21 at reset.
const uint8_t SAFE_PINS[] = {0, 1, 3, 4, 5, 6, 7, 10, 20, 21};

const uint8_t LED_CH = 0; // the one LEDC channel shared by all LED pins
const uint32_t PWM_FREQ = 5000;
const uint8_t PWM_BITS = 8; // duty 0..255

const uint32_t IDLE_TIMEOUT_MS = 5000;
const uint16_t IDLE_FADE_MS = 1000;
const uint16_t BLINK_ON_MS = 200;
const uint16_t BLINK_OFF_MS = 100;

const uint8_t CMD_HELLO = 0x01;
const uint8_t CMD_BRIGHTNESS = 0x02;
const uint8_t CMD_PINS = 0x03;
const uint8_t RSP_HELLO = 0x81;
const uint8_t RSP_PINS_OK = 0x83;
const uint8_t RSP_ERROR = 0xEE;

const uint8_t ERR_MALFORMED = 1;
const uint8_t ERR_UNSAFE_PIN = 2;
const uint8_t ERR_UNKNOWN_CMD = 3;

// Commands are queued from the BLE task and handled in loop(), so all LED and
// state changes happen in one task.
struct Cmd {
  uint8_t len;
  uint8_t data[32];
};
QueueHandle_t cmdQueue;

BLECharacteristic *tx;
volatile bool connected = false;

uint32_t safeMask = 0;
uint32_t pinMask = 0; // GPIOs currently attached to LED_CH

uint8_t level = 0; // duty currently on the pins
uint8_t fadeFrom = 0, fadeTo = 0;
uint32_t fadeStart = 0, fadeMs = 0;

uint32_t lastCmdMs = 0;
bool idleDimmed = false;

bool isSafePin(uint8_t pin) { return pin < 32 && (safeMask & (1u << pin)); }

void putU32(uint8_t *p, uint32_t v) {
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

void send(const uint8_t *data, size_t len) {
  tx->setValue((uint8_t *)data, len);
  if (connected) tx->notify();
}

void sendHello() {
  uint8_t r[11] = {RSP_HELLO, PROTOCOL_VERSION};
  putU32(r + 2, safeMask);
  putU32(r + 6, pinMask);
  r[10] = fadeTo;
  send(r, sizeof(r));
}

void sendError(uint8_t cmd, uint8_t code, uint8_t detail) {
  uint8_t r[] = {RSP_ERROR, cmd, code, detail};
  send(r, sizeof(r));
}

void startFade(uint8_t to, uint32_t ms) {
  fadeFrom = level;
  fadeTo = to;
  fadeStart = millis();
  fadeMs = ms;
}

void updateFade(uint32_t now) {
  uint32_t elapsed = now - fadeStart;
  uint8_t next = elapsed >= fadeMs
                     ? fadeTo
                     : fadeFrom + ((int32_t)fadeTo - fadeFrom) * (int32_t)elapsed / (int32_t)fadeMs;
  if (next != level) {
    level = next;
    ledcWrite(LED_CH, level);
  }
}

void applyPinMask(uint32_t mask) {
  for (uint8_t pin : SAFE_PINS) {
    bool want = mask & (1u << pin);
    bool have = pinMask & (1u << pin);
    if (want && !have) {
      ledcAttachPin(pin, LED_CH);
    } else if (!want && have) {
      ledcDetachPin(pin);
      digitalWrite(pin, LOW);
    }
  }
  pinMask = mask;
}

bool handlePins(const uint8_t *d, size_t len) {
  if (len < 2 || len < 2u + d[1]) {
    sendError(CMD_PINS, ERR_MALFORMED, 0);
    return false;
  }
  uint32_t mask = 0;
  for (uint8_t i = 0; i < d[1]; i++) {
    uint8_t pin = d[2 + i];
    if (!isSafePin(pin)) {
      sendError(CMD_PINS, ERR_UNSAFE_PIN, pin);
      return false;
    }
    mask |= 1u << pin;
  }
  applyPinMask(mask);
  Serial.printf("pins: 0x%08lx\n", (unsigned long)pinMask);

  uint8_t r[5] = {RSP_PINS_OK};
  putU32(r + 1, pinMask);
  send(r, sizeof(r));
  return true;
}

void handleCommand(const Cmd &c) {
  const uint8_t *d = c.data;
  switch (d[0]) {
  case CMD_HELLO:
    Serial.printf("hello from '%.*s'\n", c.len - 1, (const char *)d + 1);
    sendHello();
    break;

  case CMD_BRIGHTNESS:
    if (c.len < 4) return sendError(CMD_BRIGHTNESS, ERR_MALFORMED, 0);
    startFade(d[1], d[2] | (d[3] << 8));
    break;

  case CMD_PINS:
    if (!handlePins(d, c.len)) return;
    break;

  default:
    sendError(d[0], ERR_UNKNOWN_CMD, 0);
    return; // rejected commands don't count as keep-alive
  }
  lastCmdMs = millis();
  idleDimmed = false;
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    std::string v = ch->getValue();
    if (v.empty()) return;
    Cmd c;
    c.len = std::min<size_t>(v.size(), sizeof(c.data));
    memcpy(c.data, v.data(), c.len);
    xQueueSend(cmdQueue, &c, 0);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    connected = true;
    Serial.println("connected");
  }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    Serial.println("disconnected");
    BLEDevice::startAdvertising(); // the idle timeout dims the LEDs
  }
};

void blinkSafePins() {
  for (uint8_t pin : SAFE_PINS) {
    Serial.printf("blink GPIO%u\n", pin);
    digitalWrite(pin, HIGH);
    delay(BLINK_ON_MS);
    digitalWrite(pin, LOW);
    delay(BLINK_OFF_MS);
  }
}

void setup() {
  Serial.begin(115200);

  for (uint8_t pin : SAFE_PINS) {
    safeMask |= 1u << pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  blinkSafePins();

  ledcSetup(LED_CH, PWM_FREQ, PWM_BITS);
  ledcWrite(LED_CH, 0);

  cmdQueue = xQueueCreate(16, sizeof(Cmd));

  BLEDevice::init("LED-Control");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  BLECharacteristic *rx = service->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  tx = service->createCharacteristic(
      TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  tx->addDescriptor(new BLE2902());
  sendHello(); // so a plain read of TX works before HELLO
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("advertising as LED-Control");

  lastCmdMs = millis();
}

void loop() {
  Cmd c;
  while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) handleCommand(c);

  uint32_t now = millis();
  if (!idleDimmed && now - lastCmdMs >= IDLE_TIMEOUT_MS) {
    idleDimmed = true;
    if (fadeTo != 0) {
      Serial.println("idle: dimming down");
      startFade(0, IDLE_FADE_MS);
    }
  }
  updateFade(now);
  delay(1);
}
