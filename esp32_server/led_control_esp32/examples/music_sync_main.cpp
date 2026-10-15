// BLE LED server: drives several PWM pins from timestamped frames sent by a phone.
//
// Why timestamps: BLE delivers packets in order, but with uneven delay (jitter).
// Instead of applying values the moment they arrive, the phone sends frames a bit
// AHEAD of the music (e.g. 200 ms) and tags each one with the song position
// it belongs to. The ESP buffers them and plays each frame when its time comes,
// so BLE jitter is hidden as long as it's smaller than the lead time.
//
// Protocol (one characteristic, write-without-response, little-endian):
//   SYNC   [0x01][u32 songPosMs]          "the song is at songPosMs right now"
//                                         (send on play / seek / resume; clears buffer)
//   FRAMES [0x02][u8 count] then count x { [u32 tMs][u8 duty * NUM_CH] }
//                                         tMs = song position the frame should show at
//   STOP   [0x03]                         clear buffer, all LEDs off

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_UUID    "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// ESP32-C3 has 6 LEDC channels. Avoid GPIO 2/8/9 (strapping) and 18/19 (USB).
const uint8_t PINS[] = {4, 5, 6, 7};
const uint8_t NUM_CH = sizeof(PINS);
const uint32_t PWM_FREQ = 5000;
const uint8_t PWM_BITS = 8; // duty 0..255

const uint8_t CMD_SYNC = 0x01;
const uint8_t CMD_FRAMES = 0x02;
const uint8_t CMD_STOP = 0x03;

struct Frame {
  uint32_t t;
  uint8_t duty[NUM_CH];
};

// FreeRTOS queue: BLE callback (producer) and loop() (consumer) run in different tasks.
QueueHandle_t frameQueue;
const int QUEUE_LEN = 256; // ~5 s of frames at 50 fps

volatile int32_t clockOffset = 0; // songPos = millis() + clockOffset
volatile bool playing = false;
uint32_t lastQueuedT = 0; // only touched in the BLE task

uint32_t songPosNow() { return millis() + clockOffset; }

void setAll(const uint8_t *duty) {
  for (uint8_t i = 0; i < NUM_CH; i++) ledcWrite(i, duty[i]);
}

void allOff() {
  uint8_t zero[NUM_CH] = {0};
  setAll(zero);
}

uint32_t readU32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    std::string v = c->getValue();
    const uint8_t *d = (const uint8_t *)v.data();
    size_t len = v.size();
    if (len == 0) return;

    switch (d[0]) {
    case CMD_SYNC:
      if (len < 5) return;
      xQueueReset(frameQueue);
      lastQueuedT = 0;
      clockOffset = (int32_t)(readU32(d + 1) - millis());
      playing = true;
      break;

    case CMD_FRAMES: {
      if (len < 2) return;
      uint8_t count = d[1];
      const size_t frameSize = 4 + NUM_CH;
      if (len < 2 + count * frameSize) return; // malformed
      const uint8_t *p = d + 2;
      for (uint8_t i = 0; i < count; i++, p += frameSize) {
        Frame f;
        f.t = readU32(p);
        memcpy(f.duty, p + 4, NUM_CH);
        // Keep the buffer strictly time-ordered: drop duplicates / stale frames.
        if (lastQueuedT != 0 && f.t <= lastQueuedT) continue;
        if (xQueueSend(frameQueue, &f, 0) == pdTRUE) lastQueuedT = f.t;
      }
      break;
    }

    case CMD_STOP:
      playing = false;
      xQueueReset(frameQueue);
      lastQueuedT = 0;
      allOff();
      break;
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { Serial.println("connected"); }
  void onDisconnect(BLEServer *s) override {
    Serial.println("disconnected");
    playing = false;
    xQueueReset(frameQueue);
    lastQueuedT = 0;
    allOff();
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_CH; i++) {
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PINS[i], i);
  }
  allOff();

  frameQueue = xQueueCreate(QUEUE_LEN, sizeof(Frame));

  BLEDevice::init("LED-Control");
  BLEDevice::setMTU(247); // more frames per write; the phone must also request MTU
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  BLECharacteristic *rx = service->createCharacteristic(
      CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("advertising as LED-Control");
}

void loop() {
  if (playing) {
    uint32_t now = songPosNow();
    Frame f, due;
    bool haveDue = false;
    // Pop every frame whose time has come; if several are due (we fell behind),
    // only the newest one is shown.
    while (xQueuePeek(frameQueue, &f, 0) == pdTRUE && (int32_t)(f.t - now) <= 0) {
      xQueueReceive(frameQueue, &due, 0);
      haveDue = true;
    }
    if (haveDue) setAll(due.duty);
  }
  delay(1); // 1 ms resolution, and lets the BLE task run
}
