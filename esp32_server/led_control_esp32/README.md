# LED Control — ESP32-C3 BLE dimmer

Firmware for the ESP32-C3 SuperMini (HW-466AB). A phone (or any BLE client)
connects, picks which GPIOs have LEDs on them, and sends brightness fades.
All LED pins are driven by one PWM channel, so **every LED always shows the
same brightness**.

## Device behaviour

| When | What happens |
|---|---|
| Power-up | Each safe GPIO (list below) is turned on for 200 ms, one at a time, in order. Takes ~3 s; BLE starts advertising after that. |
| Idle | If no valid command arrives for **5 s**, all LEDs fade to 0 over 1 s. The next `BRIGHTNESS` command lights them again. |
| Disconnect | Advertising restarts. LEDs keep their state until the 5 s idle timeout dims them. |

The LED pin set starts **empty** after every boot and is not saved. A client
must send `PINS` before `BRIGHTNESS` has any visible effect.

## BLE layout

| | Value |
|---|---|
| Device name | `LED-Control` |
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| RX characteristic (client → ESP) | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` — write, write-without-response |
| TX characteristic (ESP → client) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` — notify, read |

Every message is a byte array whose first byte is the message type.
Multi-byte numbers are **little-endian**. Every message fits in the default
BLE MTU (20 bytes of payload), so the client doesn't have to raise the MTU.

## Connection handshake

1. Client connects and **subscribes to TX notifications**.
2. Client sends `HELLO` (optionally with its name).
3. ESP replies with `HELLO_REPLY`: protocol version, which pins are allowed,
   which pins are currently LEDs, and the current target brightness.
4. Client sends `PINS` if the active pins aren't what it wants, then
   `BRIGHTNESS` commands as needed.

Reading TX returns the **last message the ESP sent**. Right after connecting,
before any command, that is a `HELLO_REPLY`. Don't rely on it later; send
`HELLO` instead.

## Commands (client → ESP, write to RX)

### `0x01` HELLO

```
[0x01][name...]
```

`name` is optional ASCII and is only printed on the ESP's serial log. Keep it
≤ 19 characters so the message fits in the default MTU.
Answer: `HELLO_REPLY`.

### `0x02` BRIGHTNESS

```
[0x02][u8 level][u16 fadeMs]
```

Fades all LEDs from their current brightness to `level` (0 = off,
255 = full) linearly over `fadeMs` milliseconds (0 = instant, max 65535).
A new `BRIGHTNESS` in the middle of a fade starts from wherever the
current fade has reached. No response on success.

Examples:

| Bytes | Meaning |
|---|---|
| `02 FF 00 00` | full brightness, instantly |
| `02 80 F4 01` | half brightness over 500 ms |
| `02 00 E8 03` | off over 1000 ms |

### `0x03` PINS

```
[0x03][u8 count][count × u8 gpio]
```

Replaces the set of LED pins. Pins not in the list are switched off and
driven LOW; new pins immediately show the current brightness. `count = 0`
removes all LED pins. Duplicates are ignored.

The command is all-or-nothing: if any pin isn't on the safe list, nothing
changes and the ESP sends `ERROR` naming the first bad pin.
Answer: `PINS_OK` or `ERROR`.

Example: `03 03 04 05 06` → LEDs on GPIO 4, 5 and 6.

### Safe pins

Only these GPIOs are accepted: **0, 1, 3, 4, 5, 6, 7, 10, 20, 21**
(the same set is reported as `safeMask` in `HELLO_REPLY`).

| Blocked GPIO | Reason |
|---|---|
| 2, 8, 9 | Strapping pins (8 is also the onboard LED, 9 the BOOT button) |
| 11–17 | SPI flash |
| 18, 19 | Native USB (upload and serial log) |

GPIO21 flickers briefly at reset because the chip's boot log is printed on it.

## Responses (ESP → client, notify on TX)

### `0x81` HELLO_REPLY

```
[0x81][u8 version][u32 safeMask][u32 pinMask][u8 level]
```

| Field | Meaning |
|---|---|
| `version` | Protocol version, currently `1` |
| `safeMask` | GPIOs allowed in `PINS` (bit n set = GPIO n) |
| `pinMask` | GPIOs currently used as LEDs |
| `level` | Target brightness of the current/last fade (0–255) |

With the current firmware `safeMask` is `0x003004FB`.

### `0x83` PINS_OK

```
[0x83][u32 pinMask]
```

The new set of LED pins, as a bitmask.

### `0xEE` ERROR

```
[0xEE][u8 cmd][u8 code][u8 detail]
```

| `code` | Meaning | `detail` |
|---|---|---|
| 1 | Malformed: message too short for its type | 0 |
| 2 | Unsafe pin in `PINS` | the rejected GPIO number |
| 3 | Unknown command type (`cmd` = the byte received) | 0 |

## Keeping LEDs on

The idle timer is reset only by commands the ESP **accepts** (`HELLO`,
`BRIGHTNESS`, a valid `PINS`). Rejected commands don't count. To keep the
LEDs lit, send something at least every 5 s; resending the current
brightness with `fadeMs = 0` works:

```
02 <level> 00 00
```

## Example session

```
client → RX   01 70 68 6F 6E 65             HELLO "phone"
ESP    → TX   81 01 FB 04 30 00 00 00 00 00 00
                    version 1, safe pins 0x003004FB, no LED pins, level 0
client → RX   03 02 04 05                   PINS: GPIO 4 and 5
ESP    → TX   83 30 00 00 00                PINS_OK: mask 0x00000030
client → RX   02 FF E8 03                   fade to 255 over 1000 ms
client → RX   02 FF 00 00                   keep-alive every few seconds
…client stops sending…
                                            5 s later: LEDs fade to 0 over 1 s
```

## Building and uploading

See `platformio.ini`. From this folder:

```
pio run -t upload        # build + flash
pio device monitor       # serial log at 115200
```

If the upload fails or times out, put the board in flashing mode by hand:
hold **BOOT**, tap **RESET**, release **BOOT**, then upload again and press
**RESET** when it finishes.
