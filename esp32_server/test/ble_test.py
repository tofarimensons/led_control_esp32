"""BLE smoke test for the LED-Control firmware (see led_control_esp32/README.md).

Connects, does the HELLO handshake, sets the LED pins and plays a sequence of
brightness commands, printing each level as it is sent.

    pip install -r requirements.txt
    python ble_test.py                  # all safe pins, find device by name
    python ble_test.py --pins 4 5       # only GPIO 4 and 5
    python ble_test.py --address AA:BB:CC:DD:EE:FF
"""

import argparse
import asyncio
import struct
import sys

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "LED-Control"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

CMD_HELLO = 0x01
CMD_BRIGHTNESS = 0x02
CMD_PINS = 0x03
RSP_HELLO = 0x81
RSP_PINS_OK = 0x83
RSP_ERROR = 0xEE

ERROR_CODES = {1: "malformed", 2: "unsafe pin", 3: "unknown command"}

# (level, fadeMs, holdS). holdS is the wait after sending; it includes the fade
# and must stay below the firmware's 5 s idle timeout.
SEQUENCE = [
    # instant steps up
    (0, 0, 1.0),
    (32, 0, 1.0),
    (64, 0, 1.0),
    (128, 0, 1.0),
    (192, 0, 1.0),
    (255, 0, 1.5),
    # instant steps down
    (128, 0, 1.0),
    (16, 0, 1.0),
    (0, 0, 1.0),
    # fades
    (255, 2000, 2.5),
    (0, 2000, 2.5),
    (128, 500, 1.0),
    (255, 1000, 1.5),
    # interrupt a fade halfway: should reverse from ~128
    (0, 2000, 1.0),
    (255, 1000, 1.5),
    # fast blink
    (0, 0, 0.25),
    (255, 0, 0.25),
    (0, 0, 0.25),
    (255, 0, 0.25),
    (0, 0, 0.25),
    (255, 0, 0.25),
    # finish off
    (0, 1000, 1.5),
]


def mask_to_pins(mask: int) -> list[int]:
    return [n for n in range(32) if mask & (1 << n)]


def describe(data: bytes) -> str:
    if not data:
        return "empty"
    kind = data[0]
    if kind == RSP_HELLO and len(data) >= 11:
        version, safe, pins, level = struct.unpack_from("<BIIB", data, 1)
        return (f"HELLO_REPLY version={version} safe={mask_to_pins(safe)} "
                f"pins={mask_to_pins(pins)} level={level}")
    if kind == RSP_PINS_OK and len(data) >= 5:
        (pins,) = struct.unpack_from("<I", data, 1)
        return f"PINS_OK pins={mask_to_pins(pins)}"
    if kind == RSP_ERROR and len(data) >= 4:
        cmd, code, detail = data[1:4]
        return (f"ERROR cmd=0x{cmd:02X} code={code} "
                f"({ERROR_CODES.get(code, '?')}) detail={detail}")
    return f"unknown: {data.hex(' ')}"


class LedClient:
    def __init__(self, client: BleakClient):
        self.client = client
        self.responses: asyncio.Queue[bytes] = asyncio.Queue()

    def _on_notify(self, _sender, data: bytearray):
        print(f"  <- {bytes(data).hex(' ')}  {describe(bytes(data))}")
        self.responses.put_nowait(bytes(data))

    async def start(self):
        await self.client.start_notify(TX_UUID, self._on_notify)

    async def send(self, data: bytes):
        print(f"  -> {data.hex(' ')}")
        await self.client.write_gatt_char(RX_UUID, data, response=True)

    async def expect(self, kind: int, timeout: float = 3.0) -> bytes:
        data = await asyncio.wait_for(self.responses.get(), timeout)
        if data[0] != kind:
            raise RuntimeError(f"expected 0x{kind:02X}, got {describe(data)}")
        return data

    async def hello(self, name: str = "py-test") -> tuple[int, int, int, int]:
        await self.send(bytes([CMD_HELLO]) + name.encode("ascii")[:19])
        data = await self.expect(RSP_HELLO)
        return struct.unpack_from("<BIIB", data, 1)

    async def set_pins(self, pins: list[int]) -> int:
        await self.send(bytes([CMD_PINS, len(pins), *pins]))
        data = await self.expect(RSP_PINS_OK)
        return struct.unpack_from("<I", data, 1)[0]

    async def brightness(self, level: int, fade_ms: int = 0):
        await self.send(struct.pack("<BBH", CMD_BRIGHTNESS, level, fade_ms))


async def find_address(timeout: float) -> str:
    print(f"Scanning for '{DEVICE_NAME}' ({timeout:.0f} s)...")
    try:
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=timeout)
    except OSError as e:
        sys.exit(f"Can't start BLE scan ({e}). Is Bluetooth turned on?")
    if device is None:
        sys.exit(f"'{DEVICE_NAME}' not found. Is it powered and done blinking?")
    print(f"Found {device.name} at {device.address}")
    return device.address


async def run(args):
    address = args.address or await find_address(args.scan_timeout)

    async with BleakClient(address) as client:
        print(f"Connected to {address}")
        led = LedClient(client)
        await led.start()

        print("\nHELLO")
        version, safe_mask, pin_mask, level = await led.hello()
        if version != 1:
            print(f"WARNING: protocol version {version}, script expects 1")

        pins = args.pins if args.pins is not None else mask_to_pins(safe_mask)
        print(f"\nPINS {pins}")
        pin_mask = await led.set_pins(pins)

        print("\nPINS with an unsafe pin (GPIO 8), expecting ERROR")
        await led.send(bytes([CMD_PINS, 1, 8]))
        await led.expect(RSP_ERROR)

        for i in range(args.repeat):
            print(f"\nBrightness sequence (pass {i + 1}/{args.repeat}) "
                  f"on GPIO {mask_to_pins(pin_mask)}")
            for level, fade_ms, hold_s in SEQUENCE:
                pct = level * 100 / 255
                fade = f"fade {fade_ms} ms" if fade_ms else "instant"
                print(f"brightness {level:3d} ({pct:5.1f}%)  {fade}")
                await led.brightness(level, fade_ms)
                await asyncio.sleep(hold_s)

        print("\nHELLO again to check final state")
        _, _, pin_mask, level = await led.hello()
        if level != 0:
            print(f"WARNING: expected level 0, got {level}")

        await client.stop_notify(TX_UUID)
    print("\nDone.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", help="BLE address (skips scanning)")
    parser.add_argument("--pins", type=int, nargs="*",
                        help="LED GPIOs to use (default: every safe pin)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="how many times to play the sequence")
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
