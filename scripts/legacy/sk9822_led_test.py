#!/usr/bin/env python3
"""
SK9822 LED strip bench test over USB CDC (562 B plant command image).

Patches leds[0] at offset 528 (host-exchange v1):
  bits  0-4:  mode (0=test knight-rider, 1=off)
  bits  5-9:  master_brightness (0-31, SK9822 global brightness)
  bits 10-15: led_count (0 = use firmware LED_STRIP_MAX default)

Examples:
  python scripts/sk9822_led_test.py --port COM9
  python scripts/sk9822_led_test.py --port COM9 --mode 0 --brightness 8 --count 0
  python scripts/sk9822_led_test.py --port COM9 --mode 1
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

HOST_COMMAND_MAGIC = 0x434D4448
HOST_LAYOUT_VERSION = 1
IMAGE_BYTES = 562
LED_CMD_OFF = 528
USB_BAUD = 115200
DEFAULT_HZ = 40.0


def pack_led_word(mode: int, brightness: int, led_count: int) -> int:
    return (
        (mode & 0x1F)
        | ((brightness & 0x1F) << 5)
        | ((led_count & 0x3F) << 10)
    )


def build_led_command(
    seq: int,
    mode: int,
    brightness: int,
    led_count: int,
) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    struct.pack_into("<H", buf, LED_CMD_OFF, pack_led_word(mode, brightness, led_count))
    return bytes(buf)


def main() -> None:
    ap = argparse.ArgumentParser(description="SK9822 LED strip test via MCU USB")
    ap.add_argument("--port", required=True, help="USB CDC port (e.g. COM9)")
    ap.add_argument("--baud", type=int, default=USB_BAUD)
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="Host command rate")
    ap.add_argument("--mode", type=int, default=0, help="LED mode (0=test scan, 1=off)")
    ap.add_argument("--brightness", type=int, default=8,
                    help="Global brightness 0-31 (keep low on long strips)")
    ap.add_argument("--count", type=int, default=0,
                    help="LED chain length 0-63; 0 = firmware LED_STRIP_MAX")
    args = ap.parse_args()

    mode = max(0, min(31, args.mode))
    brightness = max(0, min(31, args.brightness))
    led_count = max(0, min(63, args.count))
    dt = 1.0 / max(args.hz, 1.0)

    print(f"Opening {args.port} @ {args.baud}")
    print(
        f"LED cmd: mode={mode} brightness={brightness}/31 "
        f"count={led_count} ({'firmware default' if led_count == 0 else 'host'}) @ {args.hz:.0f} Hz"
    )
    print("  mode 0 = red dot scan (test wiring + chain length)")
    print("  mode 1 = all off")
    print("Ctrl+C to stop")

    seq = 1
    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.3)
        try:
            while True:
                ser.write(build_led_command(seq, mode, brightness, led_count))
                ser.flush()
                seq = (seq + 1) & 0xFFFFFFFF
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\nSending mode=off...")
            ser.write(build_led_command(seq, 1, 0, led_count))
            ser.flush()
            print("Done.")


if __name__ == "__main__":
    main()
