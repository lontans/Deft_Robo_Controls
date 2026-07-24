#!/usr/bin/env python3
"""Jetson-side hard-ESTOP wire sense (BOARD pin 16 = GPIO08).

PDU drives the net (active-low: 1=released/power allowed, 0=asserted).
Controls PB7 is input-only; this script mirrors that sense on the Jetson.

Usage (on Jetson):
  python3 jetson_estop_sense.py
  python3 jetson_estop_sense.py --pin 16 --seconds 30
  python3 jetson_estop_sense.py --once
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--pin",
        type=int,
        default=16,
        help="BOARD pin number (default 16 = GPIO08)",
    )
    ap.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Sample duration (0 = until Ctrl+C)",
    )
    ap.add_argument(
        "--hz",
        type=float,
        default=20.0,
        help="Sample rate",
    )
    ap.add_argument(
        "--once",
        action="store_true",
        help="Print one sample and exit",
    )
    args = ap.parse_args()

    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        print("Jetson.GPIO not available — run this on the Jetson", file=sys.stderr)
        return 2

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(args.pin, GPIO.IN)

    def sample() -> tuple[int, str]:
        level = int(GPIO.input(args.pin))
        # active-low wire
        meaning = "RELEASED (power allowed)" if level == 1 else "ASSERTED (cut)"
        return level, meaning

    try:
        if args.once:
            level, meaning = sample()
            print(f"pin{args.pin} level={level}  {meaning}")
            return 0

        print(
            f"Watching BOARD pin {args.pin} (active-low ESTOP) @ {args.hz:.1f} Hz — Ctrl+C to stop"
        )
        dt = 1.0 / max(args.hz, 1.0)
        t0 = time.time()
        last = None
        edges = 0
        while True:
            level, meaning = sample()
            now = time.time() - t0
            if level != last:
                if last is not None:
                    edges += 1
                print(f"t={now:7.2f}s  level={level}  {meaning}")
                last = level
            if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                break
            time.sleep(dt)
        print(f"done edges={edges} last_level={last}")
        return 0
    except KeyboardInterrupt:
        print("\nstopped")
        return 0
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    sys.exit(main())
