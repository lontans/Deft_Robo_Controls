#!/usr/bin/env python3
"""Jetson-driven hard-ESTOP (BOARD pin 16 = GPIO08) — active-low wire drive.

Drives the hard-ESTOP net that Controls senses on PB7 (MCU is input-only).
  assert  → pin LOW  (cut / estop_sense=0)
  release → pin HIGH (power allowed / estop_sense=1)

Usage (on Jetson):
  python3 jetson_estop_drive.py assert --seconds 5   # hold LOW 5s (keeps line claimed)
  python3 jetson_estop_drive.py release --seconds 5
  python3 jetson_estop_drive.py pulse --assert-s 1.0 --release-s 1.0 --cycles 3
  python3 jetson_estop_drive.py hold --level 0 --seconds 10
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pin", type=int, default=16, help="BOARD pin (default 16)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_assert = sub.add_parser("assert", help="Drive LOW (hard ESTOP asserted)")
    p_assert.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Hold while alive (0=drive once and exit — line releases on exit)",
    )
    p_release = sub.add_parser("release", help="Drive HIGH (released)")
    p_release.add_argument("--seconds", type=float, default=0.0)

    p_hold = sub.add_parser("hold", help="Drive raw level")
    p_hold.add_argument("--level", type=int, choices=(0, 1), required=True)
    p_hold.add_argument("--seconds", type=float, default=0.0)

    p_pulse = sub.add_parser("pulse", help="Toggle assert/release N times")
    p_pulse.add_argument("--assert-s", type=float, default=1.0)
    p_pulse.add_argument("--release-s", type=float, default=1.0)
    p_pulse.add_argument("--cycles", type=int, default=3)
    p_pulse.add_argument(
        "--leave",
        choices=("release", "assert"),
        default="release",
        help="Level left after the last cycle",
    )

    args = ap.parse_args()

    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        print("Jetson.GPIO not available — run this on the Jetson", file=sys.stderr)
        return 2

    GPIO.setmode(GPIO.BOARD)
    # Output drive — Jetson owns the hard-ESTOP wire for this bench test.
    GPIO.setup(args.pin, GPIO.OUT, initial=GPIO.HIGH)

    def drive(level: int, label: str) -> None:
        GPIO.output(args.pin, GPIO.HIGH if level else GPIO.LOW)
        print(f"pin{args.pin} drive={level}  {label}", flush=True)

    def hold_for(seconds: float) -> None:
        if seconds <= 0:
            return
        print(f"holding {seconds:.1f}s (Ctrl+C → release HIGH)", flush=True)
        t0 = time.time()
        while time.time() - t0 < seconds:
            time.sleep(0.05)

    try:
        if args.cmd == "assert":
            drive(0, "ASSERTED (cut)")
            hold_for(args.seconds)
            return 0
        if args.cmd == "release":
            drive(1, "RELEASED (power allowed)")
            hold_for(args.seconds)
            return 0
        if args.cmd == "hold":
            drive(args.level, "HOLD")
            hold_for(args.seconds)
            return 0

        # pulse
        for i in range(max(1, args.cycles)):
            drive(0, f"ASSERTED cycle {i + 1}/{args.cycles}")
            time.sleep(max(0.05, args.assert_s))
            drive(1, f"RELEASED cycle {i + 1}/{args.cycles}")
            time.sleep(max(0.05, args.release_s))
        if args.leave == "assert":
            drive(0, "LEAVE ASSERTED")
        else:
            drive(1, "LEAVE RELEASED")
        return 0
    except KeyboardInterrupt:
        print("\nstopped — driving RELEASED")
        GPIO.output(args.pin, GPIO.HIGH)
        return 0
    finally:
        # No GPIO.cleanup() — cleanup floats the net; OS releases on process exit.
        pass


if __name__ == "__main__":
    sys.exit(main())
