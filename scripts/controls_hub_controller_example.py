#!/usr/bin/env python3
"""controls_hub_controller usage examples - no teleop, no ramping.

    python scripts/controls_hub_controller_example.py hold --port COM5 --slot 3 --kp 8 --kd 0.45
    python scripts/controls_hub_controller_example.py sine --port COM5 --slot 3 --hz 100
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from controls_hub_controller import ActuatorDesire, PlantSession  # noqa: E402


def cmd_hold(args: argparse.Namespace) -> None:
    """Example A: single MIT hold."""
    with PlantSession.connect(args.port) as session:
        session.set_actuator(
            args.slot, ActuatorDesire(position=args.position, velocity=0.0, kp=args.kp, kd=args.kd, torque=0.0)
        )
        print(f"Holding slot {args.slot} at p={args.position} kp={args.kp} kd={args.kd} for {args.seconds}s")
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            fb = session.read_feedback(timeout_s=1.0)
            act = fb.actuator(args.slot)
            print(
                f"tick={fb.tick} ack={fb.ack_seq} plant_block={fb.plant_block.name} "
                f"pos={act.position:+.4f} vel={act.velocity:+.4f}"
            )
            session.send_once()  # keep under the 500 ms host_stale watchdog
            time.sleep(0.1)


def cmd_sine(args: argparse.Namespace) -> None:
    """Example B: app-owned trajectory streamed at a fixed rate — no host-side ramp."""
    with PlantSession.connect(args.port) as session:
        t0 = time.perf_counter()

        def trajectory(_fb) -> dict:
            t = time.perf_counter() - t0
            pos = args.amplitude * math.sin(2 * math.pi * args.freq * t)
            return {args.slot: ActuatorDesire(position=pos, velocity=0.0, kp=args.kp, kd=args.kd, torque=0.0)}

        stop_at = time.perf_counter() + args.seconds
        session.run_at_hz(args.hz, trajectory, stop=lambda: time.perf_counter() >= stop_at)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--slot", type=int, default=3)
    sub = parser.add_subparsers(dest="mode", required=True)

    hold = sub.add_parser("hold")
    hold.add_argument("--position", type=float, default=0.0)
    hold.add_argument("--kp", type=float, default=8.0)
    hold.add_argument("--kd", type=float, default=0.45)
    hold.add_argument("--seconds", type=float, default=5.0)
    hold.set_defaults(func=cmd_hold)

    sine = sub.add_parser("sine")
    sine.add_argument("--amplitude", type=float, default=0.3)
    sine.add_argument("--freq", type=float, default=0.2)
    sine.add_argument("--kp", type=float, default=8.0)
    sine.add_argument("--kd", type=float, default=0.45)
    sine.add_argument("--hz", type=float, default=100.0)
    sine.add_argument("--seconds", type=float, default=10.0)
    sine.set_defaults(func=cmd_sine)

    args = parser.parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
