#!/usr/bin/env python3
"""HW smoke for vbeta PCB adapters. Owns COM exclusively — close dashboard first.

    python vbeta_arm_smoke.py --side left
    python vbeta_base_smoke.py
    python vbeta_neck_led_smoke.py
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from deft_controls_sdk.vbeta import (
    PcbArmDriver,
    PcbNeckDriver,
    PcbPlatformClient,
    PcbRobotSession,
    led_flash,
    led_off,
)


def arm_main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--apply-cfg", action="store_true", help="RAM-apply YAM product CFG")
    ap.add_argument("--hold-s", type=float, default=2.0)
    args = ap.parse_args(argv)

    with PcbRobotSession.connect(
        args.port, apply_yam_cfg=args.apply_cfg, stream_hz=40.0
    ) as session:
        arm = PcbArmDriver(session, side=args.side, skip_home_on_connect=True)
        arm.connect()
        q0 = arm.read("Position_Rad")
        print(f"{args.side} Position_Rad={np.array2string(q0, precision=3)}")
        target = q0.copy()
        target[0] = float(q0[0]) + 0.05
        print(f"small Goal_Position Δj0=+0.05 for {args.hold_s:.1f}s")
        arm.write("Goal_Position", target)
        time.sleep(args.hold_s)
        q1 = arm.read("Position_Rad")
        print(f"after: {np.array2string(q1, precision=3)}")
        arm.write("Zero_Torque", True)
        time.sleep(0.3)
        arm.disconnect()
    print("arm smoke done")
    return 0


def base_main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Base steer hold + drive creep smoke")
    ap.add_argument("--port", default=None)
    ap.add_argument("--apply-cfg", action="store_true")
    ap.add_argument("--creep", type=float, default=0.2, help="Bp* rad/s")
    ap.add_argument("--hold-s", type=float, default=1.5)
    args = ap.parse_args(argv)

    with PcbRobotSession.connect(
        args.port, apply_yam_cfg=args.apply_cfg, stream_hz=40.0
    ) as session:
        plat = PcbPlatformClient(session, use_neck=False)
        plat.connect()
        plat.send_target_state(
            {"BwC": 0.0, "BwR": 0.0, "BwL": 0.0},
            {"BpC": 0.0, "BpR": 0.0, "BpL": 0.0},
        )
        time.sleep(0.5)
        print("creep drive", args.creep)
        plat.send_target_state(
            {"BwC": 0.0, "BwR": 0.0, "BwL": 0.0},
            {"BpC": args.creep, "BpR": args.creep, "BpL": args.creep},
        )
        time.sleep(args.hold_s)
        plat.send_command(("base_cmd", 0.0, 0.0, 0.0))
        st = plat.get_state()
        print("state", {k: st[k] for k in ("bwc_angle", "bpc_velocity", "lift_unimplemented")})
        plat.disconnect()
    print("base smoke done")
    return 0


def neck_led_main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Neck DXL + SK9822 LED smoke")
    ap.add_argument("--port", default=None)
    ap.add_argument("--hold-s", type=float, default=2.0)
    args = ap.parse_args(argv)

    with PcbRobotSession.connect(args.port, stream_hz=40.0) as session:
        led_flash(session, brightness=8, send=False)
        session.send_once()
        neck = PcbNeckDriver(session)
        neck.go_to(0.0, 0.0)
        session.send_once()
        time.sleep(args.hold_s)
        neck.go_to(5.0, -5.0)
        session.send_once()
        time.sleep(args.hold_s)
        neck.disable()
        led_off(session, send=False)
        session.send_once()
    print("neck/led smoke done")
    return 0


if __name__ == "__main__":
    # Allow `python vbeta_arm_smoke.py` when this file is copied/symlinked;
    # default entry is arm when run as vbeta_smoke_lib — prefer dedicated scripts.
    raise SystemExit(arm_main())
