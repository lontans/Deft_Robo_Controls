#!/usr/bin/env python3
"""HW smoke implementations for vbeta PCB adapters.

Prefer the unified CLI:

    python vbeta_smoke.py arm --side left --hold --hold-s 3
    python vbeta_smoke.py arm --side left --jog --joint 0 --delta 0.05
    python vbeta_smoke.py base
    python vbeta_smoke.py neck

Owns COM exclusively — disconnect the debug dashboard first.
Goals are soft-limit clamped via deft_controls_sdk.vbeta.yam_limits.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import numpy as np

from deft_controls_sdk.vbeta import (
    PcbArmDriver,
    PcbNeckDriver,
    PcbPlatformClient,
    PcbRobotSession,
    led_flash,
    led_off,
    plan_hold_q7,
    plan_jog_q7,
)


def _print_pdb(session: PcbRobotSession, label: str) -> None:
    st = getattr(session.hub, "pdb_status", lambda: None)()
    if st is None:
        print(f"{label}: pdb_status=None")
        return
    print(
        f"{label}: kill={st.kill_state}({st.kill_state_name}) "
        f"reason={st.kill_reason}({st.kill_reason_name}) "
        f"stale={st.stale_failsafe} soft_req={st.soft_kill_req}"
    )


def _service_loop(session: PcbRobotSession, hold_s: float) -> bool:
    """Tick soft-kill during hold. Returns True if parked early."""
    t_end = time.perf_counter() + float(hold_s)
    while time.perf_counter() < t_end:
        if session.service_soft_kill():
            print("soft_kill_park_if_requested → parked")
            session.send_once()
            return True
        time.sleep(0.05)
    return False


def plan_arm_smoke_target(
    q_fb: np.ndarray,
    *,
    side: str,
    mode: str,
    joint: int = 0,
    delta: float = 0.05,
) -> tuple[np.ndarray, str]:
    """Offline-testable goal planner (no COM). mode: hold|jog|legacy."""
    q0 = np.asarray(q_fb, dtype=np.float32).reshape(7)
    if mode == "hold":
        return plan_hold_q7(q0, side), "hold present (soft-clamped)"
    if mode == "jog":
        return plan_jog_q7(q0, side, joint=joint, delta=delta)
    # legacy: small +delta on joint 0 (clamped)
    return plan_jog_q7(q0, side, joint=0, delta=0.05)


def arm_main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--port",
        default=None,
        help="CDC port (Windows COM5 / Jetson /dev/ttyACM0). Default: auto-discover.",
    )
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--apply-cfg", action="store_true", help="RAM-apply YAM product CFG")
    ap.add_argument("--hold-s", type=float, default=2.0, help="Seconds to stream command")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument(
        "--hold",
        action="store_true",
        help="Soft-hold MIT at present FB (clamped)",
    )
    mode.add_argument(
        "--jog",
        action="store_true",
        help="Small relative jog on --joint (clamped; default Δ=0.05 rad)",
    )
    ap.add_argument("--joint", type=int, default=0, help="Arm-local joint index 0..6 for --jog")
    ap.add_argument("--delta", type=float, default=0.05, help="Jog delta rad (signed)")
    ap.add_argument(
        "--no-clamp",
        action="store_true",
        help="Disable PcbArmDriver soft clamps (not recommended)",
    )
    args = ap.parse_args(argv)

    if args.hold:
        smoke_mode = "hold"
    elif args.jog:
        smoke_mode = "jog"
    else:
        smoke_mode = "legacy"

    with PcbRobotSession.connect(
        args.port, apply_yam_cfg=args.apply_cfg, stream_hz=40.0
    ) as session:
        _print_pdb(session, "pre")
        arm = PcbArmDriver(
            session,
            side=args.side,
            skip_home_on_connect=True,
            clamp_goals=not args.no_clamp,
        )
        arm.connect()
        q0 = arm.read("Position_Rad")
        print(f"{args.side} Position_Rad={np.array2string(q0, precision=3)}")
        target, note = plan_arm_smoke_target(
            q0,
            side=args.side,
            mode=smoke_mode,
            joint=args.joint,
            delta=args.delta,
        )
        print(f"mode={smoke_mode} note={note or 'ok'}")
        print(f"Goal_Position={np.array2string(target, precision=3)} for {args.hold_s:.1f}s")
        arm.write("Goal_Position", target)
        _service_loop(session, args.hold_s)
        q1 = arm.read("Position_Rad")
        print(f"after: {np.array2string(q1, precision=3)}")
        _print_pdb(session, "post")
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
    raise SystemExit(arm_main())
