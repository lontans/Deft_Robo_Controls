#!/usr/bin/env python3
"""Single-DXL range check — move only one servo so you can see awkward limits.

  cd scripts
  python _tmp_dxl_one.py --slot 0          # id 1, plant_config ranges
  python _tmp_dxl_one.py --slot 1          # id 2
  python _tmp_dxl_one.py --slot 0 --hold   # hold mid only (no sweep)

Reads present position first, then slow bounce within configured pos_min/max
at π/4 rad/s. Prints cmd vs fb every ~0.5 s. No actuator rx_sim / no LED / no RS02.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List, Optional

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, ServoDesire
from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    parse_feedback_header,
    parse_servo_feedback,
)

# Mirrors App/Src/plant/plant_config.c servo_table[]
SERVO_CFG = (
    {"slot": 0, "id": 1, "pos_min": 1024, "pos_max": 3072, "label": "servo_table[0] id=1"},
    {"slot": 1, "id": 2, "pos_min": 700, "pos_max": 2500, "label": "servo_table[1] id=2"},
)

DXL_TICKS_PER_REV = 4096.0
DEFAULT_RATE_RAD_S = math.pi / 4.0


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _drain_all(hub: ControlsPcbHub):
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        yield frame


def _mid(cfg: dict) -> int:
    return (int(cfg["pos_min"]) + int(cfg["pos_max"])) // 2


def sample_servo_fb(
    hub: ControlsPcbHub,
    slot: int,
    *,
    servo_id: int,
    hold_pos: Optional[int] = None,
    timeout_s: float = 3.0,
    hz: float = 40.0,
) -> Optional[int]:
    """Arm session and return present_position.

    Until FB is known, torque stays OFF so we never slew to a guessed mid.
    Once FB arrives, hold at that pose (or hold_pos if given) with torque ON.
    """
    dt = 1.0 / hz
    deadline = time.perf_counter() + timeout_s
    last: Optional[int] = None
    next_t = time.perf_counter()
    frames = 0
    while time.perf_counter() < deadline:
        if last is None:
            # Discover present position without commanding a distant goal.
            desire = ServoDesire(
                servo_id=servo_id,
                native_step_position=0,
                torque_enable=False,
                operating_mode=3,
            )
        else:
            goal = int(hold_pos) if hold_pos is not None else int(last)
            desire = ServoDesire(
                servo_id=servo_id,
                native_step_position=goal,
                torque_enable=True,
                operating_mode=3,
            )
        hub.set_servo(slot, desire, send=False)
        hub.set_servo(1 - slot, ServoDesire(servo_id=0), send=False)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        _conn(hub).send_once()

        for raw in _drain_all(hub):
            hdr = parse_feedback_header(raw)
            if hdr is None or hdr.get("is_debug"):
                continue
            frames += 1
            sv = parse_servo_feedback(raw, slot)
            if sv is None:
                continue
            pos = int(sv["present_position"])
            mid = int(sv.get("motor_source_id", 0) or 0)
            if mid in (0, servo_id) or pos != 0:
                last = pos

        # After first FB + short settle with torque on at present, done.
        if last is not None and hold_pos is None and frames >= 8:
            # Re-hold at present with torque for a few frames then return.
            hold_pos = last

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

        if last is not None and hold_pos is not None and frames >= 12:
            break

    if last is None:
        print(f"  arm: plant_fb_frames={frames} (no servo present_position yet)")
    return last


def run_one(
    hub: ControlsPcbHub,
    *,
    cfg: dict,
    seconds: float,
    hz: float,
    rate_rad_s: float,
    hold_only: bool,
    start_from_fb: bool,
) -> int:
    slot = int(cfg["slot"])
    sid = int(cfg["id"])
    lo = int(cfg["pos_min"])
    hi = int(cfg["pos_max"])
    mid = _mid(cfg)
    rate_steps_s = abs(rate_rad_s) / (2.0 * math.pi) * DXL_TICKS_PER_REV

    print(
        f"\n=== {cfg['label']}  slot={slot}  table range {lo}..{hi}  mid={mid} ==="
    )
    print(f"  rate={rate_rad_s:.4f} rad/s ({rate_steps_s:.0f} tick/s)  other slot disarmed")

    # Discover present pose with torque OFF, then hold there (no mid jump).
    fb0 = sample_servo_fb(hub, slot, servo_id=sid, timeout_s=1.5, hz=hz)
    if fb0 is None:
        print("  FAIL: no servo FB (check DXL power/ID/bus)")
        return 1
    print(f"  present_position after arm: fb={fb0}")

    if start_from_fb:
        cmd = float(max(lo, min(hi, fb0)))
        if int(cmd) != fb0:
            print(f"  note: FB {fb0} outside table — starting at clamped {int(cmd)}")
        # Soft hold at start pose before sweep.
        sample_servo_fb(
            hub, slot, servo_id=sid, hold_pos=int(cmd), timeout_s=0.4, hz=hz
        )
    else:
        cmd = float(mid)
        sample_servo_fb(
            hub, slot, servo_id=sid, hold_pos=mid, timeout_s=0.6, hz=hz
        )

    if hold_only:
        print(f"  HOLD at {int(cmd)} for {seconds:.1f}s (watch for awkward torque)")
    else:
        print(f"  SWEEP bounce {lo}..{hi} starting at {int(cmd)} for {seconds:.1f}s")

    direction = 1.0 if cmd < hi else -1.0
    dt = 1.0 / hz
    t0 = time.perf_counter()
    t_end = t0 + seconds
    next_t = t0
    last_print = 0.0
    fb_samples: List[int] = []
    err_n = 0
    last_fb = fb0

    while time.perf_counter() < t_end:
        if not hold_only:
            nxt = cmd + direction * rate_steps_s * dt
            if nxt >= hi:
                nxt = float(hi)
                direction = -1.0
            elif nxt <= lo:
                nxt = float(lo)
                direction = 1.0
            cmd = nxt

        goal = int(round(cmd))
        hub.set_servo(
            slot,
            ServoDesire(
                servo_id=sid,
                native_step_position=goal,
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )
        hub.set_servo(1 - slot, ServoDesire(servo_id=0), send=False)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )

        for raw in _drain_all(hub):
            hdr = parse_feedback_header(raw)
            if hdr is None or hdr.get("is_debug"):
                continue
            sv = parse_servo_feedback(raw, slot)
            if sv is None:
                continue
            last_fb = int(sv["present_position"])
            fb_samples.append(last_fb)
            if sv.get("hw_err_any"):
                err_n += 1

        _conn(hub).send_once()

        now = time.perf_counter()
        if now - last_print >= 0.5:
            last_print = now
            at_lo = abs(goal - lo) <= 2
            at_hi = abs(goal - hi) <= 2
            edge = " [LO]" if at_lo else (" [HI]" if at_hi else "")
            print(
                f"  t={now - t0:5.1f}s  cmd={goal:4d}  fb={last_fb:4d}  "
                f"err={last_fb - goal:+5d}  dir={direction:+.0f}{edge}"
            )

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    if fb_samples:
        print(
            f"  done: fb_min={min(fb_samples)} fb_max={max(fb_samples)}  "
            f"hw_err_flags={err_n}  table={lo}..{hi}"
        )
    else:
        print("  done: no FB samples")

    # Disarm
    _conn(hub).clear_servos(send=False)
    _conn(hub).send_once()
    time.sleep(0.1)
    _conn(hub).send_once()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Single DXL range check")
    ap.add_argument("--port", default=None)
    ap.add_argument(
        "--slot",
        type=int,
        required=True,
        choices=(0, 1),
        help="Which plant servo slot to move (0=id1, 1=id2)",
    )
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_RAD_S)
    ap.add_argument(
        "--hold",
        action="store_true",
        help="Hold at FB (or mid) instead of sweeping the table range",
    )
    ap.add_argument(
        "--from-mid",
        action="store_true",
        help="Start sweep at table mid instead of current FB",
    )
    args = ap.parse_args(argv)

    cfg = SERVO_CFG[int(args.slot)]
    print(
        f"Single-DXL check: slot={cfg['slot']} id={cfg['id']}  "
        f"configured {cfg['pos_min']}..{cfg['pos_max']}  "
        f"(other DXL disarmed)"
    )

    with ControlsPcbHub.connect(args.port) as hub:
        hub.recover()
        hub.set_rx_sim_mask(0)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )
        time.sleep(0.1)
        return run_one(
            hub,
            cfg=cfg,
            seconds=float(args.seconds),
            hz=float(args.hz),
            rate_rad_s=float(args.rate),
            hold_only=bool(args.hold),
            start_from_fb=not bool(args.from_mid),
        )


if __name__ == "__main__":
    raise SystemExit(main())
