"""Non-interactive single-slot plant jog — agent/script friendly hello-world.

Respects YAM joint soft limits from ``yam_limits`` (MuJoCo J1-J6 + bench J7, mirrored
onto arm2 J8-J14 — dual identical-arm bench, Jul 2026).

Examples:
  python scripts/control_hub.py --hello-world --port COM5 --joint 7 --dry-run
  python scripts/control_hub.py hello-world --port COM5 --slot 6 --delta 0.3 --hold-s 1
  python scripts/control_hub.py hello-world --port COM5 --joint 5 --delta -0.2
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

from controls_pcb_host.actuator_config import refresh_host_table_from_mcu, slot_config
from controls_pcb_host.feedback import parse_actuator_feedback
from controls_pcb_host.protocol import ACTUATOR_COUNT
from controls_pcb_host.protocol.can_bus import can_bus_label
from controls_pcb_host.session import PcbSession

from control_hub.link import (
    PlantRuntimeError,
    ensure_plant_runtime,
    release_bench_gates,
    warmup_plant_actuators,
)
from control_hub.teleop import defaults as D
from control_hub.yam_limits import (
    DEFAULT_DELTA_CAP,
    DEFAULT_DELTA_FRAC,
    arm_local_joint,
    clamp_delta,
    format_limits_table,
    joint_to_slot,
    limit_for_slot,
    load_yam_limits,
    reasonableness_notes,
    slot_to_joint,
)


def _poll_slot(session: PcbSession, slot: int) -> Optional[dict]:
    latest = None
    frame = session.reader.pop()
    while frame is not None:
        latest = frame
        frame = session.reader.pop()
    if latest is None:
        return None
    return parse_actuator_feedback(latest, slot=slot)


def _stream(
    session: PcbSession,
    slot: int,
    *,
    pos: float,
    kp: float,
    kd: float,
    hz: float,
    seconds: float,
) -> Optional[dict]:
    """Send plant desires at hz; return last feedback sample."""
    dt = 1.0 / hz
    last: Optional[dict] = None
    n = max(1, int(seconds * hz))
    for _ in range(n):
        session.send_plant({slot: (pos, 0.0, kp, kd, 0.0)})
        time.sleep(dt)
        fb = _poll_slot(session, slot)
        if fb is not None:
            last = fb
    return last


def _slew_to(
    session: PcbSession,
    slot: int,
    *,
    cmd: float,
    target: float,
    slew: float,
    kp: float,
    kd: float,
    hz: float,
) -> tuple[float, Optional[dict]]:
    """Slew cmd toward target; return (final_cmd, last_fb)."""
    dt = 1.0 / hz
    last: Optional[dict] = None
    while abs(target - cmd) > 1e-4:
        step = slew * dt
        delta = target - cmd
        if abs(delta) <= step:
            cmd = target
        else:
            cmd += math.copysign(step, delta)
        session.send_plant({slot: (cmd, 0.0, kp, kd, 0.0)})
        time.sleep(dt)
        fb = _poll_slot(session, slot)
        if fb is not None:
            last = fb
    return cmd, last


def _resolve_slot(*, slot: Optional[int], joint: Optional[int]) -> int:
    max_joint = ACTUATOR_COUNT  # dual arm: 1..7 arm1, 8..14 arm2
    if joint is not None and slot is not None and joint_to_slot(joint) != slot:
        raise ValueError(f"--joint {joint} conflicts with --slot {slot}")
    if joint is not None:
        if joint < 1 or joint > max_joint:
            raise ValueError(f"--joint must be 1..{max_joint}, got {joint}")
        return joint_to_slot(joint)
    if slot is None:
        return 0
    if slot < 0 or slot > max_joint - 1:
        raise ValueError(f"--slot must be 0..{max_joint - 1} for YAM map, got {slot}")
    return slot


def print_yam_limits(xml_path: Optional[Path] = None) -> None:
    limits = load_yam_limits(xml_path)
    print(format_limits_table(limits))
    print()
    print(reasonableness_notes())


def run_hello_world(
    port: str,
    *,
    slot: Optional[int] = None,
    joint: Optional[int] = None,
    delta: float = 0.25,
    to: Optional[float] = None,
    slew: float = 0.30,
    hold_s: float = 1.0,
    hz: float = 40.0,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
    return_home: bool = True,
    skip_arm: bool = False,
    dry_run: bool = False,
    no_limit_clamp: bool = False,
    absolute_limits: bool = False,
    i_know_zeros: bool = False,
    xml_path: Optional[Path] = None,
    show_limits: bool = False,
) -> int:
    """
    Enable one configured plant slot, jog by ``delta`` rad (limit-aware), optionally return.

    Uses MCU actuator table (``config show`` / ``config set``). Returns 0 on PASS.
    ``dry_run`` prints the plan (and optional limit table) without opening motion.

    ``to`` requests an absolute motor-frame target instead of a relative ``delta``; it
    requires ``absolute_limits=True`` and ``i_know_zeros=True`` since motor encoder frame
    is not yet aligned to the MuJoCo model frame (see docs/plan-yam-joint-commands.md §6).
    """
    if to is not None:
        if not absolute_limits:
            print("FAIL: --to requires --absolute (joint goto) / --absolute-limits (hello-world)")
            return 2
        if not i_know_zeros:
            print(
                "FAIL: absolute goto (--to) refused without --i-know-zeros — motor zero "
                "!= model zero until calibrated (docs/plan-yam-joint-commands.md §6)"
            )
            return 2

    try:
        slot_i = _resolve_slot(slot=slot, joint=joint)
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2

    limits = load_yam_limits(xml_path)
    lim = limit_for_slot(slot_i, limits)
    joint_i = slot_to_joint(slot_i)

    if show_limits or dry_run:
        print_yam_limits(xml_path)
        print()

    # Span-based default shrink when caller passes a large delta on a short joint.
    span_cap = min(DEFAULT_DELTA_CAP, max(0.05, DEFAULT_DELTA_FRAC * lim.span))

    def _plan_delta(reference: float) -> float:
        raw = (to - reference) if to is not None else delta
        d = raw
        if not no_limit_clamp and abs(d) > span_cap + 1e-9:
            d = math.copysign(span_cap, d)
        return d

    label_joint = f"J{joint_i}({lim.name})"
    print(
        f"hello-world: port={port} slot={slot_i} {label_joint}  "
        f"soft=[{lim.soft()[0]:+.3f},{lim.soft()[1]:+.3f}]  source={lim.source}"
    )
    if to is not None:
        print(f"  absolute target: to={to:+.4f} rad (motor frame; per-call step capped to {span_cap:.3f})")

    if dry_run:
        # Without live fb, show clamp assuming mid-range (and EE-joint hard-stop note).
        assume = (
            lim.mid
            if arm_local_joint(lim.joint) != 7
            else max(lim.lo + 0.05, min(lim.hi - 0.05, 1.20))
        )
        planned_delta = _plan_delta(assume)
        d, tgt, note = clamp_delta(
            assume, planned_delta, lim, absolute=absolute_limits
        )
        raw_delta = (to - assume) if to is not None else delta
        print(f"  dry-run assume_fb={assume:+.4f}  raw_delta={raw_delta:+.3f}  "
              f"planned_delta={planned_delta:+.3f}  clamped_delta={d:+.3f}  target={tgt:+.4f}")
        if note:
            print(f"  note: {note}")
        print("  dry-run: no plant session / no motion")
        return 0

    with PcbSession(port) as session:
        with session.rx_pump():
            try:
                ensure_plant_runtime(session, label="hello-world", bus=None)
            except PlantRuntimeError as exc:
                print(f"FAIL: {exc}")
                return 1

            refresh_host_table_from_mcu(session)
            cfg = slot_config(slot_i)
            if not cfg.enabled:
                print(f"FAIL: slot {slot_i} is disabled — config set --slot {slot_i} first")
                return 1
            if cfg.protocol_name not in ("damiao", "robstride"):
                print(
                    f"FAIL: slot {slot_i} protocol={cfg.protocol_name!r} (need damiao|robstride)"
                )
                return 1

            if kp is None:
                # Per-slot default (YAM J1-J4 need more than the flat wrist-tuned DM_KP;
                # see teleop/defaults.py SLOT_KP). Same table for damiao and robstride.
                kp = D.SLOT_KP[min(slot_i, len(D.SLOT_KP) - 1)]
            if kd is None:
                kd = D.DM_KD if cfg.protocol_name == "damiao" else D.KD

            label = (
                f"{label_joint} slot{slot_i} {can_bus_label(cfg.bus)} "
                f"{cfg.protocol_name} id=0x{cfg.motor_id:02X}"
            )
            request_label = f"to={to:+.3f}" if to is not None else f"delta={delta:+.3f}"
            print(
                f"  target: {label}  {request_label} rad  "
                f"slew={slew:.2f}  kp={kp:.1f} kd={kd:.2f}"
            )

            if not skip_arm:
                warmup_plant_actuators(session, [slot_i])

            # Sync feedback (kp=0)
            fb = None
            for _ in range(int(hz * 1.5)):
                session.send_plant({slot_i: (1e-6, 0.0, 0.0, 0.0, 0.0)})
                time.sleep(1.0 / hz)
                sample = _poll_slot(session, slot_i)
                if sample is not None:
                    fb = sample
            if fb is None:
                print("FAIL: no actuator feedback on slot — check config / bus / power")
                release_bench_gates(session)
                return 1

            start = float(fb["position"])
            err0 = int(fb.get("fault", 0)) & 0xF
            print(f"  synced fb={start:+.4f} rad  err=0x{err0:X}")

            planned_delta = _plan_delta(start)
            raw_request = (to - start) if to is not None else delta
            if no_limit_clamp:
                use_delta = planned_delta
                target = start + use_delta
                note = "limits disabled (--no-limit-clamp)"
            else:
                use_delta, target, note = clamp_delta(
                    start, planned_delta, lim, absolute=absolute_limits
                )
            if note:
                print(f"  limits: {note}")
            if abs(use_delta) < 1e-4 and abs(raw_request) > 1e-4:
                print(
                    f"FAIL: no safe delta from fb={start:+.4f} inside "
                    f"[{lim.lo:+.3f},{lim.hi:+.3f}] — move joint off stop or flip --delta"
                )
                release_bench_gates(session)
                return 1

            print(f"  slew {start:+.4f} -> {target:+.4f} (delta={use_delta:+.4f}) ...")
            cmd, fb = _slew_to(
                session,
                slot_i,
                cmd=start,
                target=target,
                slew=slew,
                kp=kp,
                kd=kd,
                hz=hz,
            )
            fb = _stream(
                session, slot_i, pos=cmd, kp=kp, kd=kd, hz=hz, seconds=hold_s
            ) or fb
            mid = float(fb["position"]) if fb else float("nan")
            tor = float(fb.get("torque", 0.0)) if fb else float("nan")
            print(f"  at target: cmd={cmd:+.4f} fb={mid:+.4f} torque={tor:+.3f}")

            if return_home:
                print(f"  return -> {start:+.4f} ...")
                cmd, fb = _slew_to(
                    session,
                    slot_i,
                    cmd=cmd,
                    target=start,
                    slew=slew,
                    kp=kp,
                    kd=kd,
                    hz=hz,
                )
                fb = _stream(
                    session, slot_i, pos=cmd, kp=kp, kd=kd, hz=hz, seconds=0.4
                ) or fb

            end = float(fb["position"]) if fb else float("nan")
            err1 = int(fb.get("fault", 0)) & 0xF if fb else -1
            moved = abs(mid - start) if fb else 0.0
            print(
                f"  done: start={start:+.4f} peak_fb={mid:+.4f} end={end:+.4f} "
                f"moved={moved:.4f} err=0x{err1:X}"
            )

            # Soft disable
            for _ in range(max(8, int(hz * 0.3))):
                hold = end if fb else cmd
                session.send_plant({slot_i: (hold, 0.0, 0.0, 0.0, 0.0)})
                time.sleep(1.0 / hz)
            release_bench_gates(session)

            # Pass if we saw motion in the commanded direction (or tiny delta hold).
            if abs(use_delta) < 0.02:
                ok = fb is not None
            else:
                ok = moved >= min(0.05, abs(use_delta) * 0.25)
            if ok:
                print(f"PASS: hello-world {label}")
                return 0

            # Distinguish hard-stop fight (high |τ|, tiny move) from comms death.
            if fb is not None and abs(tor) >= 4.0 and moved < 0.05:
                print(
                    f"FAIL: limit fight? moved={moved:.4f} |torque|={abs(tor):.2f} "
                    f"(want ~{abs(use_delta):.3f}) — joint against stop / wrong delta sign"
                )
            else:
                print(
                    f"FAIL: little/no motion (moved={moved:.4f}, want ~{abs(use_delta):.3f}) "
                    "— enable/TIMEOUT/bus?"
                )
            return 1
