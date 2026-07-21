"""``joint`` command group — status / goto thin wrappers over hello_world primitives.

Per docs/plan-yam-joint-commands.md §3.2 (P1). ``goto`` reuses ``hello_world.run_hello_world``
verbatim (same enable -> jog -> return codepath, same soft-limit clamp) so there is exactly
one place that talks to the plant path. ``status`` is a read-only one-shot fb poll (kp=0,
no motion) used to check position / distance-to-soft-limit without a jog.

``joint home`` (P2) is not implemented yet — deferred, see plan §3.2.

``run_joint_goto_braced`` (bench diagnostic, Jul 2026) exists because the 562 B plant wire
image carries a desire for every slot on every frame; ``send_plant({slot: (...)})`` zero-fills
every *other* slot in that frame, and firmware does ``actuator_desire_live[i] =
actuator_desire_stage[i]`` for all slots on receipt (App/Src/plant/actuator.c). So a normal
single-slot jog silently drops every other enabled slot to kp=0 (limp) on every tick. On a
loaded multi-joint daisy chain (YAM CH1), that can make an otherwise-movable joint look stuck
because the rest of the structure is going soft under it each update. This holds every other
currently-enabled slot at its captured start position while jogging the target slot, all in
one combined ``send_plant`` call per tick.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Dict, Optional

from controls_pcb_host.actuator_config import host_table, refresh_host_table_from_mcu, slot_config
from controls_pcb_host.feedback import parse_actuator_feedback
from controls_pcb_host.protocol.can_bus import can_bus_label
from controls_pcb_host.session import PcbSession

from control_hub.hello_world import _poll_slot, _resolve_slot, print_yam_limits, run_hello_world
from control_hub.link import (
    PlantRuntimeError,
    ensure_plant_runtime,
    release_bench_gates,
    warmup_plant_actuators,
)
from control_hub.teleop import defaults as D
from control_hub.yam_limits import clamp_delta, limit_for_slot, load_yam_limits, slot_to_joint


def run_joint_goto(
    port: str,
    *,
    slot: Optional[int] = None,
    joint: Optional[int] = None,
    delta: Optional[float] = None,
    to: Optional[float] = None,
    absolute: bool = False,
    i_know_zeros: bool = False,
    slew: float = 0.30,
    hold_s: float = 1.0,
    hz: float = 40.0,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
    no_return: bool = False,
    skip_arm: bool = False,
    dry_run: bool = False,
    no_limit_clamp: bool = False,
    xml_path: Optional[Path] = None,
    show_limits: bool = False,
) -> int:
    """Alias of hello-world's relative/absolute jog (same codepath, see module docstring)."""
    if delta is not None and to is not None:
        print("FAIL: joint goto takes --delta or --to, not both")
        return 2
    if delta is None and to is None:
        print("FAIL: joint goto requires --delta D (relative) or --to RAD --absolute")
        return 2

    return run_hello_world(
        port,
        slot=slot,
        joint=joint,
        delta=delta if delta is not None else 0.0,
        to=to,
        slew=slew,
        hold_s=hold_s,
        hz=hz,
        kp=kp,
        kd=kd,
        return_home=not no_return,
        skip_arm=skip_arm,
        dry_run=dry_run,
        no_limit_clamp=no_limit_clamp,
        absolute_limits=absolute,
        i_know_zeros=i_know_zeros,
        xml_path=xml_path,
        show_limits=show_limits,
    )


def run_joint_status(
    port: str,
    *,
    slot: Optional[int] = None,
    joint: Optional[int] = None,
    hz: float = 40.0,
    xml_path: Optional[Path] = None,
) -> int:
    """One-shot read-only fb + soft-limit distance (kp=0 stream; no motion)."""
    try:
        slot_i = _resolve_slot(slot=slot, joint=joint)
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2

    limits = load_yam_limits(xml_path)
    lim = limit_for_slot(slot_i, limits)
    joint_i = slot_to_joint(slot_i)
    lo, hi = lim.soft()
    label_joint = f"J{joint_i}({lim.name})"

    with PcbSession(port) as session:
        with session.rx_pump():
            try:
                ensure_plant_runtime(session, label="joint-status", bus=None)
            except PlantRuntimeError as exc:
                print(f"FAIL: {exc}")
                return 1

            refresh_host_table_from_mcu(session)
            cfg = slot_config(slot_i)
            if not cfg.enabled:
                print(f"FAIL: slot {slot_i} is disabled — config set --slot {slot_i} first")
                return 1

            fb = None
            for _ in range(max(8, int(hz * 0.75))):
                session.send_plant({slot_i: (1e-6, 0.0, 0.0, 0.0, 0.0)})
                time.sleep(1.0 / hz)
                sample = _poll_slot(session, slot_i)
                if sample is not None:
                    fb = sample
            release_bench_gates(session)

            if fb is None:
                print("FAIL: no actuator feedback on slot — check config / bus / power")
                return 1

            pos = float(fb["position"])
            tor = float(fb.get("torque", 0.0))
            err = int(fb.get("fault", 0)) & 0xF
            label = (
                f"{label_joint} slot{slot_i} {can_bus_label(cfg.bus)} "
                f"{cfg.protocol_name} id=0x{cfg.motor_id:02X}"
            )
            print(f"joint-status: {label}")
            print(
                f"  fb={pos:+.4f} rad  soft=[{lo:+.3f},{hi:+.3f}]  "
                f"dist_to_lo={pos - lo:+.3f}  dist_to_hi={hi - pos:+.3f}"
            )
            print(f"  torque={tor:+.3f} Nm  err=0x{err:X}")
            return 0


def _default_gains(protocol_name: str, slot_i: int) -> tuple[float, float]:
    # Per-slot kp (YAM J1-J4 need more than the flat wrist-tuned DM_KP — see
    # teleop/defaults.py SLOT_KP); same table for damiao and robstride.
    kp = D.SLOT_KP[min(slot_i, len(D.SLOT_KP) - 1)]
    kd = D.DM_KD if protocol_name == "damiao" else D.KD
    return kp, kd


def _poll_slots(session: PcbSession, slots: list) -> Dict[int, dict]:
    latest: Dict[int, dict] = {}
    frame = session.reader.pop()
    while frame is not None:
        for s in slots:
            fb = parse_actuator_feedback(frame, slot=s)
            if fb is not None:
                latest[s] = fb
        frame = session.reader.pop()
    return latest


def run_joint_goto_braced(
    port: str,
    *,
    slot: Optional[int] = None,
    joint: Optional[int] = None,
    delta: Optional[float] = None,
    to: Optional[float] = None,
    absolute: bool = False,
    i_know_zeros: bool = False,
    slew: float = 0.15,
    hold_s: float = 1.0,
    hz: float = 40.0,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
    brace_kp: Optional[float] = None,
    brace_kd: Optional[float] = None,
    no_return: bool = False,
    dry_run: bool = False,
    no_limit_clamp: bool = False,
    xml_path: Optional[Path] = None,
    show_limits: bool = False,
) -> int:
    """Jog one joint while holding every other enabled slot at its captured start position.

    The 562 B plant wire image carries a desire for every slot on every frame; a plain
    single-slot ``send_plant({slot: (...)})`` zero-fills every other slot in that frame, and
    firmware copies the whole staged image into ``actuator_desire_live[]`` on receipt — so a
    normal single-slot jog drops every other enabled slot to kp=0 (limp) each tick. On a
    loaded multi-joint daisy chain that can make a genuinely movable joint look stuck because
    the rest of the structure is going soft under it. This sends target + brace desires in one
    combined ``send_plant`` per tick instead. Brace slots use their normal per-protocol default
    gains unless overridden; the target slot uses ``kp``/``kd`` (same defaults as ``joint goto``).
    """
    if to is not None:
        if not absolute:
            print("FAIL: --to requires --absolute")
            return 2
        if not i_know_zeros:
            print(
                "FAIL: absolute goto (--to) refused without --i-know-zeros — motor zero "
                "!= model zero until calibrated (docs/plan-yam-joint-commands.md §6)"
            )
            return 2
    if delta is not None and to is not None:
        print("FAIL: joint goto --brace takes --delta or --to, not both")
        return 2
    if delta is None and to is None:
        print("FAIL: joint goto --brace requires --delta D (relative) or --to RAD --absolute")
        return 2

    try:
        slot_i = _resolve_slot(slot=slot, joint=joint)
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2

    limits = load_yam_limits(xml_path)
    lim = limit_for_slot(slot_i, limits)
    joint_i = slot_to_joint(slot_i)
    span_cap = min(0.35, max(0.05, 0.08 * lim.span))

    if show_limits or dry_run:
        print_yam_limits(xml_path)
        print()

    label_joint = f"J{joint_i}({lim.name})"
    print(
        f"joint-goto-braced: port={port} slot={slot_i} {label_joint}  "
        f"soft=[{lim.soft()[0]:+.3f},{lim.soft()[1]:+.3f}]  source={lim.source}"
    )

    if dry_run:
        assume = lim.mid if lim.joint != 7 else max(lim.lo + 0.05, min(lim.hi - 0.05, 1.20))
        raw = (to - assume) if to is not None else delta
        planned = raw
        if not no_limit_clamp and abs(planned) > span_cap + 1e-9:
            planned = math.copysign(span_cap, planned)
        d, tgt, note = clamp_delta(assume, planned, lim, absolute=absolute)
        print(
            f"  dry-run assume_fb={assume:+.4f}  raw_delta={raw:+.3f}  "
            f"planned_delta={planned:+.3f}  clamped_delta={d:+.3f}  target={tgt:+.4f}"
        )
        if note:
            print(f"  note: {note}")
        print("  dry-run: no plant session / no motion (brace set not resolved)")
        return 0

    with PcbSession(port) as session:
        with session.rx_pump():
            try:
                ensure_plant_runtime(session, label="joint-goto-braced", bus=None)
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

            brace_slots = [
                c.slot
                for c in host_table()
                if c.enabled and c.slot != slot_i and c.protocol_name in ("damiao", "robstride")
            ]
            all_slots = [slot_i] + brace_slots
            print(f"  bracing slots: {brace_slots if brace_slots else '(none enabled)'}")

            if kp is None or kd is None:
                dkp, dkd = _default_gains(cfg.protocol_name, slot_i)
                kp = dkp if kp is None else kp
                kd = dkd if kd is None else kd

            warmup_plant_actuators(session, all_slots)

            # Sync feedback for target + brace slots together (kp=0, no motion).
            fb_by_slot: Dict[int, dict] = {}
            for _ in range(int(hz * 1.5)):
                cmds = {s: (1e-6, 0.0, 0.0, 0.0, 0.0) for s in all_slots}
                session.send_plant(cmds)
                time.sleep(1.0 / hz)
                fb_by_slot.update(_poll_slots(session, all_slots))
            if slot_i not in fb_by_slot:
                print("FAIL: no actuator feedback on target slot — check config / bus / power")
                release_bench_gates(session)
                return 1

            start = float(fb_by_slot[slot_i]["position"])
            brace_hold: Dict[int, float] = {}
            brace_gains: Dict[int, tuple] = {}
            for s in brace_slots:
                if s not in fb_by_slot:
                    print(f"  WARNING: no feedback on brace slot {s} — excluding it from the hold set")
                    continue
                bcfg = slot_config(s)
                brace_hold[s] = float(fb_by_slot[s]["position"])
                dkp, dkd = _default_gains(bcfg.protocol_name, s)
                brace_gains[s] = (
                    brace_kp if brace_kp is not None else dkp,
                    brace_kd if brace_kd is not None else dkd,
                )
            hold_desc = ", ".join(f"{s}:{p:+.3f}" for s, p in brace_hold.items())
            print(f"  synced fb={start:+.4f} rad  brace_hold={{ {hold_desc} }}")

            raw_request = (to - start) if to is not None else delta
            planned_delta = raw_request
            if not no_limit_clamp and abs(planned_delta) > span_cap + 1e-9:
                planned_delta = math.copysign(span_cap, planned_delta)
            if no_limit_clamp:
                use_delta = planned_delta
                target = start + use_delta
                note = "limits disabled (--no-limit-clamp)"
            else:
                use_delta, target, note = clamp_delta(start, planned_delta, lim, absolute=absolute)
            if note:
                print(f"  limits: {note}")
            if abs(use_delta) < 1e-4 and abs(raw_request) > 1e-4:
                print(
                    f"FAIL: no safe delta from fb={start:+.4f} inside "
                    f"[{lim.lo:+.3f},{lim.hi:+.3f}] — move joint off stop or flip --delta"
                )
                release_bench_gates(session)
                return 1

            print(
                f"  slew {start:+.4f} -> {target:+.4f} (delta={use_delta:+.4f})  "
                f"kp={kp:.1f} kd={kd:.2f}  slew={slew:.2f} rad/s ..."
            )

            dt = 1.0 / hz

            def _tick(cmd_pos: float) -> Optional[dict]:
                cmds = {slot_i: (cmd_pos, 0.0, kp, kd, 0.0)}
                for s, p in brace_hold.items():
                    bkp, bkd = brace_gains[s]
                    cmds[s] = (p, 0.0, bkp, bkd, 0.0)
                session.send_plant(cmds)
                time.sleep(dt)
                got = _poll_slots(session, all_slots)
                fb_by_slot.update(got)
                return got.get(slot_i)

            cmd = start
            while abs(target - cmd) > 1e-4:
                step = slew * dt
                d = target - cmd
                cmd = target if abs(d) <= step else cmd + math.copysign(step, d)
                _tick(cmd)

            last_fb: Optional[dict] = None
            for _ in range(max(1, int(hold_s * hz))):
                fb = _tick(cmd)
                if fb is not None:
                    last_fb = fb
            mid = float(last_fb["position"]) if last_fb else float("nan")
            tor = float(last_fb.get("torque", 0.0)) if last_fb else float("nan")
            print(f"  at target: cmd={cmd:+.4f} fb={mid:+.4f} torque={tor:+.3f}")

            if not no_return:
                print(f"  return -> {start:+.4f} ...")
                while abs(start - cmd) > 1e-4:
                    step = slew * dt
                    d = start - cmd
                    cmd = start if abs(d) <= step else cmd + math.copysign(step, d)
                    _tick(cmd)
                for _ in range(max(8, int(hz * 0.4))):
                    last_fb = _tick(cmd) or last_fb

            end = float(last_fb["position"]) if last_fb else float("nan")
            err1 = int(last_fb.get("fault", 0)) & 0xF if last_fb else -1
            moved = abs(mid - start) if last_fb else 0.0
            print(
                f"  done: start={start:+.4f} peak_fb={mid:+.4f} end={end:+.4f} "
                f"moved={moved:.4f} err=0x{err1:X}"
            )

            # Soft disable everyone (target + braced).
            for _ in range(max(8, int(hz * 0.3))):
                hold_pos = end if last_fb else cmd
                cmds = {slot_i: (hold_pos, 0.0, 0.0, 0.0, 0.0)}
                for s, p in brace_hold.items():
                    cmds[s] = (p, 0.0, 0.0, 0.0, 0.0)
                session.send_plant(cmds)
                time.sleep(dt)
            release_bench_gates(session)

            label = (
                f"{label_joint} slot{slot_i} {can_bus_label(cfg.bus)} "
                f"{cfg.protocol_name} id=0x{cfg.motor_id:02X}"
            )
            if abs(use_delta) < 0.02:
                ok = last_fb is not None
            else:
                ok = moved >= min(0.05, abs(use_delta) * 0.25)
            if ok:
                print(f"PASS: joint-goto-braced {label}")
                return 0

            if last_fb is not None and abs(tor) >= 4.0 and moved < 0.05:
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
