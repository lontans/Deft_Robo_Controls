"""Assembly workshop TUI — one COM session, typed profiles, prove + operate.

Bare ``pcb_lab.debug test`` (no domain flag) enters here. Motion/CFG/teleop
call ``deft_controls_sdk.actions`` so dashboard/ROS can reuse the same APIs.
``test --actuators`` stays a narrower discover/CFG/motion menu.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from typing import List, Sequence

from deft_controls_sdk.actions import ActuatorAction
from deft_controls_sdk.actions.cfg_identity import (
    format_slot_cfg_lines,
    profile_cfg_status,
    profile_in_nvm,
)
from deft_controls_sdk.actions.operate import (
    feedback_positions_from_proxy,
    make_teleop_engine,
    move_arm_cruise,
    seed_for_slot,
    specs_for_cfg_map,
    spin_jog,
    stop_slots,
)
from deft_controls_sdk.config import (
    ActuatorKind,
    ActuatorProfile,
    Assembly,
    assembly_from_name,
    assembly_put_actuator,
    assembly_put_servo,
    assembly_remove_actuator,
    arm_profile,
    kind_for_component,
    lift_profile,
    neck_profile,
    wheel_profile,
)
from deft_controls_sdk.host_proxy import HostProxy

from . import test_actuators as ta


@dataclass
class WorkshopSession:
    proxy: HostProxy
    assembly: Assembly
    cfg_map: str = "bench"  # teleop SlotSpec map: bench|product
    _name: str = field(default="product", repr=False)

    def set_assembly(self, asm: Assembly) -> None:
        self.assembly = asm


def _connect(args: argparse.Namespace) -> WorkshopSession:
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz
    asm_name = str(getattr(args, "assembly", None) or "product")
    assembly = assembly_from_name(asm_name)
    # Teleop SlotSpec map: bench has live-verified base ranges; override via menu.
    cfg_map = str(getattr(args, "cfg_map", None) or "bench")
    proxy = HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        idle_first=True,
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        mode="debug",
        profile=assembly.to_demux_profile(),
    )
    return WorkshopSession(
        proxy=proxy, assembly=assembly, cfg_map=cfg_map, _name=asm_name
    )


def _print_assembly(asm: Assembly) -> None:
    print(f"\nAssembly {asm.name!r}")
    print("  actuators:")
    for name in sorted(asm.actuators):
        p = asm.actuator(name)
        cfg_n = len(p.as_cfg_rows())
        print(
            f"    {name:<16} kind={p.kind:<5} slots={list(p.slots)}  "
            f"cfg_rows={cfg_n}"
        )
    print("  servos:")
    if not asm.servos:
        print("    (none)")
    for name in sorted(asm.servos):
        p = asm.servo(name)
        print(f"    {name:<16} slots={list(p.slots)}")


def _edit_assembly_menu(session: WorkshopSession) -> None:
    print(
        "\n  --- edit assembly ---\n"
        "  1) add/replace arm (yam left|right)\n"
        "  2) add/replace wheel/base\n"
        "  3) add/replace lift\n"
        "  4) add/replace single:SLOT:PROTO:MOTOR:BUS[:kind]\n"
        "  5) add/replace neck servos\n"
        "  6) remove actuator by name\n"
        "  7) reset to product|bench stock\n"
        "  b) back"
    )
    choice = ta._prompt("edit", "b").lower()
    if choice in ("", "b", "back"):
        return
    try:
        if choice == "1":
            side = ta._prompt("side (left|right)", "left")
            name = ta._prompt("profile name", "left_arm" if side.startswith("l") else "right_arm")
            slots = ta._prompt("slots (blank=default)", "")
            prof = arm_profile(
                "yam",
                side=side,
                slots=slots or None,
                name=name or None,
            )
            session.set_assembly(assembly_put_actuator(session.assembly, prof))
            print(f"ok  {prof.name} slots={list(prof.slots)}")
        elif choice == "2":
            name = ta._prompt("profile name", "base")
            bench = ta._prompt_yn("bench spare slots (22-25)", default=True)
            slots = ta._prompt("slots (blank=default)", "")
            prof = wheel_profile(
                name=name or "base",
                slots=slots or None,
                bench=bench,
            )
            session.set_assembly(assembly_put_actuator(session.assembly, prof))
            print(f"ok  {prof.name} slots={list(prof.slots)}")
        elif choice == "3":
            name = ta._prompt("profile name", "lift")
            prof = lift_profile(name=name or "lift")
            session.set_assembly(assembly_put_actuator(session.assembly, prof))
            print(f"ok  {prof.name}")
        elif choice == "4":
            raw = ta._prompt(
                "single:SLOT:PROTO:MOTOR:BUS[:kind]",
                "single:22:robstride:0x70:5:wheel",
            )
            if not raw:
                return
            prof = ta.parse_single_target(raw)
            session.set_assembly(assembly_put_actuator(session.assembly, prof))
            print(f"ok  {prof.name}  {prof.as_cfg_row()}")
        elif choice == "5":
            name = ta._prompt("servo profile name", "neck")
            prof = neck_profile(name=name or "neck")
            session.set_assembly(assembly_put_servo(session.assembly, prof))
            print(f"ok  {prof.name} slots={list(prof.slots)}")
        elif choice == "6":
            name = ta._prompt("actuator name to remove", "")
            if not name:
                return
            session.set_assembly(assembly_remove_actuator(session.assembly, name))
            print(f"removed {name!r}")
        elif choice == "7":
            key = ta._prompt("stock (product|bench)", "product")
            session.set_assembly(assembly_from_name(key))
            session.cfg_map = (
                "bench" if session.assembly.name == "yam_bench_continuous" else session.cfg_map
            )
            print(f"reset → {session.assembly.name}")
        else:
            print(f"unknown edit choice {choice!r}")
    except Exception as exc:  # noqa: BLE001
        print(f"edit: {exc}", file=sys.stderr)


def _peek_cfg_for_slots(proxy: HostProxy, slots: Sequence[int]) -> List[dict]:
    table = proxy.hub.debug.cfg_get_table()
    print("\n".join(format_slot_cfg_lines(slots, table)))
    return list(table)


def _apply_profile_cfg(
    proxy: HostProxy,
    profile: ActuatorProfile,
    *,
    persist: bool,
) -> None:
    rows = profile.as_cfg_rows()
    if not rows:
        raise ValueError(f"{profile.name}: no cfg rows to apply")
    for row in rows:
        resp = proxy.hub.debug.cfg_set_slot(**row, persist=persist)
        print(json.dumps({"row": row, "persist": persist, "resp": resp}, default=str))


def _nudge_nvm_gate(
    session: WorkshopSession,
    action: ActuatorAction,
) -> bool:
    """Show CFG peek; ensure identity before plant nudge. Returns False if cancelled."""
    proxy = session.proxy
    slots = list(action.slots)
    print(f"\nCFG peek for {action.name} slots={slots}")
    table = _peek_cfg_for_slots(proxy, slots)

    prof = action.actuator_profile
    if prof is not None and prof.as_cfg_rows():
        status = profile_cfg_status(prof, table)
        for slot, ok, detail in status:
            print(f"  slot {slot}: {'OK' if ok else 'MISS'}  {detail}")
        in_nvm = profile_in_nvm(prof, table)
        if in_nvm:
            print("identity: matches NVM/RAM CFG")
            return True
        print("identity: not fully in NVM (or mismatch)")
        if not ta._prompt_yn("defined in NVM? (n → apply/persist first)", default=False):
            mode = ta._prompt("write CFG: apply (RAM) | persist (flash) | cancel", "apply").lower()
            if mode in ("", "c", "cancel"):
                print("cancelled")
                return False
            persist = mode in ("persist", "p", "flash", "nvm")
            if not ta._prompt_yn(
                f"cfg_set_slot ×{len(prof.as_cfg_rows())} persist={persist}",
                default=False,
            ):
                print("cancelled")
                return False
            _apply_profile_cfg(proxy, prof, persist=persist)
            table = proxy.hub.debug.cfg_get_table()
            if not profile_in_nvm(prof, table):
                print("warning: still not matching after write — continue anyway?")
                if not ta._prompt_yn("continue to nudge", default=False):
                    return False
            return True
        return True

    # No profile CFG — ask operator after peek
    print("(no ActuatorProfile.cfg — confirm from peek above)")
    if not ta._prompt_yn("actuator identity OK in NVM/CFG?", default=False):
        if ta._prompt_yn("apply single_profile CFG now?", default=True):
            ta._cfg_apply_single_menu(proxy)
            if not ta._prompt_yn("identity OK now — continue nudge?", default=True):
                return False
        else:
            print("cancelled")
            return False
    return True


def _nudge_menu(session: WorkshopSession) -> None:
    proxy = session.proxy
    assembly = session.assembly
    target = ta._prompt("target (section | slot N | single:…)", "22")
    if not target:
        return
    lower = target.strip().lower()
    default_kind: ActuatorKind = (
        "wheel"
        if lower in ("base", "base_product")
        or target.strip().isdigit()
        or lower.startswith("single:")
        else kind_for_component(target)
    )
    kind = None if target.strip().lower().startswith("single:") else ta._prompt_kind(
        default_kind
    )
    try:
        action = ta.resolve_motion_target(
            proxy, target, kind=kind, assembly=assembly
        )
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    ta._print_action_fb(action)
    if not _nudge_nvm_gate(session, action):
        return
    idx = int(ta._prompt("index within group (0-based)", "0") or "0")
    delta = ta._prompt_float("delta rad", 0.05)
    if delta is None:
        return
    hold_s = ta._prompt_float("hold seconds", 1.0)
    if hold_s is None:
        return
    print(
        f"\nnudge  target={action.name}  kind={action.kind}  index={idx}  "
        f"delta={delta:g}  plant_apply=ON"
    )
    if not ta._prompt_yn("proceed (motors will move)", default=False):
        print("cancelled")
        return
    pos = ta.apply_step(proxy, action, index=idx, delta=delta, hold_s=hold_s)
    print(f"  commanded={pos}")
    ta._print_action_fb(action)


def _cfg_apply_section_menu(session: WorkshopSession) -> None:
    known = ", ".join(sorted(session.assembly.actuators))
    name = ta._prompt(f"actuator profile ({known})", "base")
    if not name:
        return
    try:
        prof = session.assembly.actuator(name)
    except KeyError as exc:
        print(f"{exc}", file=sys.stderr)
        return
    rows = prof.as_cfg_rows()
    if not rows:
        print(f"{name}: no cfg rows on profile (use single:… or edit)", file=sys.stderr)
        return
    persist = ta._prompt_yn("persist to NVM (flash SAVE)", default=False)
    print(f"\ncfg_set_slot ×{len(rows)} persist={persist}")
    for r in rows:
        print(f"  {r}")
    if not ta._prompt_yn("proceed", default=False):
        print("cancelled")
        return
    _apply_profile_cfg(session.proxy, prof, persist=persist)


def _operate_spin(session: WorkshopSession) -> None:
    known = ", ".join(
        n for n, p in session.assembly.actuators.items() if p.kind == "wheel"
    ) or "(no wheel profiles — use slot / base)"
    target = ta._prompt(f"wheel target ({known} | slot N)", "base")
    if not target:
        return
    try:
        action = ta.resolve_motion_target(
            session.proxy, target, kind="wheel", assembly=session.assembly
        )
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    direction = 1 if ta._prompt_yn("positive direction", default=True) else -1
    cruise = ta._prompt_float("cruise rad/s", 0.3)
    if cruise is None:
        return
    specs = specs_for_cfg_map(session.cfg_map)
    missing = [s for s in action.slots if s not in specs]
    if missing:
        print(
            f"slots without teleop SlotSpec: {missing} "
            f"(try cfg_map=bench or edit assembly)",
            file=sys.stderr,
        )
        return
    print(
        f"\nspin  {action.name} slots={list(action.slots)}  "
        f"dir={direction} cruise={cruise:g}"
    )
    if not ta._prompt_yn("proceed (motors will move)", default=False):
        print("cancelled")
        return
    ta.ensure_plant_control(session.proxy, enable=True)
    engine = make_teleop_engine(
        lambda: session.proxy.hub,
        feedback_getter=lambda: feedback_positions_from_proxy(session.proxy),
    )
    seeds = {s: seed_for_slot(session.proxy, s) for s in action.slots}
    try:
        spin_jog(
            engine,
            slots=action.slots,
            specs=specs,
            seeds=seeds,
            direction=direction,
            cruise=cruise,
        )
        print("spinning — Enter to stop / blank")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print()
    finally:
        stop_slots(engine, action.slots)
        engine.stop()
        ta.apply_blank(session.proxy, action)
        print("stopped + blanked")


def _operate_move_arm(session: WorkshopSession) -> None:
    """Cruise arm slots via ``TeleopEngine`` (same core as dashboard mouse teleop)."""
    arms = [
        n
        for n, p in session.assembly.actuators.items()
        if p.kind == "joint" and "arm" in n
    ]
    default = "left_arm" if "left_arm" in session.assembly.actuators else (
        arms[0] if arms else "left_arm"
    )
    target = ta._prompt(
        f"arm section ({', '.join(arms) or 'left_arm'})",
        default,
    )
    if not target:
        return
    try:
        action = ta.resolve_motion_target(
            session.proxy, target, kind="joint", assembly=session.assembly
        )
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    specs = specs_for_cfg_map(session.cfg_map)
    unverified = [
        s
        for s in action.slots
        if s not in specs or not specs[s].verified or specs[s].lo is None
    ]
    if unverified:
        print(
            f"slots without verified teleop range: {unverified}\n"
            "left_arm on bench specs is supported; right_arm needs verified ranges.",
            file=sys.stderr,
        )
        if not ta._prompt_yn("continue with verified slots only", default=True):
            return
    usable = [
        s
        for s in action.slots
        if s in specs and specs[s].verified and specs[s].lo is not None
    ]
    if not usable:
        print("no usable slots", file=sys.stderr)
        return
    ta._print_action_fb(action)
    print(
        "move_arm uses actions.TeleopEngine (dashboard mouse teleop core).\n"
        "Commands:  t <index> <rad>  |  hold  |  stop  |  q"
    )
    cruise = ta._prompt_float("cruise rad/s", 0.35)
    if cruise is None:
        return
    if not ta._prompt_yn("proceed (motors will move)", default=False):
        print("cancelled")
        return
    ta.ensure_plant_control(session.proxy, enable=True)
    engine = make_teleop_engine(
        lambda: session.proxy.hub,
        feedback_getter=lambda: feedback_positions_from_proxy(session.proxy),
    )
    seeds = {s: seed_for_slot(session.proxy, s) for s in usable}
    # Hold current pose first
    try:
        move_arm_cruise(
            engine,
            slots=usable,
            specs=specs,
            seeds=seeds,
            targets=dict(seeds),
            cruise=cruise,
        )
        while True:
            raw = ta._prompt("move_arm", "q")
            if not raw or raw.lower() in ("q", "quit", "exit"):
                break
            parts = raw.split()
            cmd = parts[0].lower()
            if cmd in ("stop", "s"):
                stop_slots(engine, usable)
                print("frozen")
                continue
            if cmd == "hold":
                seeds = {s: seed_for_slot(session.proxy, s) for s in usable}
                move_arm_cruise(
                    engine,
                    slots=usable,
                    specs=specs,
                    seeds=seeds,
                    targets=dict(seeds),
                    cruise=cruise,
                )
                print("re-hold at FB")
                continue
            if cmd in ("t", "target") and len(parts) >= 3:
                idx = int(parts[1], 0)
                tgt = float(parts[2])
                if not (0 <= idx < len(usable)):
                    print(f"index 0..{len(usable) - 1}", file=sys.stderr)
                    continue
                slot = usable[idx]
                seeds[slot] = seed_for_slot(session.proxy, slot)
                move_arm_cruise(
                    engine,
                    slots=(slot,),
                    specs=specs,
                    seeds=seeds,
                    targets={slot: tgt},
                    cruise=cruise,
                )
                print(f"cruise slot {slot} → {tgt:g}")
                continue
            if cmd == "snap" and len(parts) >= 1:
                # snap = print snapshot
                print(json.dumps(engine.snapshot(), indent=2))
                continue
            print("usage: t <index> <rad> | hold | stop | snap | q")
    finally:
        stop_slots(engine, usable)
        engine.stop()
        ta.apply_blank(session.proxy, action)
        print("stopped + blanked")


def _operate_menu(session: WorkshopSession) -> None:
    teleop = ta._prompt_yn("teleop (interactive cruise)", default=True)
    if not teleop:
        print("teleop=n — use nudge/hold menus instead")
        return
    mode = ta._prompt("mode (spin | move_arm)", "spin").strip().lower()
    if mode in ("spin", "s", "wheel", "base"):
        _operate_spin(session)
    elif mode in ("move_arm", "arm", "m", "mouse"):
        _operate_move_arm(session)
    else:
        print(f"unknown operate mode {mode!r}")


def run_assembly_workshop(args: argparse.Namespace) -> int:
    print(
        "Assembly workshop  mode=debug  "
        "(profiles + CFG/discover/cal + ActuatorAction + TeleopEngine operate)"
    )
    print(
        "note: idle_first (plant_apply off). Motion/operate arm plant_apply after confirm.\n"
        "actions stay general — dashboard can import TeleopEngine / operate.* later."
    )
    session = _connect(args)
    with session.proxy:
        while True:
            _print_assembly(session.assembly)
            print(
                "\n  --- assembly ---\n"
                "  a) edit profiles (add/remove/reset)\n"
                "  m) cfg_map for teleop (bench|product) "
                f"[now={session.cfg_map}]\n"
                "  --- identity / device ---\n"
                "  1) show CFG table\n"
                "  2) show enabled CFG only\n"
                "  3) discover (multi-bus / multi-protocol)\n"
                "  4) calibrate robstride\n"
                "  5) CFG enabled hint (JSON)\n"
                "  11) CFG apply single_profile\n"
                "  12) CFG apply assembly section\n"
                "  --- plant motion (CMDH) ---\n"
                "  6) hold\n"
                "  7) nudge (+ NVM/CFG gate)\n"
                "  8) blank\n"
                "  9) show FB\n"
                "  10) observe (plant_apply OFF)\n"
                "  --- operate ---\n"
                "  o) operate (teleop → spin | move_arm)\n"
                "  q) quit"
            )
            choice = ta._prompt("choice", "q").lower()
            if choice in ("", "q", "quit", "exit"):
                break
            try:
                if choice in ("a", "edit", "assembly"):
                    _edit_assembly_menu(session)
                elif choice == "m":
                    cm = ta._prompt("cfg_map (bench|product)", session.cfg_map)
                    if cm in ("bench", "product"):
                        session.cfg_map = cm
                        print(f"cfg_map={cm}")
                elif choice in ("1", "cfg"):
                    ta._show_cfg(session.proxy)
                elif choice in ("2", "enabled"):
                    ta._show_cfg(session.proxy, only_enabled=True)
                elif choice in ("3", "discover", "d"):
                    ta._discover_menu(session.proxy)
                elif choice in ("4", "calibrate", "cal", "c"):
                    ta._calibrate_robstride_menu(session.proxy)
                elif choice in ("5", "hint"):
                    ta._cfg_hint(session.proxy)
                elif choice in ("6", "hold", "h"):
                    ta._motion_hold_menu(session.proxy, session.assembly)
                elif choice in ("7", "step", "s", "nudge"):
                    _nudge_menu(session)
                elif choice in ("8", "blank", "b"):
                    ta._motion_blank_menu(session.proxy, session.assembly)
                elif choice in ("9", "fb", "pos"):
                    ta._motion_fb_menu(session.proxy, session.assembly)
                elif choice in ("10", "observe"):
                    ta._motion_observe_menu(session.proxy)
                elif choice in ("11", "cfg_single"):
                    ta._cfg_apply_single_menu(session.proxy)
                elif choice in ("12", "cfg_section"):
                    _cfg_apply_section_menu(session)
                elif choice in ("o", "operate", "op"):
                    _operate_menu(session)
                else:
                    print(f"unknown choice {choice!r}")
            except Exception as exc:  # noqa: BLE001
                print(f"error: {exc}", file=sys.stderr)
                return 1
    return 0


__all__ = ["WorkshopSession", "run_assembly_workshop"]
