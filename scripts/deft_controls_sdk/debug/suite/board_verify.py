"""Bringup TUI — bare ``pcb_lab.debug test`` + filtered peripheral flags.

Scopes::

    board       — board-level CFG/doctor/observe + entry into peripherals
    actuators   — discover / CFG edit / hold / nudge / teleop actuators
    servos      — neck FB + teleop cruise
    led         — LED presets + doctor

``--actuators`` / ``--servos`` / ``--led`` open that peripheral menu only.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Dict, List, Literal, Optional, Sequence

from deft_controls_sdk.actions import ActuatorAction, ServoAction
from deft_controls_sdk.actions.operate import (
    feedback_positions_from_proxy,
    make_teleop_engine,
    neck_cruise,
    seed_for_slot,
    specs_for_cfg_map,
    spin_jog,
    stop_servos,
    stop_slots,
)
from deft_controls_sdk.actions.teleop import (
    BASE_CRUISE_DEFAULT,
    RS_HI,
    RS_KD,
    RS_KP,
    RS_LO,
    SlotSpec,
    build_servo_specs,
)
from deft_controls_sdk.config import LED_PRESETS, assembly_from_name
from deft_controls_sdk.host_proxy import HostProxy

from . import test_actuators as ta
from .pcb_tui import sample_servo_fb, servo_connection_summary
from .presets import apply_led_preset
from .show import collect_cfg

Scope = Literal["board", "actuators", "servos", "led"]


def _connect(args: argparse.Namespace) -> tuple[HostProxy, object]:
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz
    asm_name = str(getattr(args, "assembly", None) or "bench")
    assembly = assembly_from_name(asm_name)
    proxy = HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        armed=False,
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        mode="debug",
        assembly=assembly,
    )
    return proxy, assembly


def _parse_slot_list(text: str) -> List[int]:
    raw = str(text).replace(",", " ").split()
    slots = [int(x, 0) for x in raw]
    if not slots:
        raise ValueError("need at least one slot")
    for s in slots:
        if not (0 <= s < 26):
            raise ValueError(f"slot {s} out of range 0..25")
    return slots


def _cruise_specs_for_slots(slots: Sequence[int]) -> Dict[int, SlotSpec]:
    known = specs_for_cfg_map("bench")
    out: Dict[int, SlotSpec] = {}
    for slot in slots:
        s = int(slot)
        if s in known and known[s].lo is not None and known[s].hi is not None:
            out[s] = known[s]
            continue
        if s in known and known[s].seed_relative:
            out[s] = known[s]
            continue
        out[s] = SlotSpec(
            slot=s,
            group="slots",
            label=f"slot_{s}",
            protocol="robstride",
            kp=RS_KP,
            kd=RS_KD,
            verified=True,
            lo=RS_LO,
            hi=RS_HI,
            seed_relative=False,
            kp_floor=0.0,
            cruise_max=0.7,
            cruise_default=BASE_CRUISE_DEFAULT,
        )
    return out


def _doctor_menu(proxy: HostProxy) -> None:
    report = proxy.doctor()
    print(json.dumps(report, indent=2, default=str))


def _teleop_actuator_menu(proxy: HostProxy) -> None:
    raw = ta._prompt("slots (comma)", "22")
    if not raw:
        return
    try:
        slots = _parse_slot_list(raw)
    except Exception as exc:  # noqa: BLE001
        print(f"slots: {exc}", file=sys.stderr)
        return
    direction = 1 if ta._prompt_yn("positive direction", default=True) else -1
    cruise = ta._prompt_float("cruise rad/s", BASE_CRUISE_DEFAULT)
    if cruise is None:
        return
    specs = _cruise_specs_for_slots(slots)
    action = ActuatorAction.from_slots(proxy, slots, name="teleop")
    print(f"\nteleop actuator  slots={slots}  dir={direction} cruise={cruise:g}")
    if not ta._prompt_yn("proceed (motors will move)", default=False):
        print("cancelled")
        return
    ta.ensure_plant_control(proxy, enable=True)
    engine = make_teleop_engine(
        lambda: proxy.hub,
        feedback_getter=lambda: feedback_positions_from_proxy(proxy),
    )
    seeds = {s: seed_for_slot(proxy, s) for s in slots}
    try:
        spin_jog(
            engine,
            slots=slots,
            specs=specs,
            seeds=seeds,
            direction=direction,
            cruise=cruise,
        )
        print("cruising — Enter to stop / blank")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print()
    finally:
        stop_slots(engine, slots)
        engine.stop()
        ta.apply_blank(proxy, action)
        print("stopped + blanked")


def _servo_fb_dump(proxy: HostProxy) -> None:
    try:
        cfg = collect_cfg(proxy)
    except Exception as exc:  # noqa: BLE001
        print(f"collect_cfg: {exc}", file=sys.stderr)
        return
    fb = sample_servo_fb(proxy, cfg)
    summary = servo_connection_summary(cfg, fb)
    print(json.dumps({"summary": summary, "cfg_servos": cfg.get("servos")}, indent=2))
    for s in cfg.get("servos") or []:
        slot = int(s.get("slot", 0))
        row = fb.get(slot) or {}
        print(
            f"  slot {slot} en={int(bool(s.get('enabled')))} "
            f"id=0x{int(s.get('id', 0)) & 0xFF:02X} "
            f"pos={row.get('present_position')}"
        )


def _teleop_servo_menu(proxy: HostProxy) -> None:
    _servo_fb_dump(proxy)
    print(
        "\n  --- servo teleop ---\n"
        "  c) cruise to center (2048)\n"
        "  p) cruise to pitch/yaw\n"
        "  x) clear desires + disarm\n"
        "  b) back"
    )
    choice = ta._prompt("servo", "b").lower()
    if choice in ("", "b", "back"):
        return
    if choice in ("x", "clear"):
        ServoAction(proxy).neck_clear(send=True)
        ta.ensure_plant_control(proxy, enable=False)
        print("cleared + observe")
        return
    if choice in ("c", "center"):
        pitch, yaw = 2048, 2048
    elif choice in ("p", "pos", "set"):
        pitch_s = ta._prompt("pitch native", "2048")
        yaw_s = ta._prompt("yaw native", "2048")
        if not pitch_s or not yaw_s:
            return
        pitch, yaw = int(pitch_s, 0), int(yaw_s, 0)
    else:
        print(f"unknown servo choice {choice!r}")
        return
    cruise = ta._prompt_float("cruise steps/s", 200.0)
    if cruise is None:
        return
    print(f"\nteleop servo  pitch={pitch} yaw={yaw} cruise={cruise:g}")
    if not ta._prompt_yn("proceed (servos will move)", default=False):
        print("cancelled")
        return
    ta.ensure_plant_control(proxy, enable=True)
    engine = make_teleop_engine(lambda: proxy.hub)
    try:
        neck_cruise(
            engine,
            pitch=float(pitch),
            yaw=float(yaw),
            cruise=cruise,
            specs=build_servo_specs(),
        )
        print("cruising — Enter to stop")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print()
    finally:
        stop_servos(engine)
        engine.stop()
        print("stopped (desires held; use 'x' to clear)")


def _led_menu(proxy: HostProxy, args: argparse.Namespace) -> None:
    known = ", ".join(sorted(LED_PRESETS))
    default = str(getattr(args, "led_preset", None) or "idle")
    if default in ("bench", "product", "full", "yam"):
        default = "idle"
    name = ta._prompt(f"preset ({known})", default)
    if not name:
        return
    preset = LED_PRESETS.get(name.strip().lower())
    if preset is None:
        print(f"unknown preset {name!r}", file=sys.stderr)
        return
    hold = ta._prompt_float("hold seconds", float(getattr(args, "hold_s", 2.0)))
    if hold is None:
        return
    print(f"\nled  preset={preset.name}  policy={preset.policy}  hold={hold:g}s")
    if not ta._prompt_yn("proceed", default=True):
        print("cancelled")
        return
    try:
        periph = dict(proxy.hub.debug.cfg_get_periph())
    except Exception as exc:  # noqa: BLE001
        print(f"cfg_get_periph: {exc}", file=sys.stderr)
        periph = {"led": {}, "flags": 0, "servos": []}
    apply_led_preset(proxy, preset, periph)
    if hold > 0:
        time.sleep(hold)
    led = (proxy.doctor().get("led") or {})
    print(json.dumps({"preset": preset.name, "doctor_led": led}, indent=2, default=str))


def _run_inventory_quick(args: argparse.Namespace, proxy: HostProxy) -> None:
    from deft_controls_sdk.debug import run_inventory

    buses_raw = ta._prompt("buses (comma or all)", "5,6")
    if not buses_raw:
        return
    if buses_raw.strip().lower() in ("all", "*"):
        buses = (1, 2, 3, 4, 5, 6)
    else:
        buses = tuple(int(x, 0) for x in buses_raw.replace(",", " ").split())
    preset = ta._prompt("id preset (bench|product|full or empty+ranges)", "bench")
    kwargs = {
        "buses": buses,
        "include_servos": True,
        "include_pdu": True,
        "print_report": True,
    }
    if preset.strip():
        kwargs["preset"] = preset.strip().lower()
    else:
        kwargs["ranges"] = {"robstride": (0x70, 0x75), "damiao": (1, 8)}
    run_inventory(proxy, **kwargs)


def _menu_actuators(proxy: HostProxy, assembly: object) -> Optional[str]:
    """Actuator peripheral menu. Returns 'q' to quit session, None to continue/back."""
    print(
        "\n  --- actuators ---\n"
        "  d) discover (debug lanes)\n"
        "  e) edit CFG slot\n"
        "  k) calibrate robstride\n"
        "  h) hold (sample FB → stay put)\n"
        "  u) nudge (+delta rad)\n"
        "  t) teleop cruise\n"
        "  f) show FB\n"
        "  b) blank desires\n"
        "  o) observe (disarm)\n"
        "  .) back to board    q) quit"
    )
    choice = ta._prompt("actuators", ".").lower()
    if choice in ("", ".", "back"):
        return "back"
    if choice in ("q", "quit", "exit"):
        return "q"
    if choice in ("d", "discover"):
        ta._discover_menu(proxy)
    elif choice in ("e", "edit", "cfg"):
        ta._cfg_edit_slot_menu(proxy)
    elif choice in ("k", "cal", "calibrate"):
        ta._calibrate_robstride_menu(proxy)
    elif choice in ("h", "hold"):
        ta._motion_hold_menu(proxy, assembly)
    elif choice in ("u", "nudge", "step"):
        ta._motion_step_menu(proxy, assembly)
    elif choice in ("t", "teleop", "cruise"):
        _teleop_actuator_menu(proxy)
    elif choice in ("f", "fb", "pos"):
        ta._motion_fb_menu(proxy, assembly)
    elif choice in ("b", "blank"):
        ta._motion_blank_menu(proxy, assembly)
    elif choice in ("o", "observe"):
        ta._motion_observe_menu(proxy)
    else:
        print(f"unknown choice {choice!r}")
    return None


def _menu_servos(proxy: HostProxy) -> Optional[str]:
    print(
        "\n  --- servos ---\n"
        "  f) show FB / connection\n"
        "  t) teleop cruise (neck)\n"
        "  x) clear + disarm\n"
        "  o) observe (disarm)\n"
        "  .) back to board    q) quit"
    )
    choice = ta._prompt("servos", ".").lower()
    if choice in ("", ".", "back"):
        return "back"
    if choice in ("q", "quit", "exit"):
        return "q"
    if choice in ("f", "fb"):
        _servo_fb_dump(proxy)
    elif choice in ("t", "teleop", "cruise", "n", "neck"):
        _teleop_servo_menu(proxy)
    elif choice in ("x", "clear"):
        ServoAction(proxy).neck_clear(send=True)
        ta.ensure_plant_control(proxy, enable=False)
        print("cleared + observe")
    elif choice in ("o", "observe"):
        ta._motion_observe_menu(proxy)
    else:
        print(f"unknown choice {choice!r}")
    return None


def _menu_led(proxy: HostProxy, args: argparse.Namespace) -> Optional[str]:
    print(
        "\n  --- led ---\n"
        "  p) apply preset\n"
        "  d) doctor LED field\n"
        "  .) back to board    q) quit"
    )
    choice = ta._prompt("led", ".").lower()
    if choice in ("", ".", "back"):
        return "back"
    if choice in ("q", "quit", "exit"):
        return "q"
    if choice in ("p", "preset", "a", "apply"):
        _led_menu(proxy, args)
    elif choice in ("d", "doctor"):
        led = (proxy.doctor().get("led") or {})
        print(json.dumps(led, indent=2, default=str))
    else:
        print(f"unknown choice {choice!r}")
    return None


def _menu_board(
    proxy: HostProxy,
    assembly: object,
    args: argparse.Namespace,
) -> Optional[str]:
    """Top-level board menu. Returns 'q' to quit, or a peripheral scope to enter."""
    print(
        "\n  --- board ---\n"
        "  p) doctor / status\n"
        "  c) show CFG (enabled)\n"
        "  C) show CFG (all)\n"
        "  i) inventory (discover + servos + PDU)\n"
        "  o) observe (disarm)\n"
        "  --- peripherals ---\n"
        "  a) actuators…\n"
        "  s) servos…\n"
        "  l) led…\n"
        "  q) quit"
    )
    choice = ta._prompt("choice", "q")
    if choice in ("", "q", "quit", "exit"):
        return "q"
    key = choice if choice == "C" else choice.lower()
    if key in ("p", "doctor", "status"):
        _doctor_menu(proxy)
    elif key == "c":
        ta._show_cfg(proxy, only_enabled=True)
    elif key == "C":
        ta._show_cfg(proxy, only_enabled=False)
    elif key in ("o", "observe"):
        ta._motion_observe_menu(proxy)
    elif key in ("a", "actuators", "act"):
        return "actuators"
    elif key in ("s", "servos", "servo", "neck"):
        return "servos"
    elif key in ("l", "led"):
        return "led"
    elif key in ("i", "inventory", "inv"):
        _run_inventory_quick(args, proxy)
    else:
        print(f"unknown choice {choice!r}")
    return None


def run_bringup(args: argparse.Namespace, *, scope: Scope = "board") -> int:
    """Interactive bringup. ``scope`` selects top menu or a peripheral-only loop."""
    asm_name = str(getattr(args, "assembly", None) or "bench")
    print(
        f"bringup  mode=debug  assembly={asm_name}  scope={scope}\n"
        "  connect starts disarmed; motion menus arm after confirm."
    )
    proxy, assembly = _connect(args)
    current: Scope = scope
    filtered = scope != "board"
    with proxy:
        while True:
            try:
                if current == "board":
                    nxt = _menu_board(proxy, assembly, args)
                    if nxt == "q":
                        break
                    if nxt in ("actuators", "servos", "led"):
                        current = nxt  # type: ignore[assignment]
                elif current == "actuators":
                    nxt = _menu_actuators(proxy, assembly)
                    if nxt == "q":
                        break
                    if nxt == "back":
                        if filtered:
                            break
                        current = "board"
                elif current == "servos":
                    nxt = _menu_servos(proxy)
                    if nxt == "q":
                        break
                    if nxt == "back":
                        if filtered:
                            break
                        current = "board"
                elif current == "led":
                    nxt = _menu_led(proxy, args)
                    if nxt == "q":
                        break
                    if nxt == "back":
                        if filtered:
                            break
                        current = "board"
            except Exception as exc:  # noqa: BLE001
                print(f"error: {exc}", file=sys.stderr)
                return 1
    return 0


def run_board_verify(args: argparse.Namespace) -> int:
    """Bare ``test`` — board + peripheral entry points."""
    return run_bringup(args, scope="board")


__all__ = ["run_board_verify", "run_bringup", "_cruise_specs_for_slots", "_parse_slot_list"]
