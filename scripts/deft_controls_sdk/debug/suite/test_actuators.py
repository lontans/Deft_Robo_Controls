"""Actuator prove helpers — discover / CFG / hold / nudge for the bringup TUI.

The interactive menu lives in ``board_verify.py`` (bare ``test`` and
``--actuators``). This module owns prompts + plant helpers used there.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from deft_controls_sdk.actions import ActuatorAction
from deft_controls_sdk.config import (
    Assembly,
    single_profile,
    yam_product_assembly,
)
from deft_controls_sdk.debug.discover import (
    DEFAULT_ID_RANGE,
    parse_bus_list,
    parse_protocol_queue,
    summarize_queued,
)
from deft_controls_sdk.host_proxy import HostProxy
from .proto import protocol_name
from .show import format_cfg_table


def _hex_id(n: int) -> str:
    return f"0x{int(n) & 0xFF:02X}"


def _hex_ids(ids: Iterable[int]) -> List[str]:
    return [_hex_id(i) for i in ids]


def _connect_debug(args: argparse.Namespace) -> tuple[HostProxy, Assembly]:
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz
    assembly = yam_product_assembly()
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


def _prompt(msg: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        ans = input(f"{msg}{suffix}> ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return ""
    return ans if ans else default


def _prompt_yn(msg: str, *, default: bool = False) -> bool:
    d = "y" if default else "n"
    ans = _prompt(f"{msg} (y/n)", d).lower()
    if not ans:
        return default
    return ans in ("y", "yes")


def _prompt_float(msg: str, default: float) -> Optional[float]:
    raw = _prompt(msg, f"{default:g}")
    if not raw:
        return None
    return float(raw)


def _show_cfg(proxy: HostProxy, *, only_enabled: bool = False) -> None:
    from .show import collect_cfg

    cfg = collect_cfg(proxy)
    print(format_cfg_table(cfg, only_enabled=only_enabled, banner=True, box=True))


def _prompt_id_ranges(protocols: Sequence[str]) -> Dict[str, Tuple[int, int]]:
    """Ask id start/end once per protocol (label in prompt), in queue order."""
    ranges: Dict[str, Tuple[int, int]] = {}
    for proto in protocols:
        d0, d1 = DEFAULT_ID_RANGE[proto]
        if proto == "robstride":
            start_default = _hex_id(d0)
            end_default = _hex_id(d1)
        else:
            start_default = str(d0)
            end_default = str(d1)
        start_s = _prompt(f"id start ({proto})", start_default)
        end_s = _prompt(f"id end ({proto})", end_default)
        if not start_s or not end_s:
            raise ValueError(f"cancelled while prompting id range for {proto}")
        start = int(start_s, 0)
        end = int(end_s, 0)
        if start > end:
            raise ValueError(f"{proto}: id start {start} > end {end}")
        ranges[proto] = (start, end)
    return ranges


def _discover_menu(proxy: HostProxy) -> None:
    """Multi-bus / multi-protocol discover via ``hub.debug.discover_queued``."""
    bus_s = _prompt("bus (1..6, comma list, or all)", "1")
    if not bus_s:
        return
    try:
        buses = parse_bus_list(bus_s)
    except ValueError as exc:
        print(f"bus: {exc}", file=sys.stderr)
        return

    proto_s = _prompt(
        "protocol (robstride|damiao|cubemars|zeroerr|all, comma queue)",
        "robstride",
    )
    if not proto_s:
        return
    try:
        protocols = parse_protocol_queue(proto_s)
    except ValueError as exc:
        print(f"protocol: {exc}", file=sys.stderr)
        return

    try:
        ranges = _prompt_id_ranges(protocols)
    except ValueError as exc:
        print(f"id range: {exc}", file=sys.stderr)
        return

    print(
        f"\ndiscover queued  buses={buses}  protocols={protocols}  "
        f"(protocol queue first, then each bus — sequential, not parallel)"
    )
    results = proxy.hub.debug.discover_queued(
        buses=buses, protocols=protocols, ranges=ranges
    )
    for row in results:
        if not row.get("ok"):
            print(
                f"  FAIL bus={row.get('bus')} {row.get('protocol')}: "
                f"{row.get('error')}",
                file=sys.stderr,
            )
    print(json.dumps(summarize_queued(results), indent=2))


def _calibrate_robstride_menu(proxy: HostProxy) -> None:
    """RS02 encoder cal via ``hub.debug.calibrate_robstride`` (no Damiao path yet)."""
    bus_s = _prompt("bus (1..6)", "1")
    if not bus_s:
        return
    bus = int(bus_s, 0)
    mid_s = _prompt("motor id", "0x70")
    if not mid_s:
        return
    motor_id = int(mid_s, 0) & 0xFF
    listen_s = float(_prompt("cal listen seconds", "28") or "28")
    skip_iq = _prompt_yn("skip iq_test", default=False)
    print(
        f"\nRobStride encoder cal  bus={bus}  id={_hex_id(motor_id)}  "
        f"listen={listen_s:g}s  skip_iq={skip_iq}"
    )
    print("Shaft must spin freely; supply 24–60 V. Damiao calibrate is not wired.")
    if not _prompt_yn("proceed", default=False):
        print("cancelled")
        return
    ok = proxy.hub.debug.calibrate_robstride(
        bus=bus,
        motor_id=motor_id,
        cal_listen_s=listen_s,
        skip_iq_test=skip_iq,
    )
    print(
        json.dumps(
            {
                "ok": bool(ok),
                "bus": bus,
                "protocol": "robstride",
                "motor_id": _hex_id(motor_id),
                "cal_listen_s": listen_s,
                "skip_iq_test": skip_iq,
            },
            indent=2,
        )
    )


def _cfg_hint(proxy: HostProxy) -> None:
    """Print enabled CFG rows as a compact JSON hint (no mutation)."""
    table = proxy.hub.debug.cfg_get_table()
    enabled: List[dict] = []
    for row in table:
        if not row.get("enabled"):
            continue
        proto = int(row.get("protocol", 0))
        enabled.append(
            {
                "slot": int(row.get("slot", 0)),
                "bus": int(row.get("bus", 0)),
                "protocol": protocol_name(proto),
                "motor_id": _hex_id(row.get("motor_id", 0)),
                "master_id": _hex_id(row.get("master_id", 0)),
            }
        )
    print(
        json.dumps(
            {"enabled_count": len(enabled), "enabled": enabled},
            indent=2,
        )
    )


# -- plant motion (CMDH via ActuatorAction) ------------------------------------


def parse_single_target(target: str) -> ActuatorProfile:
    """Parse ``single:SLOT:PROTO:MOTOR:BUS`` into ``single_profile``.

    Trailing ``:joint|:wheel`` is accepted but ignored (gains are hold kwargs).
    """
    parts = [p.strip() for p in str(target).split(":")]
    if len(parts) < 5 or parts[0].lower() != "single":
        raise ValueError(
            "single target form: single:SLOT:PROTO:MOTOR:BUS "
            "(e.g. single:22:robstride:0x70:5)"
        )
    slot = int(parts[1], 0)
    proto = parts[2]
    motor_id = int(parts[3], 0)
    bus = int(parts[4], 0)
    return single_profile(slot, protocol=proto, motor_id=motor_id, bus=bus)


def resolve_motion_target(
    proxy: HostProxy,
    target: str,
    *,
    assembly: Optional[Assembly] = None,
) -> ActuatorAction:
    """Build an ``ActuatorAction`` from assembly section, single, or slot.

    Examples: ``left_arm``, ``base``, ``22``, ``slot:22``,
    ``single:22:robstride:0x70:5``.
    """
    raw = str(target).strip()
    if not raw:
        raise ValueError("empty motion target")
    lower = raw.lower()
    if lower.startswith("single:"):
        prof = parse_single_target(raw)
        return ActuatorAction.from_actuator_profile(proxy, prof)
    if lower.startswith("slot:"):
        slot = int(lower.split(":", 1)[1].strip(), 0)
        return ActuatorAction.from_slots(proxy, (slot,), name=f"slot_{slot}")
    try:
        slot = int(raw, 0)
    except ValueError:
        slot = None
    if slot is not None:
        return ActuatorAction.from_slots(proxy, (slot,), name=f"slot_{slot}")
    if assembly is not None and raw in assembly.actuators:
        return ActuatorAction.from_actuator_profile(proxy, assembly.actuator(raw))
    return ActuatorAction(proxy, proxy.profile, raw)


def ensure_plant_control(proxy: HostProxy, *, enable: bool = True) -> None:
    """Arm or disarm plant apply — prefers ``HostProxy.ensure_plant_control``."""
    fn = getattr(proxy, "ensure_plant_control", None)
    if callable(fn):
        fn(enable=enable)
        return
    from deft_controls_sdk.link import McuState

    proxy.hub.set_mcu_state(McuState.NORMAL, send=False)
    proxy.hub.set_plant_apply(bool(enable), send=False)
    proxy.send_once()


def apply_hold(
    proxy: HostProxy,
    action: ActuatorAction,
    positions: Optional[Sequence[float]] = None,
    *,
    hold_s: float = 1.0,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
    enable_control: bool = True,
) -> None:
    """Hold on ``action`` — default samples FB (stay put); or explicit positions."""
    if enable_control:
        ensure_plant_control(proxy, enable=True)
    action.hold(positions, kp=kp, kd=kd, send=False)
    proxy.send_once()
    if hold_s > 0:
        time.sleep(float(hold_s))


def apply_step(
    proxy: HostProxy,
    action: ActuatorAction,
    *,
    index: int = 0,
    delta: float = 0.05,
    hold_s: float = 1.0,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
    enable_control: bool = True,
) -> List[float]:
    """Step one index in the group by ``delta`` rad from FB (or 0)."""
    if enable_control:
        ensure_plant_control(proxy, enable=True)
    mounted = action.nudge(index=index, delta=delta, kp=kp, kd=kd, send=False)
    proxy.send_once()
    if hold_s > 0:
        time.sleep(float(hold_s))
    return [float(p) for p in mounted.meta.get("positions", [])]


def apply_blank(proxy: HostProxy, action: ActuatorAction, *, send: bool = True) -> None:
    action.blank(send=False)
    if send:
        proxy.send_once()


def _print_action_fb(action: ActuatorAction) -> None:
    pos = action.positions()
    print(
        f"  {action.name}  slots={list(action.slots)}  "
        f"fb={pos if pos is not None else '(no FB yet)'}"
    )


def _motion_hold_menu(proxy: HostProxy, assembly: Assembly) -> None:
    """Hold in place at FB (default), or at explicit positions."""
    known = ", ".join(sorted(assembly.actuators))
    target = _prompt(f"target (section: {known} | slot N)", "22")
    if not target:
        return
    try:
        action = resolve_motion_target(proxy, target, assembly=assembly)
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    _print_action_fb(action)
    pos_s = _prompt("positions (empty = sample FB / stay put)", "")
    positions: Optional[Sequence[float]] = None
    if pos_s.strip():
        n = len(action.slots)
        parts = [p.strip() for p in pos_s.replace(" ", ",").split(",") if p.strip()]
        if len(parts) == 1 and n > 1:
            positions = [float(parts[0])] * n
        else:
            positions = [float(p) for p in parts]
    hold_s = _prompt_float("hold seconds", 2.0)
    if hold_s is None:
        return
    kp = _prompt_float("kp", 20.0)
    kd = _prompt_float("kd", 1.0)
    if kp is None or kd is None:
        return
    print(
        f"\nhold  target={action.name}  slots={list(action.slots)}  "
        f"kp={kp:g} kd={kd:g}  sampled={positions is None}"
    )
    if not _prompt_yn("proceed (motors will track hold)", default=False):
        print("cancelled")
        return
    apply_hold(proxy, action, positions, hold_s=hold_s, kp=kp, kd=kd)
    _print_action_fb(action)


def _motion_step_menu(proxy: HostProxy, assembly: Assembly) -> None:
    target = _prompt("target (section | slot N)", "22")
    if not target:
        return
    try:
        action = resolve_motion_target(proxy, target, assembly=assembly)
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    _print_action_fb(action)
    idx = int(_prompt("index within group (0-based)", "0") or "0")
    delta = _prompt_float("delta rad", 0.05)
    if delta is None:
        return
    hold_s = _prompt_float("hold seconds", 1.0)
    if hold_s is None:
        return
    kp = _prompt_float("kp", 20.0)
    kd = _prompt_float("kd", 1.0)
    if kp is None or kd is None:
        return
    print(
        f"\nnudge  target={action.name}  index={idx}  "
        f"delta={delta:g}  kp={kp:g} kd={kd:g}"
    )
    if not _prompt_yn("proceed (motors will move)", default=False):
        print("cancelled")
        return
    pos = apply_step(
        proxy, action, index=idx, delta=delta, hold_s=hold_s, kp=kp, kd=kd
    )
    print(f"  commanded={pos}")
    _print_action_fb(action)


def _motion_blank_menu(proxy: HostProxy, assembly: Assembly) -> None:
    known = ", ".join(sorted(assembly.actuators))
    target = _prompt(f"target to blank ({known} | slot N)", "22")
    if not target:
        return
    try:
        action = resolve_motion_target(proxy, target, assembly=assembly)
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    print(f"\nblank  target={action.name}  slots={list(action.slots)}")
    if not _prompt_yn("proceed", default=True):
        print("cancelled")
        return
    apply_blank(proxy, action)
    print("blanked. Use observe (o) to disarm plant_apply.")


def _motion_fb_menu(proxy: HostProxy, assembly: Assembly) -> None:
    known = ", ".join(sorted(assembly.actuators))
    target = _prompt(f"target ({known} | slot N | all)", "all")
    if not target:
        return
    if target.strip().lower() == "all":
        for name in sorted(assembly.actuators):
            action = resolve_motion_target(proxy, name, assembly=assembly)
            _print_action_fb(action)
        return
    try:
        action = resolve_motion_target(proxy, target, assembly=assembly)
    except Exception as exc:  # noqa: BLE001
        print(f"target: {exc}", file=sys.stderr)
        return
    _print_action_fb(action)


def _cfg_edit_slot_menu(proxy: HostProxy) -> None:
    """Step-prompt CFG identity for one plant slot (RAM, optional NVM)."""
    slot_s = _prompt("plant slot", "22")
    if not slot_s:
        return
    try:
        slot = int(slot_s, 0)
    except ValueError:
        print("bad slot", file=sys.stderr)
        return
    # Show live row when available
    try:
        table = proxy.hub.debug.cfg_get_table()
        row = table[slot] if 0 <= slot < len(table) else None
        if row:
            print(
                f"  live RAM  bus={row.get('bus')}  "
                f"proto={row.get('protocol')}  "
                f"id={_hex_id(int(row.get('motor_id') or 0))}  "
                f"en={row.get('enabled')}"
            )
    except Exception as exc:  # noqa: BLE001
        print(f"  (cfg_get_table: {exc})")

    bus_s = _prompt("bus 1..6", "5")
    proto = _prompt("protocol (robstride|damiao|cubemars|zeroerr)", "robstride")
    mid_s = _prompt("motor id", "0x70")
    if not bus_s or not proto or not mid_s:
        return
    try:
        prof = single_profile(
            slot,
            protocol=proto,
            motor_id=int(mid_s, 0),
            bus=int(bus_s, 0),
        )
        cfg_row = prof.as_cfg_row()
    except Exception as exc:  # noqa: BLE001
        print(f"cfg: {exc}", file=sys.stderr)
        return
    persist = _prompt_yn("persist to NVM (flash)", default=False)
    print(f"\ncfg_set_slot  {cfg_row}  persist={persist}")
    if not _prompt_yn("proceed", default=False):
        print("cancelled")
        return
    resp = proxy.hub.debug.cfg_set_slot(**cfg_row, persist=persist)
    print(json.dumps(resp, indent=2, default=str))


# Back-compat name used by older call sites / tests
_cfg_apply_single_menu = _cfg_edit_slot_menu


def _motion_observe_menu(proxy: HostProxy) -> None:
    """plant_apply off — stop mounting desires (safe observe)."""
    print("observe: plant_apply=OFF")
    ensure_plant_control(proxy, enable=False)
    print("ok")


def run_actuators_test(args: argparse.Namespace) -> int:
    """``test --actuators`` — actuators-only peripheral menu."""
    from .board_verify import run_bringup

    if not getattr(args, "assembly", None):
        args.assembly = "bench"
    return run_bringup(args, scope="actuators")
