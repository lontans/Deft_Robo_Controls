"""Actuator prove — ``mode=debug`` discover / CFG / RobStride cal (no timing floors)."""
from __future__ import annotations

import argparse
import json
import sys
from typing import Dict, Iterable, List, Sequence, Tuple

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


def _connect_debug(args: argparse.Namespace) -> HostProxy:
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz
    return HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        idle_first=True,
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        mode="debug",
    )


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
        "protocol (robstride|damiao|all, comma queue)",
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


def run_actuators_test(args: argparse.Namespace) -> int:
    print("test --actuators  mode=debug  (functional only — no ack_lag/fb_hz gates)")
    with _connect_debug(args) as proxy:
        while True:
            print(
                "\n  1) show CFG table\n"
                "  2) show enabled CFG only\n"
                "  3) discover (multi-bus / multi-protocol)\n"
                "  4) calibrate robstride (encoder cal)\n"
                "  5) CFG enabled hint (JSON)\n"
                "  q) quit"
            )
            choice = _prompt("choice", "q").lower()
            if choice in ("", "q", "quit", "exit"):
                break
            try:
                if choice in ("1", "cfg"):
                    _show_cfg(proxy)
                elif choice in ("2", "enabled"):
                    _show_cfg(proxy, only_enabled=True)
                elif choice in ("3", "discover", "d"):
                    _discover_menu(proxy)
                elif choice in ("4", "calibrate", "cal", "c"):
                    _calibrate_robstride_menu(proxy)
                elif choice in ("5", "hint"):
                    _cfg_hint(proxy)
                else:
                    print(f"unknown choice {choice!r}")
            except Exception as exc:  # noqa: BLE001
                print(f"error: {exc}", file=sys.stderr)
                return 1
    return 0
