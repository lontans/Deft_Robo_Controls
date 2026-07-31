"""CLI / TUI for peripheral inventory — ``pcb_lab.debug test --inventory``.

Interactive by default (choose buses + ID ranges). Non-interactive for AI::

    python -m pcb_lab.debug test --inventory --preset bench --buses 5,6
    python -m pcb_lab.debug test --inventory --rs-range 0x70-0x75 --dm-range 1-8 --json
"""
from __future__ import annotations

import argparse
import json
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from deft_controls_sdk.debug.inventory import (
    PRESET_BLURBS,
    RANGE_PRESETS,
    parse_bus_list,
    parse_id_range,
    parse_protocol_queue,
    run_inventory,
)
from deft_controls_sdk.host_proxy import HostProxy


def _prompt(msg: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        ans = input(f"{msg}{suffix}> ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return ""
    return ans if ans else default


def _prompt_yn(msg: str, *, default: bool = True) -> bool:
    d = "y" if default else "n"
    ans = _prompt(f"{msg} (y/n)", d).lower()
    if not ans:
        return default
    return ans in ("y", "yes")


def _tui_plan() -> dict:
    """Ask the user for buses / ranges / sections. Returns kwargs for run_inventory."""
    print(
        "\ninventory TUI  (mode=debug)\n"
        "  Actuator discover is slow if the ID range is wide — pick a tight range.\n"
        "  Presets: " + ", ".join(f"{k}={PRESET_BLURBS[k]}" for k in ("bench", "product", "full"))
    )
    buses_s = _prompt("buses (1-6 / all)", "all")
    buses = parse_bus_list(buses_s or "all")

    include_actuators = _prompt_yn("probe actuators (RS/DM discover)", default=True)
    protocols: List[str] = []
    ranges: Dict[str, Tuple[int, int]] = {}
    preset: Optional[str] = None

    if include_actuators:
        proto_s = _prompt("protocols (robstride,damiao / rs / dm / both)", "both")
        if proto_s.strip().lower() in ("both", "all", "*"):
            protocols = ["robstride", "damiao"]
        else:
            protocols = parse_protocol_queue(proto_s)

        print("range source:")
        print("  1) preset bench   " + PRESET_BLURBS["bench"])
        print("  2) preset product " + PRESET_BLURBS["product"])
        print("  3) preset full    " + PRESET_BLURBS["full"] + "  (slow)")
        print("  4) custom start-end per protocol")
        pick = _prompt("choice", "1").lower()
        if pick in ("1", "bench", "b"):
            preset = "bench"
        elif pick in ("2", "product", "yam", "p"):
            preset = "product"
        elif pick in ("3", "full", "f"):
            if not _prompt_yn("full range is SLOW — continue?", default=False):
                preset = "bench"
                print("using bench instead")
            else:
                preset = "full"
        else:
            for proto in protocols:
                hint = "0x70-0x75" if proto == "robstride" else "1-8"
                raw = _prompt(f"{proto} id range", hint)
                ranges[proto] = parse_id_range(raw, protocol=proto)

    include_servos = _prompt_yn("sample neck servos", default=True)
    include_pdu = _prompt_yn("check PDU / kill-link wire", default=True)
    require_peer = False
    if include_pdu:
        require_peer = _prompt_yn("require live PDU peer (fail if stale/missing)", default=False)

    return {
        "buses": buses,
        "protocols": protocols or ["robstride", "damiao"],
        "ranges": ranges or None,
        "preset": preset,
        "include_actuators": include_actuators,
        "include_servos": include_servos,
        "include_pdu": include_pdu,
        "require_pdu_peer": require_peer,
    }


def _plan_from_args(args: argparse.Namespace) -> dict:
    """Non-interactive plan from CLI flags (automation-friendly)."""
    buses = parse_bus_list(getattr(args, "buses", None) or "all")
    protocols = parse_protocol_queue(getattr(args, "protocols", None) or "robstride,damiao")

    include_actuators = not bool(getattr(args, "no_actuators", False))
    include_servos = not bool(getattr(args, "no_servos", False))
    include_pdu = not bool(getattr(args, "no_pdu", False))
    # Allow section-only flags to disable the rest when explicitly set.
    if getattr(args, "actuators_only", False):
        include_servos = False
        include_pdu = False
    if getattr(args, "servos_only", False):
        include_actuators = False
        include_pdu = False
    if getattr(args, "pdu_only", False):
        include_actuators = False
        include_servos = False

    ranges: Dict[str, Tuple[int, int]] = {}
    if getattr(args, "rs_range", None):
        ranges["robstride"] = parse_id_range(str(args.rs_range), protocol="robstride")
    if getattr(args, "dm_range", None):
        ranges["damiao"] = parse_id_range(str(args.dm_range), protocol="damiao")

    preset = getattr(args, "preset", None)
    if include_actuators and not ranges and not preset:
        raise ValueError(
            "actuator inventory needs --preset bench|product|full "
            "or --rs-range / --dm-range (or run without flags for TUI)"
        )

    return {
        "buses": buses,
        "protocols": protocols,
        "ranges": ranges or None,
        "preset": preset,
        "include_actuators": include_actuators,
        "include_servos": include_servos,
        "include_pdu": include_pdu,
        "require_pdu_peer": bool(getattr(args, "peer", False)),
    }


def add_inventory_arguments(
    p: argparse.ArgumentParser,
    *,
    omit: Sequence[str] = (),
) -> None:
    """Shared flags for ``test --inventory`` (and standalone inventory parser)."""
    skip = {str(x) for x in omit}

    def _add(*opts: str, **kwargs) -> None:
        if any(o in skip for o in opts):
            return
        p.add_argument(*opts, **kwargs)

    _add(
        "--buses",
        default=None,
        help="CAN buses 1..6 or 'all' (default: all / TUI)",
    )
    _add(
        "--protocols",
        default=None,
        help="robstride,damiao (default: both)",
    )
    _add(
        "--preset",
        choices=sorted(set(RANGE_PRESETS) - {"yam"}),  # yam==product
        default=None,
        help="ID range preset (bench|product|full) — required for non-TUI actuator probe",
    )
    _add(
        "--rs-range",
        default=None,
        metavar="START-END",
        help="RobStride ID range e.g. 0x70-0x75",
    )
    _add(
        "--dm-range",
        default=None,
        metavar="START-END",
        help="Damiao ID range e.g. 1-8",
    )
    _add("--no-actuators", action="store_true", help="skip CAN discover")
    _add("--no-servos", action="store_true", help="skip neck servo sample")
    _add("--no-pdu", action="store_true", help="skip PDU wire check")
    _add("--actuators-only", action="store_true")
    _add("--servos-only", action="store_true")
    _add("--pdu-only", action="store_true")
    _add(
        "--peer",
        action="store_true",
        help="require live PDU peer (fail if missing/stale)",
    )
    _add(
        "--tui",
        action="store_true",
        help="force interactive range picker (default when no range flags)",
    )
    _add(
        "--no-tui",
        action="store_true",
        help="fail instead of opening TUI when ranges missing",
    )
    _add("--json", action="store_true", help="print JSON only (quiet banners)")
    _add(
        "--listen-ms",
        type=int,
        default=40,
        help="discover listen_ms per probe (default 40)",
    )


def run_inventory_cli(args: argparse.Namespace) -> int:
    """Entry from ``pcb_lab.debug test --inventory``."""
    want_tui = bool(getattr(args, "tui", False))
    has_range = bool(
        getattr(args, "preset", None)
        or getattr(args, "rs_range", None)
        or getattr(args, "dm_range", None)
        or getattr(args, "no_actuators", False)
        or getattr(args, "servos_only", False)
        or getattr(args, "pdu_only", False)
    )
    if want_tui or (not has_range and not getattr(args, "no_tui", False)):
        try:
            plan = _tui_plan()
        except (ValueError, EOFError) as exc:
            print(f"inventory TUI: {exc}", file=sys.stderr)
            return 2
    else:
        try:
            plan = _plan_from_args(args)
        except ValueError as exc:
            print(f"inventory: {exc}", file=sys.stderr)
            return 2

    listen = bool(getattr(args, "listen_pdu", False) or plan.get("require_pdu_peer"))
    print_report = not bool(getattr(args, "json", False))

    with HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=50.0,
        telemetry_hz=50.0,
        armed=False,
        listen_pdu=listen,
        mode="debug",
    ) as proxy:
        summary = run_inventory(
            proxy,
            buses=plan["buses"],
            protocols=plan["protocols"],
            ranges=plan.get("ranges"),
            preset=plan.get("preset"),
            include_actuators=bool(plan["include_actuators"]),
            include_servos=bool(plan["include_servos"]),
            include_pdu=bool(plan["include_pdu"]),
            require_pdu_peer=bool(plan.get("require_pdu_peer")),
            listen_ms=int(getattr(args, "listen_ms", 40) or 40),
            print_report=print_report,
        )
    if getattr(args, "json", False):
        print(json.dumps(summary, indent=2, default=str))
    return 0 if summary.get("ok") else 1


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="pcb_lab.debug test --inventory",
        description="Probe connected hardware (actuators / servos / PDU).",
    )
    p.add_argument("--port", default=None, help="CDC COM port")
    p.add_argument(
        "--listen-pdu",
        action="store_true",
        help="honor PDB kill bytes on host (also implied by --peer)",
    )
    add_inventory_arguments(p)
    return p
