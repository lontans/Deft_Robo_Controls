"""Bandwidth timing prove — reconnects ``mode=bandwidth`` (never debug).

Interactive TUI nests under **virtual** (board CDC, treat as no motors —
``rx_sim`` on) vs **hardware** (live motors — ``rx_sim`` off). Both need a
board over CDC. Hardware inventory lives at ``python -m pcb_lab inventory``.

Non-interactive: ``--virtual`` / ``--hardware``, ``--matrix``, ``--scenario``.
"""
from __future__ import annotations

import argparse
import json
import sys
from typing import Dict, List, Optional, Sequence

from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

from .bandwidth_matrix import (
    DEFAULT_HZ_LIST,
    SCENARIO_BLURBS,
    SCENARIO_NAMES,
    default_scenarios_for_matrix,
    parse_hz_list,
    render_matrix_summary,
    render_report,
    scenario_slots,
)

# virtual = USB/plant stress with synthesized ACTUATOR RX (no motor wire cost)
# hardware = live motors; measure real FDCAN/MCP path
BANDWIDTH_KINDS = ("virtual", "hardware")


def _prompt(msg: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        ans = input(f"{msg}{suffix}> ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return ""
    return ans if ans else default


def _hold_once(
    *,
    port: Optional[str],
    listen_pdu: bool,
    slots: Sequence[int],
    hz: float,
    seconds: float,
    rx_sim: bool,
    label: str,
    print_report: bool = True,
) -> dict:
    from deft_controls_sdk.debug.metrics import measure_hold

    stream_hz = float(hz)
    with HostProxy.connect(
        port,
        stream_hz=stream_hz,
        telemetry_hz=stream_hz,
        idle_first=True,
        listen_pdu=listen_pdu,
        mode="bandwidth",
    ) as proxy:
        hub = proxy.hub
        hub.set_plant_apply(True, send=True)
        if rx_sim:
            hub.set_rx_sim(True)
        desires: Dict[int, ActuatorDesire] = {
            int(s): ActuatorDesire(position=1e-6) for s in slots
        }
        report = measure_hold(
            hub,
            label,
            desires,
            seconds=float(seconds),
            hz=float(hz),
            expected_stm32_mode="bandwidth",
            print_report=print_report,
        )
        if rx_sim:
            try:
                hub.set_rx_sim(False)
            except Exception:  # noqa: BLE001
                pass
    report = dict(report)
    report["hz"] = float(hz)
    report["seconds"] = float(seconds)
    report["rx_sim"] = bool(rx_sim)
    report["slots"] = list(slots)
    return report


def run_scenario_matrix(
    *,
    port: Optional[str],
    listen_pdu: bool,
    scenarios: Sequence[str],
    hz_list: Sequence[float],
    seconds: float,
    trials: int,
    rx_sim: bool,
) -> Dict[str, List[dict]]:
    """Run scenarios × hz × trials; return results keyed by scenario name."""
    by_scenario: Dict[str, List[dict]] = {}
    trials = max(1, int(trials))
    for scenario in scenarios:
        slots = scenario_slots(scenario)
        rows: List[dict] = []
        print(
            f"\n--- scenario={scenario}  slots={slots or '∅'}  "
            f"hz={list(hz_list)}  trials={trials}  rx_sim={rx_sim} ---"
        )
        for hz in hz_list:
            for trial in range(trials):
                label = f"bandwidth {scenario} @ {hz:g} Hz (trial {trial + 1}/{trials})"
                report = _hold_once(
                    port=port,
                    listen_pdu=listen_pdu,
                    slots=slots,
                    hz=float(hz),
                    seconds=seconds,
                    rx_sim=rx_sim,
                    label=label,
                    print_report=True,
                )
                rows.append(report)
        by_scenario[scenario] = rows
        print(render_report(scenario, rows))
    return by_scenario


def _resolve_rx_sim(args: argparse.Namespace) -> bool:
    """virtual → True; hardware → False; else honor --rx-sim."""
    kind = getattr(args, "bandwidth_kind", None)
    if kind == "virtual" or bool(getattr(args, "virtual", False)):
        return True
    if kind == "hardware" or bool(getattr(args, "hardware", False)):
        return False
    return bool(getattr(args, "rx_sim", False))


def _run_single_from_args(args: argparse.Namespace) -> int:
    from .test_cmd import parse_slot_list_optional

    hz = float(args.hz)
    seconds = float(args.seconds)
    if hz <= 0 or seconds <= 0:
        print("test --bandwidth needs --hz > 0 and --seconds > 0", file=sys.stderr)
        return 2

    scenario = getattr(args, "scenario", None)
    try:
        if scenario:
            slots = scenario_slots(str(scenario))
            label = f"suite test --bandwidth scenario={scenario}"
        else:
            slot_list = parse_slot_list_optional(getattr(args, "slots", None))
            slots = list(range(ACTUATOR_COUNT)) if slot_list is None else list(slot_list)
            label = "suite test --bandwidth"
    except ValueError as exc:
        print(f"test --bandwidth: {exc}", file=sys.stderr)
        return 2

    rx_sim = _resolve_rx_sim(args)
    kind = "virtual" if rx_sim else "hardware"
    print(
        f"test --bandwidth  kind={kind}  mode=bandwidth  hold {seconds:g}s @ {hz:g} Hz  "
        f"slots={len(slots)}  rx_sim={rx_sim}"
        + (f"  scenario={scenario}" if scenario else "")
    )
    if rx_sim:
        print(
            "note: virtual / rx_sim ON — MCP SPI RX not exercised (synthesized ACTUATOR RX)",
            file=sys.stderr,
        )

    report = _hold_once(
        port=getattr(args, "port", None),
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        slots=slots,
        hz=hz,
        seconds=seconds,
        rx_sim=rx_sim,
        label=label,
    )
    ok = bool(report.get("ok"))
    print(
        json.dumps(
            {
                "ok": ok,
                "scenario": scenario,
                "raw_fb_hz": report.get("raw_fb_hz"),
                "stm32_mode": report.get("fb_stm32_mode_name"),
                "host_stm32_mode": report.get("host_stm32_mode_name"),
                "ok_lag": report.get("ok_lag"),
                "ok_fb": report.get("ok_fb"),
                "ok_mode": report.get("ok_mode"),
                "rx_sim": rx_sim,
                "slots": list(slots),
                "ack_lag_max": report.get("ack_lag_max"),
                "lap_ms_mean": report.get("lap_ms_mean"),
                "lap_max_ms": report.get("lap_max_ms"),
            },
            indent=2,
        )
    )
    return 0 if ok else 1


def _run_matrix_from_args(args: argparse.Namespace) -> int:
    try:
        if getattr(args, "hz_list", None):
            hz_list = parse_hz_list(str(args.hz_list))
        else:
            hz_list = list(DEFAULT_HZ_LIST)
        if getattr(args, "scenario", None):
            scenarios = [str(args.scenario).strip().lower()]
            scenario_slots(scenarios[0])  # validate
        else:
            scenarios = default_scenarios_for_matrix()
    except ValueError as exc:
        print(f"test --bandwidth matrix: {exc}", file=sys.stderr)
        return 2

    seconds = float(args.seconds)
    trials = int(getattr(args, "trials", 1) or 1)
    rx_sim = _resolve_rx_sim(args)
    kind = "virtual" if rx_sim else "hardware"
    if seconds <= 0:
        print("test --bandwidth needs --seconds > 0", file=sys.stderr)
        return 2
    if rx_sim:
        print(
            "note: virtual / rx_sim ON — matrix does not measure real MCP SPI RX",
            file=sys.stderr,
        )

    by_scenario = run_scenario_matrix(
        port=getattr(args, "port", None),
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        scenarios=scenarios,
        hz_list=hz_list,
        seconds=seconds,
        trials=trials,
        rx_sim=rx_sim,
    )
    print(f"\n======== MATRIX SUMMARY ({kind}) ========")
    print(render_matrix_summary(by_scenario, rx_sim=rx_sim))

    flat = [r for rows in by_scenario.values() for r in rows]
    ok_all = all(bool(r.get("ok")) for r in flat) if flat else False
    print(
        json.dumps(
            {
                "ok": ok_all,
                "kind": kind,
                "rx_sim": rx_sim,
                "scenarios": list(by_scenario.keys()),
                "hz_list": hz_list,
                "trials": trials,
                "hit_ok": sum(1 for r in flat if r.get("ok")),
                "hit_total": len(flat),
            },
            indent=2,
        )
    )
    return 0 if ok_all else 1


def _kind_submenu(
    *,
    kind: str,
    rx_sim: bool,
    args: argparse.Namespace,
    hz: float,
    seconds: float,
    hz_list: List[float],
    trials: int,
) -> tuple[float, float, List[float], int]:
    """Scenario / matrix menu for one virtual|hardware branch.

    Returns updated knobs so they persist across top-level kind switches.
    """
    port = getattr(args, "port", None)
    listen_pdu = bool(getattr(args, "listen_pdu", False))
    while True:
        print(
            f"\n[{kind}] knobs: hz={hz:g}  seconds={seconds:g}  "
            f"hz_list={hz_list}  trials={trials}  rx_sim={rx_sim}\n"
            "  1) single hold @ hz (product / all / custom slots)\n"
            "  2) one scenario × hz_list\n"
            "  3) matrix (idle, ch1, mcp, arms, all) × hz_list\n"
            "  4) full channel sweep (ch1..ch6 + mcp + all) × hz_list\n"
            "  5) set knobs (hz / seconds / hz_list / trials)\n"
            "  b) back"
        )
        choice = _prompt("choice", "").lower()
        if not choice or choice in ("b", "back", "q", "quit"):
            return hz, seconds, hz_list, trials

        if choice in ("5", "knobs", "k"):
            hz_s = _prompt("single-hold hz", f"{hz:g}")
            sec_s = _prompt("seconds per hold", f"{seconds:g}")
            list_s = _prompt("hz_list (comma)", ",".join(f"{h:g}" for h in hz_list))
            trials_s = _prompt("trials", str(trials))
            try:
                hz = float(hz_s)
                seconds = float(sec_s)
                hz_list = parse_hz_list(list_s)
                trials = max(1, int(trials_s, 0))
            except ValueError as exc:
                print(f"knobs: {exc}", file=sys.stderr)
            continue

        if choice in ("1", "hold", "h"):
            slot_mode = _prompt("slots: all|product|custom comma list", "product")
            try:
                if slot_mode in ("all", "*"):
                    slots = list(range(ACTUATOR_COUNT))
                elif slot_mode in ("product", "p", ""):
                    slots = scenario_slots("all")
                else:
                    from .test_cmd import parse_slot_list_optional

                    parsed = parse_slot_list_optional(slot_mode)
                    slots = list(parsed) if parsed is not None else scenario_slots("all")
            except ValueError as exc:
                print(f"slots: {exc}", file=sys.stderr)
                continue
            report = _hold_once(
                port=port,
                listen_pdu=listen_pdu,
                slots=slots,
                hz=hz,
                seconds=seconds,
                rx_sim=rx_sim,
                label=f"bandwidth {kind} hold @ {hz:g} Hz",
            )
            print(json.dumps({"ok": report.get("ok"), "raw_fb_hz": report.get("raw_fb_hz")}, indent=2))
            continue

        if choice in ("2", "scenario", "s"):
            print("scenarios:")
            for i, name in enumerate(SCENARIO_NAMES, start=1):
                print(f"  {i}) {name:<6}  {SCENARIO_BLURBS.get(name, '')}")
            pick = _prompt("scenario", "mcp").lower()
            if pick.isdigit():
                idx = int(pick)
                if 1 <= idx <= len(SCENARIO_NAMES):
                    pick = SCENARIO_NAMES[idx - 1]
            try:
                scenario_slots(pick)
            except ValueError as exc:
                print(f"scenario: {exc}", file=sys.stderr)
                continue
            by_scenario = run_scenario_matrix(
                port=port,
                listen_pdu=listen_pdu,
                scenarios=[pick],
                hz_list=hz_list,
                seconds=seconds,
                trials=trials,
                rx_sim=rx_sim,
            )
            print(render_matrix_summary(by_scenario, rx_sim=rx_sim))
            continue

        if choice in ("3", "matrix", "m"):
            scenarios = default_scenarios_for_matrix()
            by_scenario = run_scenario_matrix(
                port=port,
                listen_pdu=listen_pdu,
                scenarios=scenarios,
                hz_list=hz_list,
                seconds=seconds,
                trials=trials,
                rx_sim=rx_sim,
            )
            print(f"\n======== MATRIX SUMMARY ({kind}) ========")
            print(render_matrix_summary(by_scenario, rx_sim=rx_sim))
            continue

        if choice in ("4", "sweep", "full"):
            scenarios = [
                "idle",
                "ch1",
                "ch2",
                "ch3",
                "ch4",
                "ch5",
                "ch6",
                "mcp",
                "fdcan",
                "all",
            ]
            by_scenario = run_scenario_matrix(
                port=port,
                listen_pdu=listen_pdu,
                scenarios=scenarios,
                hz_list=hz_list,
                seconds=seconds,
                trials=trials,
                rx_sim=rx_sim,
            )
            print(f"\n======== CHANNEL SWEEP SUMMARY ({kind}) ========")
            print(render_matrix_summary(by_scenario, rx_sim=rx_sim))
            continue

        print(f"unknown choice: {choice!r}", file=sys.stderr)


def _bandwidth_tui(args: argparse.Namespace) -> int:
    """Top-level virtual | hardware, then scenario matrix menu."""
    hz = float(getattr(args, "hz", 200.0) or 200.0)
    seconds = float(getattr(args, "seconds", 3.0) or 3.0)
    hz_list = list(DEFAULT_HZ_LIST)
    if getattr(args, "hz_list", None):
        try:
            hz_list = parse_hz_list(str(args.hz_list))
        except ValueError as exc:
            print(f"hz-list: {exc}", file=sys.stderr)
    trials = int(getattr(args, "trials", 1) or 1)

    # CLI can jump straight into a kind
    forced = None
    if bool(getattr(args, "virtual", False)):
        forced = "virtual"
    elif bool(getattr(args, "hardware", False)):
        forced = "hardware"
    elif getattr(args, "bandwidth_kind", None) in BANDWIDTH_KINDS:
        forced = str(args.bandwidth_kind)

    print(
        "\nbandwidth TUI  (board CDC required)\n"
        "  virtual  — treat as no motors; rx_sim ON (USB / plant TX stress)\n"
        "  hardware — live motors; rx_sim OFF (real FDCAN / MCP SPI path)\n"
        "  (hardware inventory: python -m pcb_lab inventory)\n"
    )

    while True:
        if forced:
            kind = forced
            forced = None  # only once from CLI; then show top menu again
        else:
            print(
                "\n  1) virtual   (rx_sim ON)\n"
                "  2) hardware  (rx_sim OFF)\n"
                "  q) quit"
            )
            choice = _prompt("choice", "").lower()
            if not choice or choice in ("q", "quit", "exit"):
                return 0
            if choice in ("1", "v", "virtual"):
                kind = "virtual"
            elif choice in ("2", "h", "hardware", "hw"):
                kind = "hardware"
            else:
                print(f"unknown choice: {choice!r}", file=sys.stderr)
                continue

        rx_sim = kind == "virtual"
        hz, seconds, hz_list, trials = _kind_submenu(
            kind=kind,
            rx_sim=rx_sim,
            args=args,
            hz=hz,
            seconds=seconds,
            hz_list=hz_list,
            trials=trials,
        )


def run_bandwidth_test(args: argparse.Namespace) -> int:
    """Entry from ``test --bandwidth`` / domain picker."""
    if getattr(args, "matrix", False):
        return _run_matrix_from_args(args)
    if getattr(args, "hz_list", None) and getattr(args, "scenario", None):
        return _run_matrix_from_args(args)
    if getattr(args, "scenario", None):
        return _run_single_from_args(args)
    if getattr(args, "slots", None):
        return _run_single_from_args(args)
    if getattr(args, "no_tui", False):
        return _run_single_from_args(args)
    return _bandwidth_tui(args)
