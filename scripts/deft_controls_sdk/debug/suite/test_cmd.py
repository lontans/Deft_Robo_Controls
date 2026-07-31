"""``test`` subcommand: Assembly workshop by default + domain flags.

Ownership: all leaves live under ``debug.suite`` (no ``vbeta`` imports).

| Entry        | Link mode   | Pass gates                          |
|--------------|-------------|-------------------------------------|
| test (bare)  | debug       | Assembly workshop (profiles / operate)|
| --bandwidth  | bandwidth   | measure_hold (ack_lag / fb_hz / mode)|
| --actuators  | debug       | functional discover/CFG + motion     |
| --led        | debug       | functional observe only              |
| --servo      | debug       | functional observe only              |
| --pdu-link   | debug       | wire + policy observe only           |
"""
from __future__ import annotations

import argparse
import sys
from typing import Callable, Optional, Sequence

def add_test_parser(sub: argparse._SubParsersAction) -> argparse.ArgumentParser:
    t = sub.add_parser(
        "test",
        help=(
            "Assembly workshop (default), or domain flags: "
            "--bandwidth|--actuators|--led|--servo|--pdu-link"
        ),
    )
    g = t.add_mutually_exclusive_group()
    g.add_argument(
        "--bandwidth",
        action="store_true",
        help="timing prove via measure_hold (reconnects mode=bandwidth)",
    )
    g.add_argument(
        "--actuators",
        action="store_true",
        help="actuator discover / CFG / motion menu (mode=debug)",
    )
    g.add_argument(
        "--led",
        action="store_true",
        help="LED preset hold + doctor observe (mode=debug)",
    )
    g.add_argument(
        "--servo",
        action="store_true",
        help="neck servo FB sample via pcb_tui helpers (mode=debug)",
    )
    g.add_argument(
        "--pdu-link",
        action="store_true",
        help="pdb_status wire + listen_pdu policy (mode=debug)",
    )

    # Assembly workshop knobs (bare ``test``)
    t.add_argument(
        "--assembly",
        default="product",
        metavar="NAME",
        help="with bare test (workshop): stock assembly product|bench (default product)",
    )
    t.add_argument(
        "--cfg-map",
        default="bench",
        dest="cfg_map",
        metavar="MAP",
        help="with bare test (workshop): teleop SlotSpec map bench|product (default bench)",
    )

    # bandwidth knobs
    t.add_argument(
        "--hz",
        type=float,
        default=200.0,
        help="host TX rate for --bandwidth single hold (default 200)",
    )
    t.add_argument(
        "--seconds",
        type=float,
        default=3.0,
        help="hold duration for --bandwidth (default 3)",
    )
    t.add_argument(
        "--virtual",
        action="store_true",
        help=(
            "with --bandwidth: virtual path (rx_sim ON — board CDC, treat as "
            "no motors; USB/plant TX stress only)"
        ),
    )
    t.add_argument(
        "--hardware",
        action="store_true",
        help=(
            "with --bandwidth: hardware path (rx_sim OFF — live motors / "
            "real FDCAN+MCP SPI cost)"
        ),
    )
    t.add_argument(
        "--rx-sim",
        action="store_true",
        help=(
            "with --bandwidth: synthesize ACTUATOR RX (same as --virtual; "
            "ignored when --hardware is set)"
        ),
    )
    t.add_argument(
        "--slots",
        default=None,
        metavar="LIST",
        help="with --bandwidth: comma/space slot list (skips TUI; default product via TUI)",
    )
    t.add_argument(
        "--scenario",
        default=None,
        metavar="NAME",
        help=(
            "with --bandwidth: idle|ch1..ch6|fdcan|mcp|arms|all "
            "(skips TUI; with --hz-list runs that scenario as a mini-matrix)"
        ),
    )
    t.add_argument(
        "--matrix",
        action="store_true",
        help="with --bandwidth: run default scenario matrix (idle,ch1,mcp,arms,all) × hz_list",
    )
    t.add_argument(
        "--hz-list",
        default=None,
        metavar="LIST",
        help="with --bandwidth matrix/scenario: comma Hz list (default 40,100,200,500)",
    )
    t.add_argument(
        "--trials",
        type=int,
        default=1,
        help="with --bandwidth matrix: repeats per (scenario,hz) (default 1)",
    )
    t.add_argument(
        "--no-tui",
        action="store_true",
        help="with --bandwidth: single hold at --hz (all 26 slots), skip interactive TUI",
    )

    # led knobs
    t.add_argument(
        "--preset",
        default="idle",
        choices=("idle", "pdu", "follow", "gen_2"),
        help="with --led: suite LED_PRESETS name (default idle)",
    )
    t.add_argument(
        "--hold-s",
        type=float,
        default=2.0,
        help="with --led: hold desire seconds (default 2)",
    )

    # pdu-link knobs
    t.add_argument(
        "--peer",
        action="store_true",
        help="with --pdu-link: require non-stale pdb wire (peer present)",
    )

    t.set_defaults(_cmd="test")
    return t


def _domain_from_args(args: argparse.Namespace) -> Optional[str]:
    if args.bandwidth:
        return "bandwidth"
    if args.actuators:
        return "actuators"
    if args.led:
        return "led"
    if args.servo:
        return "servo"
    if getattr(args, "pdu_link", False):
        return "pdu-link"
    return None


def run_test(args: argparse.Namespace) -> int:
    domain = _domain_from_args(args)
    if domain is None:
        # Bare ``test`` → Assembly workshop (not the domain picker).
        return _run_workshop(args)

    runners: dict[str, Callable[[argparse.Namespace], int]] = {
        "bandwidth": _run_bandwidth,
        "actuators": _run_actuators,
        "led": _run_led,
        "servo": _run_servo,
        "pdu-link": _run_pdu_link,
    }
    fn = runners.get(domain)
    if fn is None:
        print(f"unknown domain {domain!r}", file=sys.stderr)
        return 2
    return int(fn(args))


def _run_workshop(args: argparse.Namespace) -> int:
    from .workshop import run_assembly_workshop

    return run_assembly_workshop(args)


def _run_bandwidth(args: argparse.Namespace) -> int:
    from .test_bandwidth import run_bandwidth_test

    return run_bandwidth_test(args)


def _run_actuators(args: argparse.Namespace) -> int:
    from .test_actuators import run_actuators_test

    return run_actuators_test(args)


def _run_led(args: argparse.Namespace) -> int:
    from .test_led import run_led_test

    return run_led_test(args)


def _run_servo(args: argparse.Namespace) -> int:
    from .test_servo import run_servo_test

    return run_servo_test(args)


def _run_pdu_link(args: argparse.Namespace) -> int:
    from .test_pdu_link import run_pdu_link_test

    return run_pdu_link_test(args)


def parse_slot_list_optional(text: Optional[str]) -> Optional[Sequence[int]]:
    """Parse ``0,1,2`` / ``0 1 2``; empty/None → None (caller uses all slots)."""
    if text is None or not str(text).strip():
        return None
    raw = str(text).replace(",", " ").split()
    slots = [int(x, 0) for x in raw]
    if len(set(slots)) != len(slots):
        raise ValueError(f"duplicate slots in {slots}")
    for s in slots:
        if not (0 <= s < 26):
            raise ValueError(f"slot {s} out of range 0..25")
    return slots
