"""Plant debug suite CLI: ``show|set|test``.

Canonical::

    python -m deft_controls_sdk.debug.suite [--port COM5] show|set|test …

Lab alias (same argv)::

    python -m pcb_lab.debug …

``test`` owns its own connect per domain (bandwidth vs debug) — see test_cmd.
Board USB / Soft-DFU lives on ``python -m pcb_lab`` (not here).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Optional, Sequence

from deft_controls_sdk.debug.stream_pause import pause_plant_stream
from deft_controls_sdk.host_proxy import HostProxy

from .cfg_editor import run_cfg_editor
from .proto import parse_protocol
from .show import (
    collect_bandwidth,
    collect_cfg,
    format_cfg_table,
)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="pcb_lab.debug",
        description=(
            "Peripheral / CFG debug suite: show | set | test. "
            "Timing proves: test --bandwidth (mode=bandwidth). "
            "Peripheral proves: test --inventory|--actuators|--led|--servo|--pdu-link "
            "(mode=debug). Board USB/flash: python -m pcb_lab."
        ),
    )
    p.add_argument("--port", default=None, help="CDC COM port (auto if omitted)")
    p.add_argument(
        "--listen-pdu",
        action="store_true",
        help="honor PDU kill in status / soft-kill (default off)",
    )
    p.add_argument(
        "--stream-hz",
        type=float,
        default=200.0,
        help="plant stream rate while connected (default 200)",
    )
    p.add_argument(
        "--telemetry-hz",
        type=float,
        default=None,
        help="telemetry publish rate (default: same as --stream-hz)",
    )
    p.add_argument(
        "--persist-telemetry",
        action="store_true",
        help="write TelemetryCache to state.json (implied by show --pcb)",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sh = sub.add_parser(
        "show",
        help="snapshot one or more views (combine flags)",
    )
    sh.add_argument("--cfg", action="store_true", help="actuator CFG table")
    sh.add_argument(
        "--cfg-enabled",
        action="store_true",
        help="CFG table, enabled slots only (implies --cfg)",
    )
    sh.add_argument(
        "--bandwidth",
        action="store_true",
        help="host stream_hz + plant/periph lap timings (snapshot; not a stress test)",
    )
    sh.add_argument(
        "--pcb",
        action="store_true",
        help="live TUI: CFG connections + TelemetryCache FB snapshot + cache age",
    )
    sh.add_argument(
        "--once",
        action="store_true",
        help="with --pcb: print one frame and exit (no refresh loop)",
    )
    sh.add_argument(
        "--json",
        action="store_true",
        help="emit JSON object (default: human text for cfg, JSON otherwise)",
    )
    sh.add_argument(
        "--settle-ms",
        type=float,
        default=200.0,
        help="wait after connect before sampling FB (default 200)",
    )
    sh.set_defaults(_cmd="show")

    st = sub.add_parser("set", help="edit CFG / stream / LED policy")
    st.add_argument("--cfg", action="store_true", help="edit actuator CFG table")
    st.add_argument(
        "--stream",
        type=float,
        default=None,
        metavar="HZ",
        help="set plant stream_hz and telemetry_hz together (e.g. 200)",
    )
    st.add_argument(
        "--led",
        choices=("debug", "pdu", "follow"),
        default=None,
        help="LedDesire policy: debug|pdu|follow",
    )
    st.add_argument(
        "--led-mode",
        type=int,
        default=None,
        metavar="N",
        help="numeric LED pattern 1..8 (implies --led debug)",
    )
    st.add_argument(
        "--led-brightness",
        type=int,
        default=8,
        help="LED master brightness 0..31 (default 8)",
    )
    st.add_argument(
        "--hold-s",
        type=float,
        default=1.0,
        help="after --led/--led-mode, keep streaming that desire (default 1.0s)",
    )
    st.add_argument("--slot", type=int, default=None, help="one-shot: slot index")
    st.add_argument("--bus", type=int, default=None)
    st.add_argument(
        "--protocol",
        default=None,
        help="none|robstride|cubemars|damiao|zeroerr|int",
    )
    st.add_argument(
        "--motor-id",
        default=None,
        help="motor id (hex ok, e.g. 0x70)",
    )
    st.add_argument(
        "--master-id",
        default=None,
        help="Damiao master id (hex; default 0xFF=auto)",
    )
    st.add_argument(
        "--enabled",
        dest="enabled",
        action="store_true",
        help="one-shot: enable slot",
    )
    st.add_argument(
        "--disabled",
        dest="enabled",
        action="store_false",
        help="one-shot: disable slot",
    )
    st.add_argument(
        "--persist",
        action="store_true",
        help="flash NVM SAVE after RAM apply (survives reboot)",
    )
    st.set_defaults(_cmd="set", enabled=None)

    from .test_cmd import add_test_parser

    add_test_parser(sub)
    return p


def _connect(args: argparse.Namespace) -> HostProxy:
    stream_hz = float(args.stream_hz)
    tel = args.telemetry_hz
    telemetry_hz = float(tel) if tel is not None else stream_hz
    # --pcb needs TelemetryCache → state.json so the TUI can show cache age /
    # FB strip the same way debug_dashboard does.
    persist = bool(getattr(args, "pcb", False) or getattr(args, "persist_telemetry", False))
    # Bandwidth-only runs must not arm the debug lanes (ADR-004).
    want_bw_only = bool(getattr(args, "bandwidth", False)) and not (
        getattr(args, "cfg", False)
        or getattr(args, "cfg_enabled", False)
        or getattr(args, "pcb", False)
    )
    return HostProxy.connect(
        args.port,
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        armed=False,
        listen_pdu=bool(args.listen_pdu),
        persist_telemetry=persist,
        mode="bandwidth" if want_bw_only else "debug",
    )


def _cmd_show(proxy: HostProxy, args: argparse.Namespace) -> int:
    want_pcb = bool(getattr(args, "pcb", False))
    want_cfg = bool(args.cfg or args.cfg_enabled)
    want_bw = bool(args.bandwidth)
    if want_pcb and not (want_cfg or want_bw):
        settle = max(0.0, float(args.settle_ms) / 1000.0)
        if settle:
            time.sleep(settle)
        from .pcb_tui import run_pcb_dashboard

        return run_pcb_dashboard(
            proxy,
            once=bool(getattr(args, "once", False) or args.json),
        )

    if not (want_cfg or want_bw or want_pcb):
        # Bare show → CFG table (live board: show --pcb)
        want_cfg = True

    settle = max(0.0, float(args.settle_ms) / 1000.0)
    if settle:
        time.sleep(settle)

    payload: dict = {}
    if want_cfg:
        payload["cfg"] = collect_cfg(proxy)
    if want_bw:
        payload["bandwidth"] = collect_bandwidth(proxy)

    if args.json:
        print(json.dumps(payload, indent=2))
        return 0

    if want_cfg:
        print(
            format_cfg_table(
                payload["cfg"],
                only_enabled=bool(args.cfg_enabled),
                banner=True,
                box=True,
            )
        )
        if want_bw or want_pcb:
            print()
    rest = {k: v for k, v in payload.items() if k != "cfg"}
    if rest:
        print(json.dumps(rest, indent=2))
    if want_pcb:
        from .pcb_tui import run_pcb_dashboard

        if rest or want_cfg:
            print()
        return run_pcb_dashboard(
            proxy,
            once=bool(getattr(args, "once", False) or args.json),
        )
    return 0


def _cmd_set(proxy: HostProxy, args: argparse.Namespace) -> int:
    did = False
    if args.stream is not None:
        hz = float(args.stream)
        if hz <= 0:
            print("set --stream needs HZ > 0", file=sys.stderr)
            return 2
        hub = proxy.hub
        if hub.is_streaming:
            hub.stop_streaming()
        hub.start_streaming(hz=hz, telemetry_hz=hz, auto_soft_kill=hub.listen_pdu)
        proxy._stream_hz = hz
        proxy._telemetry_hz = hz
        print(f"stream_hz={hz:g} telemetry_hz={hz:g}")
        did = True

    led_policy = args.led
    led_pattern = args.led_mode
    if led_pattern is not None and led_policy is None:
        led_policy = "debug"
    if led_policy is not None:
        from deft_controls_sdk.link import LedDesire
        from deft_controls_sdk.link.api_types import led_mode_name

        if led_policy == "debug" and led_pattern is None:
            print("set --led debug needs --led-mode N (1..8)", file=sys.stderr)
            return 2
        if led_policy == "debug":
            n = int(led_pattern)
            if not (1 <= n <= 8):
                print("set --led-mode needs N in 1..8", file=sys.stderr)
                return 2
        desire = LedDesire(
            mode=led_policy,
            pattern=int(led_pattern or 0),
            master_brightness=int(args.led_brightness),
        )
        # Ensure plant stream is pushing the held desire (idle_first already
        # started streaming; restart if somehow stopped).
        hub = proxy.hub
        if not hub.is_streaming:
            hub.start_streaming(
                hz=float(proxy._stream_hz),
                telemetry_hz=float(proxy._telemetry_hz),
                auto_soft_kill=hub.listen_pdu,
            )
        proxy.set_led(desire, send=True)
        hold = max(0.0, float(args.hold_s))
        if hold:
            time.sleep(hold)
        print(
            f"led policy={desire.mode} pattern={desire.pattern} "
            f"({led_mode_name(desire.wire_mode)}) "
            f"wire={desire.wire_mode} brightness={desire.master_brightness} "
            f"listen_pdu={proxy.listen_pdu} held={hold:g}s"
        )
        did = True

    if not args.cfg and args.slot is None:
        if did:
            return 0
        print(
            "set: pass --cfg / --stream HZ / --led …\n"
            "  python -m pcb_lab.debug set --stream 200\n"
            "  python -m pcb_lab.debug set --led-mode 8\n"
            "  python -m pcb_lab.debug set --led pdu\n"
            "  python -m pcb_lab.debug set --led follow\n"
            "  python -m pcb_lab.debug set --cfg",
            file=sys.stderr,
        )
        return 2

    if args.cfg or args.slot is not None:
        one_shot = args.slot is not None
        if one_shot:
            if args.bus is None or args.protocol is None or args.motor_id is None:
                print(
                    "one-shot set --cfg needs --slot --bus --protocol --motor-id",
                    file=sys.stderr,
                )
                return 2
            enabled = True if args.enabled is None else bool(args.enabled)
            proto = parse_protocol(args.protocol)
            motor_id = int(args.motor_id, 0) & 0xFF
            master_id = 0xFF if args.master_id is None else int(args.master_id, 0) & 0xFF
            with pause_plant_stream(proxy.hub):
                proxy.hub.debug.cfg_set_slot(
                    slot=int(args.slot),
                    bus=int(args.bus),
                    protocol=proto,
                    motor_id=motor_id,
                    master_id=master_id,
                    enabled=enabled,
                    persist=bool(args.persist),
                )
            print(
                f"CFG slot {args.slot}: bus={args.bus} proto={args.protocol} "
                f"id=0x{motor_id:02X} en={enabled} "
                f"{'persisted' if args.persist else 'RAM only'}"
            )
            return 0

        return run_cfg_editor(proxy)

    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(list(argv) if argv is not None else None)
    if args._cmd == "test":
        from .test_cmd import run_test

        return run_test(args)
    with _connect(args) as proxy:
        if args._cmd == "show":
            return _cmd_show(proxy, args)
        if args._cmd == "set":
            return _cmd_set(proxy, args)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
