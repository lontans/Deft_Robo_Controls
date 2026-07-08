"""Unified CLI for Deft controls PCB host bring-up."""
from __future__ import annotations

import argparse
import subprocess
import sys
from typing import List, Optional

from .._bootstrap import ensure_scripts_path
from ..actuator_config import (
    apply_host_config,
    format_table,
    parse_protocol,
    slot_config,
)
from ..feedback import format_status_line
from ..plugins import damiao, dynamixel, led, robstride, uart_bridge
from ..session import PcbSession
from ..teleop import run_calibrate, run_plant_teleop_for_slot, run_servo_teleop
from ..transport import auto_pick_port, list_serial_ports


def _port(args: argparse.Namespace) -> str:
    if args.port:
        return args.port
    return auto_pick_port()


def cmd_ports(_args: argparse.Namespace) -> int:
    list_serial_ports()
    return 0


def cmd_link_test(args: argparse.Namespace) -> int:
    with PcbSession(_port(args)) as session:
        return 0 if session.link_test() else 1


def cmd_status(args: argparse.Namespace) -> int:
    with PcbSession(_port(args)) as session:
        with session.rx_pump():
            hdr = session.poll_status(timeout_s=1.0)
        if hdr is None:
            print("No feedback.")
            return 1
        print(format_status_line(hdr, hdr.get("actuator_slots")))
        return 0


def cmd_recover(args: argparse.Namespace) -> int:
    with PcbSession(_port(args)) as session:
        with session.rx_pump():
            session.wake_from_diag(args.bus)
        print("Recovery sent (DIAG session end + RECOVERY + NORMAL).")
        return 0


def cmd_discover(args: argparse.Namespace) -> int:
    proto = args.protocol.lower()
    start = args.start
    end = args.end
    if proto == "damiao" and start == 0x40 and end == 0x80:
        start, end = 1, 16
    with PcbSession(_port(args)) as session:
        if proto == "damiao":
            damiao.discover(
                session,
                bus=args.bus,
                start=start,
                end=end,
                listen_ms=args.listen_ms,
            )
        elif proto in ("robstride", "rs02"):
            robstride.discover_id(session, args.bus, start=start, end=end)
        else:
            print(f"discover not implemented for protocol {proto}", file=sys.stderr)
            return 2
    return 0


def cmd_probe(args: argparse.Namespace) -> int:
    with PcbSession(_port(args)) as session:
        if args.slot is not None:
            cfg = slot_config(args.slot)
            bus = cfg.bus
            motor_id = cfg.motor_id
            proto = cfg.protocol_name
        else:
            if args.bus is None or args.id is None:
                print("probe requires --slot N or --bus and --id", file=sys.stderr)
                return 2
            bus = args.bus
            motor_id = args.id
            proto = args.protocol or "robstride"

        if proto == "damiao":
            if args.enable and args.hold_ms > 0:
                ok = damiao.enable_and_hold(
                    session, bus, motor_id, hold_ms=args.hold_ms,
                    listen_ms=args.listen_ms,
                )
                return 0 if ok else 1
            damiao.probe(
                session,
                bus,
                motor_id,
                enable=args.enable,
                listen_ms=args.listen_ms,
            )
        else:
            robstride.probe_id(session, bus, motor_id)
    return 0


def cmd_teleop(args: argparse.Namespace) -> int:
    port = _port(args)
    if args.servo:
        run_servo_teleop(port)
        return 0
    slot = args.slot if args.slot is not None else 0
    with PcbSession(port) as session:
        with session.rx_pump():
            session.wake_from_diag(slot_config(slot).bus)
    run_plant_teleop_for_slot(port, slot, skip_home=args.skip_home)
    return 0


def cmd_calibrate(args: argparse.Namespace) -> int:
    slot = args.slot if args.slot is not None else 0
    run_calibrate(_port(args), slot)
    return 0


def cmd_config_show(_args: argparse.Namespace) -> int:
    print(format_table())
    return 0


def cmd_config_set(args: argparse.Namespace) -> int:
    slot = args.slot
    proto = parse_protocol(args.protocol) if args.protocol else None
    apply_host_config(
        slot,
        bus=args.bus,
        protocol=proto,
        motor_id=args.motor_id,
        master_id=args.master_id,
        enabled=not args.disable,
        persist=args.persist,
    )
    print(format_table())
    print(
        "\nHost mirror updated. Reflash or firmware config PDU required for MCU to match.",
    )
    return 0


def cmd_led(args: argparse.Namespace) -> int:
    with PcbSession(_port(args)) as session:
        led.run_test(
            session,
            mode=args.mode,
            brightness=args.brightness,
            count=args.count,
            hz=args.hz,
        )
    return 0


def cmd_uart_bridge(args: argparse.Namespace) -> int:
    with PcbSession(_port(args)) as session:
        uart_bridge.run_bridge(session, args.bridge_port, bridge_baud=args.bridge_baud)
    return 0


def cmd_expert(args: argparse.Namespace) -> int:
    ensure_scripts_path()
    script_map = {
        "rs02": "rs02_can_scan.py",
        "damiao": "damiao_scan.py",
        "teleop": "host_teleop_laptop_usb.py",
        "dynamixel": "dynamixel_scan.py",
        "dxl-teleop": "dynamixel_teleop.py",
    }
    script = script_map.get(args.tool)
    if script is None:
        print(f"unknown expert tool {args.tool!r}", file=sys.stderr)
        return 2
    import os

    scripts_dir = ensure_scripts_path()
    path = os.path.join(scripts_dir, script)
    cmd: List[str] = [sys.executable, path] + args.extra
    return subprocess.call(cmd)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="controls_pcb_host",
        description="Deft controls PCB — unified host bring-up over USB CDC (562 B images)",
    )
    ap.add_argument("--port", help="USB CDC port (e.g. COM5)")
    ap.add_argument(
        "--list-ports",
        action="store_true",
        help="List serial ports (STM32 USB CDC hint) and exit",
    )
    sub = ap.add_subparsers(dest="command", required=False)

    sub.add_parser("ports", help="List serial ports").set_defaults(func=cmd_ports)
    sub.add_parser("list-ports", help="Alias for ports").set_defaults(func=cmd_ports)

    p = sub.add_parser("link-test", help="Send plant frame and wait for feedback")
    p.set_defaults(func=cmd_link_test)

    p = sub.add_parser("status", help="One-shot plant status snapshot")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("recover", help="End diag session and RECOVERY mount")
    p.add_argument("--bus", type=int, default=3, help="Damiao bus for session end (default 3)")
    p.set_defaults(func=cmd_recover)

    p = sub.add_parser("discover", help="Scan for motor CAN ID")
    p.add_argument("--protocol", default="robstride", help="robstride | damiao")
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--start", type=lambda x: int(x, 0), default=0x40)
    p.add_argument("--end", type=lambda x: int(x, 0), default=0x80)
    p.add_argument("--listen-ms", type=int, default=40, help="Damiao listen window")
    p.set_defaults(func=cmd_discover)

    p = sub.add_parser("probe", help="Probe one motor ID")
    p.add_argument("--slot", type=int, default=None)
    p.add_argument("--bus", type=int, default=None)
    p.add_argument("--id", type=lambda x: int(x, 0), default=None, dest="id")
    p.add_argument("--protocol", default=None, help="Override slot protocol")
    p.add_argument("--enable", action="store_true", help="Damiao: clear-fault + enable")
    p.add_argument("--listen-ms", type=int, default=15)
    p.add_argument("--hold-ms", type=int, default=0, help="Damiao MIT hold after --enable")
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("teleop", help="Interactive teleop (delegates to legacy runner)")
    p.add_argument("--slot", type=int, default=None)
    p.add_argument("--servo", action="store_true", help="Dynamixel neck servos")
    p.add_argument("--skip-home", action="store_true")
    p.set_defaults(func=cmd_teleop)

    p = sub.add_parser("calibrate", help="RS02 encoder cal (slot 0 default)")
    p.add_argument("--slot", type=int, default=0)
    p.set_defaults(func=cmd_calibrate)

    cfg = sub.add_parser("config", help="Actuator slot configuration (host mirror)")
    cfg_sub = cfg.add_subparsers(dest="config_cmd", required=True)
    show_p = cfg_sub.add_parser("show")
    show_p.set_defaults(func=cmd_config_show)
    p = cfg_sub.add_parser("set", help="Update host mirror (firmware PDU pending)")
    p.add_argument("--slot", type=int, required=True)
    p.add_argument("--bus", type=int, default=None)
    p.add_argument("--protocol", default=None)
    p.add_argument("--motor-id", type=lambda x: int(x, 0), default=None)
    p.add_argument("--master-id", type=lambda x: int(x, 0), default=None)
    p.add_argument("--disable", action="store_true")
    p.add_argument("--persist", action="store_true", help="Request NVM (not on MCU yet)")
    p.set_defaults(func=cmd_config_set)

    p = sub.add_parser("led", help="SK9822 strip test")
    p.add_argument("--mode", type=int, default=0)
    p.add_argument("--brightness", type=int, default=8)
    p.add_argument("--count", type=int, default=0)
    p.add_argument("--hz", type=float, default=40.0)
    p.set_defaults(func=cmd_led)

    p = sub.add_parser("uart-bridge", help="UART4 Damiao debug bridge")
    p.add_argument("--bridge-port", required=True)
    p.add_argument("--bridge-baud", type=int, default=921600)
    p.set_defaults(func=cmd_uart_bridge)

    p = sub.add_parser("expert", help="Run legacy script with passthrough args")
    p.add_argument("tool", choices=["rs02", "damiao", "teleop", "dynamixel", "dxl-teleop"])
    p.add_argument("extra", nargs=argparse.REMAINDER, help="Args passed to legacy script")
    p.set_defaults(func=cmd_expert)

    return ap


def main(argv: Optional[List[str]] = None) -> int:
    ensure_scripts_path()
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.list_ports:
        list_serial_ports()
        return 0
    if args.command is None:
        ap.print_help()
        return 2
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
