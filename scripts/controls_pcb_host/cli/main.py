"""Unified CLI for Deft controls PCB host bring-up."""
from __future__ import annotations

import argparse
import subprocess
import sys
from typing import List, Optional

from .._bootstrap import ensure_scripts_path
from ..actuator_config import (
    PROTOCOL_NAMES,
    apply_host_config,
    format_table,
    parse_protocol,
    slot_config,
)
from ..feedback import format_status_line
from ..plugins import damiao, dynamixel, led, robstride, uart_bridge
from ..protocol import PROBE_ENABLE_ONLY, PROBE_FULL
from ..session import PcbSession
from ..teleop import (
    parse_slot_list,
    run_calibrate,
    run_plant_extremity_teleop_for_slot,
    run_plant_teleop_for_slot,
    run_plant_teleop_for_slots,
    run_servo_teleop,
)
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
    from control_hub.link import heal_usb, release_bench_gates

    with PcbSession(_port(args)) as session:
        with session.rx_pump():
            release_bench_gates(session)
            hdr = session.poll_status(timeout_s=1.5)
            if hdr is not None:
                print(
                    f"Recovery OK  plant_block={hdr.get('plant_block_name', '?')}  "
                    f"pdu={hdr.get('pdu_tag', '?')}  ack_seq={hdr.get('last_cmd_seq')}"
                )
                return 0
            if heal_usb(session, rounds=16):
                hdr = session.poll_status(timeout_s=1.0)
                if hdr is not None:
                    print(
                        f"Recovery OK  plant_block={hdr.get('plant_block_name', '?')}  "
                        f"pdu={hdr.get('pdu_tag', '?')}"
                    )
                    return 0
            print("Recovery sent; USB feedback not confirmed — retry link-test.")
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
            kind = PROBE_FULL if getattr(args, "full", False) else PROBE_ENABLE_ONLY
            robstride.probe_id(session, bus, motor_id, kind=kind)
    return 0


def cmd_teleop(args: argparse.Namespace) -> int:
    port = _port(args)
    if args.servo:
        run_servo_teleop(port)
        return 0
    slot = args.slot if args.slot is not None else 0
    try:
        if args.extremity:
            run_plant_extremity_teleop_for_slot(
                port,
                slot,
                skip_home=args.skip_home,
                hz=args.hz,
                kd=args.kd,
                slew_rate=args.arrow_vel if args.arrow_vel is not None else None,
                kp=args.kp,
                home_kp=args.home_kp,
                home_slew=args.home_slew,
            )
        else:
            run_plant_teleop_for_slot(
                port,
                slot,
                skip_home=args.skip_home,
                hz=args.hz,
                kd=args.kd,
                arrow_vel=args.arrow_vel,
                ramp_up_s=args.ramp_up,
                ramp_down_s=args.ramp_down,
                kp=args.kp,
                home_kp=args.home_kp,
                home_slew=args.home_slew,
                debug_trace=args.debug_trace,
            )
    except Exception as exc:
        from control_hub.link import PlantRuntimeError

        if isinstance(exc, PlantRuntimeError):
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        raise
    return 0


def cmd_plant_teleop(args: argparse.Namespace) -> int:
    port = _port(args)
    if args.damiao_teleop:
        slots = [2]
        damiao_only = True
    else:
        slots = parse_slot_list(args.plant_slots)
        if not slots:
            print("--plant-slots: need at least one slot index", file=sys.stderr)
            return 2
        damiao_only = False
    try:
        run_plant_teleop_for_slots(
            port,
            slots,
            skip_home=args.skip_home,
            hz=args.hz,
            kd=args.kd,
            arrow_vel=args.arrow_vel,
            ramp_up_s=args.ramp_up,
            ramp_down_s=args.ramp_down,
            kp=args.kp,
            home_kp=args.home_kp,
            home_slew=args.home_slew,
            debug_trace=args.debug_trace,
            damiao_only=damiao_only,
        )
    except Exception as exc:
        from control_hub.link import PlantRuntimeError

        if isinstance(exc, PlantRuntimeError):
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        raise
    return 0


def cmd_calibrate(args: argparse.Namespace) -> int:
    if args.slot is not None:
        cfg = slot_config(args.slot)
        bus = cfg.bus
        motor_id = cfg.motor_id
    elif args.bus is not None and args.id is not None:
        bus = args.bus
        motor_id = args.id
    else:
        print("calibrate requires --slot N or --bus and --id", file=sys.stderr)
        return 2
    return run_calibrate(
        _port(args),
        bus,
        motor_id,
        cal_timeout=args.timeout,
        strict_cali=args.strict_cali,
    )


def cmd_config_show(args: argparse.Namespace) -> int:
    if getattr(args, "mirror_only", False):
        print(format_table())
        return 0
    with PcbSession(_port(args)) as session:
        with session.rx_pump():
            from ..plugins import plant_config as cfg_pdu
            from ..actuator_config import sync_host_table_from_mcu

            slots = cfg_pdu.fetch_table(session)
            sync_host_table_from_mcu(slots)
    print(format_table(source="MCU"))
    return 0


def cmd_config_set(args: argparse.Namespace) -> int:
    slot = args.slot
    proto = parse_protocol(args.protocol) if args.protocol else None
    port = _port(args)
    changes: list[str] = []
    if args.bus is not None:
        changes.append(f"bus=CH{args.bus}")
    if proto is not None:
        changes.append(f"protocol={PROTOCOL_NAMES[proto]}")
    if args.motor_id is not None:
        changes.append(f"motor_id=0x{args.motor_id & 0xFF:02X}")
    if args.master_id is not None:
        changes.append(f"master_id=0x{args.master_id & 0xFF:02X}")
    if args.disable:
        changes.append("enabled=off")
    with PcbSession(port) as session:
        with session.rx_pump():
            try:
                apply_host_config(
                    slot,
                    bus=args.bus,
                    protocol=proto,
                    motor_id=args.motor_id,
                    master_id=args.master_id,
                    enabled=not args.disable,
                    persist=args.persist,
                    session=session,
                )
            except RuntimeError as exc:
                if args.persist and "flash" in str(exc).lower():
                    print(format_table(source="MCU RAM"))
                    print(
                        f"\nWarning: {exc}\n"
                        "SET updated MCU RAM, but flash SAVE failed. "
                        "Power cycle will revert unless you reflash firmware and retry --persist.\n"
                        "Use:  python scripts/control_hub.py config show --port "
                        f"{port}"
                    )
                    return 1
                raise
    if changes:
        print(f"Updated slot {slot}: {', '.join(changes)}")
    print(format_table(source="MCU"))
    if args.persist:
        print("\nSaved to MCU flash NVM (survives power cycle).")
    else:
        print("\nApplied to MCU RAM (lost on power cycle unless you --persist).")
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
    port_help = "USB CDC port (e.g. COM5)"
    port_parent = argparse.ArgumentParser(add_help=False)
    port_parent.add_argument("--port", help=port_help)

    ap = argparse.ArgumentParser(
        prog="controls_pcb_host",
        description="Deft controls PCB — unified host bring-up over USB CDC (562 B images)",
    )
    ap.add_argument("--port", help=port_help)
    ap.add_argument(
        "--list-ports",
        action="store_true",
        help="List serial ports (STM32 USB CDC hint) and exit",
    )
    ap.add_argument(
        "--plant-teleop",
        action="store_true",
        help="500 Hz plant teleop (no RS2 bench PDU); use with --plant-slots",
    )
    ap.add_argument(
        "--damiao-teleop",
        action="store_true",
        help="Damiao plant teleop on slot 2 (alias for --plant-teleop --plant-slots 2)",
    )
    ap.add_argument(
        "--plant-slots",
        default="0",
        metavar="LIST",
        help="Actuator slot indices for --plant-teleop (comma-separated, e.g. 0,1 or 2)",
    )
    ap.add_argument(
        "--skip-home",
        action="store_true",
        help="Skip auto-homing at plant-teleop start",
    )
    ap.add_argument("--hz", type=float, default=None, help="Host command rate (plant teleop)")
    ap.add_argument("--arrow-vel", type=float, default=None, dest="arrow_vel")
    ap.add_argument("--ramp-up", type=float, default=None, dest="ramp_up")
    ap.add_argument("--ramp-down", type=float, default=None, dest="ramp_down")
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--home-kp", type=float, default=None, dest="home_kp")
    ap.add_argument("--home-slew", type=float, default=None, dest="home_slew")
    ap.add_argument("--debug-trace", type=str, default=None, dest="debug_trace")
    sub = ap.add_subparsers(dest="command", required=False)

    sub.add_parser("ports", help="List serial ports").set_defaults(func=cmd_ports)
    sub.add_parser("list-ports", help="Alias for ports").set_defaults(func=cmd_ports)

    p = sub.add_parser("link-test", parents=[port_parent], help="Send plant frame and wait for feedback")
    p.set_defaults(func=cmd_link_test)

    p = sub.add_parser("status", parents=[port_parent], help="One-shot plant status snapshot")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser(
        "recover",
        parents=[port_parent],
        help="Clear bench gates and RECOVERY mount (before plant teleop)",
    )
    p.add_argument(
        "--bus",
        type=int,
        default=2,
        help="Bus used for last bench session (default 2 = CH2 RS02)",
    )
    p.set_defaults(func=cmd_recover)

    p = sub.add_parser("discover", parents=[port_parent], help="Scan for motor CAN ID")
    p.add_argument("--protocol", default="robstride", help="robstride | damiao")
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--start", type=lambda x: int(x, 0), default=0x40)
    p.add_argument("--end", type=lambda x: int(x, 0), default=0x80)
    p.add_argument("--listen-ms", type=int, default=40, help="Damiao listen window")
    p.set_defaults(func=cmd_discover)

    p = sub.add_parser("probe", parents=[port_parent], help="Probe one motor ID")
    p.add_argument("--slot", type=int, default=None)
    p.add_argument("--bus", type=int, default=None)
    p.add_argument("--id", type=lambda x: int(x, 0), default=None, dest="id")
    p.add_argument("--protocol", default=None, help="Override slot protocol")
    p.add_argument("--enable", action="store_true", help="Damiao: clear-fault + enable")
    p.add_argument("--listen-ms", type=int, default=15)
    p.add_argument("--hold-ms", type=int, default=0, help="Damiao MIT hold after --enable")
    p.add_argument(
        "--full",
        action="store_true",
        help="RS02: PROBE_FULL init (default: reset + enable-only, same as discover)",
    )
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("teleop", parents=[port_parent], help="FDCAN plant teleop (562 B desires, no RS2 PDU)")
    p.add_argument("--slot", type=int, default=None, help="Actuator slot (default 0; slot 2 = Damiao CH3)")
    p.add_argument("--servo", action="store_true", help="Dynamixel neck servos")
    p.add_argument(
        "--extremity",
        action="store_true",
        help="RS02: press Up/Down once to go to ±position limit at --arrow-vel rad/s",
    )
    p.add_argument(
        "--skip-home",
        action="store_true",
        help="Skip auto-homing to 0 (use after calibrate or when shaft is already at zero)",
    )
    p.add_argument("--hz", type=float, default=None, help="Host command rate (default 40)")
    p.add_argument("--arrow-vel", type=float, default=None, dest="arrow_vel", help="Arrow hold speed rad/s (default 5)")
    p.add_argument("--ramp-up", type=float, default=None, dest="ramp_up", help="Velocity ramp-up time constant s (default 0.4)")
    p.add_argument(
        "--ramp-down",
        type=float,
        default=None,
        dest="ramp_down",
        help="Velocity coast-down time constant s (default 1.2)",
    )
    p.add_argument("--kd", type=float, default=None, help="D gain while moving (default 0.5)")
    p.add_argument("--kp", type=float, default=None, help="Override slot stiffness while moving")
    p.add_argument("--home-kp", type=float, default=None, dest="home_kp", help="Homing P gain (default 6)")
    p.add_argument(
        "--home-slew",
        type=float,
        default=None,
        dest="home_slew",
        help="Homing slew rad/s (default 0.18)",
    )
    p.add_argument(
        "--debug-trace",
        type=str,
        default=None,
        dest="debug_trace",
        help="Write per-frame CSV (t, dir, rate, cmd, fb, lead, d_fb, block, tx, ack)",
    )
    p.set_defaults(func=cmd_teleop)

    p = sub.add_parser(
        "calibrate",
        parents=[port_parent],
        help="RS02 encoder cal (comm 0x05/0x06/0x16 via MCU probe path)",
    )
    p.add_argument("--slot", type=int, default=None, help="Actuator slot (e.g. 1 = CH2 0x70)")
    p.add_argument("--bus", type=int, default=None)
    p.add_argument("--id", type=lambda x: int(x, 0), default=None, dest="id")
    p.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Cali listen seconds on MCU (default 28; datasheet: one comm 0x05 window)",
    )
    p.add_argument(
        "--strict-cali",
        action="store_true",
        help="Require mms→rest/running in readback before zero/save (old behavior)",
    )
    p.set_defaults(func=cmd_calibrate)

    cfg = sub.add_parser("config", help="Actuator slot table on MCU (CFG PDU + flash NVM)")
    cfg_sub = cfg.add_subparsers(dest="config_cmd", required=True)
    show_p = cfg_sub.add_parser(
        "show",
        parents=[port_parent],
        help="Read actuator table from MCU (CFG GET)",
    )
    show_p.add_argument(
        "--mirror-only",
        action="store_true",
        help="Print host mirror only (no USB)",
    )
    show_p.set_defaults(func=cmd_config_show)
    p = cfg_sub.add_parser(
        "set",
        parents=[port_parent],
        help="Update actuator slot on MCU (CFG SET); use --bus to move a slot to another channel",
    )
    p.add_argument("--slot", type=int, required=True)
    p.add_argument(
        "--bus",
        "--channel",
        type=int,
        default=None,
        dest="bus",
        metavar="CH",
        help="Schematic CAN bus CH1–CH6 (same channel for daisy-chain / mixed-protocol slots)",
    )
    p.add_argument(
        "--protocol",
        default=None,
        metavar="NAME",
        help="Slot protocol: robstride | damiao | cubemars | none",
    )
    p.add_argument("--motor-id", type=lambda x: int(x, 0), default=None)
    p.add_argument("--master-id", type=lambda x: int(x, 0), default=None)
    p.add_argument("--disable", action="store_true")
    p.add_argument("--persist", action="store_true", help="Write table to MCU flash NVM")
    p.set_defaults(func=cmd_config_set)

    p = sub.add_parser("led", parents=[port_parent], help="SK9822 strip test")
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


def _hoist_port_flag(argv: List[str]) -> List[str]:
    """Allow `control_hub.py --port COM5 discover` as well as `discover --port COM5`."""
    out = list(argv)
    for i, arg in enumerate(out):
        if arg in ("--port", "-p") and i + 1 < len(out):
            port = out[i + 1]
            del out[i : i + 2]
            out.extend(["--port", port])
            break
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ensure_scripts_path()
    ap = build_parser()
    raw = sys.argv[1:] if argv is None else argv
    args = ap.parse_args(_hoist_port_flag(raw))
    if args.list_ports:
        list_serial_ports()
        return 0
    if args.plant_teleop or args.damiao_teleop:
        return cmd_plant_teleop(args)
    if args.command is None:
        ap.print_help()
        return 2
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
