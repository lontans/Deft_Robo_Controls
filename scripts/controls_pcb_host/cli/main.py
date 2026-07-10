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
    slots_for_arm_local_joints,
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


def cmd_hello_world(args: argparse.Namespace) -> int:
    from pathlib import Path

    from control_hub.hello_world import print_yam_limits, run_hello_world

    if getattr(args, "limits_only", False):
        xml = getattr(args, "xml", None)
        print_yam_limits(Path(xml) if xml else None)
        return 0

    slot = getattr(args, "slot", None)
    joint = getattr(args, "joint", None)
    if slot is None and joint is None:
        slot = 0

    xml = getattr(args, "xml", None)
    return run_hello_world(
        _port(args),
        slot=slot,
        joint=joint,
        delta=getattr(args, "delta", 0.25) if getattr(args, "delta", None) is not None else 0.25,
        slew=getattr(args, "slew", 0.30) if getattr(args, "slew", None) is not None else 0.30,
        hold_s=getattr(args, "hold_s", 1.0) if getattr(args, "hold_s", None) is not None else 1.0,
        hz=args.hz if getattr(args, "hz", None) is not None else 40.0,
        kp=getattr(args, "kp", None),
        kd=getattr(args, "kd", None),
        return_home=not getattr(args, "no_return", False),
        skip_arm=getattr(args, "skip_arm", False),
        dry_run=getattr(args, "dry_run", False),
        no_limit_clamp=getattr(args, "no_limit_clamp", False),
        absolute_limits=getattr(args, "absolute_limits", False),
        xml_path=Path(xml) if xml else None,
        show_limits=getattr(args, "show_limits", False),
    )


def cmd_joint_status(args: argparse.Namespace) -> int:
    from pathlib import Path

    from control_hub.joint_cmd import run_joint_status

    xml = getattr(args, "xml", None)
    return run_joint_status(
        _port(args),
        slot=getattr(args, "slot", None),
        joint=getattr(args, "joint", None),
        hz=args.hz if getattr(args, "hz", None) is not None else 40.0,
        xml_path=Path(xml) if xml else None,
    )


def cmd_joint_goto(args: argparse.Namespace) -> int:
    from pathlib import Path

    xml = getattr(args, "xml", None)

    if getattr(args, "brace", False):
        from control_hub.joint_cmd import run_joint_goto_braced

        return run_joint_goto_braced(
            _port(args),
            slot=getattr(args, "slot", None),
            joint=getattr(args, "joint", None),
            delta=getattr(args, "delta", None),
            to=getattr(args, "to", None),
            absolute=getattr(args, "absolute", False),
            i_know_zeros=getattr(args, "i_know_zeros", False),
            slew=args.slew if getattr(args, "slew", None) is not None else 0.15,
            hold_s=args.hold_s if getattr(args, "hold_s", None) is not None else 1.0,
            hz=args.hz if getattr(args, "hz", None) is not None else 40.0,
            kp=getattr(args, "kp", None),
            kd=getattr(args, "kd", None),
            brace_kp=getattr(args, "brace_kp", None),
            brace_kd=getattr(args, "brace_kd", None),
            no_return=getattr(args, "no_return", False),
            dry_run=getattr(args, "dry_run", False),
            no_limit_clamp=getattr(args, "no_limit_clamp", False),
            xml_path=Path(xml) if xml else None,
            show_limits=getattr(args, "show_limits", False),
        )

    from control_hub.joint_cmd import run_joint_goto

    return run_joint_goto(
        _port(args),
        slot=getattr(args, "slot", None),
        joint=getattr(args, "joint", None),
        delta=getattr(args, "delta", None),
        to=getattr(args, "to", None),
        absolute=getattr(args, "absolute", False),
        i_know_zeros=getattr(args, "i_know_zeros", False),
        slew=args.slew if getattr(args, "slew", None) is not None else 0.30,
        hold_s=args.hold_s if getattr(args, "hold_s", None) is not None else 1.0,
        hz=args.hz if getattr(args, "hz", None) is not None else 40.0,
        kp=getattr(args, "kp", None),
        kd=getattr(args, "kd", None),
        no_return=getattr(args, "no_return", False),
        skip_arm=getattr(args, "skip_arm", False),
        dry_run=getattr(args, "dry_run", False),
        no_limit_clamp=getattr(args, "no_limit_clamp", False),
        xml_path=Path(xml) if xml else None,
        show_limits=getattr(args, "show_limits", False),
    )


def cmd_joint_home(args: argparse.Namespace) -> int:
    _ = args
    print(
        "FAIL: joint home is not implemented yet (P2, deferred — "
        "see docs/plan-yam-joint-commands.md §3.2)"
    )
    return 2


def cmd_plant_teleop(args: argparse.Namespace) -> int:
    port = _port(args)
    if args.damiao_teleop:
        slots = [2]
        damiao_only = True
    elif getattr(args, "plant_joints", None):
        try:
            joints = parse_slot_list(args.plant_joints)
            slots = slots_for_arm_local_joints(joints)
        except ValueError as exc:
            print(f"--joints: {exc}", file=sys.stderr)
            return 2
        if not slots:
            print("--joints: need at least one joint number", file=sys.stderr)
            return 2
        damiao_only = False
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
        description="Deft controls PCB - unified host bring-up over USB CDC (562 B images)",
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
        "--hello-world",
        action="store_true",
        help="Non-interactive single-slot plant jog (agent/script smoke test)",
    )
    ap.add_argument("--slot", type=int, default=None, help="Actuator slot (hello-world / teleop)")
    ap.add_argument(
        "--joint",
        type=int,
        default=None,
        help="YAM joint 1..14 (hello-world; slot = joint-1; 1-7 arm1, 8-14 arm2)",
    )
    ap.add_argument(
        "--delta",
        type=float,
        default=None,
        help="hello-world: relative move rad from current fb (default +0.25)",
    )
    ap.add_argument(
        "--slew",
        type=float,
        default=None,
        help="hello-world: slew rate rad/s (default 0.30)",
    )
    ap.add_argument(
        "--hold-s",
        type=float,
        default=None,
        dest="hold_s",
        help="hello-world: hold at target seconds (default 1.0)",
    )
    ap.add_argument(
        "--no-return",
        action="store_true",
        help="hello-world: stay at target (do not slew back)",
    )
    ap.add_argument(
        "--skip-arm",
        action="store_true",
        help="hello-world: skip Damiao 0xFB/0xFC enable probe",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="hello-world: print limit plan only (no motion)",
    )
    ap.add_argument(
        "--limits",
        action="store_true",
        dest="limits_only",
        help="Print YAM joint soft limits and exit",
    )
    ap.add_argument(
        "--show-limits",
        action="store_true",
        help="hello-world: print limit table before jog",
    )
    ap.add_argument(
        "--no-limit-clamp",
        action="store_true",
        help="hello-world: do not clamp delta to YAM soft limits",
    )
    ap.add_argument(
        "--absolute-limits",
        action="store_true",
        help="hello-world: treat fb as model frame when clamping (needs zero cal)",
    )
    ap.add_argument(
        "--xml",
        type=str,
        default=None,
        help="Override path to yam.xml for joint limits",
    )
    ap.add_argument(
        "--plant-slots",
        default="0",
        metavar="LIST",
        help="Actuator slot indices for --plant-teleop (comma-separated, e.g. 0,1 or 2)",
    )
    ap.add_argument(
        "--joints",
        default=None,
        dest="plant_joints",
        metavar="LIST",
        help="--plant-teleop: arm-local joint numbers 1..7 (comma-separated, e.g. 1,2,3,4) — "
        "expands to that joint on every arm/bus at once (e.g. 1,2 on a 2-arm bench = "
        "slots 0,1,7,8). Overrides --plant-slots.",
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
    p.add_argument("--slot", type=int, default=None, help="Actuator slot (default 0; uses MCU config bus/id)")
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
        "hello-world",
        parents=[port_parent],
        help="Non-interactive single-joint plant jog (limit-aware; enable -> delta -> return)",
    )
    p.add_argument("--slot", type=int, default=None, help="Actuator slot 0..13 (MCU config; 0-6 arm1, 7-13 arm2)")
    p.add_argument("--joint", type=int, default=None, help="YAM joint 1..14 (overrides --slot; 1-7 arm1, 8-14 arm2)")
    p.add_argument(
        "--delta",
        type=float,
        default=0.25,
        help="Relative move in rad from current fb (default +0.25; soft-clamped)",
    )
    p.add_argument("--slew", type=float, default=0.30, help="Slew rate rad/s (default 0.30)")
    p.add_argument("--hold-s", type=float, default=1.0, dest="hold_s", help="Hold at target seconds")
    p.add_argument("--hz", type=float, default=40.0, help="Host command rate")
    p.add_argument("--kp", type=float, default=None, help="MIT kp while moving")
    p.add_argument("--kd", type=float, default=None, help="MIT kd while moving")
    p.add_argument("--no-return", action="store_true", help="Stay at target (do not slew back)")
    p.add_argument("--skip-arm", action="store_true", help="Skip Damiao 0xFB/0xFC enable probe")
    p.add_argument("--dry-run", action="store_true", help="Print limit plan only (no motion)")
    p.add_argument(
        "--limits",
        action="store_true",
        dest="limits_only",
        help="Print YAM joint soft limits and exit",
    )
    p.add_argument("--show-limits", action="store_true", help="Print limit table before jog")
    p.add_argument(
        "--no-limit-clamp",
        action="store_true",
        help="Do not clamp delta to YAM soft limits",
    )
    p.add_argument(
        "--absolute-limits",
        action="store_true",
        help="Treat fb as model frame when clamping (needs zero cal)",
    )
    p.add_argument("--xml", type=str, default=None, help="Override path to yam.xml")
    p.set_defaults(func=cmd_hello_world)

    joint = sub.add_parser(
        "joint",
        help="Joint-level status / goto (YAM 1..14, dual arm; see docs/plan-yam-joint-commands.md)",
    )
    joint_sub = joint.add_subparsers(dest="joint_cmd", required=True)

    jp = joint_sub.add_parser(
        "status", parents=[port_parent], help="Read-only fb + soft-limit distance (no motion)"
    )
    jp.add_argument("--slot", type=int, default=None, help="Actuator slot 0..13 (MCU config; 0-6 arm1, 7-13 arm2)")
    jp.add_argument("--joint", type=int, default=None, help="YAM joint 1..14 (overrides --slot; 1-7 arm1, 8-14 arm2)")
    jp.add_argument("--hz", type=float, default=None, help="Poll rate (default 40)")
    jp.add_argument("--xml", type=str, default=None, help="Override path to yam.xml")
    jp.set_defaults(func=cmd_joint_status)

    jp = joint_sub.add_parser(
        "goto",
        parents=[port_parent],
        help="Relative (--delta) or absolute motor-frame (--to --absolute) jog; same "
        "codepath as hello-world",
    )
    jp.add_argument("--slot", type=int, default=None, help="Actuator slot 0..13 (MCU config; 0-6 arm1, 7-13 arm2)")
    jp.add_argument("--joint", type=int, default=None, help="YAM joint 1..14 (overrides --slot; 1-7 arm1, 8-14 arm2)")
    jp.add_argument("--delta", type=float, default=None, help="Relative move in rad from current fb")
    jp.add_argument("--to", type=float, default=None, help="Absolute motor-frame target rad (needs --absolute)")
    jp.add_argument(
        "--absolute",
        action="store_true",
        help="Treat --to as an absolute motor-frame target (needs --i-know-zeros)",
    )
    jp.add_argument(
        "--i-know-zeros",
        action="store_true",
        dest="i_know_zeros",
        help="Acknowledge motor zero != model zero until calibrated (required with --to)",
    )
    jp.add_argument("--slew", type=float, default=None, help="Slew rate rad/s (default 0.30)")
    jp.add_argument("--hold-s", type=float, default=None, dest="hold_s", help="Hold at target seconds")
    jp.add_argument("--hz", type=float, default=None, help="Host command rate (default 40)")
    jp.add_argument("--kp", type=float, default=None, help="MIT kp while moving")
    jp.add_argument("--kd", type=float, default=None, help="MIT kd while moving")
    jp.add_argument("--no-return", action="store_true", help="Stay at target (do not slew back)")
    jp.add_argument("--skip-arm", action="store_true", help="Skip Damiao 0xFB/0xFC enable probe")
    jp.add_argument("--dry-run", action="store_true", help="Print limit plan only (no motion)")
    jp.add_argument("--show-limits", action="store_true", help="Print limit table before jog")
    jp.add_argument("--no-limit-clamp", action="store_true", help="Do not clamp delta to YAM soft limits")
    jp.add_argument(
        "--brace",
        action="store_true",
        help="Hold every other enabled slot at its current position while moving the "
        "target (single-slot sends otherwise zero-fill/limp every other slot each tick)",
    )
    jp.add_argument("--brace-kp", type=float, default=None, dest="brace_kp", help="Override brace-slot kp (default: per-protocol default)")
    jp.add_argument("--brace-kd", type=float, default=None, dest="brace_kd", help="Override brace-slot kd (default: per-protocol default)")
    jp.add_argument("--xml", type=str, default=None, help="Override path to yam.xml")
    jp.set_defaults(func=cmd_joint_goto)

    jp = joint_sub.add_parser(
        "home", parents=[port_parent], help="Per-joint home (P2 — not implemented yet)"
    )
    jp.add_argument("--joints", type=str, default=None, help="e.g. 1-6 (unused; not implemented)")
    jp.set_defaults(func=cmd_joint_home)

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
        help="Require mms->rest/running in readback before zero/save (old behavior)",
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
    if getattr(args, "limits_only", False) and args.command is None and not args.hello_world:
        return cmd_hello_world(args)
    if args.hello_world:
        if args.slot is None and getattr(args, "joint", None) is None:
            args.slot = 0
        if args.delta is None:
            args.delta = 0.25
        if args.slew is None:
            args.slew = 0.30
        if args.hold_s is None:
            args.hold_s = 1.0
        return cmd_hello_world(args)
    if args.plant_teleop or args.damiao_teleop:
        return cmd_plant_teleop(args)
    if args.command is None:
        ap.print_help()
        return 2
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
