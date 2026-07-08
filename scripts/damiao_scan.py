#!/usr/bin/env python3
"""
Damiao MIT / position / speed ID discover over MCU USB (PDU tag DM0).

Expert CLI — daily use: python scripts/control_hub.py discover --protocol damiao

Examples:
  python scripts/damiao_scan.py --port COM9 --discover --bus 3
  python scripts/damiao_scan.py --port COM9 --probe-id 1 --bus 3 --reg-scan
  python scripts/damiao_scan.py --port COM9 --fw-sweep --bus 3 --start 0 --end 127
"""
from __future__ import annotations

import argparse
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from controls_pcb_host.plugins import damiao_expert as expert  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="Damiao CAN ID discover via MCU USB")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bus", type=int, default=3, help="Schematic bus 1-6 (Damiao default CH3)")
    ap.add_argument("--discover", action="store_true",
                    help="Find motor ID (MCU sweep first, then per-ID reg_scan fallback)")
    ap.add_argument("--host-only", action="store_true",
                    help="With --discover: skip MCU sweep, per-ID reg_scan only")
    ap.add_argument("--fw-sweep", action="store_true",
                    help="Sweep --start..--end in one MCU command (no per-ID USB round trip)")
    ap.add_argument("--probe-id", type=lambda x: int(x, 0), default=None, help="Probe one ID")
    ap.add_argument("--start", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--end", type=lambda x: int(x, 0), default=127)
    ap.add_argument("--deep", action="store_true", help="Priority IDs first; wider retry")
    ap.add_argument("--all-modes", action="store_true", help="After reg-scan, retry MIT+POS+VEL")
    ap.add_argument("--reg-scan", action="store_true",
                    help="Register read scan (default for discover and --probe-id)")
    ap.add_argument("--enable", action="store_true",
                    help="Send single 0xFC enable with --probe-id")
    ap.add_argument("--clear-fault", action="store_true",
                    help="With --enable: send 0xFB before 0xFC")
    ap.add_argument("--feedback", action="store_true",
                    help="MIT feedback only (motor must already be enabled)")
    ap.add_argument("--no-enable", action="store_true", help="Deprecated (ignored)")
    ap.add_argument("--mit-fallback", action="store_true",
                    help="Single MIT frame for live pos/temp (no enable)")
    ap.add_argument("--keep-enabled", action="store_true",
                    help="With --enable: MIT hold stream before exit (see --hold-ms)")
    ap.add_argument("--hold-ms", type=int, default=None, metavar="MS")
    ap.add_argument("--can-enable", action="store_true", help="Alias for --mit-fallback")
    ap.add_argument("--master-id", default="scan",
                    help="Master filter for --mit-fallback only (reg-scan uses ANY)")
    ap.add_argument("--listen-ms", type=int, default=24)
    ap.add_argument("--sweep-listen-ms", type=int, default=4)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--link-test", action="store_true")
    ap.add_argument("--ack-debug", action="store_true")
    ap.add_argument("--debug-usb", action="store_true")
    ap.add_argument("--verbose", action="store_true", help="Deprecated (ignored)")
    args = ap.parse_args()

    if args.can_enable:
        args.mit_fallback = True

    if args.list_ports:
        for p in list_ports.comports():
            print(p.device, p.description)
        return

    if not args.port:
        ap.error("--port required (or --list-ports)")

    if args.link_test:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            time.sleep(0.3)
            raise SystemExit(expert.run_link_test(ser, args))

    if args.ack_debug:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            time.sleep(0.3)
            raise SystemExit(expert.run_ack_debug(ser, args))

    if not args.discover and not args.fw_sweep and args.probe_id is None:
        ap.error("use --discover, --fw-sweep, or --probe-id")

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.3)
        if args.fw_sweep:
            raise SystemExit(expert.run_fw_sweep(ser, args))
        if args.discover:
            raise SystemExit(expert.run_discover(ser, args))
        raise SystemExit(expert.run_single_probe(ser, args))


if __name__ == "__main__":
    main()
