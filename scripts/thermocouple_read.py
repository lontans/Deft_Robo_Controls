#!/usr/bin/env python3
"""
Bench thermocouple (MAX31855) read over MCU USB.

With SPI3_ROLE_DEFAULT=THERMO, every 562 B feedback image carries a 't' PDU
with the latest cached reading — no TMP probe required.

Example:
  python scripts/thermocouple_read.py --port COM9
  python scripts/thermocouple_read.py --port COM9 --watch --hz 2
  python scripts/thermocouple_read.py --port COM9 --probe   # also send TMP tag
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from controls_pcb_host.commands import build_thermo_probe_command  # noqa: E402
from controls_pcb_host.feedback import parse_thermo_feedback  # noqa: E402
from controls_pcb_host.session import PcbSession  # noqa: E402
from controls_pcb_host.transport import auto_pick_port, list_serial_ports  # noqa: E402


def _print_reading(reading: dict) -> None:
    status = "OK" if reading["ok"] else f"FAULT(0x{reading['fault_bits']:X})"
    print(
        f"thermocouple={reading['thermocouple_c']:+.2f}C  "
        f"cold_junction={reading['cold_junction_c']:+.2f}C  "
        f"age={reading['age_ms']}ms  {status}"
    )


def read_once(session: PcbSession, *, probe: bool, timeout_s: float = 1.5) -> None:
    if probe:
        session.write_command(build_thermo_probe_command(session.next_seq()))
    frame = session.wait_feedback(timeout_s)
    if frame is None:
        print("no 562 B feedback within timeout — check COM port / flash / USB CDC")
        return
    reading = parse_thermo_feedback(frame)
    if reading is None:
        print(
            "feedback frame has no thermo 't' PDU — is SPI3_ROLE_DEFAULT=THERMO "
            "flashed, and is plant_feedback whitelist including 't'?"
        )
        return
    _print_reading(reading)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--port", default=None)
    ap.add_argument("--watch", action="store_true", help="Poll continuously instead of a single read")
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument(
        "--probe",
        action="store_true",
        help="Also send a TMP-tagged command (optional; readings stream without it)",
    )
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return 0

    port = args.port or auto_pick_port()
    with PcbSession.open(port) as session:
        with session.rx_pump():
            if not args.watch:
                read_once(session, probe=args.probe)
                return 0
            try:
                while True:
                    read_once(session, probe=args.probe)
                    time.sleep(1.0 / args.hz)
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
