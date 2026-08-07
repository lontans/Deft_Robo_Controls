#!/usr/bin/env python3
"""Demo 01 — board alive (scan + bandwidth status).

Talk track:
  Soft-DFU / USB CDC is up; host can see the plant stream without arming motors.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import add_port_args, require_one_owner, say_done, step  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    add_port_args(p)
    args = p.parse_args(argv)

    require_one_owner()
    step(1, "Scan for STM32 CDC / DFU", "pcb_lab board scan")
    from pcb_lab import board

    rc = board.cmd_scan(port=args.port, serial=args.serial)
    if rc != 0:
        return rc

    step(2, "Bandwidth link health", "fb_hz / ack_lag — plant USB duplex")
    rc = board.cmd_status(
        port=args.port,
        serial=args.serial,
        seconds=2.0,
        hz=200.0,
        listen_pdu=False,
    )
    say_done("01_board_alive")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
