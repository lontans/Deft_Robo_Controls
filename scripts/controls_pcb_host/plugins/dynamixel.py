"""Dynamixel bench PDU (scan/ping)."""
from __future__ import annotations

import time
from typing import Optional

from .. import commands as cmd
from ..protocol.diag_pdu import PLANT_DXL_PROBE_PING, PLANT_DXL_PROBE_SCAN
from ..session import PcbSession


def run_scan(
    session: PcbSession,
    target_id: int = 1,
    id_start: int = 1,
    id_end: int = 20,
    *,
    timeout_s: float = 3.0,
) -> None:
    with session.rx_pump():
        frame = cmd.build_dxl_probe_command(
            session.next_seq(),
            target_id,
            PLANT_DXL_PROBE_SCAN,
            id_start=id_start,
            id_end=id_end,
        )
        session.write_command(frame)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            fb = session.reader.pop()
            if fb is not None:
                print(f"Feedback received ({len(fb)} B) — check dynamixel_scan for PDU parse.")
                return
            time.sleep(0.01)
        print("No feedback within timeout.")


def run_ping(session: PcbSession, target_id: int = 1) -> None:
    with session.rx_pump():
        frame = cmd.build_dxl_probe_command(
            session.next_seq(),
            target_id,
            PLANT_DXL_PROBE_PING,
        )
        session.write_command(frame)
        time.sleep(0.5)
        fb = session.reader.pop()
        if fb is None:
            print(f"Ping ID {target_id}: no response.")
        else:
            print(f"Ping ID {target_id}: feedback received.")
