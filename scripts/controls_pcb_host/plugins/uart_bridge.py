"""UART4 Damiao debug bridge (PDU UB)."""
from __future__ import annotations

import threading
import time

from .. import commands as cmd
from ..feedback import parse_ub_feedback
from ..session import PcbSession
from ..transport import open_serial


def run_bridge(session: PcbSession, bridge_port: str, bridge_baud: int = 921600) -> None:
    bridge = open_serial(bridge_port, bridge_baud)
    stop = threading.Event()

    def mcu_to_bridge() -> None:
        with session.rx_pump():
            while not stop.is_set():
                frame = session.reader.pop()
                if frame is None:
                    time.sleep(0.002)
                    continue
                chunk = parse_ub_feedback(frame)
                if chunk:
                    bridge.write(chunk)
                    bridge.flush()

    def bridge_to_mcu() -> None:
        while not stop.is_set():
            chunk = bridge.read(max(1, bridge.in_waiting))
            if not chunk:
                time.sleep(0.002)
                continue
            session.write_command(cmd.build_ub_command(chunk, session.next_seq()))

    t1 = threading.Thread(target=mcu_to_bridge, name="mcu-rx", daemon=True)
    t2 = threading.Thread(target=bridge_to_mcu, name="bridge-rx", daemon=True)
    t1.start()
    t2.start()
    print(f"Bridge active: MCU {session.port} <-> {bridge_port} @ {bridge_baud}")
    print("Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop.set()
        t1.join(timeout=1.0)
        t2.join(timeout=1.0)
        bridge.close()
        print("Bridge stopped.")
