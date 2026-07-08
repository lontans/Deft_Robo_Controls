"""MCU USB session: serial link, command sequencing, MCU state, diag sessions."""
from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Generator, Optional

import serial

from . import commands as cmd
from .actuator_config import slot_config  # noqa: F401 — re-export for plugins
from .feedback import parse_actuator_feedback, parse_feedback_header
from .protocol import (
    PLANT_MCU_STATE_NORMAL,
    PLANT_MCU_STATE_RECOVERY,
    SESSION_BEGIN,
    SESSION_END,
)
from .transport import DEFAULT_BAUD, FrameReader, SerialRxPump, describe_open_port, open_serial


class PcbSession:
    def __init__(
        self,
        port: str,
        *,
        baud: int = DEFAULT_BAUD,
        mcu_state: int = PLANT_MCU_STATE_NORMAL,
    ) -> None:
        self.port = port
        self.baud = baud
        self.mcu_state = mcu_state
        self.seq = 1
        self._ser: Optional[serial.Serial] = None
        self.reader = FrameReader()

    @classmethod
    def open(cls, port: str, **kwargs: object) -> "PcbSession":
        sess = cls(port, **kwargs)  # type: ignore[arg-type]
        sess.connect()
        return sess

    def connect(self) -> None:
        describe_open_port(self.port)
        self._ser = open_serial(self.port, self.baud)
        time.sleep(0.3)
        self.reader.clear()

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def ser(self) -> serial.Serial:
        if self._ser is None:
            raise RuntimeError("session not connected")
        return self._ser

    @contextmanager
    def rx_pump(self) -> Generator[FrameReader, None, None]:
        with SerialRxPump(self.ser, self.reader):
            yield self.reader

    def next_seq(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return s

    def write_command(self, frame: bytes) -> None:
        self.reader.drain()
        self.ser.write(frame)
        self.ser.flush()

    def send_plant(
        self,
        slot_commands: Optional[dict] = None,
        *,
        mcu_state: Optional[int] = None,
    ) -> None:
        state = self.mcu_state if mcu_state is None else mcu_state
        frame = cmd.build_plant_command(self.next_seq(), slot_commands, mcu_state=state)
        self.write_command(frame)

    def set_mcu_state(self, state: int) -> None:
        self.mcu_state = state
        self.send_plant(mcu_state=state)

    def recover(self) -> None:
        """RECOVERY mount + return to NORMAL (matches plant_recovery_all intent)."""
        self.set_mcu_state(PLANT_MCU_STATE_RECOVERY)
        time.sleep(0.05)
        self.set_mcu_state(PLANT_MCU_STATE_NORMAL)

    def wait_feedback(self, timeout_s: float = 1.0) -> Optional[bytes]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.reader.pop()
            if frame is not None:
                return frame
            time.sleep(0.005)
        return None

    def poll_status(self, timeout_s: float = 0.5) -> Optional[dict]:
        self.send_plant()
        frame = self.wait_feedback(timeout_s)
        if frame is None:
            return None
        hdr = parse_feedback_header(frame)
        if hdr is None:
            return None
        slots = []
        for i in range(6):
            slots.append(parse_actuator_feedback(frame, i))
        hdr["actuator_slots"] = slots
        return hdr

    def rs2_session_begin(self, bus: int = 1) -> None:
        from .plugins import robstride as rs2

        rs2.send_diag(self, 0, SESSION_BEGIN, timeout_s=2.0, bus=bus)

    def rs2_session_end(self, bus: int = 1) -> None:
        from .plugins import robstride as rs2

        rs2.send_diag(self, 0, SESSION_END, timeout_s=2.0, bus=bus)

    def dm_session_begin(self, bus: int = 3) -> None:
        from .plugins import damiao as dm

        dm.send_probe(self, 0, SESSION_BEGIN, bus=bus, timeout_s=3.0)

    def dm_session_end(self, bus: int = 3) -> None:
        from .plugins import damiao as dm

        dm.send_probe(self, 0, SESSION_END, bus=bus, timeout_s=3.0)

    def wake_from_diag(self, bus: int = 3) -> None:
        """Release DIAG_ONLY gates before plant teleop (Damiao path)."""
        try:
            self.dm_session_end(bus)
        except Exception:
            pass
        self.recover()
        time.sleep(0.15)

    def link_test(self, timeout_s: float = 2.0) -> bool:
        with self.rx_pump():
            # Firmware may stream feedback before any command (bare-metal superloop).
            time.sleep(0.15)
            frame = self.wait_feedback(0.5)
            if frame is not None:
                hdr = parse_feedback_header(frame)
                if hdr is not None:
                    print(
                        f"OK (unsolicited)  tick={hdr['tick']}  mcu_state={hdr['mcu_state']}  "
                        f"ack_seq={hdr['last_cmd_seq']}"
                    )
                    return True

            self.send_plant()
            frame = self.wait_feedback(timeout_s)
            if frame is None:
                print(
                    f"No 562 B feedback within timeout "
                    f"(rx_bytes={self.reader.total_bytes}  frames={self.reader.total_frames})."
                )
                return False
            hdr = parse_feedback_header(frame)
            if hdr is None:
                print("Received bytes but feedback header invalid.")
                return False
            print(
                f"OK  tick={hdr['tick']}  mcu_state={hdr['mcu_state']}  "
                f"ack_seq={hdr['last_cmd_seq']}  pdu={hdr['pdu_tag']}"
            )
            return True

    def __enter__(self) -> "PcbSession":
        self.connect()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
