"""Plant TX must match legacy: write + flush every frame.

Skipping flush (an earlier SDK experiment) let Windows CDC coalesce OUT
traffic so the board saw bursty command arrival — ACT LEDs blink then freeze
even though host_send_ms looked fine. Bench probes use the same path.
"""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.api_types import ActuatorDesire, IDLE
from deft_controls_sdk.link.exchange import PROBE_RESET, build_rs2_scan_command


class _FakeSerial:
    def __init__(self) -> None:
        self.write_calls = 0
        self.flush_calls = 0

    def write(self, frame: bytes) -> None:
        self.write_calls += 1

    def flush(self) -> None:
        self.flush_calls += 1


def _fake_connection() -> tuple[Connection, _FakeSerial]:
    conn = Connection("TESTPORT")
    fake = _FakeSerial()
    conn._ser = fake
    return conn, fake


def test_send_once_flushes_like_legacy() -> None:
    conn, fake = _fake_connection()
    conn.set_actuator(0, ActuatorDesire(kp=8.0), send=False)
    conn.send_once()
    assert fake.write_calls == 1
    assert fake.flush_calls == 1


def test_write_command_flushes() -> None:
    conn, fake = _fake_connection()
    conn.write_command(conn.build_command())
    assert fake.write_calls == 1
    assert fake.flush_calls == 1


def test_write_raw_drain_false_skips_flush() -> None:
    conn, fake = _fake_connection()
    frame = build_rs2_scan_command(0x70, PROBE_RESET, seq=1, bus=1)
    conn.write_raw(frame, drain=False)
    assert fake.write_calls == 1
    assert fake.flush_calls == 0


def test_streaming_loop_flushes_every_tick() -> None:
    conn, fake = _fake_connection()
    conn.set_actuator(3, IDLE, send=False)
    for _ in range(50):
        conn.send_once()
    assert fake.write_calls == 50
    assert fake.flush_calls == 50
