"""DebugLease — RS2 SESSION_BEGIN/END bracket around a DEBUG-mode bench op.

Ported from scripts/legacy/controls_hub_api/hub.py's DebugAPI.lease() (itself
built on PcbSession.rs2_session_begin/end, scripts/legacy/controls_pcb_host/
session.py:133-141). While held, firmware gates the 500 Hz plant apply
(plant_block=BENCH_SESSION) — see docs/architecture.md "Plant runtime gates".

This does not open a second serial port: it borrows the caller's Connection
under the single-writer-lease model (docs/architecture.md#host-api-modes).
"""
from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Generator, Optional

from deft_controls_sdk.link.exchange import (
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_scan_command,
    can_bus_label,
    parse_probe_pdu,
)

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

_SESSION_TIMEOUT_S = 2.0


def rs2_session_begin(connection: "Connection", bus: int = 1) -> Optional[dict]:
    connection.reader.drain()
    frame = build_rs2_scan_command(0, SESSION_BEGIN, connection.next_seq(), bus=bus)
    return connection.exchange_raw(
        frame, parse_probe_pdu, timeout_s=_SESSION_TIMEOUT_S, predicate=lambda p: p["probe_kind"] == SESSION_BEGIN
    )


def rs2_session_end(connection: "Connection", bus: int = 1) -> Optional[dict]:
    connection.reader.drain()
    frame = build_rs2_scan_command(0, SESSION_END, connection.next_seq(), bus=bus)
    return connection.exchange_raw(
        frame, parse_probe_pdu, timeout_s=_SESSION_TIMEOUT_S, predicate=lambda p: p["probe_kind"] == SESSION_END
    )


@contextmanager
def lease(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
) -> Generator[None, None, None]:
    """RS2 session_begin/end bracket. Publishes mode="debug_lease" to telemetry
    (if attached) so the dashboard shows *why* plant_block is non-zero instead
    of just going quiet, then restores streaming-vs-idle mode on exit."""
    if telemetry is not None:
        telemetry.set_connected(True, mode="debug_lease")
    rs2_session_begin(connection, bus)
    try:
        yield
    finally:
        rs2_session_end(connection, bus)
        if telemetry is not None:
            telemetry.set_connected(True, mode="plant_stream" if connection.is_streaming else "idle")


def describe_bus(bus: int) -> str:
    return can_bus_label(bus)
