"""ZeroErr (eDriver) discover over the ZE0 bench PDU.

Node discover is a CANopen SDO-read of the Identity Object (0x1018), not a
MIT-frame sweep — see zeroerr.h's ``zeroerr_probe_node[_range]``. Bus must be
1 Mbps (EDS BaudRate_1000 only); FDCAN CH1-3 and MCP CH4-6 both work (see
canopen.c's MCP-aware TX path).

Configure a slot once a node is found::

    hub.debug.cfg_set_slot(
        slot=0, bus=1, protocol=PROTO_ZEROERR, motor_id=<node_id>, persist=False
    )

Never touches ``zeroerr_apply_cycle``'s boot FSM — bench discover only.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional

from deft_controls_sdk.debug.stream_pause import pause_plant_stream
from deft_controls_sdk.link.exchange import (
    SESSION_BEGIN,
    SESSION_END,
    ZE_PROBE_NODE,
    ZE_PROBE_SWEEP,
    ZEROERR_PRODUCT_CODE,
    ZEROERR_VENDOR_ID,
    build_ze_probe_command,
    can_bus_label,
    parse_ze_probe_pdu,
)

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

PROTO_ZEROERR = 4

_SESSION_TIMEOUT_S = 3.0
_DEFAULT_SDO_TIMEOUT_MS = 30


def _clamp_timeout_ms(ms: int, *, minimum: int = 5, maximum: int = 255) -> int:
    return max(minimum, min(maximum, int(ms)))


def _ze_session_begin(connection: "Connection", bus: int) -> Optional[dict]:
    frame = build_ze_probe_command(0, SESSION_BEGIN, connection.next_seq(), bus=bus)
    return connection.exchange_raw(
        frame, parse_ze_probe_pdu, timeout_s=_SESSION_TIMEOUT_S,
        predicate=lambda p: True,
    )


def _ze_session_end(connection: "Connection", bus: int) -> Optional[dict]:
    frame = build_ze_probe_command(0, SESSION_END, connection.next_seq(), bus=bus)
    return connection.exchange_raw(
        frame, parse_ze_probe_pdu, timeout_s=_SESSION_TIMEOUT_S,
        predicate=lambda p: True,
    )


def _format_hit(resp: dict, node_id: int) -> str:
    node = resp.get("discovered_id", 0) & 0xFF
    tag = "ZeroErr" if resp.get("vendor_match") else "unknown CANopen node"
    return (
        f"FOUND  probe=0x{node_id:02X}  node={node}  {tag}  "
        f"vendor=0x{resp.get('vendor', 0):08X}  product=0x{resp.get('product', 0):08X}  "
        f"revision=0x{resp.get('revision', 0):08X}"
    )


def discover_all(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
    start: int = 1,
    end: int = 16,
    sdo_timeout_ms: int = _DEFAULT_SDO_TIMEOUT_MS,
) -> List[int]:
    """Collect every ZeroErr node id in ``start..end`` (CANopen 1..127).

    Firmware ``ZE_PROBE_SWEEP`` stops at the first responder, so the host
    re-sweeps from ``hit+1`` until the range is exhausted — same contract as
    Damiao/CubeMars ID_SWEEP.
    """
    if end < start:
        start, end = end, start
    print(f"ZeroErr discover on {can_bus_label(bus)}  node ids {start}..{end}")
    if telemetry is not None:
        telemetry.set_connected(True, mode="discover")
    timeout_ms = _clamp_timeout_ms(sdo_timeout_ms)
    found: List[int] = []
    seen = set()
    with pause_plant_stream(connection):
        try:
            begin = _ze_session_begin(connection, bus)
            if begin is None:
                print("  WARN: ZE SESSION_BEGIN missed — probes may fail")

            cursor = start
            while cursor <= end:
                span = end - cursor + 1
                # Each SDO read can block up to timeout_ms on the MCU; a full
                # sweep call may walk the whole remaining range.
                sweep_timeout_s = (span * (timeout_ms + 5)) / 1000.0 + 3.0
                frame = build_ze_probe_command(
                    cursor, ZE_PROBE_SWEEP, connection.next_seq(),
                    bus=bus, sdo_timeout_ms=timeout_ms, end_id=end,
                )
                connection.reader.drain()
                resp = connection.exchange_raw(
                    frame, parse_ze_probe_pdu, timeout_s=sweep_timeout_s,
                    predicate=lambda p: True,
                )
                if resp is not None and resp.get("found"):
                    hit = resp.get("discovered_id", cursor) & 0xFF
                    print(_format_hit(resp, hit))
                    if hit not in seen:
                        seen.add(hit)
                        found.append(hit)
                    nxt = max(hit, cursor) + 1
                    cursor = nxt if nxt > cursor else cursor + 1
                    continue
                if resp is not None:
                    print(f"  ZE_PROBE_SWEEP no-hit  from=0x{cursor:02X}")
                break

            if not found:
                print("No ZeroErr node found in range.")
            else:
                ids_s = ", ".join(f"0x{i:02X}" for i in found)
                print(f"ZeroErr discover summary: {len(found)} node(s) — {ids_s}")
            return found
        finally:
            _ze_session_end(connection, bus)
            if telemetry is not None:
                telemetry.set_connected(
                    True, mode="plant_stream" if connection.is_streaming else "idle",
                )


def discover(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
    start: int = 1,
    end: int = 16,
    sdo_timeout_ms: int = _DEFAULT_SDO_TIMEOUT_MS,
) -> Optional[int]:
    """Sweep start..end; returns the first responding node id, or None."""
    hits = discover_all(
        connection, telemetry, bus=bus, start=start, end=end, sdo_timeout_ms=sdo_timeout_ms,
    )
    return hits[0] if hits else None


def probe_node(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    node_id: int,
    sdo_timeout_ms: int = _DEFAULT_SDO_TIMEOUT_MS,
) -> Optional[dict]:
    """Single-node identity probe (0x1018) — no session/sweep bookkeeping."""
    timeout_ms = _clamp_timeout_ms(sdo_timeout_ms)
    with pause_plant_stream(connection):
        if telemetry is not None:
            telemetry.set_connected(True, mode="discover")
        frame = build_ze_probe_command(
            node_id, ZE_PROBE_NODE, connection.next_seq(), bus=bus, sdo_timeout_ms=timeout_ms,
        )
        connection.reader.drain()
        resp = connection.exchange_raw(
            frame, parse_ze_probe_pdu,
            timeout_s=(timeout_ms + 5) / 1000.0 + 2.0,
            predicate=lambda p: True,
        )
        if telemetry is not None:
            telemetry.set_connected(
                True, mode="plant_stream" if connection.is_streaming else "idle",
            )
        if resp is None:
            print(f"MISS  node=0x{node_id:02X}")
        else:
            print(_format_hit(resp, node_id) if resp.get("found") else f"MISS  node=0x{node_id:02X}")
        return resp


__all__ = [
    "PROTO_ZEROERR",
    "ZEROERR_PRODUCT_CODE",
    "ZEROERR_VENDOR_ID",
    "discover",
    "discover_all",
    "probe_node",
]
