"""CubeMars AK-series discover over the CM0 bench PDU.

Same shape as :mod:`deft_controls_sdk.debug.damiao` (Damiao-shaped MIT wire),
but CubeMars has no PDF-documented register-read scheme (see docs/vendor.md),
so probing is MIT-frame-only: firmware sends a zero-effort MIT command per
candidate Drive ID and matches the FB echo it triggers (PDF: FB D[0] = Drive
ID). ``cubemars_probe_id_range`` on the MCU stops at the first hit in a
sweep — same "host re-sweeps from hit+1" contract as Damiao ID_SWEEP.

Never touches ``cubemars_apply_cycle`` (the hot MIT control path) — bench
discover only.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence

from deft_controls_sdk.debug.stream_pause import pause_plant_stream
from deft_controls_sdk.link.exchange import (
    CM_PROBE_ID_SWEEP,
    CM_PROBE_KIND_NAMES,
    CM_PROBE_MIT,
    SESSION_BEGIN,
    SESSION_END,
    build_cm_probe_command,
    can_bus_label,
    is_mcp_bus,
    parse_cm_probe_pdu,
)

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

_SESSION_TIMEOUT_S = 3.0


def _clamp_listen_ms(ms: int, *, minimum: int = 20, maximum: int = 255) -> int:
    return max(minimum, min(maximum, int(ms)))


def _probe_timeout_s(listen_ms: int, id_span: int = 1) -> float:
    return (id_span * (listen_ms + 5)) / 1000.0 + 3.0


def _cm_session_begin(connection: "Connection", bus: int) -> Optional[dict]:
    frame = build_cm_probe_command(0, SESSION_BEGIN, connection.next_seq(), bus=bus)
    return connection.exchange_raw(
        frame, parse_cm_probe_pdu, timeout_s=_SESSION_TIMEOUT_S,
        predicate=lambda p: p["probe_kind"] == SESSION_BEGIN,
    )


def _cm_session_end(connection: "Connection", bus: int) -> Optional[dict]:
    frame = build_cm_probe_command(0, SESSION_END, connection.next_seq(), bus=bus)
    return connection.exchange_raw(
        frame, parse_cm_probe_pdu, timeout_s=_SESSION_TIMEOUT_S,
        predicate=lambda p: p["probe_kind"] == SESSION_END,
    )


def _send_probe(
    connection: "Connection",
    motor_id: int,
    probe_kind: int,
    *,
    bus: int,
    listen_ms: int,
    end_id: int = 0,
    timeout_s: Optional[float] = None,
) -> Optional[dict]:
    listen_ms = _clamp_listen_ms(listen_ms)
    if timeout_s is None:
        span = max(1, (end_id - motor_id + 1) if probe_kind == CM_PROBE_ID_SWEEP else 1)
        timeout_s = _probe_timeout_s(listen_ms, span)
    frame = build_cm_probe_command(
        motor_id, probe_kind, connection.next_seq(),
        bus=bus, listen_ms=listen_ms, end_id=end_id,
    )

    def _matches(parsed: dict) -> bool:
        return parsed.get("probe_kind") == probe_kind

    connection.reader.drain()
    return connection.exchange_raw(frame, parse_cm_probe_pdu, timeout_s=timeout_s, predicate=_matches)


def _format_hit(resp: dict, motor_id: int) -> str:
    kind = CM_PROBE_KIND_NAMES.get(resp.get("probe_kind"), str(resp.get("probe_kind")))
    esc = resp.get("discovered_id", 0) & 0xFF
    fault = resp.get("fault", 0) & 0xFF
    return (
        f"FOUND  probe=0x{motor_id:02X}  drive_id=0x{esc:02X}  mode={kind}  "
        f"pos={resp.get('position', 0):+.4f}  fault=0x{fault:x}"
    )


def _hit_id(resp: dict, fallback: int) -> int:
    hit = int(resp.get("discovered_id", 0)) & 0xFF
    return hit if hit != 0 else int(resp.get("probe_id", fallback)) & 0xFF


def discover_all(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
    start: int = 1,
    end: int = 16,
    listen_ms: int = 40,
) -> List[int]:
    """Collect every CubeMars Drive ID in ``start..end``.

    Firmware ``ID_SWEEP`` stops at the first responder, so the host re-sweeps
    from ``hit+1`` until the range is exhausted (same pattern as Damiao
    discover). No REG_SCAN fallback — CubeMars has nothing to fall back to.
    """
    if end < start:
        start, end = end, start
    mcp = is_mcp_bus(bus)
    print(f"CubeMars discover on {can_bus_label(bus)}  IDs {start}..{end}")
    if telemetry is not None:
        telemetry.set_connected(True, mode="discover")
    sweep_listen = _clamp_listen_ms(listen_ms, minimum=40)
    found: List[int] = []
    seen = set()
    with pause_plant_stream(connection):
        try:
            begin = _cm_session_begin(connection, bus)
            if begin is None:
                print("  WARN: CM SESSION_BEGIN missed — probes may fail")

            cursor = start
            while cursor <= end:
                span = end - cursor + 1
                sweep_timeout = _probe_timeout_s(sweep_listen, span)
                if mcp:
                    sweep_timeout = max(sweep_timeout, span * (sweep_listen / 1000.0) + 2.0)
                resp = _send_probe(
                    connection, cursor, CM_PROBE_ID_SWEEP,
                    bus=bus, listen_ms=sweep_listen, end_id=end, timeout_s=sweep_timeout,
                )
                if resp is not None and resp.get("found"):
                    hit = _hit_id(resp, cursor)
                    print(_format_hit(resp, hit))
                    if hit not in seen:
                        seen.add(hit)
                        found.append(hit)
                    nxt = max(hit, cursor) + 1
                    cursor = nxt if nxt > cursor else cursor + 1
                    continue
                if resp is not None:
                    print(f"  ID_SWEEP no-hit  from=0x{cursor:02X}  found={resp.get('found')}")
                break

            if not found:
                print("No CubeMars motor found in range.")
            else:
                ids_s = ", ".join(f"0x{i:02X}" for i in found)
                print(f"CubeMars discover summary: {len(found)} motor(s) — {ids_s}")
            return found
        finally:
            _cm_session_end(connection, bus)
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
    listen_ms: int = 40,
) -> Optional[int]:
    """ID_SWEEP across the range; returns first hit or None.

    Prefer :func:`discover_all` when more than one CubeMars may share the bus.
    """
    hits = discover_all(connection, telemetry, bus=bus, start=start, end=end, listen_ms=listen_ms)
    return hits[0] if hits else None
