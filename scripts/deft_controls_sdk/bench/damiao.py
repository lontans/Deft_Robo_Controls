"""Damiao DM0 discover over the DM0 bench PDU.

Ported from scripts/legacy/controls_pcb_host/plugins/damiao.py (discover,
send_probe, _discover_id_order, probe_timeout_s). Preserves the known-IDs-
first scan order fix from docs/lessons.md ("Discover scan-order flood" —
probing ID 1 upward before a motor at 0x06 floods the bus with ~60 frames
per wrong ID and the drive stops replying). The SDK has no actuator config
table (yet — see docs/architecture.md Deferred), so callers pass
`known_ids` explicitly instead of this module reading a host-side slot table.

DM0 uses its own SESSION_BEGIN/END bracket (DM-tagged, not RS2-tagged) — kept
local to this module since nothing else needs a bare "DM lease" today.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Sequence

from deft_controls_sdk.link.exchange import (
    DM_MASTER_ANY,
    DM_PROBE_ID_SWEEP,
    DM_PROBE_KIND_NAMES,
    DM_PROBE_REG_SCAN,
    DM_REG_ESC_ID,
    SESSION_BEGIN,
    SESSION_END,
    build_dm_probe_command,
    can_bus_label,
    dm_fault_found,
    parse_dm_from_actuator,
    parse_dm_probe_pdu,
    probe_kind_matches,
)

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

_SESSION_TIMEOUT_S = 3.0


def _clamp_listen_ms(ms: int, *, minimum: int = 20, maximum: int = 255) -> int:
    return max(minimum, min(maximum, int(ms)))


def _probe_timeout_s(probe_kind: int, listen_ms: int, id_span: int = 1) -> float:
    if probe_kind == DM_PROBE_ID_SWEEP:
        return (id_span * (listen_ms + 5)) / 1000.0 + 3.0
    if probe_kind == DM_PROBE_REG_SCAN:
        return max(3.0, (listen_ms * 3 + 140) / 1000.0 + 1.5)
    return max(1.0, listen_ms / 1000.0 + 0.75)


def _dm_session_begin(connection: "Connection", bus: int) -> Optional[dict]:
    frame = build_dm_probe_command(0, SESSION_BEGIN, connection.next_seq(), bus=bus, master_id=DM_MASTER_ANY)
    return connection.exchange_raw(
        frame, parse_dm_probe_pdu, timeout_s=_SESSION_TIMEOUT_S, predicate=lambda p: p["probe_kind"] == SESSION_BEGIN
    )


def _dm_session_end(connection: "Connection", bus: int) -> Optional[dict]:
    frame = build_dm_probe_command(0, SESSION_END, connection.next_seq(), bus=bus, master_id=DM_MASTER_ANY)
    return connection.exchange_raw(
        frame, parse_dm_probe_pdu, timeout_s=_SESSION_TIMEOUT_S, predicate=lambda p: p["probe_kind"] == SESSION_END
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
    slot: int = 2,
) -> Optional[dict]:
    listen_ms = _clamp_listen_ms(listen_ms)
    if timeout_s is None:
        span = max(1, (end_id - motor_id + 1) if probe_kind == DM_PROBE_ID_SWEEP else 1)
        timeout_s = _probe_timeout_s(probe_kind, listen_ms, span)
    frame = build_dm_probe_command(
        motor_id,
        probe_kind,
        connection.next_seq(),
        bus=bus,
        master_id=DM_MASTER_ANY,
        listen_ms=listen_ms,
        param_rid=DM_REG_ESC_ID,
        end_id=end_id,
    )

    def _parse_either(raw: bytes) -> Optional[dict]:
        parsed = parse_dm_probe_pdu(raw)
        if parsed is not None:
            return parsed
        return parse_dm_from_actuator(raw, motor_id, slot=slot)

    def _matches(parsed: dict) -> bool:
        if probe_kind == DM_PROBE_ID_SWEEP:
            return parsed.get("probe_kind") == DM_PROBE_ID_SWEEP
        if parsed.get("probe_id") == (motor_id & 0xFF):
            return probe_kind_matches(parsed.get("probe_kind"), probe_kind)
        return False

    connection.reader.drain()
    return connection.exchange_raw(frame, _parse_either, timeout_s=timeout_s, predicate=_matches)


def _discover_id_order(start: int, end: int, known_ids: Sequence[int]) -> list[int]:
    """Configured Damiao IDs on this bus first — probing wrong IDs first floods
    the bus (docs/known-issues.md). Caller passes known_ids (e.g. from its own
    actuator table); this module has no config table of its own."""
    head = [mid & 0xFF for mid in known_ids if start <= mid <= end]
    full = list(range(start, end + 1))
    seen = set(head)
    return head + [mid for mid in full if mid not in seen]


def _format_hit(resp: dict, motor_id: int) -> str:
    kind = DM_PROBE_KIND_NAMES.get(resp.get("probe_kind"), str(resp.get("probe_kind")))
    esc = resp.get("discovered_id", resp.get("param_value", 0)) & 0xFF
    master = resp.get("master_id", 0) & 0xFF
    err = resp.get("err", 0) & 0xFF
    return (
        f"FOUND  probe=0x{motor_id:02X}  esc_id=0x{esc:02X}  master_rx=0x{master:02X}  "
        f"mode={kind}  pos={resp.get('position', 0):+.4f}  err=0x{err:x}"
    )


def discover(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
    start: int = 1,
    end: int = 16,
    listen_ms: int = 40,
    known_ids: Sequence[int] = (),
) -> Optional[int]:
    """ID_SWEEP first, then per-ID REG_SCAN fallback (known_ids first). Mirrors
    legacy damiao.discover() exactly, minus the actuator-config-table lookup —
    pass known_ids explicitly (e.g. [0x01..0x07] for one YAM arm's daisy chain)."""
    print(f"Damiao discover on {can_bus_label(bus)}  IDs {start}..{end}")
    if telemetry is not None:
        telemetry.set_connected(True, mode="discover")
    span = max(1, end - start + 1)
    sweep_listen = _clamp_listen_ms(listen_ms, minimum=40)
    try:
        _dm_session_begin(connection, bus)
        sweep_timeout = _probe_timeout_s(DM_PROBE_ID_SWEEP, sweep_listen, span)
        resp = _send_probe(
            connection, start, DM_PROBE_ID_SWEEP, bus=bus, listen_ms=sweep_listen, end_id=end, timeout_s=sweep_timeout
        )
        if resp is not None and (resp.get("found") or dm_fault_found(resp.get("err", 0))):
            hit = int(resp.get("discovered_id", resp.get("param_value", 0))) & 0xFF
            if hit == 0:
                hit = int(resp.get("probe_id", start)) & 0xFF
            print(_format_hit(resp, hit))
            return hit

        per_timeout = _probe_timeout_s(DM_PROBE_REG_SCAN, listen_ms)
        for motor_id in _discover_id_order(start, end, known_ids):
            resp = _send_probe(connection, motor_id, DM_PROBE_REG_SCAN, bus=bus, listen_ms=listen_ms, timeout_s=per_timeout)
            if resp is not None and (resp.get("found") or dm_fault_found(resp.get("err", 0))):
                print(_format_hit(resp, motor_id))
                return motor_id
        print("No Damiao motor found in range.")
        return None
    finally:
        _dm_session_end(connection, bus)
        if telemetry is not None:
            telemetry.set_connected(True, mode="plant_stream" if connection.is_streaming else "idle")
