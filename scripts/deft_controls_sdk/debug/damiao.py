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

from typing import TYPE_CHECKING, List, Optional, Sequence

from deft_controls_sdk.debug.stream_pause import pause_plant_stream
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
    is_mcp_bus,
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
    """Optional hint order only — ID_SWEEP runs first. Do not invent known_ids;
    a wrong head list just burns REG_SCAN airtime before the real ESC."""
    head = [mid & 0xFF for mid in known_ids if start <= (mid & 0xFF) <= end]
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


def _hit_esc_id(resp: dict, fallback: int) -> int:
    hit = int(resp.get("discovered_id", resp.get("param_value", 0))) & 0xFF
    if hit == 0:
        hit = int(resp.get("probe_id", fallback)) & 0xFF
    return hit


def discover_all(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
    start: int = 1,
    end: int = 16,
    listen_ms: int = 40,
    known_ids: Sequence[int] = (),
) -> List[int]:
    """Collect every Damiao ESC_ID in ``start..end``.

    Firmware ``ID_SWEEP`` stops at the first responder, so the host re-sweeps
    from ``hit+1`` until the range is exhausted (same multi-hit idea as
    RobStride discover). ``REG_SCAN`` is only a fallback when sweeps miss, or
    for leftover ``known_ids`` — avoid a full 1→N REG_SCAN flood after a hit.

    ``known_ids`` is an optional REG_SCAN order hint — leave empty unless you
    already know configured ESC IDs on this bus.
    """
    if end < start:
        start, end = end, start
    mcp = is_mcp_bus(bus)
    print(f"Damiao discover on {can_bus_label(bus)}  IDs {start}..{end}")
    if telemetry is not None:
        telemetry.set_connected(True, mode="discover")
    # Same listen budget as YAM arm; MCP FW does one TX burst then RX-only.
    sweep_listen = _clamp_listen_ms(listen_ms, minimum=40)
    found: List[int] = []
    seen = set()
    with pause_plant_stream(connection):
        try:
            begin = _dm_session_begin(connection, bus)
            if begin is None:
                print("  WARN: DM SESSION_BEGIN missed — probes may fail")

            # Successive ID_SWEEP: FW returns on first hit; continue past it.
            cursor = start
            while cursor <= end:
                span = end - cursor + 1
                sweep_timeout = _probe_timeout_s(DM_PROBE_ID_SWEEP, sweep_listen, span)
                if mcp:
                    sweep_timeout = max(
                        sweep_timeout, span * (sweep_listen / 1000.0) + 2.0
                    )
                resp = _send_probe(
                    connection,
                    cursor,
                    DM_PROBE_ID_SWEEP,
                    bus=bus,
                    listen_ms=sweep_listen,
                    end_id=end,
                    timeout_s=sweep_timeout,
                )
                if resp is not None and (
                    resp.get("found") or dm_fault_found(resp.get("err", 0))
                ):
                    hit = _hit_esc_id(resp, cursor)
                    print(_format_hit(resp, hit))
                    if hit not in seen:
                        seen.add(hit)
                        found.append(hit)
                    nxt = max(hit, cursor) + 1
                    cursor = nxt if nxt > cursor else cursor + 1
                    continue
                if resp is not None:
                    print(
                        f"  ID_SWEEP no-hit  from=0x{cursor:02X}  "
                        f"found={resp.get('found')} "
                        f"raw={resp.get('raw_frames_seen', resp.get('raw_frames', 0))} "
                        f"tx={resp.get('tx_frames_sent', '?')}"
                    )
                break

            # REG_SCAN: full range only when sweeps found nothing; otherwise
            # only leftover known_ids (avoid 1→N flood after a hit).
            per_listen = _clamp_listen_ms(listen_ms, minimum=20)
            per_timeout = _probe_timeout_s(DM_PROBE_REG_SCAN, per_listen)
            if not found:
                scan_ids = _discover_id_order(start, end, known_ids)
            elif known_ids:
                scan_ids = [
                    mid & 0xFF
                    for mid in known_ids
                    if start <= (mid & 0xFF) <= end and (mid & 0xFF) not in seen
                ]
            else:
                scan_ids = []
            for motor_id in scan_ids:
                resp = _send_probe(
                    connection,
                    motor_id,
                    DM_PROBE_REG_SCAN,
                    bus=bus,
                    listen_ms=per_listen,
                    timeout_s=per_timeout,
                )
                if resp is not None and (
                    resp.get("found") or dm_fault_found(resp.get("err", 0))
                ):
                    hit = _hit_esc_id(resp, motor_id)
                    print(_format_hit(resp, hit))
                    if hit not in seen:
                        seen.add(hit)
                        found.append(hit)
            if not found:
                print("No Damiao motor found in range.")
            else:
                ids_s = ", ".join(f"0x{i:02X}" for i in found)
                print(f"Damiao discover summary: {len(found)} motor(s) — {ids_s}")
            return found
        finally:
            _dm_session_end(connection, bus)
            if telemetry is not None:
                telemetry.set_connected(
                    True,
                    mode="plant_stream" if connection.is_streaming else "idle",
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
    """ID_SWEEP first, then REG_SCAN fallback. Returns first hit or None.

    Prefer :func:`discover_all` when more than one Damiao may share the bus.
    Leave ``known_ids`` empty unless you already know configured ESC IDs.
    """
    hits = discover_all(
        connection,
        telemetry,
        bus=bus,
        start=start,
        end=end,
        listen_ms=listen_ms,
        known_ids=known_ids,
    )
    return hits[0] if hits else None
