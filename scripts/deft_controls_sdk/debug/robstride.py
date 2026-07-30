"""RobStride RS2 discover / probe / calibrate over DEBUG frames."""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence

from deft_controls_sdk.link.exchange import (
    PROBE_ENABLE_ONLY,
    PROBE_PROMISC,
    PROBE_RESET,
    build_rs2_probe_command,
    can_bus_label,
    is_mcp_bus,
    parse_probe_pdu,
)

from .lease import lease
from .stream_pause import pause_plant_stream

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache


def _send_diag(
    connection: "Connection",
    motor_id: int,
    probe_kind: int,
    timeout_s: float,
    *,
    kp: float = 0.0,
    kd: float = 0.0,
    bus: int = 1,
) -> Optional[dict]:
    connection.reader.drain()
    frame = build_rs2_probe_command(
        motor_id, probe_kind, connection.next_seq(), kp=kp, kd=kd, bus=bus
    )
    return connection.exchange_raw(
        frame,
        parse_probe_pdu,
        timeout_s=timeout_s,
        predicate=lambda p: p["probe_id"] == (motor_id & 0xFF)
        and p["probe_kind"] == probe_kind,
    )


def _send_diag_collect(
    connection: "Connection",
    motor_id: int,
    probe_kind: int,
    timeout_s: float,
    *,
    bus: int = 1,
) -> List[dict]:
    """Collect progress + final DBGF hits (multi-bus discover)."""
    connection.reader.drain()
    frame = build_rs2_probe_command(
        motor_id, probe_kind, connection.next_seq(), bus=bus
    )
    return connection.exchange_raw_collect(
        frame,
        parse_probe_pdu,
        timeout_s=timeout_s,
        predicate=lambda p: p["probe_id"] == (motor_id & 0xFF)
        and p["probe_kind"] == probe_kind,
    )


def format_probe_line(resp: dict) -> str:
    found = int(resp.get("found", 0))
    bus = resp.get("bus")
    bus_s = f"  bus={bus}" if bus is not None else ""
    return (
        f"probe_id=0x{resp['probe_id']:02X}  kind={resp.get('probe_kind')}  "
        f"found={found}  comm={resp.get('comm_mode')}  pos={resp.get('position', 0):+.4f}  "
        f"raw={resp.get('raw_frames', 0)}{bus_s}"
    )


def _replied_motor_id(probed_id: int, resp: dict) -> int:
    """Prefer FW discovered_id when present; else the probed address."""
    disc = int(resp.get("discovered_id") or 0) & 0xFF
    if disc != 0:
        return disc
    return probed_id & 0xFF


def _mcp_discover_timeout(base_s: float) -> float:
    """MCP probes block USB until listen finishes (~420 ms enable + TX flush)."""
    return max(base_s * 3.0, base_s + 0.6, 2.0)


def _normalize_id_range(start: int, end: int) -> tuple[int, int]:
    if end < start:
        start, end = end, start
    start = max(0, int(start) & 0xFF)
    end = min(0x7F, int(end) & 0xFF)
    return start, end


def discover_all_by_bus(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    buses: Sequence[int],
    start: int = 0x40,
    end: int = 0x80,
) -> Dict[int, List[int]]:
    """Multi-bus overlapping discover — one SESSION_BEGIN mask, RR listen.

    Returns ``{host_bus: [motor_ids…]}`` in discovery order per bus.
    Requires FW with ``robstride_probe_id_buses`` + bus echo on DBGF.
    """
    bus_list = sorted({int(b) for b in buses if 1 <= int(b) <= 6})
    if not bus_list:
        raise ValueError("buses must include at least one of 1..6")
    start, end = _normalize_id_range(start, end)

    mcp_any = any(is_mcp_bus(b) for b in bus_list)
    enable_s = _mcp_discover_timeout(0.55) if mcp_any else 0.55
    promisc_s = _mcp_discover_timeout(0.40) if mcp_any else 0.40
    # Multi-bus: one listen window covers all rails — keep MCP-sized budget once.
    labels = ", ".join(can_bus_label(b) for b in bus_list)
    print(
        f"RS2 multi-bus discover on [{labels}]  "
        f"IDs 0x{start:02X}..0x{end:02X}"
        + (
            f"  (timeouts enable={enable_s:.1f}s promisc={promisc_s:.1f}s)"
            if mcp_any
            else ""
        )
    )

    by_bus: Dict[int, List[int]] = {b: [] for b in bus_list}
    seen_pair = set()
    primary = bus_list[0]

    with pause_plant_stream(connection):
        with lease(connection, telemetry, bus=primary, buses=bus_list):
            if telemetry is not None:
                telemetry.set_connected(True, mode="discover")
            for motor_id in range(start, end + 1):
                collected: List[dict] = []
                for kind, label, timeout_s in (
                    (PROBE_ENABLE_ONLY, "enable", enable_s),
                    (PROBE_PROMISC, "promisc", promisc_s),
                ):
                    batch = _send_diag_collect(
                        connection, motor_id, kind, timeout_s, bus=primary
                    )
                    for resp in batch:
                        if resp.get("found"):
                            collected.append({**resp, "_via": label})
                        elif int(resp.get("raw_frames", 0) or 0) > 0:
                            print(
                                f"  traffic  id=0x{motor_id:02X}  {label}  "
                                f"raw={resp['raw_frames']}"
                            )
                for resp in collected:
                    disc = _replied_motor_id(motor_id, resp)
                    bus_hit = int(resp.get("bus") or primary)
                    if bus_hit not in by_bus:
                        # Attribute to primary if FW omitted bus (old image).
                        bus_hit = primary
                    key = (bus_hit, disc)
                    if key in seen_pair:
                        continue
                    seen_pair.add(key)
                    by_bus.setdefault(bus_hit, []).append(disc)
                    print(
                        f"FOUND  bus={bus_hit}  id=0x{disc:02X}  "
                        f"via {resp.get('_via', '?')}  {format_probe_line(resp)}"
                        + (
                            f"  (probed 0x{motor_id:02X})"
                            if disc != (motor_id & 0xFF)
                            else ""
                        )
                    )
                time.sleep(0.05)

    total = sum(len(v) for v in by_bus.values())
    if total == 0:
        print("No RS2 motor found in range (multi-bus).")
    else:
        parts = [
            f"{can_bus_label(b)}:[{', '.join(f'0x{i:02X}' for i in ids)}]"
            for b, ids in by_bus.items()
            if ids
        ]
        print(f"RS2 multi-bus summary: {total} hit(s) — " + "; ".join(parts))
    return by_bus


def discover_all(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int = 1,
    buses: Optional[Sequence[int]] = None,
    start: int = 0x40,
    end: int = 0x80,
) -> List[int]:
    """Sweep start..end; return every unique responding motor_id.

    Pass ``buses=[…]`` (len>1) for overlapping multi-bus discover; returns
    unique ids across those buses (see :func:`discover_all_by_bus` for per-bus).
    """
    if buses is not None and len(list(buses)) > 1:
        by_bus = discover_all_by_bus(
            connection, telemetry, buses=buses, start=start, end=end
        )
        out: List[int] = []
        seen = set()
        for b in sorted(by_bus.keys()):
            for mid in by_bus[b]:
                if mid not in seen:
                    seen.add(mid)
                    out.append(mid)
        return out

    bus_i = int(buses[0]) if buses else int(bus)
    start, end = _normalize_id_range(start, end)

    mcp = is_mcp_bus(bus_i)
    enable_s = _mcp_discover_timeout(0.55) if mcp else 0.55
    promisc_s = _mcp_discover_timeout(0.40) if mcp else 0.40
    print(
        f"RS2 discover on {can_bus_label(bus_i)}  IDs 0x{start:02X}..0x{end:02X}"
        + (
            f"  (MCP timeouts enable={enable_s:.1f}s promisc={promisc_s:.1f}s)"
            if mcp
            else ""
        )
    )
    found_ids: List[int] = []
    seen = set()
    with pause_plant_stream(connection):
        with lease(connection, telemetry, bus=bus_i):
            if telemetry is not None:
                telemetry.set_connected(True, mode="discover")
            for motor_id in range(start, end + 1):
                hit_resp: Optional[dict] = None
                hit_label = ""
                for kind, label, timeout_s in (
                    (PROBE_ENABLE_ONLY, "enable", enable_s),
                    (PROBE_PROMISC, "promisc", promisc_s),
                ):
                    resp = _send_diag(
                        connection, motor_id, kind, timeout_s, bus=bus_i
                    )
                    if resp is not None and resp.get("found"):
                        hit_resp = resp
                        hit_label = label
                        break
                    if resp is not None and resp.get("raw_frames", 0) > 0:
                        print(
                            f"  traffic  id=0x{motor_id:02X}  {label}  "
                            f"raw={resp['raw_frames']}"
                        )
                if hit_resp is None:
                    continue
                disc = _replied_motor_id(motor_id, hit_resp)
                print(
                    f"FOUND  id=0x{disc:02X}  via {hit_label}  "
                    f"{format_probe_line(hit_resp)}"
                    + (
                        f"  (probed 0x{motor_id:02X})"
                        if disc != (motor_id & 0xFF)
                        else ""
                    )
                )
                if disc not in seen:
                    seen.add(disc)
                    found_ids.append(disc)
                time.sleep(0.05)
    if not found_ids:
        print("No RS2 motor found in range.")
    else:
        ids_s = ", ".join(f"0x{i:02X}" for i in found_ids)
        print(f"RS2 discover summary: {len(found_ids)} motor(s) — {ids_s}")
    return found_ids


def discover(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    start: int = 0x40,
    end: int = 0x80,
) -> Optional[int]:
    """Sweep start..end; return the first responding motor_id, or None."""
    hits = discover_all(connection, telemetry, bus=bus, start=start, end=end)
    return hits[0] if hits else None


def probe(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    motor_id: int,
    timeout_s: float = 0.55,
    reset: bool = True,
) -> Optional[dict]:
    """Bench probe: optional reset then enable-only."""
    mcp = is_mcp_bus(bus)
    enable_timeout_s = _mcp_discover_timeout(timeout_s) if mcp else timeout_s
    reset_timeout_s = _mcp_discover_timeout(0.45) if mcp else 0.45
    with pause_plant_stream(connection):
        with lease(connection, telemetry, bus=bus):
            if telemetry is not None:
                telemetry.set_connected(True, mode="discover")
            if reset:
                _send_diag(
                    connection, motor_id, PROBE_RESET, reset_timeout_s, bus=bus
                )
                time.sleep(0.05)
            resp = _send_diag(
                connection, motor_id, PROBE_ENABLE_ONLY, enable_timeout_s, bus=bus
            )
            if resp is None:
                print(f"MISS  id=0x{motor_id:02X}")
            else:
                print(f"OK  {format_probe_line(resp)}")
            return resp
