"""RobStride RS2 discover / probe / calibrate over DEBUG frames."""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

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
    frame = build_rs2_probe_command(motor_id, probe_kind, connection.next_seq(), kp=kp, kd=kd, bus=bus)
    return connection.exchange_raw(
        frame,
        parse_probe_pdu,
        timeout_s=timeout_s,
        predicate=lambda p: p["probe_id"] == (motor_id & 0xFF) and p["probe_kind"] == probe_kind,
    )


def format_probe_line(resp: dict) -> str:
    found = int(resp.get("found", 0))
    return (
        f"probe_id=0x{resp['probe_id']:02X}  kind={resp.get('probe_kind')}  "
        f"found={found}  comm={resp.get('comm_mode')}  pos={resp.get('position', 0):+.4f}  "
        f"raw={resp.get('raw_frames', 0)}"
    )


def _mcp_discover_timeout(base_s: float) -> float:
    """MCP probes block USB until listen finishes (~420 ms enable + TX flush).

    FDCAN budgets (0.55 / 0.40) are too short — host times out after TX is on
    the wire but before the HIT PDU arrives. Mirror legacy rs02_can_scan.
    """
    return max(base_s * 3.0, base_s + 0.6, 2.0)


def discover(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    start: int = 0x40,
    end: int = 0x80,
) -> Optional[int]:
    """Sweep start..end on `bus`, trying enable-then-promiscuous per ID. Returns
    the first responding motor_id, or None."""
    mcp = is_mcp_bus(bus)
    enable_s = _mcp_discover_timeout(0.55) if mcp else 0.55
    promisc_s = _mcp_discover_timeout(0.40) if mcp else 0.40
    print(
        f"RS2 discover on {can_bus_label(bus)}  IDs 0x{start:02X}..0x{end:02X}"
        + (f"  (MCP timeouts enable={enable_s:.1f}s promisc={promisc_s:.1f}s)" if mcp else "")
    )
    with lease(connection, telemetry, bus=bus):
        if telemetry is not None:
            telemetry.set_connected(True, mode="discover")
        for motor_id in range(start, end + 1):
            for kind, label, timeout_s in (
                (PROBE_ENABLE_ONLY, "enable", enable_s),
                (PROBE_PROMISC, "promisc", promisc_s),
            ):
                resp = _send_diag(connection, motor_id, kind, timeout_s, bus=bus)
                if resp is not None and resp.get("found"):
                    print(f"FOUND  id=0x{motor_id:02X}  via {label}  {format_probe_line(resp)}")
                    return motor_id
                if resp is not None and resp.get("raw_frames", 0) > 0:
                    print(f"  traffic  id=0x{motor_id:02X}  {label}  raw={resp['raw_frames']}")
        print("No RS2 motor found in range.")
        return None


def probe(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    motor_id: int,
    timeout_s: float = 0.55,
) -> Optional[dict]:
    """Bench probe: reset (FDCAN only) then enable-only. Mirrors legacy probe_id's
    default path (kind=PROBE_ENABLE_ONLY); the kp/kd ctrl-probe kinds are not
    exposed here yet — add if a caller needs PROBE_FULL/PROBE_CTRL_ONLY."""
    mcp = is_mcp_bus(bus)
    # MCP: firmware enable listen ~420 ms + blocking SPI TX; keep ≥2 s like discover.
    enable_timeout_s = _mcp_discover_timeout(timeout_s) if mcp else timeout_s
    with lease(connection, telemetry, bus=bus):
        if telemetry is not None:
            telemetry.set_connected(True, mode="discover")
        if not mcp:
            _send_diag(connection, motor_id, PROBE_RESET, 0.45, bus=bus)
        resp = _send_diag(connection, motor_id, PROBE_ENABLE_ONLY, enable_timeout_s, bus=bus)
        if resp is None:
            print(f"MISS  id=0x{motor_id:02X}")
        else:
            print(f"OK  {format_probe_line(resp)}")
        return resp
