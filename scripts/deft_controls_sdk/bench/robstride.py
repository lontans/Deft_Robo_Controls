"""RobStride RS2 discover / probe over the RS2 bench PDU.

Ported from scripts/legacy/controls_pcb_host/plugins/robstride.py
(discover_id, probe_id, send_diag, format_probe_line) — control flow and
timing kept close to the original; calibrate (comm 0x04->0x05->0x06->0x16,
RS02_Firmware_Documentation.pdf) is NOT ported yet. It depends on four more
legacy modules (control_hub/rs02/{calibrate,probe,display}.py, link.py) with
timing-sensitive listen windows and a spinning shaft — that's a deeper, more
hardware-sensitive port than discover/probe and deserves its own pass rather
than a rushed one bundled in here. Use `scripts/legacy` for calibrate today.
"""
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


def discover(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    start: int = 0x40,
    end: int = 0x80,
) -> Optional[int]:
    """Sweep start..end on `bus`, trying enable-then-promiscuous per ID. Returns
    the first responding motor_id, or None. Mirrors legacy discover_id exactly."""
    print(f"RS2 discover on {can_bus_label(bus)}  IDs 0x{start:02X}..0x{end:02X}")
    with lease(connection, telemetry, bus=bus):
        if telemetry is not None:
            telemetry.set_connected(True, mode="discover")
        for motor_id in range(start, end + 1):
            for kind, label, timeout_s in (
                (PROBE_ENABLE_ONLY, "enable", 0.55),
                (PROBE_PROMISC, "promisc", 0.40),
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
    enable_timeout_s = max(timeout_s, 1.0) if mcp else timeout_s
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
