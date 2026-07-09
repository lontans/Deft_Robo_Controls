"""RobStride RS2 bench probes over RS2 PDU."""
from __future__ import annotations

import time
from typing import Optional

from .. import commands as cmd
from ..feedback import parse_probe_pdu
from ..protocol import (
    PROBE_CTRL_ONLY,
    PROBE_ENABLE_CTRL,
    PROBE_ENABLE_ONLY,
    PROBE_FULL,
    PROBE_PROMISC,
    PROBE_RESET,
    SESSION_BEGIN,
    SESSION_END,
)
from ..protocol.can_bus import can_bus_label, is_mcp_bus, print_can_bus_note
from ..session import PcbSession
from ..transport import FrameReader


def wait_probe_response(
    reader: FrameReader,
    probe_id: int,
    timeout_s: float,
    probe_kind: Optional[int] = None,
) -> Optional[dict]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_probe_pdu(frame)
            if parsed is not None:
                if probe_kind in (SESSION_BEGIN, SESSION_END):
                    if parsed["probe_kind"] == probe_kind:
                        return parsed
                elif parsed["probe_id"] == (probe_id & 0xFF):
                    if probe_kind is not None and parsed["probe_kind"] != probe_kind:
                        frame = reader.pop()
                        continue
                    return parsed
            frame = reader.pop()
        time.sleep(0.005)
    return None


def send_diag(
    session: PcbSession,
    motor_id: int,
    probe_kind: int,
    timeout_s: float,
    *,
    probe_param: int = 0,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    bus: int = 1,
) -> Optional[dict]:
    seq = session.next_seq()
    session.reader.drain()
    if probe_param != 0:
        frame = cmd.build_rs2_probe_command(
            motor_id,
            probe_kind,
            seq,
            probe_param,
            position=position,
            kp=kp,
            kd=kd,
            bus=bus,
        )
    else:
        buf = bytearray(cmd.build_rs2_scan_command(motor_id, probe_kind, seq, bus=bus))
        if position != 0.0 or velocity != 0.0 or kp != 0.0 or kd != 0.0:
            cmd.patch_actuator_desire(buf, position=position, velocity=velocity, kp=kp, kd=kd)
        frame = bytes(buf)
    session.write_command(frame)
    return wait_probe_response(session.reader, motor_id, timeout_s, probe_kind=probe_kind)


def format_probe_line(resp: dict) -> str:
    found = int(resp.get("found", 0))
    pos = float(resp.get("position", 0))
    pos_s = f"{pos:+.4f}"
    if abs(pos) > 3.0 and resp.get("comm_mode") == 0x02:
        pos_s += " (comm0x02 p_raw — pararead 0x7019 for truth)"
    return (
        f"probe_id=0x{resp['probe_id']:02X}  kind={resp.get('probe_kind')}  "
        f"found={found}  comm={resp.get('comm_mode')}  "
        f"pos={pos_s}  raw={resp.get('raw_frames', 0)}"
    )


def discover_id(
    session: PcbSession,
    bus: int,
    start: int = 0x40,
    end: int = 0x80,
) -> Optional[int]:
    print(f"RS2 discover on {can_bus_label(bus)}  IDs 0x{start:02X}..0x{end:02X}")
    print_can_bus_note(bus)
    with session.rx_pump():
        session.rs2_session_begin(bus)
        try:
            for mid in range(start, end + 1):
                for kind, label, tmo in (
                    (PROBE_ENABLE_ONLY, "enable", 0.55),
                    (PROBE_PROMISC, "promisc", 0.40),
                ):
                    resp = send_diag(session, mid, kind, tmo, bus=bus)
                    if resp is not None and resp.get("found"):
                        print(f"FOUND  id=0x{mid:02X}  via {label}  {format_probe_line(resp)}")
                        return mid
                    if resp is not None and resp.get("raw_frames", 0) > 0:
                        print(f"  traffic  id=0x{mid:02X}  {label}  raw={resp['raw_frames']}")
            print("No RS2 motor found in range.")
            return None
        finally:
            session.rs2_session_end(bus)


def probe_id(
    session: PcbSession,
    bus: int,
    motor_id: int,
    *,
    kind: int = PROBE_ENABLE_ONLY,
    timeout_s: float = 0.55,
) -> Optional[dict]:
    """Bench probe: reset then enable-only by default (matches discover hit path).

    MCP (CH4–6): skip pre-reset (discover path); longer listen timeout for SPI-CAN.
    Use kind=PROBE_FULL for run_mode+enable+ctrl init (--full on CLI).
    """
    mcp = is_mcp_bus(bus)
    enable_tmo = max(timeout_s, 1.0) if mcp else timeout_s
    with session.rx_pump():
        session.rs2_session_begin(bus)
        try:
            if not mcp:
                send_diag(session, motor_id, PROBE_RESET, 0.45, bus=bus)
            kw: dict = {}
            if kind in (PROBE_FULL, PROBE_ENABLE_CTRL, PROBE_CTRL_ONLY):
                kw = dict(kp=50.0, kd=1.0)
            resp = send_diag(
                session, motor_id, kind, enable_tmo, bus=bus, **kw
            )
            if resp is None:
                print(f"MISS  id=0x{motor_id:02X}  kind={kind}")
            else:
                print(f"OK  {format_probe_line(resp)}")
            return resp
        finally:
            session.rs2_session_end(bus)


def enable_for_plant(
    session: PcbSession,
    bus: int,
    motor_id: int,
    *,
    timeout_s: Optional[float] = None,
) -> bool:
    """RS2 enable-only on one bus — caller must hold session.rx_pump().

    Used at plant teleop start so motors are armed after recover/exit-reset
  without a manual ``probe`` per branch. Does not print on success.
    """
    mcp = is_mcp_bus(bus)
    tmo = timeout_s if timeout_s is not None else (1.0 if mcp else 0.55)
    session.rs2_session_begin(bus)
    try:
        resp = send_diag(session, motor_id & 0xFF, PROBE_ENABLE_ONLY, tmo, bus=bus)
        return resp is not None and bool(resp.get("found"))
    finally:
        session.rs2_session_end(bus)
        time.sleep(0.08 if mcp else 0.04)
