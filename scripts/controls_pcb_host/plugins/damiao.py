"""Damiao DM0 bench probes."""
from __future__ import annotations

import time
from typing import Optional

from .. import commands as cmd
from ..actuator_config import slot_config
from ..feedback import (
    dm_fault_found,
    parse_dm_from_actuator,
    parse_dm_probe_pdu,
    probe_kind_matches,
)
from ..protocol.diag_pdu import (
    DM_PROBE_CLEAR_FAULT,
    DM_PROBE_ENABLE,
    DM_PROBE_ID_SWEEP,
    DM_PROBE_KIND_NAMES,
    DM_PROBE_MIT,
    DM_PROBE_REG_SCAN,
    DM_REG_ESC_ID,
    SESSION_BEGIN,
    SESSION_END,
)
from ..protocol.can_bus import can_bus_label
from ..session import PcbSession
from ..transport import FrameReader


def clamp_listen_ms(ms: int, *, minimum: int = 20, maximum: int = 255) -> int:
    return max(minimum, min(maximum, int(ms)))


def probe_timeout_s(probe_kind: int, listen_ms: int, id_span: int = 1) -> float:
    if probe_kind == DM_PROBE_ID_SWEEP:
        return (id_span * (listen_ms + 5)) / 1000.0 + 3.0
    if probe_kind == DM_PROBE_REG_SCAN:
        return max(3.0, (listen_ms * 3 + 140) / 1000.0 + 1.5)
    if probe_kind == DM_PROBE_MIT:
        return max(2.0, listen_ms / 1000.0 + 1.0)
    return max(1.0, listen_ms / 1000.0 + 0.75)


def wait_probe_response(
    reader: FrameReader,
    probe_id: int,
    timeout_s: float,
    probe_kind: Optional[int] = None,
    *,
    slot: int = 2,
) -> Optional[dict]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_dm_probe_pdu(frame)
            if parsed is not None:
                if probe_kind in (SESSION_BEGIN, SESSION_END):
                    if parsed["probe_kind"] == probe_kind:
                        return parsed
                elif probe_kind == DM_PROBE_ID_SWEEP:
                    if parsed["probe_kind"] == DM_PROBE_ID_SWEEP:
                        return parsed
                elif parsed["probe_id"] == (probe_id & 0xFF):
                    if probe_kind_matches(parsed["probe_kind"], probe_kind):
                        return parsed
            slot_parsed = parse_dm_from_actuator(frame, probe_id, slot=slot)
            if slot_parsed is not None and probe_kind not in (SESSION_BEGIN, SESSION_END):
                if probe_kind is None or probe_kind_matches(slot_parsed["probe_kind"], probe_kind):
                    return slot_parsed
            frame = reader.pop()
        time.sleep(0.005)
    return None


def send_probe(
    session: PcbSession,
    motor_id: int,
    probe_kind: int,
    *,
    bus: int = 3,
    master_id: int = 0,
    listen_ms: int = 15,
    param_rid: int = DM_REG_ESC_ID,
    end_id: int = 0,
    timeout_s: Optional[float] = None,
    slot: int = 2,
) -> Optional[dict]:
    listen_ms = clamp_listen_ms(listen_ms)
    if timeout_s is None:
        span = max(1, (end_id - motor_id + 1) if probe_kind == DM_PROBE_ID_SWEEP else 1)
        timeout_s = probe_timeout_s(probe_kind, listen_ms, span)
    seq = session.next_seq()
    frame = cmd.build_dm_probe_command(
        motor_id,
        probe_kind,
        seq,
        bus=bus,
        master_id=master_id,
        listen_ms=listen_ms,
        param_rid=param_rid,
        end_id=end_id,
    )
    session.write_command(frame)
    pk = probe_kind if probe_kind in (SESSION_BEGIN, SESSION_END) else probe_kind
    return wait_probe_response(session.reader, motor_id, timeout_s, probe_kind=pk, slot=slot)


def format_hit(resp: dict, motor_id: int) -> str:
    kind = DM_PROBE_KIND_NAMES.get(resp.get("probe_kind"), str(resp.get("probe_kind")))
    esc = resp.get("discovered_id", resp.get("param_value", 0)) & 0xFF
    master = resp.get("master_id", 0) & 0xFF
    err = resp.get("err", 0) & 0xFF
    return (
        f"FOUND  probe=0x{motor_id:02X}  esc_id=0x{esc:02X}  master_rx=0x{master:02X}  "
        f"mode={kind}  pos={resp.get('position', 0):+.4f}  err=0x{err:x}"
    )


def discover(
    session: PcbSession,
    bus: int = 3,
    start: int = 1,
    end: int = 16,
    *,
    listen_ms: int = 40,
) -> Optional[int]:
    print(f"Damiao reg-scan discover on {can_bus_label(bus)}  IDs {start}..{end}")
    with session.rx_pump():
        session.dm_session_begin(bus)
        try:
            for mid in range(start, end + 1):
                resp = send_probe(
                    session,
                    mid,
                    DM_PROBE_REG_SCAN,
                    bus=bus,
                    listen_ms=listen_ms,
                )
                if resp is not None and (resp.get("found") or dm_fault_found(resp.get("err", 0))):
                    print(format_hit(resp, mid))
                    return mid
            print("No Damiao motor found in range.")
            return None
        finally:
            session.dm_session_end(bus)


def probe(
    session: PcbSession,
    bus: int,
    motor_id: int,
    *,
    kind: int = DM_PROBE_REG_SCAN,
    enable: bool = False,
    listen_ms: int = 15,
) -> Optional[dict]:
    with session.rx_pump():
        session.dm_session_begin(bus)
        try:
            if enable:
                send_probe(session, motor_id, DM_PROBE_CLEAR_FAULT, bus=bus, listen_ms=listen_ms)
                resp = send_probe(session, motor_id, DM_PROBE_ENABLE, bus=bus, listen_ms=listen_ms)
            else:
                resp = send_probe(
                    session,
                    motor_id,
                    kind,
                    bus=bus,
                    listen_ms=listen_ms,
                )
            if resp is None:
                print(f"MISS  id=0x{motor_id:02X}")
            else:
                print(format_hit(resp, motor_id))
            return resp
        finally:
            session.dm_session_end(bus)


def enable_and_hold(
    session: PcbSession,
    bus: int,
    motor_id: int,
    hold_ms: int = 3000,
    *,
    listen_ms: int = 32,
) -> bool:
    """Clear fault, enable, verify ERR=1, MIT-hold bridge (Damiao comm timeout)."""
    slot = slot_config(2).slot if motor_id == slot_config(2).motor_id else 2
    with session.rx_pump():
        session.dm_session_begin(bus)
        try:
            send_probe(session, motor_id, DM_PROBE_CLEAR_FAULT, bus=bus, listen_ms=listen_ms)
            resp = send_probe(session, motor_id, DM_PROBE_ENABLE, bus=bus, listen_ms=listen_ms)
            if resp is None or (resp.get("err", 0) & 0xF) != 1:
                act = None
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    frame = session.reader.pop()
                    if frame is not None:
                        act_fb = parse_dm_from_actuator(frame, motor_id, slot=slot)
                        if act_fb is not None:
                            act = act_fb
                            break
                    time.sleep(0.01)
                if act is None or (act.get("err", 0) & 0xF) != 1:
                    print("Enable verify failed (ERR nibble != 1).")
                    return False
            print("OK: motor reports enabled (ERR=0x1).")
            print(f"MIT hold {hold_ms} ms — start teleop before TIMEOUT expires.")
            deadline = time.monotonic() + hold_ms / 1000.0
            bursts = 0
            while time.monotonic() < deadline:
                send_probe(
                    session,
                    motor_id,
                    DM_PROBE_MIT,
                    bus=bus,
                    listen_ms=listen_ms,
                    timeout_s=0.5,
                )
                bursts += 1
            print(f"Hold done ({bursts} MIT burst(s)).")
            return True
        finally:
            session.dm_session_end(bus)
