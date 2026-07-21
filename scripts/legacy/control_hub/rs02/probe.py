"""RS2 bench probes via PcbSession (RS2 PDU on 562 B image)."""
from __future__ import annotations

import time
from typing import Callable, Optional, Tuple

from controls_pcb_host import commands as cmd
from controls_pcb_host.feedback import parse_probe_pdu
from controls_pcb_host.protocol import (
    PROBE_CALI,
    PROBE_DATA_SAVE,
    PROBE_ENABLE_ONLY,
    PROBE_PARAREAD,
    PROBE_PARAWRITE,
    PROBE_RESET,
    PROBE_ZERO,
)
from controls_pcb_host.session import PcbSession

from ..protocol.rs02 import CALI_LISTEN_ONLY, comm_label, decode_ext_id, mms_label
from .display import format_probe_line

__all__ = [
    "format_probe_line",
    "mms_label",
    "pararead",
    "parawrite",
    "probe",
    "probe_cali",
    "probe_response_valid",
    "wait_cali_done",
]

_VALID_COMM = (0x02, 0x11, 0x12, 0x18)


def probe_response_valid(resp: dict, motor_id: int) -> bool:
    if not resp.get("ext_id"):
        return False
    ext = decode_ext_id(resp["ext_id"])
    disc = resp.get("discovered_id") or ext.motor_id
    if disc != (motor_id & 0xFF):
        return False
    comm = resp["comm_mode"]
    if comm != ext.mode or comm not in _VALID_COMM:
        return False
    return True


def cali_finished(resp: dict, motor_id: int, *, saw_cali: bool) -> bool:
    if not saw_cali or not probe_response_valid(resp, motor_id):
        return False
    ext = decode_ext_id(resp["ext_id"])
    return ext.mode_status in (0, 2)


def wait_cali_probe_response(
    session: PcbSession,
    motor_id: int,
    timeout_s: float,
    *,
    on_progress: Optional[Callable[[dict, bool], None]] = None,
) -> Tuple[Optional[dict], bool]:
    """Wait for cali to finish — track mms=cali from progress/final PDUs."""
    reader = session.reader
    deadline = time.monotonic() + timeout_s
    latest_valid: Optional[dict] = None
    saw_cali = False
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_probe_pdu(frame)
            if (
                parsed is not None
                and parsed["probe_id"] == (motor_id & 0xFF)
                and parsed["probe_kind"] == PROBE_CALI
            ):
                if parsed.get("ext_id"):
                    ext = decode_ext_id(parsed["ext_id"])
                    if ext.mode == 0x02 and ext.motor_id == (motor_id & 0xFF):
                        if ext.mode_status == 1:
                            saw_cali = True
                        if probe_response_valid(parsed, motor_id):
                            latest_valid = parsed
                        if cali_finished(parsed, motor_id, saw_cali=saw_cali):
                            return parsed, True
                if on_progress is not None:
                    on_progress(parsed, saw_cali)
            frame = reader.pop()
        time.sleep(0.005)
    return latest_valid, saw_cali


def probe(
    session: PcbSession,
    motor_id: int,
    kind: int,
    timeout_s: float,
    *,
    bus: int,
    probe_param: int = 0,
) -> Optional[dict]:
    if kind == PROBE_CALI:
        listen_s = float(probe_param & 0xFF) if probe_param else 28.0
        listen_only = bool(probe_param & CALI_LISTEN_ONLY)
        resp, _ = probe_cali(
            session,
            motor_id,
            listen_s,
            bus=bus,
            listen_only=listen_only,
            usb_wait_s=timeout_s,
        )
        return resp

    from controls_pcb_host.plugins import robstride as rs2

    return rs2.send_diag(
        session,
        motor_id,
        kind,
        timeout_s,
        probe_param=probe_param,
        bus=bus,
    )


def probe_cali(
    session: PcbSession,
    motor_id: int,
    listen_s: float,
    *,
    bus: int,
    listen_only: bool = False,
    usb_wait_s: Optional[float] = None,
    probe_param: Optional[int] = None,
    on_progress: Optional[Callable[[dict, bool], None]] = None,
) -> Tuple[Optional[dict], bool]:
    listen_s = max(8.0, listen_s)
    if probe_param is not None:
        param = probe_param
    elif listen_only:
        param = CALI_LISTEN_ONLY | int(listen_s)
    else:
        param = int(listen_s) & 0xFF
    wait_s = usb_wait_s if usb_wait_s is not None else listen_s + 15.0
    session.reader.drain()
    seq = session.next_seq()
    frame = cmd.build_rs2_probe_command(motor_id, PROBE_CALI, seq, param, bus=bus)
    session.write_command(frame)
    return wait_cali_probe_response(session, motor_id, wait_s, on_progress=on_progress)


def parawrite(
    session: PcbSession,
    motor_id: int,
    index: int,
    raw_value: int,
    timeout_s: float,
    *,
    bus: int,
) -> Optional[dict]:
    from controls_pcb_host.plugins.robstride import wait_probe_response

    seq = session.next_seq()
    frame = cmd.build_rs2_probe_command(
        motor_id, PROBE_PARAWRITE, seq, index, raw_value, bus=bus
    )
    session.write_command(frame)
    return wait_probe_response(
        session.reader, motor_id, timeout_s, probe_kind=PROBE_PARAWRITE
    )


def pararead(
    session: PcbSession,
    motor_id: int,
    index: int,
    timeout_s: float,
    *,
    bus: int,
) -> Tuple[Optional[dict], Optional[dict]]:
    """Return (hit, sniff) — hit is comm 0x11 with float in position field."""
    session.reader.drain()
    seq = session.next_seq()
    frame = cmd.build_rs2_probe_command(motor_id, PROBE_PARAREAD, seq, index, bus=bus)
    session.write_command(frame)
    deadline = time.monotonic() + timeout_s
    hit: Optional[dict] = None
    sniff: Optional[dict] = None
    while time.monotonic() < deadline:
        raw = session.reader.pop()
        while raw is not None:
            parsed = parse_probe_pdu(raw)
            if parsed is not None and parsed["probe_id"] == (motor_id & 0xFF):
                if parsed["comm_mode"] == 0x11 and parsed.get("found"):
                    idx_echo = parsed["can_data"][0] | (parsed["can_data"][1] << 8)
                    if idx_echo == index:
                        hit = parsed
                        return hit, sniff
                if sniff is None:
                    sniff = parsed
            raw = session.reader.pop()
        time.sleep(0.005)
    return hit, sniff


def wait_cali_done(
    session: PcbSession,
    motor_id: int,
    *,
    bus: int,
    poll_s: float = 15.0,
) -> Optional[dict]:
    """Deprecated: follow-up listen chunks disturb an in-flight cal. Prefer one long listen."""
    deadline = time.monotonic() + poll_s
    chunk_s = min(20.0, max(8.0, poll_s * 0.35))
    while time.monotonic() < deadline:
        resp, saw_cali = probe_cali(
            session,
            motor_id,
            chunk_s,
            bus=bus,
            listen_only=True,
            usb_wait_s=chunk_s + 8.0,
        )
        if resp is not None and cali_finished(resp, motor_id, saw_cali=saw_cali):
            return resp
        time.sleep(0.2)
    return None
