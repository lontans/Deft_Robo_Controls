"""Unit tests for RobStride SET_CAN_ID (comm=0x07) helpers — no hardware.

Mirrors test_deft_controls_sdk_robstride_calibrate.py's fake-connection
harness. Exercises deft_controls_sdk.debug.robstride.set_can_id and its wire
encoding (PROBE_SET_CAN_ID param_index packing, discover-range clamp bug
fixed alongside it).
"""
from __future__ import annotations

import os
import struct
import sys
from typing import Optional

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug.robstride import _normalize_id_range, set_can_id
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import (
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    RS2_RESP_TAG,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_probe_command,
    extract_rs2_mailbox,
)
from deft_controls_sdk.link.exchange.bench import (
    PROBE_DATA_SAVE,
    PROBE_ENABLE_ONLY,
    PROBE_SET_CAN_ID,
)


def _fake_connection(responder) -> Connection:
    """Stubbed Connection: write_raw → responder → FrameReader (no real COM)."""
    conn = Connection("TESTPORT")
    conn._ser = object()

    def _write_raw(frame: bytes, *, drain: bool = False) -> None:
        reply = responder(frame)
        if reply is not None:
            conn._reader.feed(reply)

    conn.write_raw = _write_raw  # type: ignore[method-assign]
    return conn


def _rs2_feedback(
    *,
    motor_id: int,
    probe_kind: int,
    comm_mode: int = 0x02,
    found: bool = True,
    raw_frames: int = 1,
    discovered_id: Optional[int] = None,
) -> bytes:
    """Build a DBGF RS2 probe PDU (same shape as the calibrate test helper)."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_DEBUG_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    pdu = bytearray(32)
    pdu[0] = RS2_RESP_TAG
    pdu[1] = motor_id & 0xFF
    pdu[2] = 1 if found else 0
    pdu[3] = comm_mode & 0xFF
    pdu[24] = (discovered_id if discovered_id is not None else motor_id) & 0xFF
    pdu[25] = probe_kind & 0xFF
    pdu[26] = raw_frames & 0xFF
    buf[PDU_OFF : PDU_OFF + 32] = pdu
    return bytes(buf)


def test_build_rs2_probe_encodes_set_can_id_param() -> None:
    """param_index (new id) lands at mbox[5:7], target motor_id/kind/bus correct."""
    frame = build_rs2_probe_command(0x7F, PROBE_SET_CAN_ID, seq=1, param_index=0x78, bus=6)
    pdu = extract_rs2_mailbox(frame)
    assert pdu[3] == 0x7F  # addressed to the OLD id
    assert pdu[4] == PROBE_SET_CAN_ID
    assert pdu[5] == 0x78  # new id, low byte
    assert pdu[6] == 0
    assert pdu[11] == 6  # bus


def test_normalize_id_range_clamps_start_above_0x7f() -> None:
    """Regression: entering 0x82 used to produce the backwards/empty range
    0x82..0x7F (only `end` was clamped) — TUI discover then looped zero
    times and silently reported 'No RS2 motor found' with no explanation."""
    assert _normalize_id_range(0x82, 0x82) == (0x7F, 0x7F)
    assert _normalize_id_range(0x40, 0x90) == (0x40, 0x7F)


def test_set_can_id_full_sequence_reports_ok(monkeypatch: pytest.MonkeyPatch) -> None:
    """End-to-end happy path: SET_CAN_ID → verify (ENABLE_ONLY) → DATA_SAVE,
    each addressed to the right id, in order."""
    monkeypatch.setattr("deft_controls_sdk.debug.robstride.time.sleep", lambda *_a, **_k: None)
    monkeypatch.setattr("deft_controls_sdk.link.connection.time.sleep", lambda *_a, **_k: None)

    old_id, new_id, bus = 0x7F, 0x78, 5
    seen_kinds: list[int] = []

    def responder(frame: bytes):
        pdu = extract_rs2_mailbox(frame)
        kind = pdu[4]
        if kind == SESSION_BEGIN:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_BEGIN, found=False, raw_frames=0)
        if kind == SESSION_END:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_END, found=False, raw_frames=0)
        seen_kinds.append(kind)
        if kind == PROBE_SET_CAN_ID:
            assert pdu[3] == old_id
            assert pdu[5] == new_id
            return _rs2_feedback(motor_id=old_id, probe_kind=PROBE_SET_CAN_ID, comm_mode=0x07, found=True)
        if kind == PROBE_ENABLE_ONLY:
            assert pdu[3] == new_id
            return _rs2_feedback(motor_id=new_id, probe_kind=PROBE_ENABLE_ONLY, comm_mode=0x02, found=True)
        if kind == PROBE_DATA_SAVE:
            assert pdu[3] == new_id
            return _rs2_feedback(motor_id=new_id, probe_kind=PROBE_DATA_SAVE, comm_mode=0x16, found=True)
        return None

    conn = _fake_connection(responder)
    result = set_can_id(conn, None, bus=bus, old_id=old_id, new_id=new_id)

    assert seen_kinds == [PROBE_SET_CAN_ID, PROBE_ENABLE_ONLY, PROBE_DATA_SAVE]
    assert result["set_ok"] is True
    assert result["verify_ok"] is True
    assert result["save_ok"] is True


def test_set_can_id_skips_save_when_verify_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    """A doubtful id change (verify probe misses) must not be persisted."""
    monkeypatch.setattr("deft_controls_sdk.debug.robstride.time.sleep", lambda *_a, **_k: None)
    monkeypatch.setattr("deft_controls_sdk.link.connection.time.sleep", lambda *_a, **_k: None)

    old_id, new_id, bus = 0x7F, 0x78, 5
    seen_kinds: list[int] = []

    def responder(frame: bytes):
        pdu = extract_rs2_mailbox(frame)
        kind = pdu[4]
        if kind == SESSION_BEGIN:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_BEGIN, found=False, raw_frames=0)
        if kind == SESSION_END:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_END, found=False, raw_frames=0)
        seen_kinds.append(kind)
        if kind == PROBE_SET_CAN_ID:
            return _rs2_feedback(motor_id=old_id, probe_kind=PROBE_SET_CAN_ID, comm_mode=0x07, found=True)
        if kind == PROBE_ENABLE_ONLY:
            return None  # motor never answers on new_id — change didn't take
        return None

    conn = _fake_connection(responder)
    result = set_can_id(conn, None, bus=bus, old_id=old_id, new_id=new_id)

    assert PROBE_DATA_SAVE not in seen_kinds
    assert result["verify_ok"] is False
    assert result["save_ok"] is False


def test_set_can_id_rejects_same_id() -> None:
    conn = _fake_connection(lambda frame: None)
    with pytest.raises(ValueError):
        set_can_id(conn, None, bus=5, old_id=0x7F, new_id=0x7F)


def test_set_can_id_rejects_zero_new_id() -> None:
    conn = _fake_connection(lambda frame: None)
    with pytest.raises(ValueError):
        set_can_id(conn, None, bus=5, old_id=0x7F, new_id=0)
