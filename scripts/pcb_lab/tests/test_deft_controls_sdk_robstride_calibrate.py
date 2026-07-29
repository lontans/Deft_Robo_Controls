"""Unit tests for RS02 calibrate helpers and probe param packing (no hardware)."""
from __future__ import annotations

import os
import struct
import sys
from typing import Optional

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug.robstride_calibrate import (
    PARAM_BUS_VOLT,
    PARAM_MECH_POS,
    PARAM_RUN_MODE,
    _pararead,
    cali_finished,
    calibrate,
    decode_ext_id,
    mms_label,
    pararead_index_echo,
    pararead_is_hit,
    probe_response_valid,
)
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import (
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    RS2_RESP_TAG,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_probe_command,
)
from deft_controls_sdk.link.exchange.bench import (
    PROBE_CALI,
    PROBE_DATA_SAVE,
    PROBE_PARAREAD,
    PROBE_PARAWRITE,
    PROBE_RESET,
    PROBE_ZERO,
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
    mms: int = 0,
    position: float = 0.0,
    can_data: bytes = b"\x00" * 8,
    raw_frames: int = 1,
    discovered_id: Optional[int] = None,
) -> bytes:
    """Build a DBGF RS2 probe PDU. ``can_data`` is the 8 B CAN payload."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_DEBUG_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    pdu = bytearray(32)
    pdu[0] = RS2_RESP_TAG
    pdu[1] = motor_id & 0xFF
    pdu[2] = 1 if found else 0
    pdu[3] = comm_mode & 0xFF
    data16 = (motor_id & 0xFF) | ((mms & 0x3) << 14)
    ext_id = ((comm_mode & 0x1F) << 24) | (data16 << 8)
    struct.pack_into("<I", pdu, 4, ext_id)
    pad = bytes(can_data[:8]).ljust(8, b"\x00")
    pdu[8:16] = pad
    struct.pack_into("<ff", pdu, 16, 25.0, float(position))
    pdu[24] = (discovered_id if discovered_id is not None else motor_id) & 0xFF
    pdu[25] = probe_kind & 0xFF
    pdu[26] = raw_frames & 0xFF
    buf[PDU_OFF : PDU_OFF + 32] = pdu
    return bytes(buf)


def _pararead_can_data_zero_echo(value: float) -> bytes:
    """Real RS02 shape: index bytes 0x0000, float at data[4:7]."""
    can = bytearray(8)
    struct.pack_into("<f", can, 4, float(value))
    return bytes(can)


def test_build_rs2_probe_packs_param_bytes() -> None:
    frame = build_rs2_probe_command(0x70, PROBE_PARAWRITE, 3, 0x702D, 1, bus=4)
    assert len(frame) == IMAGE_BYTES
    magic, = struct.unpack_from("<I", frame, 0)
    assert magic == HOST_DEBUG_COMMAND_MAGIC
    assert frame[PDU_OFF : PDU_OFF + 5] == bytes([ord("R"), ord("S"), ord("2"), 0x70, PROBE_PARAWRITE])
    assert frame[PDU_OFF + 5] == 0x2D
    assert frame[PDU_OFF + 6] == 0x70
    assert frame[PDU_OFF + 7] == 1
    assert frame[PDU_OFF + 11] == 4  # bus


def test_build_rs2_cali_listen_seconds() -> None:
    frame = build_rs2_probe_command(0x70, PROBE_CALI, 1, 28, bus=2)
    assert frame[PDU_OFF + 4] == PROBE_CALI
    assert frame[PDU_OFF + 5] == 28
    assert frame[PDU_OFF + 11] == 2


def test_decode_ext_id_mms() -> None:
    # mode=0x02 feedback, motor 0x70, mms=cali (bits 14..15 of data16 = 1)
    data16 = 0x70 | (1 << 14)
    ext_id = (0x02 << 24) | (data16 << 8)
    ext = decode_ext_id(ext_id)
    assert ext.mode == 0x02
    assert ext.motor_id == 0x70
    assert ext.mode_status == 1
    assert mms_label(1) == "cali"


def test_cali_finished_requires_saw_cali() -> None:
    data16 = 0x70  # mms=rest
    ext_id = (0x02 << 24) | (data16 << 8)
    resp = {
        "ext_id": ext_id,
        "comm_mode": 0x02,
        "discovered_id": 0x70,
        "probe_id": 0x70,
    }
    assert probe_response_valid(resp, 0x70)
    assert not cali_finished(resp, 0x70, saw_cali=False)
    assert cali_finished(resp, 0x70, saw_cali=True)


def test_pararead_is_hit_accepts_zero_echo() -> None:
    """RS02 leaves data[0:1]=0x0000; rejecting that made calibrate always FAIL."""
    can = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F])  # float 1.0 LE at [4:7]
    resp = {
        "comm_mode": 0x11,
        "found": True,
        "raw_frames": 1,
        "can_data": can,
        "position": 1.0,
    }
    assert pararead_index_echo(resp) == 0
    assert pararead_is_hit(resp, 0x7019)
    assert pararead_is_hit(resp, 0x701C)
    assert pararead_is_hit(resp, 0x7005)


def test_pararead_is_hit_accepts_matching_echo() -> None:
    can = bytes([0x19, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    resp = {"comm_mode": 0x11, "found": True, "raw_frames": 1, "can_data": can}
    assert pararead_index_echo(resp) == 0x7019
    assert pararead_is_hit(resp, 0x7019)
    assert not pararead_is_hit(resp, 0x701C)


def test_pararead_is_hit_rejects_wrong_comm() -> None:
    can = bytes(8)
    resp = {"comm_mode": 0x02, "found": True, "raw_frames": 1, "can_data": can}
    assert not pararead_is_hit(resp, 0x7019)


def test_pararead_accepts_zero_index_echo_from_hardware(monkeypatch: pytest.MonkeyPatch) -> None:
    """Regression: RS02 leaves can_data[0:1]=0; treating that as a miss made
    every calibrate() verify fail even when cal physically succeeded.

    Pins ``_pararead`` (not just ``pararead_is_hit``) against a real-shaped
    zero-echo reply so a future rewrite of the echo check cannot regress.
    """
    monkeypatch.setattr("deft_controls_sdk.debug.robstride_calibrate.time.sleep", lambda *_a, **_k: None)

    motor_id = 0x70
    value = 1.25
    can = _pararead_can_data_zero_echo(value)

    def responder(frame: bytes):
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        assert pdu[4] == PROBE_PARAREAD
        assert (pdu[5] | (pdu[6] << 8)) == PARAM_MECH_POS
        return _rs2_feedback(
            motor_id=motor_id,
            probe_kind=PROBE_PARAREAD,
            comm_mode=0x11,
            position=value,
            can_data=can,
            raw_frames=1,
        )

    conn = _fake_connection(responder)
    hit, sniff = _pararead(conn, motor_id, PARAM_MECH_POS, 0.5, bus=4)
    assert sniff is None or hit is not None
    assert hit is not None, "zero-echo pararead reply must count as a hit"
    assert pararead_index_echo(hit) == 0
    assert hit["position"] == pytest.approx(value)


def test_calibrate_succeeds_with_zero_echo_pararead_verify(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """End-to-end: fake RS02 that never echoes param index in data[0:1].

    Exercises full ``_cal_body`` (reset → iq_test → cali → zero → save →
    VERIFY_READS). This is the path that should have caught the original bug
    where only unit helpers were tested and ``_pararead`` still rejected
    hardware replies.
    """
    monkeypatch.setattr("deft_controls_sdk.debug.robstride_calibrate.time.sleep", lambda *_a, **_k: None)
    monkeypatch.setattr("deft_controls_sdk.link.connection.time.sleep", lambda *_a, **_k: None)

    motor_id = 0x70
    bus = 4

    def _ext_feedback(kind: int, *, mms: int = 0, comm: int = 0x02) -> bytes:
        return _rs2_feedback(
            motor_id=motor_id,
            probe_kind=kind,
            comm_mode=comm,
            mms=mms,
            position=0.0,
            raw_frames=1,
        )

    def _zero_echo_pararead(param_index: int) -> bytes:
        # VERIFY wants mechPos near 0; VBUS in range; run_mode arbitrary.
        values = {
            PARAM_MECH_POS: 0.01,
            PARAM_BUS_VOLT: 48.0,
            PARAM_RUN_MODE: 0.0,
        }
        value = float(values.get(param_index & 0xFFFF, 0.0))
        return _rs2_feedback(
            motor_id=motor_id,
            probe_kind=PROBE_PARAREAD,
            comm_mode=0x11,
            position=value,
            can_data=_pararead_can_data_zero_echo(value),
            raw_frames=1,
        )

    def responder(frame: bytes):
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        kind = pdu[4]
        param_index = pdu[5] | (pdu[6] << 8)

        if kind == SESSION_BEGIN:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_BEGIN, found=False, raw_frames=0)
        if kind == SESSION_END:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_END, found=False, raw_frames=0)
        if kind == PROBE_RESET:
            return _ext_feedback(PROBE_RESET, mms=0)
        if kind == PROBE_PARAWRITE:
            return _ext_feedback(PROBE_PARAWRITE, mms=0, comm=0x12)
        if kind == PROBE_PARAREAD:
            return _zero_echo_pararead(param_index)
        if kind == PROBE_CALI:
            # Progress: enter cali, then leave to rest (saw_cali + finished).
            return _ext_feedback(PROBE_CALI, mms=1) + _ext_feedback(PROBE_CALI, mms=0)
        if kind == PROBE_ZERO:
            return _ext_feedback(PROBE_ZERO, mms=0)
        if kind == PROBE_DATA_SAVE:
            return _ext_feedback(PROBE_DATA_SAVE, mms=0)
        return None

    conn = _fake_connection(responder)
    ok = calibrate(
        conn,
        None,
        bus=bus,
        motor_id=motor_id,
        cal_listen_s=10.0,
        skip_iq_test=False,
        strict_cali=False,
    )
    assert ok is True


def test_calibrate_partial_pararead_does_not_crash_on_none_mechpos(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Regression: hardware report was "SDK crashed formatting mechPos=None
    after save". VBUS/run_mode pararead can succeed while the mechPos read
    specifically times out -- calibrate() must report a clean FAIL, not raise
    TypeError formatting None with `:+.4f`.
    """
    monkeypatch.setattr("deft_controls_sdk.debug.robstride_calibrate.time.sleep", lambda *_a, **_k: None)
    monkeypatch.setattr("deft_controls_sdk.link.connection.time.sleep", lambda *_a, **_k: None)

    motor_id = 0x70
    bus = 4

    def _ext_feedback(kind: int, *, mms: int = 0, comm: int = 0x02) -> bytes:
        return _rs2_feedback(
            motor_id=motor_id,
            probe_kind=kind,
            comm_mode=comm,
            mms=mms,
            position=0.0,
            raw_frames=1,
        )

    def responder(frame: bytes):
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        kind = pdu[4]
        param_index = pdu[5] | (pdu[6] << 8)

        if kind == SESSION_BEGIN:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_BEGIN, found=False, raw_frames=0)
        if kind == SESSION_END:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_END, found=False, raw_frames=0)
        if kind == PROBE_RESET:
            return _ext_feedback(PROBE_RESET, mms=0)
        if kind == PROBE_PARAWRITE:
            return _ext_feedback(PROBE_PARAWRITE, mms=0, comm=0x12)
        if kind == PROBE_PARAREAD:
            # mechPos read: no reply (times out). VBUS/run_mode: reply fine.
            if (param_index & 0xFFFF) == PARAM_MECH_POS:
                return None
            value = 48.0 if (param_index & 0xFFFF) == PARAM_BUS_VOLT else 0.0
            return _rs2_feedback(
                motor_id=motor_id,
                probe_kind=PROBE_PARAREAD,
                comm_mode=0x11,
                position=value,
                can_data=_pararead_can_data_zero_echo(value),
                raw_frames=1,
            )
        if kind == PROBE_CALI:
            return _ext_feedback(PROBE_CALI, mms=1) + _ext_feedback(PROBE_CALI, mms=0)
        if kind == PROBE_ZERO:
            return _ext_feedback(PROBE_ZERO, mms=0)
        if kind == PROBE_DATA_SAVE:
            return _ext_feedback(PROBE_DATA_SAVE, mms=0)
        return None

    conn = _fake_connection(responder)
    ok = calibrate(
        conn,
        None,
        bus=bus,
        motor_id=motor_id,
        cal_listen_s=10.0,
        skip_iq_test=False,
        strict_cali=False,
    )
    assert ok is False
