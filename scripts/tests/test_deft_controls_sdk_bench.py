"""Golden tests for deft_controls_sdk/bench (no hardware).

Connection is never opened for real — `_ser` is stubbed and `write_raw` is
monkeypatched to synthesize a reply frame and feed it straight into the
FrameReader, exercising the same exchange_raw()/predicate matching path a
real bench session would use.
"""
from __future__ import annotations

import os
import struct
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.bench import DebugAPI
from deft_controls_sdk.bench.config import fetch_table
from deft_controls_sdk.bench.lease import lease
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import (
    CFG_OP_GET,
    CFG_RESP_TAG0,
    CFG_RESP_TAG1,
    CFG_RESP_TAG2,
    CFG_STATUS_OK,
    DM_PROBE_ID_SWEEP,
    DM_PROBE_REG_SCAN,
    DM_RESP_TAG,
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    PROBE_ENABLE_ONLY,
    RS2_RESP_TAG,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_scan_command,
    parse_probe_pdu,
)


def _fake_connection(responder) -> Connection:
    """A Connection with a stubbed socket; write_raw hands the outgoing frame to
    `responder(frame) -> Optional[bytes]`, and any returned reply is fed back
    into the reader — same shape as a real device's request/response."""
    conn = Connection("TESTPORT")
    conn._ser = object()  # is_open True; nothing actually opens a port

    def _write_raw(frame: bytes, *, drain: bool = False) -> None:
        reply = responder(frame)
        if reply is not None:
            conn._reader.feed(reply)

    conn.write_raw = _write_raw  # type: ignore[method-assign]
    return conn


def _blank_feedback() -> bytearray:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_DEBUG_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    return buf


# -- wire round trip (no Connection involved) -----------------------------------------


def test_rs2_scan_command_places_tag_id_kind_and_bus() -> None:
    frame = build_rs2_scan_command(0x70, SESSION_BEGIN, seq=5, bus=2)
    assert len(frame) == IMAGE_BYTES
    magic, = struct.unpack_from("<I", frame, 0)
    assert magic == HOST_DEBUG_COMMAND_MAGIC
    assert frame[PDU_OFF : PDU_OFF + 3] == b"RS2"
    assert frame[PDU_OFF + 3] == 0x70
    assert frame[PDU_OFF + 4] == SESSION_BEGIN
    assert frame[PDU_OFF + 11] == 2  # bus routing byte


def test_parse_probe_pdu_round_trip() -> None:
    resp = _blank_feedback()
    pdu = bytearray(32)
    pdu[0] = RS2_RESP_TAG
    pdu[1] = 0x70
    pdu[2] = 1  # found
    struct.pack_into("<f", pdu, 20, 1.25)  # position
    pdu[25] = SESSION_BEGIN
    resp[PDU_OFF : PDU_OFF + 32] = bytes(pdu)
    parsed = parse_probe_pdu(bytes(resp))
    assert parsed is not None
    assert parsed["found"] is True
    assert parsed["probe_kind"] == SESSION_BEGIN
    assert parsed["position"] == pytest.approx(1.25)


# -- DebugLease (RS2 session begin/end) -----------------------------------------------


def test_lease_sends_session_begin_then_end() -> None:
    sent_kinds = []

    def responder(frame: bytes):
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        kind = pdu[4]
        sent_kinds.append(kind)
        resp = _blank_feedback()
        rpdu = bytearray(32)
        rpdu[0] = RS2_RESP_TAG
        rpdu[25] = kind
        resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
        return bytes(resp)

    conn = _fake_connection(responder)
    with lease(conn, None, bus=2):
        pass
    assert sent_kinds == [SESSION_BEGIN, SESSION_END]


def test_lease_sends_session_end_even_on_exception() -> None:
    sent_kinds = []

    def responder(frame: bytes):
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        kind = pdu[4]
        sent_kinds.append(kind)
        resp = _blank_feedback()
        rpdu = bytearray(32)
        rpdu[0] = RS2_RESP_TAG
        rpdu[25] = kind
        resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
        return bytes(resp)

    conn = _fake_connection(responder)
    with pytest.raises(ValueError):
        with lease(conn, None, bus=1):
            raise ValueError("boom")
    assert sent_kinds == [SESSION_BEGIN, SESSION_END]


# -- CFG get (paged) -------------------------------------------------------------------


def test_fetch_table_pages_through_dual_arm_slot_count() -> None:
    page, total = 4, 14

    def responder(frame: bytes):
        pdu_in = frame[PDU_OFF : PDU_OFF + 32]
        start = pdu_in[4]
        resp = _blank_feedback()
        rpdu = bytearray(32)
        rpdu[0], rpdu[1], rpdu[2] = CFG_RESP_TAG0, CFG_RESP_TAG1, CFG_RESP_TAG2
        rpdu[3] = CFG_OP_GET
        rpdu[4] = CFG_STATUS_OK
        n = min(page, total - start)
        rpdu[5] = n
        for i in range(n):
            off = 6 + i * 3
            rpdu[off : off + 3] = bytes([1, (1 | 0x80), start + i + 1])
        rpdu[-2] = total
        rpdu[-1] = start
        resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
        return bytes(resp)

    conn = _fake_connection(responder)
    table = fetch_table(conn)
    assert len(table) == total
    assert [s["slot"] for s in table] == list(range(total))
    assert [s["motor_id"] for s in table] == list(range(1, total + 1))


# -- discover (RobStride + Damiao) ------------------------------------------------------


def test_discover_robstride_finds_target_id() -> None:
    target = 0x72

    def responder(frame: bytes):
        pdu_in = frame[PDU_OFF : PDU_OFF + 32]
        motor_id, kind = pdu_in[3], pdu_in[4]
        resp = _blank_feedback()
        rpdu = bytearray(32)
        rpdu[0] = RS2_RESP_TAG
        rpdu[1] = motor_id
        rpdu[25] = kind
        if motor_id == target and kind == PROBE_ENABLE_ONLY:
            rpdu[2] = 1
        resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
        return bytes(resp)

    conn = _fake_connection(responder)
    debug = DebugAPI(conn, None)
    hit = debug.discover_robstride(bus=2, start=0x70, end=0x75)
    assert hit == target


def test_discover_damiao_prefers_known_ids_order() -> None:
    """The ID-sweep misses; known_ids=[target] must be tried before the raw
    numeric range even though target isn't the first ID in that range."""
    target = 0x06

    def responder(frame: bytes):
        pdu_in = frame[PDU_OFF : PDU_OFF + 32]
        motor_id, kind = pdu_in[3], pdu_in[4]
        resp = _blank_feedback()
        rpdu = bytearray(32)
        rpdu[0] = DM_RESP_TAG
        rpdu[1] = motor_id
        rpdu[3] = kind
        if kind == DM_PROBE_REG_SCAN and motor_id == target:
            rpdu[2] = 1
            rpdu[24] = target
        resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
        return bytes(resp)

    conn = _fake_connection(responder)
    debug = DebugAPI(conn, None)
    hit = debug.discover_damiao(bus=1, start=1, end=8, known_ids=[target])
    assert hit == target


def test_calibrate_robstride_explicitly_not_implemented() -> None:
    """Deliberately deferred (see bench/__init__.py docstring) — must fail
    loudly with a pointer to the legacy CLI, not silently no-op."""
    conn = _fake_connection(lambda frame: None)
    debug = DebugAPI(conn, None)
    with pytest.raises(NotImplementedError, match="legacy/control_hub.py calibrate"):
        debug.calibrate_robstride(bus=2, motor_id=0x70)
