"""Wire-level tests for CubeMars (CM0) / ZeroErr (ZE0) bench discover PDUs.

No hardware, no COM5 — pure byte-level pack/unpack against
App/Inc/plant/diag/diag.h and App/Src/plant/diag/diag_{cubemars,zeroerr}.c /
diag_feedback.c. Mirrors test_debug_lanes.py / test_cubemars_mit_wire.py.
"""
from __future__ import annotations

import struct

import pytest

from deft_controls_sdk.link.exchange import (
    CFG_RESP_TAG0,
    CM_PROBE_ID_SWEEP,
    CM_PROBE_MIT,
    CM_RESP_TAG,
    DEBUG_LANE_CM,
    DEBUG_LANE_DM,
    DEBUG_LANE_ZE,
    DM_RESP_TAG,
    RS2_RESP_TAG,
    SESSION_BEGIN,
    ZE_PROBE_NODE,
    ZE_PROBE_SWEEP,
    ZE_RESP_TAG,
    ZEROERR_PRODUCT_CODE,
    ZEROERR_VENDOR_ID,
    build_cm_probe_command,
    build_ze_probe_command,
    debug_lanes_header_present,
    normalize_can_bus,
    parse_cm_probe_pdu,
    parse_ze_probe_pdu,
    wrap_mailbox_as_debug_lanes,
)
from deft_controls_sdk.link.exchange.wire_layout import DEBUG_LANE0_OFF


def test_cm_resp_tag_does_not_collide_with_any_other_protocol():
    """Regression pin: plant_feedback.c routes/excludes by data[0] alone, so
    every protocol's response tag must be globally unique, not just unique
    among DM/RS2/CM. CM_RESP_TAG was originally picked as 'c', which collides
    with CFG_RESP_TAG0 ('c') — that would have silently misrouted CFG replies
    into the CubeMars lane (or vice versa)."""
    assert CM_RESP_TAG not in (CFG_RESP_TAG0, DM_RESP_TAG, RS2_RESP_TAG, ZE_RESP_TAG)
    assert ZE_RESP_TAG not in (CFG_RESP_TAG0, DM_RESP_TAG, RS2_RESP_TAG, CM_RESP_TAG)


def test_cm_probe_command_uses_its_own_debug_lane():
    frame = build_cm_probe_command(3, CM_PROBE_MIT, seq=1, bus=2, listen_ms=25, end_id=9)
    assert debug_lanes_header_present(frame)
    assert DEBUG_LANE_CM != DEBUG_LANE_DM
    assert DEBUG_LANE_CM != DEBUG_LANE_ZE

    off = DEBUG_LANE0_OFF + DEBUG_LANE_CM * 32
    lane = frame[off : off + 32]
    assert lane[0:3] == b"CM0"
    assert lane[3] == 3  # motor_id
    assert lane[4] == CM_PROBE_MIT
    assert lane[6] == 25  # listen_ms
    assert lane[8] == 9  # end_id
    assert lane[11] == normalize_can_bus(2)

    # Not written into any other lane.
    dm_off = DEBUG_LANE0_OFF + DEBUG_LANE_DM * 32
    assert frame[dm_off : dm_off + 3] != b"CM0"


def test_cm_session_begin_encodes_bus_mask():
    frame = build_cm_probe_command(0, SESSION_BEGIN, seq=1, bus=1, bus_mask=0b000101)
    off = DEBUG_LANE0_OFF + DEBUG_LANE_CM * 32
    lane = frame[off : off + 32]
    assert lane[4] == SESSION_BEGIN
    assert lane[9] == 0b000101


def _cm_response_frame(*, motor_id, found, probe_kind, discovered_id, position, fault):
    mbox = bytearray(32)
    mbox[0] = CM_RESP_TAG
    mbox[1] = motor_id & 0xFF
    mbox[2] = 1 if found else 0
    mbox[3] = probe_kind & 0xFF
    struct.pack_into("<I", mbox, 4, 0x123)  # rx_can_id
    mbox[8:16] = bytes(range(8))
    struct.pack_into("<f", mbox, 20, position)
    mbox[24] = discovered_id & 0xFF
    mbox[26] = 7  # raw_frames_seen
    mbox[27] = fault & 0xFF
    mbox[28] = 2  # tx_frames_sent
    return bytes(wrap_mailbox_as_debug_lanes(9, mbox, DEBUG_LANE_CM))


def test_parse_cm_probe_pdu_round_trip():
    frame = _cm_response_frame(
        motor_id=5, found=True, probe_kind=CM_PROBE_ID_SWEEP,
        discovered_id=5, position=1.25, fault=0,
    )
    parsed = parse_cm_probe_pdu(frame)
    assert parsed is not None
    assert parsed["probe_id"] == 5
    assert parsed["found"] is True
    assert parsed["probe_kind"] == CM_PROBE_ID_SWEEP
    assert parsed["discovered_id"] == 5
    assert parsed["position"] == pytest.approx(1.25, abs=1e-5)
    assert parsed["raw_frames"] == 7
    assert parsed["tx_frames_sent"] == 2


def test_parse_cm_probe_pdu_rejects_other_tags():
    """A CFG reply ('c'/'f'/'g') must never be mistaken for a CM reply, even
    though both protocols' command tags start with 'C'."""
    mbox = bytearray(32)
    mbox[0:3] = b"cfg"
    frame = bytes(wrap_mailbox_as_debug_lanes(1, mbox, DEBUG_LANE_CM))
    assert parse_cm_probe_pdu(frame) is None


def test_ze_probe_command_uses_its_own_debug_lane():
    frame = build_ze_probe_command(12, ZE_PROBE_SWEEP, seq=1, bus=3, sdo_timeout_ms=40, end_id=20)
    off = DEBUG_LANE0_OFF + DEBUG_LANE_ZE * 32
    lane = frame[off : off + 32]
    assert lane[0:3] == b"ZE0"
    assert lane[3] == 12
    assert lane[4] == ZE_PROBE_SWEEP
    assert lane[6] == 40
    assert lane[8] == 20
    assert lane[11] == normalize_can_bus(3)


def test_ze_probe_node_kind_default():
    frame = build_ze_probe_command(1, ZE_PROBE_NODE, seq=1, bus=1)
    off = DEBUG_LANE0_OFF + DEBUG_LANE_ZE * 32
    assert frame[off + 4] == ZE_PROBE_NODE == 0


def _ze_response_frame(*, node_id, found, vendor_match, vendor, product, revision, discovered_id):
    mbox = bytearray(32)
    mbox[0] = ZE_RESP_TAG
    mbox[1] = node_id & 0xFF
    mbox[2] = 1 if found else 0
    mbox[3] = 1 if vendor_match else 0
    struct.pack_into("<I", mbox, 4, vendor)
    struct.pack_into("<I", mbox, 8, product)
    struct.pack_into("<I", mbox, 12, revision)
    mbox[24] = discovered_id & 0xFF
    return bytes(wrap_mailbox_as_debug_lanes(3, mbox, DEBUG_LANE_ZE))


def test_parse_ze_probe_pdu_confirmed_zeroerr():
    frame = _ze_response_frame(
        node_id=4, found=True, vendor_match=True,
        vendor=ZEROERR_VENDOR_ID, product=ZEROERR_PRODUCT_CODE, revision=0x10001,
        discovered_id=4,
    )
    parsed = parse_ze_probe_pdu(frame)
    assert parsed is not None
    assert parsed["found"] is True
    assert parsed["vendor_match"] is True
    assert parsed["vendor"] == ZEROERR_VENDOR_ID
    assert parsed["product"] == ZEROERR_PRODUCT_CODE
    assert parsed["discovered_id"] == 4


def test_parse_ze_probe_pdu_unconfirmed_canopen_node():
    """A node that answers SDO 0x1018 but isn't a ZeroErr eDriver: found=True,
    vendor_match=False — still reported, not silently dropped."""
    frame = _ze_response_frame(
        node_id=9, found=True, vendor_match=False,
        vendor=0xDEADBEEF, product=0, revision=0, discovered_id=9,
    )
    parsed = parse_ze_probe_pdu(frame)
    assert parsed is not None
    assert parsed["found"] is True
    assert parsed["vendor_match"] is False


def test_parse_ze_probe_pdu_rejects_other_tags():
    mbox = bytearray(32)
    mbox[0:3] = b"DM0"
    frame = bytes(wrap_mailbox_as_debug_lanes(1, mbox, DEBUG_LANE_ZE))
    assert parse_ze_probe_pdu(frame) is None
