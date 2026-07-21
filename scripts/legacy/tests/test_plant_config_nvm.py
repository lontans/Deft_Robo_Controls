"""CFG PDU pack/parse tests (no hardware)."""
from __future__ import annotations

import struct

from controls_pcb_host.commands import build_cfg_command
from controls_pcb_host.feedback import parse_cfg_feedback
from controls_pcb_host.protocol import (
    CFG_OP_GET,
    CFG_OP_SET,
    CFG_RESP_TAG0,
    CFG_RESP_TAG1,
    CFG_RESP_TAG2,
    CFG_STATUS_OK,
    CFG_TAG0,
    CFG_TAG1,
    CFG_TAG2,
    IMAGE_BYTES,
    PDU_OFF,
)


def test_build_cfg_get_command() -> None:
    buf = build_cfg_command(CFG_OP_GET, seq=7)
    assert len(buf) == IMAGE_BYTES
    assert buf[PDU_OFF + 0] == CFG_TAG0
    assert buf[PDU_OFF + 1] == CFG_TAG1
    assert buf[PDU_OFF + 2] == CFG_TAG2
    assert buf[PDU_OFF + 3] == CFG_OP_GET


def test_build_cfg_set_command() -> None:
    buf = build_cfg_command(
        CFG_OP_SET,
        seq=1,
        slot=4,
        bus=5,
        protocol=1,
        motor_id=0x70,
        enabled=True,
    )
    assert buf[PDU_OFF + 4] == 4
    assert buf[PDU_OFF + 8] == 5
    assert buf[PDU_OFF + 9] == 1
    assert buf[PDU_OFF + 10] == 0x70
    assert buf[PDU_OFF + 12] == 1


def test_parse_cfg_feedback_roundtrip() -> None:
    pdu = bytearray(32)
    pdu[0] = CFG_RESP_TAG0
    pdu[1] = CFG_RESP_TAG1
    pdu[2] = CFG_RESP_TAG2
    pdu[3] = CFG_OP_GET | 0x80
    pdu[4] = CFG_STATUS_OK
    pdu[5] = 2
    # 3 B/slot: bus, protocol|(enabled<<7), motor_id
    pdu[6:9] = bytes([4, 0x81, 0x73])
    pdu[9:12] = bytes([5, 0x81, 0x70])
    parsed = parse_cfg_feedback(bytes(pdu))
    assert parsed is not None
    assert parsed["status_name"] == "ok"
    assert len(parsed["slots"]) == 2
    assert parsed["slots"][0] == {
        "slot": 0,
        "bus": 4,
        "protocol": 1,
        "motor_id": 0x73,
        "enabled": True,
    }


def test_parse_cfg_feedback_seven_slots() -> None:
    pdu = bytearray(32)
    pdu[0] = CFG_RESP_TAG0
    pdu[1] = CFG_RESP_TAG1
    pdu[2] = CFG_RESP_TAG2
    pdu[3] = CFG_OP_GET | 0x80
    pdu[4] = CFG_STATUS_OK
    pdu[5] = 7
    for i in range(7):
        off = 6 + i * 3
        pdu[off] = 1
        pdu[off + 1] = 0x80 | 3  # damiao + enabled
        pdu[off + 2] = i + 1
    parsed = parse_cfg_feedback(bytes(pdu))
    assert parsed is not None
    assert parsed["slot_count"] == 7
    assert len(parsed["slots"]) == 7
    assert parsed["slots"][6]["motor_id"] == 7
    assert parsed["slots"][6]["protocol"] == 3
    assert parsed["slots"][6]["enabled"] is True
