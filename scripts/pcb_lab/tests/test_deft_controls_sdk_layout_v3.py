"""Contract tests for host exchange layout v3 (694 B) — no hardware."""
from __future__ import annotations

import os
import struct
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.link.exchange import (
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    build_debug_command,
    build_plant_command,
)
from deft_controls_sdk.link.exchange.parse import (
    parse_actuator_feedback,
    parse_feedback_header,
    parse_system_timing,
)
from deft_controls_sdk.link.exchange.wire_layout import (
    ACTUATOR0_OFF,
    ACTUATOR_COUNT,
    ACTUATOR_SLOT_BYTES,
    DEBUG_MAILBOX_OFF,
    HOST_COMMAND_MAGIC,
    HOST_EXCHANGE_ACTUATOR_SLOTS,
    PDB_OFF,
    SYSTEM_BYTES,
)


def test_layout_constants() -> None:
    assert HOST_LAYOUT_VERSION == 3
    assert IMAGE_BYTES == 694
    assert HOST_EXCHANGE_ACTUATOR_SLOTS == ACTUATOR_COUNT == 26
    assert ACTUATOR0_OFF == 44
    assert ACTUATOR_SLOT_BYTES == 22
    assert SYSTEM_BYTES == 32
    assert PDB_OFF == 630
    assert PDU_OFF == PDB_OFF
    assert DEBUG_MAILBOX_OFF == PDU_OFF


def test_plant_command_size_and_header() -> None:
    frame = build_plant_command(7)
    assert len(frame) == IMAGE_BYTES
    magic, ver, size, seq = struct.unpack_from("<IHHI", frame, 0)
    assert magic == HOST_COMMAND_MAGIC
    assert ver == 3
    assert size == 694
    assert seq == 7


def test_system_timing_parse() -> None:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 1
    )
    struct.pack_into("<IHHBB", buf, 12, 0, 15, 40, 2, 3)
    struct.pack_into("<IIHH", buf, 12 + 18, 0xAABBCCDD, 0x11223344, 7, 12)
    timing = parse_system_timing(bytes(buf))
    assert timing is not None
    assert timing["act_lap_ms"] == 15
    assert timing["act_lap_peak_ms"] == 40
    assert timing["ticks_svc"] == 2
    assert timing["ticks_pending"] == 3
    assert timing["cmd_rx_seq"] == 0xAABBCCDD
    assert timing["cmd_applied_seq"] == 0x11223344
    assert timing["periph_lap_ms"] == 7
    assert timing["periph_lap_peak_ms"] == 12
    # legacy aliases
    assert timing["lap_ms"] == 15
    assert timing["last_image_id"] == 0xAABBCCDD
    hdr = parse_feedback_header(bytes(buf))
    assert hdr is not None
    assert hdr["act_lap_ms"] == 15
    assert hdr["cmd_rx_seq"] == 0xAABBCCDD
    assert hdr["cmd_applied_seq"] == 0x11223344
    assert hdr["periph_lap_ms"] == 7
    assert hdr["periph_lap_peak_ms"] == 12


def test_debug_command_magic() -> None:
    frame = build_debug_command(3)
    assert len(frame) == IMAGE_BYTES
    magic, ver, size, seq = struct.unpack_from("<IHHI", frame, 0)
    assert magic == HOST_DEBUG_COMMAND_MAGIC
    assert ver == 3
    assert size == 694
    assert seq == 3
    hdr = parse_feedback_header(
        struct.pack(
            "<IHHI",
            HOST_DEBUG_FEEDBACK_MAGIC,
            HOST_LAYOUT_VERSION,
            IMAGE_BYTES,
            0,
        )
        + bytes(IMAGE_BYTES - 12)
    )
    assert hdr is not None
    assert hdr["is_debug"] is True


def test_actuator_meta_slot() -> None:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    struct.pack_into("<ffffIH", buf, ACTUATOR0_OFF, 1.0, 0.0, 0.0, 0.0, 0, 0x4109)
    act = parse_actuator_feedback(bytes(buf), 0)
    assert act is not None
    assert act["position"] == 1.0
    assert "meta" in act
