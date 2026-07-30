"""Unit tests for debug lanes + stm32_mode packing (ADR-004)."""
from __future__ import annotations

import struct

from deft_controls_sdk.link.exchange import (
    DEBUG_LANE_RS,
    HOST_DEBUG_COMMAND_MAGIC,
    IMAGE_BYTES,
    PDU_OFF,
    STM32_MODE_BANDWIDTH,
    STM32_MODE_DEBUG,
    STM32_MODE_SOFT_DFU,
    build_rs2_scan_command,
    build_debug_lanes_command,
    patch_system_stm32_mode,
    debug_lanes_header_present,
    wrap_mailbox_as_debug_lanes,
)
from deft_controls_sdk.link.exchange.pack import build_plant_command
from deft_controls_sdk.link.exchange.wire_layout import (
    DEBUG_LANES_HDR_OFF,
    DEBUG_LANE0_OFF,
    SYSTEM_CMD_OFF,
)


def test_debug_lanes_header_and_lane_roundtrip() -> None:
    mbox = bytearray(32)
    mbox[:3] = b"RS2"
    mbox[3] = 0x70
    mbox[4] = 12
    frame = wrap_mailbox_as_debug_lanes(7, mbox, DEBUG_LANE_RS)
    assert len(frame) == IMAGE_BYTES
    assert struct.unpack_from("<I", frame, 0)[0] == HOST_DEBUG_COMMAND_MAGIC
    assert debug_lanes_header_present(frame)
    assert frame[DEBUG_LANES_HDR_OFF : DEBUG_LANES_HDR_OFF + 3] == b"DL\x01"
    arm = struct.unpack_from("<H", frame, DEBUG_LANES_HDR_OFF + 4)[0]
    assert arm == (1 << DEBUG_LANE_RS)
    off = DEBUG_LANE0_OFF + DEBUG_LANE_RS * 32
    assert frame[off : off + 5] == bytes(mbox[:5])


def test_build_rs2_scan_uses_debug_lanes() -> None:
    frame = build_rs2_scan_command(0x70, 12, seq=3, bus=1)
    assert debug_lanes_header_present(frame)
    assert frame[DEBUG_LANE0_OFF : DEBUG_LANE0_OFF + 3] == b"RS2"


def test_builders_never_emit_legacy_mailbox() -> None:
    """Host TX always uses DL lanes — no offset-630 packing path."""
    mbox = bytearray(32)
    mbox[:3] = b"CFG"
    frame = wrap_mailbox_as_debug_lanes(1, mbox, 7)
    assert debug_lanes_header_present(frame)
    assert frame[PDU_OFF : PDU_OFF + 3] != b"CFG"


def test_parser_still_accepts_legacy_inbound_mailbox() -> None:
    from deft_controls_sdk.link.exchange.debug_lanes import mailbox_or_lane

    buf = bytearray(IMAGE_BYTES)
    buf[PDU_OFF : PDU_OFF + 3] = b"RS2"
    assert mailbox_or_lane(bytes(buf), DEBUG_LANE_RS)[:3] == b"RS2"


def test_stm32_mode_bits() -> None:
    buf = bytearray(build_plant_command(1))
    patch_system_stm32_mode(buf, STM32_MODE_DEBUG)
    word = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)[0]
    assert (word >> 9) & 3 == STM32_MODE_DEBUG
    patch_system_stm32_mode(buf, STM32_MODE_SOFT_DFU)
    word = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)[0]
    assert (word >> 9) & 3 == STM32_MODE_SOFT_DFU
    patch_system_stm32_mode(buf, STM32_MODE_BANDWIDTH)
    word = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)[0]
    assert (word >> 9) & 3 == STM32_MODE_BANDWIDTH


def test_empty_armed_debug_lanes() -> None:
    frame = bytes(build_debug_lanes_command(1, arm_mask=0))
    assert debug_lanes_header_present(frame)
    assert struct.unpack_from("<H", frame, DEBUG_LANES_HDR_OFF + 4)[0] == 0


def test_dl_header_decodes_as_soft_dfu_bits() -> None:
    """Regression: 'D','L',ver=1 at system word ⇒ stm32_mode bits == 2.

    FW must not Soft-DFU from debug-lanes frames (plant_command.c).
    """
    frame = bytes(build_debug_lanes_command(1, arm_mask=0))
    word = struct.unpack_from("<I", frame, DEBUG_LANES_HDR_OFF)[0]
    assert (word >> 9) & 3 == STM32_MODE_SOFT_DFU
