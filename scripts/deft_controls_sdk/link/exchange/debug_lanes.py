"""Debug lanes pack/parse — ADR-004 / docs/host-contract.md.

DBGC/DBGF frames with header ``DL\\x01`` at offset 12 and 10×32 B lanes
starting at offset 18. Host builders always emit lanes. Parsers still accept
legacy offset-630 mailboxes on inbound frames (older FW replies).
"""
from __future__ import annotations

import struct
from typing import Mapping, Optional, Union

from .wire_layout import (
    DEBUG_LANES_HDR_OFF,
    DEBUG_LANE0_OFF,
    DEBUG_LANE_BYTES,
    DEBUG_LANE_CFG,
    DEBUG_LANE_COUNT,
    DEBUG_LANE_DM,
    DEBUG_LANE_RS,
    DEBUG_LANES_TAG0,
    DEBUG_LANES_TAG1,
    DEBUG_LANES_VER,
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
)


def debug_lane_offset(lane: int) -> int:
    if lane < 0 or lane >= DEBUG_LANE_COUNT:
        raise ValueError(f"debug lane must be 0..{DEBUG_LANE_COUNT - 1}, got {lane}")
    return DEBUG_LANE0_OFF + int(lane) * DEBUG_LANE_BYTES


def debug_lanes_header_present(frame: bytes) -> bool:
    if len(frame) < DEBUG_LANES_HDR_OFF + 3:
        return False
    return (
        frame[DEBUG_LANES_HDR_OFF] == DEBUG_LANES_TAG0
        and frame[DEBUG_LANES_HDR_OFF + 1] == DEBUG_LANES_TAG1
        and frame[DEBUG_LANES_HDR_OFF + 2] == DEBUG_LANES_VER
    )


def read_debug_lane(frame: bytes, lane: int) -> bytes:
    off = debug_lane_offset(lane)
    return bytes(frame[off : off + DEBUG_LANE_BYTES])


def mailbox_or_lane(frame: bytes, lane: int) -> bytes:
    """Prefer debug lane when DL header present; else legacy PDU_OFF mailbox."""
    if debug_lanes_header_present(frame):
        return read_debug_lane(frame, lane)
    return bytes(frame[PDU_OFF : PDU_OFF + DEBUG_LANE_BYTES])


def build_debug_lanes_command(
    seq: int,
    lanes: Optional[Mapping[int, Union[bytes, bytearray]]] = None,
    *,
    arm_mask: Optional[int] = None,
    flags: int = 0,
) -> bytearray:
    """Blank DBGC with DL\\x01 header; optional lane payloads.

    ``arm_mask`` defaults to bits set for every key in ``lanes``.
    """
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI",
        buf,
        0,
        HOST_DEBUG_COMMAND_MAGIC,
        HOST_LAYOUT_VERSION,
        IMAGE_BYTES,
        seq & 0xFFFFFFFF,
    )
    lanes = dict(lanes or {})
    if arm_mask is None:
        arm_mask = 0
        for lane in lanes:
            arm_mask |= 1 << int(lane)
    buf[DEBUG_LANES_HDR_OFF + 0] = DEBUG_LANES_TAG0
    buf[DEBUG_LANES_HDR_OFF + 1] = DEBUG_LANES_TAG1
    buf[DEBUG_LANES_HDR_OFF + 2] = DEBUG_LANES_VER
    buf[DEBUG_LANES_HDR_OFF + 3] = int(flags) & 0xFF
    struct.pack_into("<H", buf, DEBUG_LANES_HDR_OFF + 4, int(arm_mask) & 0xFFFF)
    for lane, payload in lanes.items():
        off = debug_lane_offset(int(lane))
        blob = bytes(payload)
        if len(blob) > DEBUG_LANE_BYTES:
            raise ValueError(f"lane {lane} payload exceeds {DEBUG_LANE_BYTES} B")
        buf[off : off + len(blob)] = blob
    return buf


def wrap_mailbox_as_debug_lanes(
    seq: int,
    mailbox: Union[bytes, bytearray],
    lane: int,
    *,
    flags: int = 0,
) -> bytes:
    """Pack a 32 B payload into one debug lane on a DBGC image (always lanes)."""
    blob = bytes(mailbox)[:DEBUG_LANE_BYTES]
    return bytes(
        build_debug_lanes_command(
            seq,
            {int(lane): blob},
            flags=flags,
        )
    )


def extract_rs2_mailbox(frame: bytes) -> bytes:
    return mailbox_or_lane(frame, DEBUG_LANE_RS)


def extract_dm_mailbox(frame: bytes) -> bytes:
    return mailbox_or_lane(frame, DEBUG_LANE_DM)


def extract_cfg_mailbox(frame: bytes) -> bytes:
    return mailbox_or_lane(frame, DEBUG_LANE_CFG)
