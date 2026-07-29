"""Golden-vector tests for the PDB UART frame contract (docs/pdb-uart-v1.md).

No hardware, no COM5 — pure byte-level pack/parse/CRC checks against the
controls-side C contract in App/Inc/host/pdb_link.h / App/Src/host/pdb_link.c.
"""
from __future__ import annotations

import os
import struct
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.link.exchange import IMAGE_BYTES
from deft_controls_sdk.link.exchange.wire_layout import (
    HOST_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    PDB_OFF,
    SYSTEM_FB_OFF,
)
from deft_controls_sdk.pdb import (
    FRAME_BYTES,
    KILL_HARD_ESTOP,
    KILL_NORMAL,
    KILL_REASON_COMMS_LOSS,
    KILL_REASON_NONE,
    KILL_REASON_OVERCURRENT,
    KILL_REASON_UNDERVOLTAGE,
    KILL_SOFT_READY,
    KILL_SOFT_REQ,
    MAGIC_CMD,
    MAGIC_FB,
    PdbFrameReader,
    PdbStatus,
    counts_to_amps,
    counts_to_volts,
    crc16,
    is_frame_valid,
    pack_command,
    pack_feedback,
    parse_command,
    parse_feedback,
    parse_system_kill,
    pdb_status_from_frame,
)


def test_crc16_ccitt_false_check_value():
    """Standard CRC-16/CCITT-FALSE check vector: ASCII '123456789' -> 0x29B1.
    Confirms our bit-banged crc16() matches the documented poly/init/no-reflect/
    no-final-xor semantics independent of our own pack/parse code."""
    assert crc16(b"123456789") == 0x29B1


def test_command_frame_size_and_magic():
    buf = pack_command(seq=7, rail_enable_cmd=0b0101, kill_request=0, heartbeat=42)
    assert len(buf) == FRAME_BYTES == 64
    magic, version = struct.unpack_from("<IB", buf, 0)
    assert magic == MAGIC_CMD
    assert version == 1


def test_command_round_trip():
    buf = pack_command(
        seq=200,
        rail_enable_cmd=0b1010,
        kill_request=KILL_SOFT_READY,
        heartbeat=17,
        flags=0x01,
    )
    parsed = parse_command(buf)
    assert parsed == {
        "seq": 200,
        "flags": 0x01,
        "rail_enable_cmd": 0b1010,
        "kill_request": KILL_SOFT_READY,
        "heartbeat": 17,
    }


def test_command_seq_wraps_to_uint8():
    buf = pack_command(seq=257)  # 257 & 0xFF == 1
    parsed = parse_command(buf)
    assert parsed["seq"] == 1


def test_feedback_round_trip():
    buf = pack_feedback(
        seq=3,
        pack_v=(4800, 4801, 0, 0),
        rail_v=(4800, 1900, 1200, 500),
        pack_i=(150, 0, 0, 0),
        rail_i=(10, 20, 30, 40),
        contactor_state=0b1111,
        kill_state=KILL_HARD_ESTOP,
        kill_reason=KILL_REASON_OVERCURRENT,
        estop_sense=1,
        fault_flags=0,
        heartbeat_echo=99,
    )
    parsed = parse_feedback(buf)
    assert parsed == {
        "seq": 3,
        "pack_v": (4800, 4801, 0, 0),
        "rail_v": (4800, 1900, 1200, 500),
        "pack_i": (150, 0, 0, 0),
        "rail_i": (10, 20, 30, 40),
        "contactor_state": 0b1111,
        "kill_state": KILL_HARD_ESTOP,
        "kill_reason": KILL_REASON_OVERCURRENT,
        "estop_sense": 1,
        "fault_flags": 0,
        "heartbeat_echo": 99,
    }


def test_feedback_rejects_wrong_element_count():
    import pytest

    with pytest.raises(ValueError):
        pack_feedback(pack_v=(1, 2, 3))


def test_wrong_direction_magic_is_rejected():
    """A well-formed feedback frame must not parse as a command, and vice
    versa — the two directions share no wire compatibility."""
    fb = pack_feedback(seq=1)
    cmd = pack_command(seq=1)
    assert parse_command(fb) is None
    assert parse_feedback(cmd) is None
    assert not is_frame_valid(fb, MAGIC_CMD)
    assert not is_frame_valid(cmd, MAGIC_FB)


def test_crc_reject_on_single_bit_flip():
    """Corrupting any payload byte must invalidate CRC — parse returns None,
    matching the firmware's 'no frame this cycle' fail-safe (never partially
    trust a frame whose CRC doesn't match)."""
    buf = bytearray(pack_command(seq=5, rail_enable_cmd=1, heartbeat=9))
    buf[9] ^= 0x01  # flip a bit inside kill_request
    assert parse_command(bytes(buf)) is None
    assert not is_frame_valid(bytes(buf), MAGIC_CMD)


def test_crc_reject_on_version_mismatch():
    buf = bytearray(pack_feedback(seq=1))
    buf[4] = 2  # version byte
    assert parse_feedback(bytes(buf)) is None


def test_crc_reject_on_wrong_length():
    assert parse_command(pack_command()[:-1]) is None
    assert parse_feedback(pack_feedback() + b"\x00") is None


def test_crc_stored_le_at_62_63():
    buf = pack_command(seq=1)
    crc = crc16(buf[:62])
    assert buf[62] == (crc & 0xFF)
    assert buf[63] == (crc >> 8) & 0xFF


def test_frame_reader_extracts_single_frame():
    reader = PdbFrameReader(MAGIC_FB)
    buf = pack_feedback(seq=1, heartbeat_echo=5)
    frames = reader.feed(buf)
    assert frames == [buf]


def test_frame_reader_extracts_back_to_back_frames():
    reader = PdbFrameReader(MAGIC_FB)
    f1 = pack_feedback(seq=1)
    f2 = pack_feedback(seq=2)
    frames = reader.feed(f1 + f2)
    assert frames == [f1, f2]


def test_frame_reader_handles_byte_at_a_time_feed():
    reader = PdbFrameReader(MAGIC_FB)
    buf = pack_feedback(seq=9, heartbeat_echo=3)
    frames = []
    for i in range(len(buf)):
        frames.extend(reader.feed(buf[i : i + 1]))
    assert frames == [buf]


def test_frame_reader_resyncs_past_corrupt_prefix():
    """Garbage bytes ahead of a valid frame must not stall the reader forever
    — it should slide forward to the next magic occurrence and recover,
    mirroring pdb_rx_resync() in the firmware."""
    reader = PdbFrameReader(MAGIC_FB)
    good = pack_feedback(seq=42, heartbeat_echo=7)
    junk = b"\x00\x11\x22\x33\x44\x55"
    frames = reader.feed(junk + good)
    assert frames == [good]


def test_frame_reader_drops_corrupt_frame_with_no_embedded_magic():
    reader = PdbFrameReader(MAGIC_FB)
    corrupt = bytearray(pack_feedback(seq=1))
    corrupt[10] ^= 0xFF  # corrupt payload, no magic bytes introduced
    frames = reader.feed(bytes(corrupt))
    assert frames == []


def test_si_placeholder_scales():
    assert counts_to_volts(4800) == 48.0
    assert counts_to_amps(150) == 1.5


def test_parse_system_kill_and_pdb_status_from_usb_image():
    buf = bytearray(IMAGE_BYTES)
    import struct

    struct.pack_into(
        "<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 1
    )
    buf[SYSTEM_FB_OFF + 14] = KILL_SOFT_REQ
    buf[SYSTEM_FB_OFF + 15] = KILL_REASON_OVERCURRENT
    buf[SYSTEM_FB_OFF + 16] = 1
    pdb = pack_feedback(
        seq=9,
        pack_v=(4800, 0, 0, 0),
        rail_v=(4800, 1900, 1200, 500),
        pack_i=(100, 0, 0, 0),
        rail_i=(10, 20, 30, 40),
        kill_state=KILL_SOFT_REQ,
        kill_reason=KILL_REASON_OVERCURRENT,
        estop_sense=1,
    )
    buf[PDB_OFF : PDB_OFF + FRAME_BYTES] = pdb

    sys_kill = parse_system_kill(bytes(buf))
    assert sys_kill is not None
    assert sys_kill["kill_state"] == KILL_SOFT_REQ
    assert sys_kill["kill_reason"] == KILL_REASON_OVERCURRENT
    assert sys_kill["estop_sense"] == 1
    assert sys_kill["stale_failsafe"] is False

    status = pdb_status_from_frame(bytes(buf))
    assert isinstance(status, PdbStatus)
    assert status.soft_kill_req
    assert status.pack_v_V == (48.0, 0.0, 0.0, 0.0)
    assert status.rail_v_V == (48.0, 19.0, 12.0, 5.0)
    assert status.pdb is not None
    assert status.pdb["kill_state"] == KILL_SOFT_REQ


def test_stale_failsafe_flag_on_hard_comms_loss():
    buf = bytearray(IMAGE_BYTES)
    import struct

    struct.pack_into(
        "<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    buf[SYSTEM_FB_OFF + 14] = KILL_HARD_ESTOP
    buf[SYSTEM_FB_OFF + 15] = KILL_REASON_COMMS_LOSS
    buf[SYSTEM_FB_OFF + 16] = 1
    status = pdb_status_from_frame(bytes(buf))
    assert status is not None
    assert status.stale_failsafe is True
    assert status.hard_estop
    assert status.normal is False
    assert status.kill_state_name == "hard_estop"


def test_soft_kill_ready_constant_exported():
    assert KILL_SOFT_READY == 2
    assert KILL_NORMAL == 0


# FW pdb_link_eval_kill() USB overlay — uses shared host mirror of the C policy.
from deft_controls_sdk.pdb.limits import pdb_vi_reject_reason as _fw_vi_reject_reason


def _fw_usb_kill_overlay(fb: dict, *, fresh: bool) -> tuple[int, int]:
    """Mirror pdb_link_eval_kill() USB system kill presentation."""
    if not fresh:
        return KILL_HARD_ESTOP, KILL_REASON_COMMS_LOSS
    peer_state = int(fb["kill_state"])
    peer_reason = int(fb["kill_reason"])
    if peer_state != KILL_NORMAL:
        return peer_state, peer_reason
    vi = _fw_vi_reject_reason(fb)
    if vi != KILL_REASON_NONE:
        return KILL_SOFT_REQ, vi
    return peer_state, peer_reason


def test_fw_vi_overlay_pack_undervoltage_soft_kill():
    fb = parse_feedback(
        pack_feedback(
            pack_v=(3900, 0, 0, 0),  # 39.0 V — below 40 V
            rail_v=(4800, 1900, 1200, 500),
            pack_i=(100, 0, 0, 0),
            rail_i=(10, 20, 30, 40),
            contactor_state=0b1111,
            kill_state=KILL_NORMAL,
            kill_reason=0,
        )
    )
    assert _fw_usb_kill_overlay(fb, fresh=True) == (
        KILL_SOFT_REQ,
        KILL_REASON_UNDERVOLTAGE,
    )


def test_fw_vi_overlay_overcurrent_beats_uv():
    fb = parse_feedback(
        pack_feedback(
            pack_v=(3900, 0, 0, 0),
            rail_v=(4800, 1900, 1200, 500),
            pack_i=(3100, 0, 0, 0),  # 31 A
            rail_i=(10, 20, 30, 40),
            contactor_state=0b1111,
            kill_state=KILL_NORMAL,
            kill_reason=0,
        )
    )
    assert _fw_usb_kill_overlay(fb, fresh=True) == (
        KILL_SOFT_REQ,
        KILL_REASON_OVERCURRENT,
    )


def test_fw_vi_overlay_skips_zero_pack_slots():
    fb = parse_feedback(
        pack_feedback(
            pack_v=(4800, 0, 0, 0),  # unused packs at 0 must not UV
            rail_v=(4800, 1900, 1200, 500),
            pack_i=(100, 0, 0, 0),
            rail_i=(10, 20, 30, 40),
            contactor_state=0b1111,
            kill_state=KILL_NORMAL,
            kill_reason=0,
        )
    )
    assert _fw_usb_kill_overlay(fb, fresh=True) == (KILL_NORMAL, 0)


def test_fw_vi_overlay_stale_still_comms_loss():
    fb = parse_feedback(
        pack_feedback(
            pack_v=(3900, 0, 0, 0),
            rail_v=(4800, 0, 0, 0),
            kill_state=KILL_NORMAL,
            kill_reason=0,
        )
    )
    assert _fw_usb_kill_overlay(fb, fresh=False) == (
        KILL_HARD_ESTOP,
        KILL_REASON_COMMS_LOSS,
    )


def test_fw_vi_overlay_does_not_demote_peer_hard():
    fb = parse_feedback(
        pack_feedback(
            pack_v=(3900, 0, 0, 0),
            rail_v=(4800, 0, 0, 0),
            kill_state=KILL_HARD_ESTOP,
            kill_reason=KILL_REASON_OVERCURRENT,
        )
    )
    assert _fw_usb_kill_overlay(fb, fresh=True) == (
        KILL_HARD_ESTOP,
        KILL_REASON_OVERCURRENT,
    )
