"""562 B host feedback parsers — parse wire bytes into dicts."""
from __future__ import annotations

import struct
from typing import Optional

from .pack import actuator_slot_offset
from .wire_layout import HOST_FEEDBACK_MAGIC, IMAGE_BYTES, PDU_OFF

PDU_TAG_NAMES = {
    "r": "RS2 bench reply",
    "m": "Damiao bench reply",
    "d": "Dynamixel probe",
    "S": "servo diag (SVD)",
    "u": "UART4 bridge",
    "c": "config reply",
    "t": "thermocouple bench reply",
}

PLANT_BLOCK_NAMES = {
    0: "none",
    1: "bench_session",
    2: "probe_busy",
    3: "quiet_period",
    4: "diag_only",
    5: "host_stale",
    6: "servo_session",
}


def parse_svd_plant_timing(pdu: bytes) -> Optional[dict]:
    """Superloop timing from firmware plant_timing.c.

    SVD PDU: bytes 23..28. Thermo 't' PDU: same 6 bytes at 16..21 (while
    SPI3_ROLE_THERMO owns the mailbox and would otherwise hide SVD).
    """
    if len(pdu) < 17:
        return None
    if len(pdu) >= 29 and pdu[:3] == b"SVD":
        off = 23
    elif pdu[0:1] == b"t" and len(pdu) >= 22:
        off = 16
    else:
        return None
    lap_ms = pdu[off] | (pdu[off + 1] << 8)
    lap_max_ms = pdu[off + 4] | (pdu[off + 5] << 8)
    return {
        "lap_ms": lap_ms,
        "lap_max_ms": lap_max_ms,
        "ticks_svc": pdu[off + 2],
        "ticks_pending": pdu[off + 3],
    }


def parse_thermo_led_debug(pdu: bytes) -> Optional[dict]:
    """MCP ACT LED / try_send counters from thermo 't' PDU bytes 22..31.

    Layout (wrap 255):
      22..24  mark_count[CH4..CH6]
      25..27  toggle_count[CH4..CH6]
      28..30  try_ok[CH4..CH6]
      31      try_busy[CH4]
    """
    if len(pdu) < 32 or pdu[0:1] != b"t":
        return None
    return {
        "mark_ch4": pdu[22],
        "mark_ch5": pdu[23],
        "mark_ch6": pdu[24],
        "toggle_ch4": pdu[25],
        "toggle_ch5": pdu[26],
        "toggle_ch6": pdu[27],
        "try_ok_ch4": pdu[28],
        "try_ok_ch5": pdu[29],
        "try_ok_ch6": pdu[30],
        "try_busy_ch4": pdu[31],
    }


def parse_feedback_header(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, layout_version, byte_size, fb_seq = struct.unpack_from("<IHHI", frame, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    sys_word, = struct.unpack_from("<I", frame, 12)
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    tag = chr(pdu[0]) if 32 <= pdu[0] < 127 else f"0x{pdu[0]:02X}"
    plant_block = (sys_word >> 25) & 0x7F
    timing = parse_svd_plant_timing(pdu)
    led = parse_thermo_led_debug(pdu)
    out = {
        "magic_ok": True,
        "magic_hex": f"0x{magic:08X}",
        "layout_version": layout_version,
        "byte_size": byte_size,
        "fb_seq": fb_seq,
        "tick": sys_word & 0xFFF,
        "mcu_state": (sys_word >> 13) & 0x7,
        "last_cmd_seq": (sys_word >> 17) & 0xFF,
        "plant_block": plant_block & 0x7F,
        "plant_block_name": PLANT_BLOCK_NAMES.get(plant_block & 0x7F, f"unknown({plant_block})"),
        "pdu_tag": tag,
        "pdu_tag_name": PDU_TAG_NAMES.get(tag, "unknown"),
        "pdu_head_hex": pdu[:12].hex(),
    }
    if timing is not None:
        out.update(timing)
    if led is not None:
        out["led_debug"] = led
    return out


def parse_actuator_feedback(frame: bytes, slot: int = 0) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    sys_word, = struct.unpack_from("<I", frame, 12)
    off = actuator_slot_offset(slot)
    pos, vel, torque, temp, fault = struct.unpack_from("<ffffI", frame, off)
    return {
        "tick": sys_word & 0xFFF,
        "ack": (sys_word >> 17) & 0xFF,
        "position": pos,
        "velocity": vel,
        "torque": torque,
        "temperature": temp,
        "fault": fault,
    }
