"""672 B host feedback parsers — parse wire bytes into dicts."""
from __future__ import annotations

import struct
from typing import Optional

from .pack import actuator_slot_offset
from .wire_layout import (
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_FEEDBACK_MAGIC,
    IMAGE_BYTES,
    PDU_OFF,
    SERVO0_FB_OFF,
    SERVO_SLOT_BYTES,
    SYSTEM_FB_OFF,
)

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


def parse_system_timing(frame: bytes) -> Optional[dict]:
    """Plant timing + image-id readbacks from system[32] (layout v2)."""
    if len(frame) < SYSTEM_FB_OFF + 26:
        return None
    lap_ms, lap_max_ms, ticks_svc, ticks_pending = struct.unpack_from(
        "<HHBB", frame, SYSTEM_FB_OFF + 4
    )
    last_image_id, last_applied_image_id = struct.unpack_from(
        "<II", frame, SYSTEM_FB_OFF + 18
    )
    return {
        "lap_ms": lap_ms,
        "lap_max_ms": lap_max_ms,
        "ticks_svc": ticks_svc,
        "ticks_pending": ticks_pending,
        "last_image_id": last_image_id,
        "last_applied_image_id": last_applied_image_id,
    }


def parse_svd_plant_timing(pdu: bytes) -> Optional[dict]:
    """Deprecated: timing moved to system[] in layout v2. Kept for old traces."""
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


def parse_actuator_meta(meta: int) -> dict:
    return {
        "protocol": meta & 7,
        "bus": (meta >> 3) & 7,
        "motor_id": (meta >> 6) & 0xFF,
        "enabled": bool((meta >> 14) & 1),
        "fb_valid": bool((meta >> 15) & 1),
    }


def parse_feedback_header(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, layout_version, byte_size, fb_seq = struct.unpack_from("<IHHI", frame, 0)
    if magic not in (HOST_FEEDBACK_MAGIC, HOST_DEBUG_FEEDBACK_MAGIC):
        return None
    sys_word, = struct.unpack_from("<I", frame, SYSTEM_FB_OFF)
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    tag = chr(pdu[0]) if 32 <= pdu[0] < 127 else f"0x{pdu[0]:02X}"
    plant_block = (sys_word >> 25) & 0x7F
    timing = parse_system_timing(frame)
    out = {
        "magic_ok": True,
        "magic_hex": f"0x{magic:08X}",
        "is_debug": magic == HOST_DEBUG_FEEDBACK_MAGIC,
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
    return out


def parse_actuator_feedback(frame: bytes, slot: int = 0) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic not in (HOST_FEEDBACK_MAGIC, HOST_DEBUG_FEEDBACK_MAGIC):
        return None
    sys_word, = struct.unpack_from("<I", frame, SYSTEM_FB_OFF)
    off = actuator_slot_offset(slot)
    pos, vel, torque, temp, fault, meta = struct.unpack_from("<ffffIH", frame, off)
    out = {
        "tick": sys_word & 0xFFF,
        "ack": (sys_word >> 17) & 0xFF,
        "position": pos,
        "velocity": vel,
        "torque": torque,
        "temperature": temp,
        "fault": fault,
        "meta": meta,
    }
    out.update(parse_actuator_meta(meta))
    return out


def parse_servo_feedback(frame: bytes, slot: int = 0) -> Optional[dict]:
    """Parse one host_servo_feedback_t (6 B) at SERVO0_FB_OFF + slot*6."""
    if len(frame) != IMAGE_BYTES or slot not in (0, 1):
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic not in (HOST_FEEDBACK_MAGIC, HOST_DEBUG_FEEDBACK_MAGIC):
        return None
    off = SERVO0_FB_OFF + slot * SERVO_SLOT_BYTES
    pos, speed, flags = struct.unpack_from("<hhH", frame, off)
    return {
        "present_position": pos,
        "present_speed": speed,
        "moving": bool(flags & 0x1),
        "target_reached": bool(flags & 0x2),
        "err_input_volt": bool(flags & 0x4),
        "err_overheating": bool(flags & 0x8),
        "err_overload": bool(flags & 0x10),
        "motor_source_id": (flags >> 5) & 0xFF,
        "hw_err_any": bool(flags & 0x1C),
    }
