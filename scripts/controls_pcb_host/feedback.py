"""562 B host feedback image parsers."""
from __future__ import annotations

import struct
from typing import List, Optional

from .commands import actuator_slot_offset, servo_slot_cmd_offset
from .protocol import (
    CFG_RESP_TAG0,
    CFG_RESP_TAG1,
    CFG_RESP_TAG2,
    CFG_STATUS_NAMES,
    DM_FB_MAGIC,
    DM_FB_MAGIC_FOUND_OLD,
    DM_PROBE_REG_SCAN,
    DM_RESP_TAG,
    DXL_RESP_TAG,
    HOST_FEEDBACK_MAGIC,
    IMAGE_BYTES,
    PDU_OFF,
    RS2_RESP_TAG,
    SERVO_SLOT_BYTES,
    SESSION_BEGIN,
    SESSION_END,
    UB_FB_MAX_BYTES,
    UB_RESP_TAG,
)


PDU_TAG_NAMES = {
    "r": "RS2 bench reply",
    "m": "Damiao bench reply",
    "d": "Dynamixel probe",
    "S": "servo diag (SVD)",
    "u": "UART4 bridge",
    "c": "config reply",
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
    """Bytes 23..28 in SVD PDU — superloop timing from firmware plant_timing.c."""
    if len(pdu) < 29 or pdu[:3] != b"SVD":
        return None
    lap_ms = pdu[23] | (pdu[24] << 8)
    lap_max_ms = pdu[27] | (pdu[28] << 8)
    return {
        "lap_ms": lap_ms,
        "lap_max_ms": lap_max_ms,
        "ticks_svc": pdu[25],
        "ticks_pending": pdu[26],
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
    return out


def parse_cfg_feedback(pdu: bytes) -> Optional[dict]:
    """Parse CFG response PDU (cf g) from firmware plant_config_nvm."""
    if len(pdu) < 6:
        return None
    if pdu[0] != CFG_RESP_TAG0 or pdu[1] != CFG_RESP_TAG1 or pdu[2] != CFG_RESP_TAG2:
        return None
    op = pdu[3] & 0x7F
    status = pdu[4]
    count = pdu[5]
    slots: List[dict] = []
    for i in range(min(count, 6)):
        off = 6 + i * 4
        if off + 4 > len(pdu):
            break
        bus = pdu[off]
        proto = pdu[off + 1]
        motor_id = pdu[off + 2]
        flags = pdu[off + 3]
        slots.append(
            {
                "slot": i,
                "bus": bus,
                "protocol": proto,
                "motor_id": motor_id,
                "enabled": bool(flags & 1),
            }
        )
    return {
        "op": op,
        "status": status,
        "status_name": CFG_STATUS_NAMES.get(status, f"unknown({status})"),
        "slot_count": count,
        "slots": slots,
    }


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


def parse_servo_feedback(frame: bytes, slot: int = 0) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    off = servo_slot_cmd_offset(slot)
    if off + SERVO_SLOT_BYTES > IMAGE_BYTES:
        return None
    present, speed, flags = struct.unpack_from("<hhH", frame, off)
    return {
        "present_position": present,
        "present_speed": speed,
        "motor_source_id": (flags >> 5) & 0xFF,
        "moving": flags & 1,
    }


def parse_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    if pdu[0] != RS2_RESP_TAG:
        return None
    ext_id, = struct.unpack_from("<I", pdu, 4)
    temperature, position = struct.unpack_from("<ff", pdu, 16)
    return {
        "probe_id": pdu[1],
        "found": pdu[2] != 0,
        "comm_mode": pdu[3],
        "ext_id": ext_id,
        "can_data": bytes(pdu[8:16]),
        "temperature": temperature,
        "position": position,
        "discovered_id": pdu[24],
        "probe_kind": pdu[25],
        "raw_frames": pdu[26],
        "mcp_init_mask": pdu[27],
        "mcp_rail_opmod": pdu[28],
        "mcp_tx_ok": pdu[29],
        "mcp_tx_fail": pdu[30],
        "mcp_tec": pdu[31],
    }


def parse_dm_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    if pdu[0] != DM_RESP_TAG:
        return None
    rx_can_id, = struct.unpack_from("<I", pdu, 4)
    param_value, = struct.unpack_from("<I", pdu, 16)
    position, = struct.unpack_from("<f", pdu, 20)
    can_data = bytes(pdu[8:16])
    temperature = float(can_data[6]) if len(can_data) > 6 else 0.0
    return {
        "probe_id": pdu[1],
        "found": pdu[2] != 0,
        "probe_kind": pdu[3],
        "rx_can_id": rx_can_id,
        "can_data": can_data,
        "param_value": param_value,
        "param_rid": pdu[31],
        "temperature": temperature,
        "position": position,
        "discovered_id": pdu[24],
        "master_id": pdu[25],
        "raw_frames": pdu[26],
        "err": pdu[27],
        "tx_frames": pdu[28],
    }


def dm_fault_is_probe(fault: int) -> bool:
    top = fault & 0xFF000000
    return top in (DM_FB_MAGIC, DM_FB_MAGIC_FOUND_OLD)


def dm_fault_found(fault: int) -> bool:
    top = fault & 0xFF000000
    if top == DM_FB_MAGIC_FOUND_OLD:
        return True
    return bool((fault >> 23) & 1)


def parse_dm_from_actuator(frame: bytes, motor_id: int, slot: int = 2) -> Optional[dict]:
    act = parse_actuator_feedback(frame, slot)
    if act is None:
        return None
    fault = act["fault"]
    if not dm_fault_is_probe(fault):
        return None
    probed = int(act["velocity"]) & 0xFF
    if probed != (motor_id & 0xFF):
        return None
    esc_id = int(act["torque"]) & 0xFF
    master_id = int(act["temperature"]) & 0xFF
    return {
        "probe_id": probed,
        "found": dm_fault_found(fault),
        "probe_kind": DM_PROBE_REG_SCAN,
        "rx_can_id": 0,
        "can_data": b"",
        "param_value": esc_id,
        "param_rid": 0x08,
        "temperature": act["temperature"],
        "position": act["position"],
        "discovered_id": esc_id,
        "master_id": master_id,
        "raw_frames": (fault >> 8) & 0xFF,
        "err": fault & 0xFF,
        "tx_frames": (fault >> 16) & 0xFF,
        "via_actuator_slot": True,
    }


def probe_kind_matches(resp_kind: int, expect_kind: Optional[int]) -> bool:
    if expect_kind is None:
        return True
    if expect_kind == resp_kind:
        return True
    if expect_kind == 16 and resp_kind in (16, 14):  # REG_SCAN / READ_PARAM
        return True
    if expect_kind == 15 and resp_kind in (15, 11, 0, 1, 2, 3):  # DISCOVER family
        return True
    return False


def parse_ub_feedback(frame: bytes) -> Optional[bytes]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    if pdu[0] != UB_RESP_TAG:
        return None
    n = min(pdu[1], UB_FB_MAX_BYTES)
    return bytes(pdu[2 : 2 + n])


def format_status_line(hdr: dict, actuator_slots: Optional[List[dict]] = None) -> str:
    parts = [
        f"tick={hdr['tick']}",
        f"mcu_state={hdr['mcu_state']}",
        f"ack_seq={hdr['last_cmd_seq']}",
        f"plant_block={hdr.get('plant_block_name', '?')}",
        f"pdu={hdr['pdu_tag']}",
    ]
    line = "  ".join(parts)
    if actuator_slots:
        for i, act in enumerate(actuator_slots):
            if act is None:
                continue
            line += (
                f"  | slot{i}: pos={act['position']:+.4f}"
                f" err/fault=0x{act['fault'] & 0xFF:X}"
            )
    return line
