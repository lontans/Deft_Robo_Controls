"""Tagged-pdu (DEBUG mode) wire helpers — RS2 / DM0 / CFG bench PDU tags.

Mirrors `App/Inc/plant/plant_diag.h` and `plant_config_nvm.h`. Ported from
scripts/pcb_lab/legacy/controls_pcb_host/protocol/diag_pdu.py + commands.py +
feedback.py — kept byte-for-byte identical to the legacy encode/decode so a
DEBUG-mode probe behaves the same whether it goes through the SDK or the
legacy CLI. Plant-path pack/parse stays in pack.py/parse.py; this module is
only imported by `deft_controls_sdk/debug/`.
"""
from __future__ import annotations

import struct
from typing import Dict, Optional

from .pack import patch_actuator_desire, patch_system_mcu_state
from .parse import parse_actuator_feedback
from .wire_layout import (
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    MAX_CAN_BUS,
    MIN_CAN_BUS,
    PDU_OFF,
    PLANT_MCU_STATE_DIAG_ONLY,
    PLANT_MCU_STATE_NORMAL,
)

# -- CAN bus routing (pdu.data[11] = schematic branch 1..6) ---------------------------

PDU_BUS_OFF = PDU_OFF + 11

MCP_CAN_BUSES = frozenset({4, 5, 6})

_CAN_PINS: Dict[int, str] = {
    1: "PB8/9 FDCAN1",
    2: "PA8/PA15 FDCAN3",
    3: "PB12/13 FDCAN2",
    4: "PB11 MCP SPI-CAN",
    5: "PB1 MCP SPI-CAN",
    6: "PA4 MCP SPI-CAN",
}


def normalize_can_bus(bus: int) -> int:
    b = int(bus)
    if b < MIN_CAN_BUS or b > MAX_CAN_BUS:
        raise ValueError(f"CAN bus must be {MIN_CAN_BUS}..{MAX_CAN_BUS}, got {b}")
    return b


def is_mcp_bus(bus: int) -> bool:
    return normalize_can_bus(bus) in MCP_CAN_BUSES


def can_bus_label(bus: int) -> str:
    b = normalize_can_bus(bus)
    return f"CH{b} ({_CAN_PINS[b]})"


def build_debug_command(seq: int) -> bytearray:
    """Blank DEBUG command image (DBGC) — mailbox at PDU_OFF / pdb[0..31]."""
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
    return buf


# Internal alias used by RS2/DM/CFG builders in this module.
_blank_command = build_debug_command


def _patch_pdu_bus(buf: bytearray, bus: int) -> None:
    buf[PDU_BUS_OFF] = normalize_can_bus(bus) & 0xFF


# -- RS2 bench PDU (RobStride discover / probe / session) ----------------------------

RS2_TAG0, RS2_TAG1, RS2_TAG2 = ord("R"), ord("S"), ord("2")
RS2_RESP_TAG = ord("r")

PROBE_FULL = 0
PROBE_ENABLE_CTRL = 1
PROBE_CTRL_ONLY = 2
PROBE_PROMISC = 10
PROBE_RESET = 11
PROBE_ENABLE_ONLY = 12
PROBE_CTRL_FAST = 13
PROBE_PARAREAD = 14
PROBE_PROACTIVE = 15
PROBE_CALI = 16
PROBE_ZERO = 17
PROBE_DATA_SAVE = 18
PROBE_PARAWRITE = 19

SESSION_BEGIN = 254
SESSION_END = 255


def build_rs2_scan_command(motor_id: int, probe_kind: int, seq: int, bus: int = 1) -> bytes:
    buf = _blank_command(seq)
    buf[PDU_OFF + 0] = RS2_TAG0
    buf[PDU_OFF + 1] = RS2_TAG1
    buf[PDU_OFF + 2] = RS2_TAG2
    buf[PDU_OFF + 3] = motor_id & 0xFF
    buf[PDU_OFF + 4] = probe_kind & 0xFF
    _patch_pdu_bus(buf, bus)
    return bytes(buf)


def build_rs2_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    param_index: int = 0,
    param_raw_value: int = 0,
    *,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    bus: int = 1,
) -> bytes:
    """Scan command + optional param bytes + optional actuator desire.

    ``param_index`` / ``param_raw_value`` map to pdu[5..10] (pararead/write, cali
    listen seconds, etc.). Desire is mounted for PROBE_ENABLE_CTRL / CTRL_ONLY /
    FULL when kp/kd/position are non-zero.
    """
    buf = bytearray(build_rs2_scan_command(motor_id, probe_kind, seq, bus=bus))
    if position != 0.0 or velocity != 0.0 or kp != 0.0 or kd != 0.0:
        patch_actuator_desire(buf, position=position, velocity=velocity, kp=kp, kd=kd)
    buf[PDU_OFF + 5] = param_index & 0xFF
    buf[PDU_OFF + 6] = (param_index >> 8) & 0xFF
    buf[PDU_OFF + 7] = param_raw_value & 0xFF
    buf[PDU_OFF + 8] = (param_raw_value >> 8) & 0xFF
    buf[PDU_OFF + 9] = (param_raw_value >> 16) & 0xFF
    buf[PDU_OFF + 10] = (param_raw_value >> 24) & 0xFF
    return bytes(buf)


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
    }


# -- Damiao DM0 bench PDU (discover) --------------------------------------------------

DM_TAG0, DM_TAG1, DM_TAG2 = ord("D"), ord("M"), ord("0")
DM_RESP_TAG = ord("m")

DM_PROBE_MIT = 0
DM_PROBE_ENABLE = 11
DM_PROBE_DISABLE = 12
DM_PROBE_CLEAR_FAULT = 13
DM_PROBE_REG_SCAN = 16
DM_PROBE_ID_SWEEP = 17

DM_REG_ESC_ID = 0x08
DM_MASTER_ANY = 0xFF

DM_FB_MAGIC = 0xDA000000
DM_FB_MAGIC_FOUND_OLD = 0xDB000000

DM_PROBE_KIND_NAMES = {
    DM_PROBE_MIT: "mit",
    DM_PROBE_ENABLE: "enable",
    DM_PROBE_DISABLE: "disable",
    DM_PROBE_CLEAR_FAULT: "clear_fault",
    DM_PROBE_REG_SCAN: "reg_scan",
    DM_PROBE_ID_SWEEP: "id_sweep",
}


def build_dm_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    *,
    bus: int = 1,
    master_id: int = DM_MASTER_ANY,
    listen_ms: int = 15,
    param_rid: int = DM_REG_ESC_ID,
    end_id: int = 0,
) -> bytes:
    buf = _blank_command(seq)
    patch_system_mcu_state(buf, PLANT_MCU_STATE_DIAG_ONLY)
    buf[PDU_OFF + 0] = DM_TAG0
    buf[PDU_OFF + 1] = DM_TAG1
    buf[PDU_OFF + 2] = DM_TAG2
    buf[PDU_OFF + 3] = motor_id & 0xFF
    buf[PDU_OFF + 4] = probe_kind & 0xFF
    buf[PDU_OFF + 5] = master_id & 0xFF
    buf[PDU_OFF + 6] = listen_ms & 0xFF
    buf[PDU_OFF + 7] = param_rid & 0xFF
    buf[PDU_OFF + 8] = end_id & 0xFF
    _patch_pdu_bus(buf, bus)
    return bytes(buf)


def parse_dm_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    if pdu[0] != DM_RESP_TAG:
        return None
    rx_can_id, = struct.unpack_from("<I", pdu, 4)
    param_value, = struct.unpack_from("<I", pdu, 16)
    position, = struct.unpack_from("<f", pdu, 20)
    return {
        "probe_id": pdu[1],
        "found": pdu[2] != 0,
        "probe_kind": pdu[3],
        "rx_can_id": rx_can_id,
        "can_data": bytes(pdu[8:16]),
        "param_value": param_value,
        "position": position,
        "discovered_id": pdu[24],
        "master_id": pdu[25],
        "raw_frames": pdu[26],
        "err": pdu[27],
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
    """Damiao register-scan replies can also land in actuator_feedback[slot] (not just
    the pdu region) — see App/Src/plant/plugins/damiao.c. Same fault-word encoding as
    parse_dm_probe_pdu, decoded from the plant feedback slot instead."""
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
    return {
        "probe_id": probed,
        "found": dm_fault_found(fault),
        "probe_kind": DM_PROBE_REG_SCAN,
        "can_data": b"",
        "param_value": esc_id,
        "position": act["position"],
        "discovered_id": esc_id,
        "master_id": int(act["temperature"]) & 0xFF,
        "raw_frames": 0,
        "err": 0,
    }


def probe_kind_matches(resp_kind: int, expect_kind: Optional[int]) -> bool:
    if expect_kind is None:
        return True
    if expect_kind == resp_kind:
        return True
    if expect_kind == DM_PROBE_REG_SCAN and resp_kind in (16, 14):  # REG_SCAN / READ_PARAM
        return True
    return False


# -- CFG bench PDU (actuator table get/set/save) --------------------------------------

CFG_TAG0, CFG_TAG1, CFG_TAG2 = ord("C"), ord("F"), ord("G")
CFG_RESP_TAG0, CFG_RESP_TAG1, CFG_RESP_TAG2 = ord("c"), ord("f"), ord("g")

CFG_OP_GET = 1
CFG_OP_SET = 2
CFG_OP_SAVE = 3

CFG_STATUS_OK = 0
CFG_STATUS_BAD_ARG = 1
CFG_STATUS_FLASH_ERR = 2
CFG_STATUS_BAD_CRC = 3

CFG_STATUS_NAMES = {
    CFG_STATUS_OK: "ok",
    CFG_STATUS_BAD_ARG: "bad_arg",
    CFG_STATUS_FLASH_ERR: "flash_err",
    CFG_STATUS_BAD_CRC: "bad_crc",
}


def build_cfg_command(
    op: int,
    seq: int,
    *,
    slot: int = 0,
    bus: int = 1,
    protocol: int = 0,
    motor_id: int = 0,
    master_id: int = 0,
    enabled: bool = True,
    mcu_state: int = PLANT_MCU_STATE_NORMAL,
) -> bytes:
    buf = _blank_command(seq)
    patch_system_mcu_state(buf, mcu_state)
    buf[PDU_OFF + 0] = CFG_TAG0
    buf[PDU_OFF + 1] = CFG_TAG1
    buf[PDU_OFF + 2] = CFG_TAG2
    buf[PDU_OFF + 3] = op & 0xFF
    buf[PDU_OFF + 4] = slot & 0xFF
    buf[PDU_OFF + 8] = bus & 0xFF
    buf[PDU_OFF + 9] = protocol & 0xFF
    buf[PDU_OFF + 10] = motor_id & 0xFF
    buf[PDU_OFF + 11] = master_id & 0xFF
    buf[PDU_OFF + 12] = 1 if enabled else 0
    return bytes(buf)


def parse_cfg_feedback(pdu: bytes) -> Optional[dict]:
    """Parse a CFG response pdu (32 B slice at PDU_OFF, not the full 672 B frame).

    Paginated when ACTUATOR_COUNT exceeds one PDU's worth of slots (dual-arm, 14
    actuators): header (0..4) + count (5) + up to N slot records, 3 bytes each
    ([bus][protocol|(enabled<<7)][motor_id]), + trailer [total_count][start_slot]
    in the last 2 bytes. Mirrors plant_config_feedback_fill() in
    App/Src/plant/plant_config_nvm.c.
    """
    if len(pdu) < 8:
        return None
    if pdu[0] != CFG_RESP_TAG0 or pdu[1] != CFG_RESP_TAG1 or pdu[2] != CFG_RESP_TAG2:
        return None
    op = pdu[3] & 0x7F
    status = pdu[4]
    count = pdu[5]
    total_count = pdu[-2]
    start_slot = pdu[-1]
    max_fit = (len(pdu) - 6 - 2) // 3
    slots = []
    for i in range(min(count, max_fit)):
        off = 6 + i * 3
        bus = pdu[off]
        packed = pdu[off + 1]
        motor_id = pdu[off + 2]
        slots.append(
            {
                "slot": start_slot + i,
                "bus": bus,
                "protocol": packed & 0x7F,
                "motor_id": motor_id,
                "enabled": bool(packed & 0x80),
            }
        )
    return {
        "op": op,
        "status": status,
        "status_name": CFG_STATUS_NAMES.get(status, f"unknown({status})"),
        "slot_count": count,
        "total_count": total_count,
        "start_slot": start_slot,
        "slots": slots,
    }
