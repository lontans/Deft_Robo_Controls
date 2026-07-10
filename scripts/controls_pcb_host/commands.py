"""562 B host command image builders."""
from __future__ import annotations

import struct
from typing import Dict, Optional, Tuple

from .protocol import (
    ACTUATOR0_OFF,
    ACTUATOR_SLOT_BYTES,
    CFG_OP_DEFAULTS,
    CFG_OP_GET,
    CFG_OP_LOAD,
    CFG_OP_SAVE,
    CFG_OP_SET,
    CFG_RESP_TAG0,
    CFG_TAG0,
    CFG_TAG1,
    CFG_TAG2,
    DM_TAG0,
    DM_TAG1,
    DM_TAG2,
    DXL_TAG0,
    DXL_TAG1,
    DXL_TAG2,
    HOST_COMMAND_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    LED_CMD_OFF,
    PDU_BUS_OFF,
    PDU_OFF,
    PLANT_MCU_STATE_DIAG_ONLY,
    PLANT_MCU_STATE_NORMAL,
    RS2_TAG0,
    RS2_TAG1,
    RS2_TAG2,
    SERVO0_CMD_OFF,
    SERVO_SLOT_BYTES,
    SYSTEM_CMD_OFF,
    UB_CMD_MAX_BYTES,
    UB_TAG0,
    UB_TAG1,
)
from .protocol.can_bus import normalize_can_bus


def actuator_slot_offset(slot: int) -> int:
    return ACTUATOR0_OFF + slot * ACTUATOR_SLOT_BYTES


def patch_system_mcu_state(buf: bytearray, mcu_state: int) -> None:
    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~0x0E) | ((int(mcu_state) & 7) << 1)
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def patch_pdu_bus(buf: bytearray, bus: int) -> None:
    buf[PDU_BUS_OFF] = normalize_can_bus(bus) & 0xFF


def patch_actuator_desire(
    buf: bytearray,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    torque: float = 0.0,
    slot: int = 0,
) -> None:
    struct.pack_into(
        "<fffff",
        buf,
        actuator_slot_offset(slot),
        position,
        velocity,
        kp,
        kd,
        torque,
    )


def _blank_command(seq: int) -> bytearray:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    return buf


def build_plant_command(
    seq: int,
    slot_commands: Optional[Dict[int, Tuple[float, float, float, float, float]]] = None,
    mcu_state: int = PLANT_MCU_STATE_NORMAL,
) -> bytes:
    buf = _blank_command(seq)
    patch_system_mcu_state(buf, mcu_state)
    if slot_commands:
        for slot, (pos, vel, kp, kd, tau) in slot_commands.items():
            patch_actuator_desire(buf, pos, vel, kp, kd, tau, slot=slot)
    return bytes(buf)


def build_actuator_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    seq: int,
    slot: int = 0,
    mcu_state: int = PLANT_MCU_STATE_NORMAL,
) -> bytes:
    buf = _blank_command(seq)
    patch_system_mcu_state(buf, mcu_state)
    patch_actuator_desire(buf, position, velocity, kp, kd, torque, slot=slot)
    return bytes(buf)


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


def build_rs2_scan_command(motor_id: int, probe_kind: int, seq: int, bus: int = 1) -> bytes:
    buf = _blank_command(seq)
    buf[PDU_OFF + 0] = RS2_TAG0
    buf[PDU_OFF + 1] = RS2_TAG1
    buf[PDU_OFF + 2] = RS2_TAG2
    buf[PDU_OFF + 3] = motor_id & 0xFF
    buf[PDU_OFF + 4] = probe_kind & 0xFF
    patch_pdu_bus(buf, bus)
    return bytes(buf)


def build_rs2_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    param_index: int = 0,
    param_raw_value: int = 0,
    position: float = 0.0,
    kp: float = 0.0,
    kd: float = 1.0,
    bus: int = 1,
) -> bytes:
    buf = bytearray(build_rs2_scan_command(motor_id, probe_kind, seq, bus=bus))
    if position != 0.0 or kp != 0.0 or kd != 0.0:
        patch_actuator_desire(buf, position=position, kp=kp, kd=kd)
    buf[PDU_OFF + 5] = param_index & 0xFF
    buf[PDU_OFF + 6] = (param_index >> 8) & 0xFF
    buf[PDU_OFF + 7] = param_raw_value & 0xFF
    buf[PDU_OFF + 8] = (param_raw_value >> 8) & 0xFF
    buf[PDU_OFF + 9] = (param_raw_value >> 16) & 0xFF
    buf[PDU_OFF + 10] = (param_raw_value >> 24) & 0xFF
    return bytes(buf)


def build_dm_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    bus: int = 3,
    master_id: int = 0,
    listen_ms: int = 15,
    param_rid: int = 0x08,
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
    patch_pdu_bus(buf, bus)
    return bytes(buf)


def build_dxl_probe_command(
    seq: int,
    target_id: int,
    kind: int,
    id_start: int = 1,
    id_end: int = 20,
) -> bytes:
    buf = _blank_command(seq)
    patch_system_mcu_state(buf, PLANT_MCU_STATE_DIAG_ONLY)
    buf[PDU_OFF + 0] = DXL_TAG0
    buf[PDU_OFF + 1] = DXL_TAG1
    buf[PDU_OFF + 2] = DXL_TAG2
    buf[PDU_OFF + 3] = target_id & 0xFF
    buf[PDU_OFF + 4] = kind & 0xFF
    buf[PDU_OFF + 5] = id_start & 0xFF
    buf[PDU_OFF + 6] = id_end & 0xFF
    return bytes(buf)


def pack_led_word(mode: int, brightness: int, led_count: int) -> int:
    return (
        (mode & 0x1F)
        | ((brightness & 0x1F) << 5)
        | ((led_count & 0x3F) << 10)
    )


def build_led_command(seq: int, mode: int, brightness: int, led_count: int) -> bytes:
    buf = _blank_command(seq)
    struct.pack_into("<H", buf, LED_CMD_OFF, pack_led_word(mode, brightness, led_count))
    return bytes(buf)


def build_ub_command(chunk: bytes, seq: int) -> bytes:
    buf = _blank_command(seq)
    patch_system_mcu_state(buf, PLANT_MCU_STATE_DIAG_ONLY)
    n = min(len(chunk), UB_CMD_MAX_BYTES)
    buf[PDU_OFF + 0] = UB_TAG0
    buf[PDU_OFF + 1] = UB_TAG1
    buf[PDU_OFF + 2] = n
    buf[PDU_OFF + 3 : PDU_OFF + 3 + n] = chunk[:n]
    return bytes(buf)


def servo_slot_cmd_offset(slot: int) -> int:
    return SERVO0_CMD_OFF + slot * SERVO_SLOT_BYTES


def pack_servo_cmd_flags(
    servo_id: int,
    torque_enable: int = 1,
    operating_mode: int = 3,
) -> int:
    return (
        (torque_enable & 1)
        | ((operating_mode & 7) << 2)
        | ((servo_id & 0xFF) << 5)
    )


def patch_servo_command(
    buf: bytearray,
    slot: int,
    position: int,
    servo_id: int,
    torque_enable: int = 1,
    speed: int = 0,
    operating_mode: int = 3,
) -> None:
    off = servo_slot_cmd_offset(slot)
    flags = pack_servo_cmd_flags(servo_id, torque_enable, operating_mode)
    struct.pack_into("<hhH", buf, off, position, speed, flags)


def build_plant_servo_command(
    seq: int,
    slot_positions: Optional[Dict[int, Tuple[int, int]]] = None,
) -> bytes:
    buf = bytearray(build_plant_command(seq, {}))
    if slot_positions:
        for slot, (position, servo_id) in slot_positions.items():
            patch_servo_command(buf, slot, position, servo_id)
    return bytes(buf)
