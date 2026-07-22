"""672 B host command image builders — pack desires into wire bytes."""
from __future__ import annotations

import struct
from typing import Dict, Optional, Tuple

from .wire_layout import (
    ACTUATOR0_OFF,
    ACTUATOR_SLOT_BYTES,
    HOST_COMMAND_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PLANT_MCU_STATE_NORMAL,
    SYSTEM_CMD_OFF,
)


def actuator_slot_offset(slot: int) -> int:
    return ACTUATOR0_OFF + slot * ACTUATOR_SLOT_BYTES


def patch_system_mcu_state(buf: bytearray, mcu_state: int) -> None:
    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~0x0E) | ((int(mcu_state) & 7) << 1)
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


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
