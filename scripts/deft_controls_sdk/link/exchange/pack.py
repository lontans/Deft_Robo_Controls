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
    LED_CMD_OFF,
    PLANT_MCU_STATE_NORMAL,
    SERVO0_CMD_OFF,
    SERVO_SLOT_BYTES,
    SYSTEM_CMD_OFF,
)


def actuator_slot_offset(slot: int) -> int:
    return ACTUATOR0_OFF + slot * ACTUATOR_SLOT_BYTES


def patch_system_mcu_state(buf: bytearray, mcu_state: int) -> None:
    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~0x0E) | ((int(mcu_state) & 7) << 1)
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def patch_system_rx_sim(buf: bytearray, enable: bool) -> None:
    """Back-compat: True → ACTUATOR only; False → clear bits0..3."""
    patch_system_rx_sim_mask(buf, 0x1 if enable else 0)


def patch_system_rx_sim_mask(buf: bytearray, mask: int) -> None:
    """system.reserved bits0..3 (wire bits5..8): ACTUATOR|SERVO|LED|PDU."""
    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~(0xF << 5)) | ((int(mask) & 0xF) << 5)
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def patch_system_stm32_mode(buf: bytearray, mode: int) -> None:
    """system.reserved bits4..5 (wire bits9..10): plant/debug/soft_dfu."""
    from .wire_layout import STM32_MODE_MASK, STM32_MODE_SHIFT

    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~(STM32_MODE_MASK << STM32_MODE_SHIFT)) | (
        (int(mode) & STM32_MODE_MASK) << STM32_MODE_SHIFT
    )
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def patch_system_plant_apply(buf: bytearray, enable: bool) -> None:
    """system wire bit11: plant_apply arm (1=apply, 0=observe)."""
    from .wire_layout import PLANT_APPLY_MASK, PLANT_APPLY_SHIFT

    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word &= ~(PLANT_APPLY_MASK << PLANT_APPLY_SHIFT)
    if enable:
        word |= PLANT_APPLY_MASK << PLANT_APPLY_SHIFT
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def patch_servo_command(
    buf: bytearray,
    slot: int,
    *,
    servo_id: int,
    native_step_position: int,
    native_speed_unit: int = 0,
    torque_enable: bool = True,
    led_control: bool = False,
    operating_mode: int = 3,
) -> None:
    """Pack one host_servo_command_t (6 B) at SERVO0_CMD_OFF + slot*6."""
    if slot < 0 or slot > 1:
        raise ValueError(f"servo slot must be 0..1, got {slot}")
    flags = (
        (1 if torque_enable else 0)
        | ((1 if led_control else 0) << 1)
        | ((int(operating_mode) & 7) << 2)
        | ((int(servo_id) & 0xFF) << 5)
    )
    off = SERVO0_CMD_OFF + slot * SERVO_SLOT_BYTES
    struct.pack_into(
        "<hhH",
        buf,
        off,
        int(native_step_position),
        int(native_speed_unit),
        flags,
    )


def patch_led_command(
    buf: bytearray,
    *,
    mode: int = 0,
    master_brightness: int = 0,
    led_count: int = 0,
) -> None:
    """Pack host_led_command_t (2 B) at LED_CMD_OFF.

    ``mode`` is 5-bit (0=OFF, 1=TEST, 2=FLASH, 3..7 factory traffic-light —
    see ``LED_MODE_*`` / docs/legacy/rfc/rfc-led-factory-patterns.md). ``led_count`` 0 ⇒
    firmware LED_STRIP_MAX (300). Layout stays 672 B / 2 B LED word.
    """
    word = (
        (int(mode) & 0x1F)
        | ((int(master_brightness) & 0x1F) << 5)
        | ((int(led_count) & 0x3F) << 10)
    )
    struct.pack_into("<H", buf, LED_CMD_OFF, word)


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
