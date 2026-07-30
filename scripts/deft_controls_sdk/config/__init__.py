"""config — component identity, profiles, CFG row builders, firmware paths.

Not plant command TX (see ``actions``) and not DEBUG wire RPC (see ``debug.config``).
"""
from __future__ import annotations

from .actuator import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    DEFAULT_DRIVE_KD,
    DEFAULT_STEER_KD,
    DEFAULT_STEER_KP,
    PROTO_CUBEMARS,
    PROTO_DAMIAO,
    PROTO_NONE,
    PROTO_ROBSTRIDE,
    PROTO_ZEROERR,
    arm_slots,
    cubemars_yam_rows,
    slots_by_bus,
    yam_left_arm_rows,
    yam_product_rows,
)
from .firmware import default_firmware_elf
from .led import LED_PRESETS, LedPreset
from .pdu_link import DEFAULT_BENCH_LISTEN_PDU, DEFAULT_PRODUCT_LISTEN_PDU
from .profile import (
    BASE_DRIVE_SLOTS,
    BASE_SLOTS,
    BASE_STEER_SLOTS,
    BENCH_BASE_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_SERVO_SLOT,
    Profile,
    RIGHT_ARM_SLOTS,
    SPARE_SLOTS,
    bench_continuous_profile,
    yam_product_profile,
)
from .servo import (
    NECK_PITCH_DXL_ID,
    NECK_YAW_DXL_ID,
    SERVO_MODEL_NAMES,
    SERVO_MODEL_XL330_M288,
    SERVO_MODEL_XL430,
)

__all__ = [
    "BASE_DRIVE_SLOTS",
    "BASE_SLOTS",
    "BASE_STEER_SLOTS",
    "BENCH_BASE_SLOTS",
    "DEFAULT_ARM_KD",
    "DEFAULT_ARM_KP",
    "DEFAULT_BENCH_LISTEN_PDU",
    "DEFAULT_DRIVE_KD",
    "DEFAULT_PRODUCT_LISTEN_PDU",
    "DEFAULT_STEER_KD",
    "DEFAULT_STEER_KP",
    "LEFT_ARM_SLOTS",
    "LED_PRESETS",
    "LIFT_SLOT",
    "LedPreset",
    "NECK_PITCH_DXL_ID",
    "NECK_PITCH_SERVO_SLOT",
    "NECK_YAW_DXL_ID",
    "NECK_YAW_SERVO_SLOT",
    "PROTO_CUBEMARS",
    "PROTO_DAMIAO",
    "PROTO_NONE",
    "PROTO_ROBSTRIDE",
    "PROTO_ZEROERR",
    "Profile",
    "RIGHT_ARM_SLOTS",
    "SERVO_MODEL_NAMES",
    "SERVO_MODEL_XL330_M288",
    "SERVO_MODEL_XL430",
    "SPARE_SLOTS",
    "arm_slots",
    "bench_continuous_profile",
    "cubemars_yam_rows",
    "default_firmware_elf",
    "slots_by_bus",
    "yam_left_arm_rows",
    "yam_product_profile",
    "yam_product_rows",
]
