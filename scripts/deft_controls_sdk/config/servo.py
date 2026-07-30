"""Neck / DXL servo identity (slots, DXL ids, NVM model codes)."""
from __future__ import annotations

from .profile import NECK_PITCH_SERVO_SLOT, NECK_YAW_SERVO_SLOT

NECK_PITCH_DXL_ID = 1
NECK_YAW_DXL_ID = 2

# NVM servo_table.model (matches App plant_config_nvm.h)
SERVO_MODEL_XL430 = 0
SERVO_MODEL_XL330_M288 = 1  # XL330-M288-T
SERVO_MODEL_NAMES = {
    SERVO_MODEL_XL430: "XL430",
    SERVO_MODEL_XL330_M288: "XL330-M288-T",
}

__all__ = [
    "NECK_PITCH_SERVO_SLOT",
    "NECK_YAW_SERVO_SLOT",
    "NECK_PITCH_DXL_ID",
    "NECK_YAW_DXL_ID",
    "SERVO_MODEL_XL430",
    "SERVO_MODEL_XL330_M288",
    "SERVO_MODEL_NAMES",
]
