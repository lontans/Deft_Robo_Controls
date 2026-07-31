"""Shim — teleop lives in ``deft_controls_sdk.actions.teleop``.

Kept so older ``from .teleop import …`` imports keep working.
"""
from __future__ import annotations

from deft_controls_sdk.actions.teleop import (  # noqa: F401
    ARM_CRUISE_DEFAULT,
    ARM_CRUISE_MAX,
    BASE_BENCH_DAMIAO_SLOTS,
    BASE_BENCH_ROWS,
    BASE_CRUISE_DEFAULT,
    BASE_CRUISE_MAX,
    DM_BASE_TRAVEL,
    DXL_CRUISE_DEFAULT,
    DXL_CRUISE_MAX,
    DXL_HI,
    DXL_IDS,
    DXL_LO,
    SlotSpec,
    TeleopEngine,
    build_actuator_specs,
    build_servo_specs,
    read_dxl_present_position,
)

__all__ = [
    "ARM_CRUISE_DEFAULT",
    "ARM_CRUISE_MAX",
    "BASE_BENCH_DAMIAO_SLOTS",
    "BASE_BENCH_ROWS",
    "BASE_CRUISE_DEFAULT",
    "BASE_CRUISE_MAX",
    "DM_BASE_TRAVEL",
    "DXL_CRUISE_DEFAULT",
    "DXL_CRUISE_MAX",
    "DXL_HI",
    "DXL_IDS",
    "DXL_LO",
    "SlotSpec",
    "TeleopEngine",
    "build_actuator_specs",
    "build_servo_specs",
    "read_dxl_present_position",
]
