"""Actuator CFG row builders (identity) — apply via hub.debug.cfg_* wire RPC."""
from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

from .profile import (
    BASE_DRIVE_SLOTS,
    BASE_SLOTS,
    BASE_STEER_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    RIGHT_ARM_SLOTS,
    SPARE_SLOTS,
)

PROTO_NONE = 0
PROTO_ROBSTRIDE = 1
PROTO_CUBEMARS = 2
PROTO_DAMIAO = 3
PROTO_ZEROERR = 4

# Damiao master IDs (bringup nominal)
DAMIAO_MASTER_IDS = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
# Back-compat private alias (suite presets / older imports)
_DAMIAO_MASTER = DAMIAO_MASTER_IDS

# Dual-YAM teleop baseline
DEFAULT_ARM_KP: Tuple[float, ...] = (40.0, 60.0, 90.0, 60.0, 25.0, 25.0, 20.0)
DEFAULT_ARM_KD: Tuple[float, ...] = (2.5, 3.75, 5.6, 3.75, 1.5, 1.5, 1.25)
DEFAULT_STEER_KP = 40.0
DEFAULT_STEER_KD = 1.0
DEFAULT_DRIVE_KD = 2.0


def arm_slots(side: str) -> Tuple[int, ...]:
    s = side.strip().lower()
    if s in ("left", "l", "can_deft_l"):
        return LEFT_ARM_SLOTS
    if s in ("right", "r", "can_deft_r"):
        return RIGHT_ARM_SLOTS
    raise ValueError(f"side must be left|right, got {side!r}")


def yam_product_rows() -> List[Tuple[int, bool, int, int, int]]:
    """(bus, enabled, protocol, motor_id, master_id) per slot 0..25."""
    rows: List[Tuple[int, bool, int, int, int]] = []

    for i in range(7):
        rows.append((1, True, PROTO_DAMIAO, 0x01 + i, DAMIAO_MASTER_IDS[i]))
    for i in range(7):
        rows.append((2, True, PROTO_DAMIAO, 0x01 + i, DAMIAO_MASTER_IDS[i]))

    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 0x01, 0))
    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 0x02, 0))

    rows.append((3, False, PROTO_NONE, 0, 0))
    for _ in range(5):
        rows.append((3, False, PROTO_NONE, 0, 0))

    if len(rows) != ACTUATOR_COUNT:
        raise RuntimeError(f"YAM CFG rows={len(rows)} != ACTUATOR_COUNT={ACTUATOR_COUNT}")
    return rows


def yam_left_arm_rows() -> List[Tuple[int, bool, int, int, int]]:
    """Bench characterize: left arm CH1 only — slots 0–6 on, everything else off."""
    rows: List[Tuple[int, bool, int, int, int]] = []
    for i in range(7):
        rows.append((1, True, PROTO_DAMIAO, 0x01 + i, DAMIAO_MASTER_IDS[i]))
    for i in range(7):
        rows.append((2, False, PROTO_DAMIAO, 0x01 + i, DAMIAO_MASTER_IDS[i]))
    for bus in (4, 5, 6):
        rows.append((bus, False, PROTO_ROBSTRIDE, 0x01, 0))
    for bus in (4, 5, 6):
        rows.append((bus, False, PROTO_ROBSTRIDE, 0x02, 0))
    rows.append((3, False, PROTO_NONE, 0, 0))
    for _ in range(5):
        rows.append((3, False, PROTO_NONE, 0, 0))
    if len(rows) != ACTUATOR_COUNT:
        raise RuntimeError(
            f"YAM left-arm CFG rows={len(rows)} != ACTUATOR_COUNT={ACTUATOR_COUNT}"
        )
    return rows


def cubemars_yam_rows() -> List[Tuple[int, bool, int, int, int]]:
    """Bench-only scaffold: YAM map with CubeMars MIT on arms. Not a live default."""
    rows: List[Tuple[int, bool, int, int, int]] = []

    for i in range(7):
        rows.append((1, True, PROTO_CUBEMARS, 0x01 + i, 0))
    for i in range(7):
        rows.append((2, True, PROTO_CUBEMARS, 0x01 + i, 0))

    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 0x01, 0))
    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 0x02, 0))

    rows.append((3, False, PROTO_NONE, 0, 0))
    for _ in range(5):
        rows.append((3, False, PROTO_NONE, 0, 0))

    if len(rows) != ACTUATOR_COUNT:
        raise RuntimeError(f"CubeMars CFG rows={len(rows)} != ACTUATOR_COUNT={ACTUATOR_COUNT}")
    return rows


def slots_by_bus(rows: Sequence[Tuple[int, bool, int, int, int]] | None = None) -> Dict[int, List[int]]:
    if rows is None:
        rows = yam_product_rows()
    by_bus: Dict[int, List[int]] = {b: [] for b in range(1, 7)}
    for slot, (bus, enabled, _p, _m, _mas) in enumerate(rows):
        if enabled and 1 <= bus <= 6:
            by_bus[bus].append(slot)
    return by_bus


__all__ = [
    "BASE_DRIVE_SLOTS",
    "BASE_SLOTS",
    "BASE_STEER_SLOTS",
    "DEFAULT_ARM_KD",
    "DEFAULT_ARM_KP",
    "DEFAULT_DRIVE_KD",
    "DEFAULT_STEER_KD",
    "DEFAULT_STEER_KP",
    "LEFT_ARM_SLOTS",
    "LIFT_SLOT",
    "PROTO_CUBEMARS",
    "PROTO_DAMIAO",
    "PROTO_NONE",
    "PROTO_ROBSTRIDE",
    "PROTO_ZEROERR",
    "RIGHT_ARM_SLOTS",
    "SPARE_SLOTS",
    "arm_slots",
    "cubemars_yam_rows",
    "slots_by_bus",
    "yam_left_arm_rows",
    "yam_product_rows",
]
