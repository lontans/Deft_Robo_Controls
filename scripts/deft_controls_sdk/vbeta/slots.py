"""YAM product slot map and CFG rows for Controls PCB."""
from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

PROTO_NONE = 0
PROTO_ROBSTRIDE = 1
PROTO_CUBEMARS = 2
PROTO_DAMIAO = 3

# Arms
LEFT_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 7))
RIGHT_ARM_SLOTS: Tuple[int, ...] = tuple(range(7, 14))

# Base: 3 steer + 3 drive on MCP CH4–6 (2 motors per rail)
BASE_STEER_SLOTS: Dict[str, int] = {"BwC": 14, "BwR": 15, "BwL": 16}
BASE_DRIVE_SLOTS: Dict[str, int] = {"BpC": 17, "BpR": 18, "BpL": 19}
BASE_SLOTS: Tuple[int, ...] = (14, 15, 16, 17, 18, 19)

LIFT_SLOT = 20  # reserved, CFG disabled
SPARE_SLOTS: Tuple[int, ...] = (21, 22, 23, 24)

# DXL neck host servo slots
NECK_PITCH_SERVO_SLOT = 0
NECK_YAW_SERVO_SLOT = 1
NECK_PITCH_DXL_ID = 1
NECK_YAW_DXL_ID = 2

# Damiao master IDs (bringup nominal)
_DAMIAO_MASTER = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)

# Default MIT gains for YAM Damiao (bringup.md)
DEFAULT_ARM_KP: Tuple[float, ...] = (40.0, 60.0, 90.0, 60.0, 25.0, 25.0, 20.0)
DEFAULT_ARM_KD: float = 1.0

# Base RobStride gains (conservative hold / creep)
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
    """(bus, enabled, protocol, motor_id, master_id) per slot 0..24."""
    rows: List[Tuple[int, bool, int, int, int]] = []

    # Left arm CH1 Damiao ESC 0x01..0x07
    for i in range(7):
        rows.append((1, True, PROTO_DAMIAO, 0x01 + i, _DAMIAO_MASTER[i]))
    # Right arm CH2
    for i in range(7):
        rows.append((2, True, PROTO_DAMIAO, 0x01 + i, _DAMIAO_MASTER[i]))

    # Base steer: CH4, CH5, CH6 — one each, motor_id 0x01
    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 0x01, 0))
    # Base drive: CH4, CH5, CH6 — motor_id 0x02
    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 0x02, 0))

    # Lift reserved disabled on CH3
    rows.append((3, False, PROTO_NONE, 0, 0))
    # Spare
    for _ in range(4):
        rows.append((3, False, PROTO_NONE, 0, 0))

    if len(rows) != ACTUATOR_COUNT:
        raise RuntimeError(f"YAM CFG rows={len(rows)} != ACTUATOR_COUNT={ACTUATOR_COUNT}")
    return rows


def cubemars_yam_rows() -> List[Tuple[int, bool, int, int, int]]:
    """Bench-only scaffold: same slot map as `yam_product_rows()` with the
    arm rows swapped from Damiao MIT to CubeMars MIT (PROTO_CUBEMARS) —
    see docs/rfc-cubemars-mit-plant.md. **Not used by any default/live
    path** (YAM product CFG stays Damiao; this exists for future bench work
    against real CubeMars hardware only). `master_id=0` per arm slot means
    "no RX-ID override" (feedback arrives on the motor's own ID per the
    CubeMars PDF — see `resolve_rx_can_id()` in
    `deft_controls_sdk/protocol/cubemars_mit.py`), unlike Damiao's non-zero
    `_DAMIAO_MASTER` table above.
    """
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
    for _ in range(4):
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
