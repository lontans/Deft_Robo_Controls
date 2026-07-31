"""Actuator CFG row builders + optional teleop gain helpers.

CFG rows are applied via ``hub.debug.cfg_*`` wire RPC (identity only).
``joint`` / ``wheel`` gain kinds are host teleop sugar (``TeleopEngine`` /
suite prompts) — not NVM and not part of ``ActuatorProfile`` / ``ActuatorAction``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Literal, Sequence, Tuple

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

# Dual-YAM teleop baseline (CH1/CH2 arm joints — load-bearing)
DEFAULT_ARM_KP: Tuple[float, ...] = (40.0, 60.0, 90.0, 60.0, 25.0, 25.0, 20.0)
DEFAULT_ARM_KD: Tuple[float, ...] = (2.5, 3.75, 5.6, 3.75, 1.5, 1.5, 1.25)
# Product base steer (position hold on swerve) — stiffer than bench wheel cruise
DEFAULT_STEER_KP = 40.0
DEFAULT_STEER_KD = 1.0
DEFAULT_DRIVE_KD = 2.0

# --- plant action kinds (smooth arm teleop ≠ base/wheel teleop) ---------------
# ``joint``: gravity / load-bearing (arms). ``wheel``: CH5–6 style base (often
# not load-bearing the same way; gentler position gains from continuous/teleop).
ActuatorKind = Literal["joint", "wheel"]

DEFAULT_JOINT_KP = float(DEFAULT_ARM_KP[0])
DEFAULT_JOINT_KD = float(DEFAULT_ARM_KD[0])
DEFAULT_JOINT_TORQUE = 0.0
# Bench / yam_continuous RobStride base (teleop RS_KP/RS_KD)
DEFAULT_WHEEL_KP = 20.0
DEFAULT_WHEEL_KD = 1.0
DEFAULT_WHEEL_TORQUE = 0.0

COMPONENT_ACTUATOR_KIND: Dict[str, ActuatorKind] = {
    "left_arm": "joint",
    "right_arm": "joint",
    "torso": "joint",
    "lift": "joint",  # bench alias of torso
    "base": "wheel",
    "base_product": "wheel",
    "base_wheel_1": "wheel",
    "base_wheel_2": "wheel",
    "base_wheel_3": "wheel",
}


@dataclass(frozen=True)
class ActuatorGains:
    """Scalar MIT desire defaults for one kind (per-slot sequences optional)."""

    kp: float
    kd: float
    torque: float = 0.0


JOINT_GAINS = ActuatorGains(
    kp=DEFAULT_JOINT_KP, kd=DEFAULT_JOINT_KD, torque=DEFAULT_JOINT_TORQUE
)
WHEEL_GAINS = ActuatorGains(
    kp=DEFAULT_WHEEL_KP, kd=DEFAULT_WHEEL_KD, torque=DEFAULT_WHEEL_TORQUE
)


def kind_for_component(name: str) -> ActuatorKind:
    """Profile component → joint|wheel (unknown names default to joint)."""
    return COMPONENT_ACTUATOR_KIND.get(str(name), "joint")


def gains_for_kind(kind: ActuatorKind) -> ActuatorGains:
    if kind == "wheel":
        return WHEEL_GAINS
    if kind == "joint":
        return JOINT_GAINS
    raise ValueError(f"unknown ActuatorKind {kind!r}; use joint|wheel")


# Neutral hold defaults when ``ActuatorAction.hold`` omits kp/kd (bringup-safe).
DEFAULT_HOLD_KP = DEFAULT_WHEEL_KP
DEFAULT_HOLD_KD = DEFAULT_WHEEL_KD
DEFAULT_HOLD_TORQUE = 0.0


def resolve_hold_gains(
    n: int,
    *,
    kp: float | Sequence[float] | None = None,
    kd: float | Sequence[float] | None = None,
    torque: float | Sequence[float] | None = None,
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    """Broadcast hold gains — no joint/wheel kind. Pass kp/kd for product teleop."""

    def _vec(
        override: float | Sequence[float] | None,
        default_scalar: float,
    ) -> Tuple[float, ...]:
        if override is None:
            return tuple(float(default_scalar) for _ in range(n))
        if isinstance(override, (int, float)):
            return tuple(float(override) for _ in range(n))
        out = tuple(float(x) for x in override)
        if len(out) != n:
            raise ValueError(f"expected {n} gain values, got {len(out)}")
        return out

    return (
        _vec(kp, DEFAULT_HOLD_KP),
        _vec(kd, DEFAULT_HOLD_KD),
        _vec(torque, DEFAULT_HOLD_TORQUE),
    )


def resolve_gain_vectors(
    kind: ActuatorKind,
    n: int,
    *,
    kp: float | Sequence[float] | None = None,
    kd: float | Sequence[float] | None = None,
    torque: float | Sequence[float] | None = None,
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    """Teleop helper: broadcast scalars or pad arm tables by ``joint``/``wheel`` kind."""
    gains = gains_for_kind(kind)

    def _vec(
        override: float | Sequence[float] | None,
        default_scalar: float,
        arm_table: Tuple[float, ...] | None,
    ) -> Tuple[float, ...]:
        if override is None:
            if kind == "joint" and arm_table is not None and n == len(arm_table):
                return arm_table
            return tuple(float(default_scalar) for _ in range(n))
        if isinstance(override, (int, float)):
            return tuple(float(override) for _ in range(n))
        out = tuple(float(x) for x in override)
        if len(out) != n:
            raise ValueError(f"expected {n} gain values, got {len(out)}")
        return out

    kps = _vec(kp, gains.kp, DEFAULT_ARM_KP)
    kds = _vec(kd, gains.kd, DEFAULT_ARM_KD)
    tqs = _vec(torque, gains.torque, None)
    return kps, kds, tqs


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
    "ActuatorGains",
    "ActuatorKind",
    "BASE_DRIVE_SLOTS",
    "BASE_SLOTS",
    "BASE_STEER_SLOTS",
    "COMPONENT_ACTUATOR_KIND",
    "DEFAULT_ARM_KD",
    "DEFAULT_ARM_KP",
    "DEFAULT_DRIVE_KD",
    "DEFAULT_JOINT_KD",
    "DEFAULT_JOINT_KP",
    "DEFAULT_HOLD_KD",
    "DEFAULT_HOLD_KP",
    "DEFAULT_HOLD_TORQUE",
    "DEFAULT_JOINT_TORQUE",
    "DEFAULT_STEER_KD",
    "DEFAULT_STEER_KP",
    "DEFAULT_WHEEL_KD",
    "DEFAULT_WHEEL_KP",
    "DEFAULT_WHEEL_TORQUE",
    "JOINT_GAINS",
    "LEFT_ARM_SLOTS",
    "LIFT_SLOT",
    "PROTO_CUBEMARS",
    "PROTO_DAMIAO",
    "PROTO_NONE",
    "PROTO_ROBSTRIDE",
    "PROTO_ZEROERR",
    "RIGHT_ARM_SLOTS",
    "SPARE_SLOTS",
    "WHEEL_GAINS",
    "arm_slots",
    "cubemars_yam_rows",
    "gains_for_kind",
    "kind_for_component",
    "resolve_gain_vectors",
    "resolve_hold_gains",
    "slots_by_bus",
    "yam_left_arm_rows",
    "yam_product_rows",
]
