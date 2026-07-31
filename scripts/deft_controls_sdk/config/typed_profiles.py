"""Peripheral-homogeneous profiles — actuators XOR servos (never mixed).

Assemblies compose these; HostProxy demux still uses ``Profile`` via
``Assembly.to_demux_profile()``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union

from .actuator import (
    ActuatorKind,
    PROTO_CUBEMARS,
    PROTO_DAMIAO,
    PROTO_NONE,
    PROTO_ROBSTRIDE,
    PROTO_ZEROERR,
    arm_slots,
    yam_product_rows,
)
from .profile import (
    BASE_SLOTS,
    BENCH_BASE_SLOTS,
    LIFT_SLOT,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_SERVO_SLOT,
)
from .servo import (
    NECK_PITCH_DXL_ID,
    NECK_YAW_DXL_ID,
    SERVO_MODEL_XL430,
)

SlotsIn = Union[str, Sequence[int], None]

_PROTO_BY_NAME: Dict[str, int] = {
    "none": PROTO_NONE,
    "robstride": PROTO_ROBSTRIDE,
    "rs": PROTO_ROBSTRIDE,
    "cubemars": PROTO_CUBEMARS,
    "cm": PROTO_CUBEMARS,
    "damiao": PROTO_DAMIAO,
    "dm": PROTO_DAMIAO,
    "zeroerr": PROTO_ZEROERR,
    "ze": PROTO_ZEROERR,
}


def parse_protocol(value: Union[str, int]) -> int:
    if isinstance(value, int):
        return int(value)
    key = str(value).strip().lower()
    if key not in _PROTO_BY_NAME:
        known = ", ".join(sorted(_PROTO_BY_NAME))
        raise ValueError(f"unknown protocol {value!r}; known: {known}")
    return _PROTO_BY_NAME[key]


def parse_slots_spec(spec: SlotsIn, default: Tuple[int, ...]) -> Tuple[int, ...]:
    """Parse ``None`` | ``(0,1,…)`` | ``\"0,1,2\"`` | ``\"0-6\"`` into slots."""
    if spec is None:
        return tuple(int(s) for s in default)
    if isinstance(spec, str):
        text = spec.strip().replace(" ", "")
        if not text:
            return tuple(int(s) for s in default)
        if "-" in text and "," not in text:
            a, b = text.split("-", 1)
            start, end = int(a, 0), int(b, 0)
            if start > end:
                raise ValueError(f"slot range start {start} > end {end}")
            return tuple(range(start, end + 1))
        parts = [p for p in text.split(",") if p]
        return tuple(int(p, 0) for p in parts)
    return tuple(int(s) for s in spec)


@dataclass(frozen=True)
class CfgSlotSpec:
    """One MCU CFG row (identity) for a host actuator slot."""

    bus: int
    protocol: int
    motor_id: int
    master_id: int = 0
    enabled: bool = True


@dataclass(frozen=True)
class ActuatorProfile:
    """Homogeneous actuator section or single — slots + kind + optional CFG."""

    name: str
    slots: Tuple[int, ...]
    kind: ActuatorKind
    cfg: Tuple[Optional[CfgSlotSpec], ...] = ()

    def __post_init__(self) -> None:
        if not self.slots:
            raise ValueError(f"{self.name}: ActuatorProfile needs at least one slot")
        if self.cfg and len(self.cfg) != len(self.slots):
            raise ValueError(
                f"{self.name}: cfg length {len(self.cfg)} != slots {len(self.slots)}"
            )

    def as_cfg_rows(self) -> List[dict]:
        """Rows suitable for ``hub.debug.cfg_set_slot(**row)`` (skips missing cfg)."""
        out: List[dict] = []
        specs = self.cfg
        if not specs:
            return out
        for slot, spec in zip(self.slots, specs):
            if spec is None:
                continue
            out.append(
                {
                    "slot": int(slot),
                    "bus": int(spec.bus),
                    "protocol": int(spec.protocol),
                    "motor_id": int(spec.motor_id),
                    "master_id": int(spec.master_id),
                    "enabled": bool(spec.enabled),
                }
            )
        return out

    def as_cfg_row(self) -> dict:
        """Single-slot helper — raises if not exactly one cfg row."""
        rows = self.as_cfg_rows()
        if len(rows) != 1:
            raise ValueError(
                f"{self.name}: as_cfg_row requires exactly one cfg entry, got {len(rows)}"
            )
        return rows[0]


@dataclass(frozen=True)
class ServoEntry:
    slot: int
    dxl_id: int
    model: int = SERVO_MODEL_XL430


@dataclass(frozen=True)
class ServoProfile:
    """Homogeneous servo section (e.g. neck) — never mixed with actuators."""

    name: str
    entries: Tuple[ServoEntry, ...]

    @property
    def slots(self) -> Tuple[int, ...]:
        return tuple(e.slot for e in self.entries)


def _cfg_from_yam_rows(slots: Sequence[int]) -> Tuple[Optional[CfgSlotSpec], ...]:
    rows = yam_product_rows()
    out: List[Optional[CfgSlotSpec]] = []
    for slot in slots:
        s = int(slot)
        if s < 0 or s >= len(rows):
            out.append(None)
            continue
        bus, enabled, proto, mid, master = rows[s]
        out.append(
            CfgSlotSpec(
                bus=int(bus),
                protocol=int(proto),
                motor_id=int(mid),
                master_id=int(master),
                enabled=bool(enabled),
            )
        )
    return tuple(out)


def arm_profile(
    arm_type: str = "yam",
    *,
    side: str = "left",
    slots: SlotsIn = None,
    name: Optional[str] = None,
) -> ActuatorProfile:
    """Section profile for an arm (actuators only)."""
    at = (arm_type or "yam").strip().lower()
    if at not in ("yam", "yam_damiao", "damiao"):
        raise ValueError(f"unsupported arm_type {arm_type!r}; use yam")
    default_slots = arm_slots(side)
    resolved = parse_slots_spec(slots, default_slots)
    side_key = side.strip().lower()
    if side_key in ("left", "l", "can_deft_l"):
        default_name = "left_arm"
    elif side_key in ("right", "r", "can_deft_r"):
        default_name = "right_arm"
    else:
        default_name = f"arm_{side_key}"
    return ActuatorProfile(
        name=name or default_name,
        slots=resolved,
        kind="joint",
        cfg=_cfg_from_yam_rows(resolved) if resolved == default_slots else (),
    )


def wheel_profile(
    *,
    name: str = "base",
    slots: SlotsIn = None,
    bench: bool = False,
) -> ActuatorProfile:
    """Section profile for base/wheel actuators."""
    default = BENCH_BASE_SLOTS if bench else BASE_SLOTS
    resolved = parse_slots_spec(slots, default)
    cfg = _cfg_from_yam_rows(resolved) if (not bench and resolved == BASE_SLOTS) else ()
    return ActuatorProfile(name=name, slots=resolved, kind="wheel", cfg=cfg)


def lift_profile(*, slots: SlotsIn = None, name: str = "lift") -> ActuatorProfile:
    resolved = parse_slots_spec(slots, (LIFT_SLOT,))
    return ActuatorProfile(
        name=name,
        slots=resolved,
        kind="joint",
        cfg=_cfg_from_yam_rows(resolved) if resolved == (LIFT_SLOT,) else (),
    )


def single_profile(
    slot: int,
    *,
    protocol: Union[str, int],
    motor_id: int,
    bus: int,
    kind: ActuatorKind = "joint",
    master_id: int = 0,
    enabled: bool = True,
    name: Optional[str] = None,
) -> ActuatorProfile:
    """One actuator + CFG identity (motion + NVM)."""
    s = int(slot)
    proto = parse_protocol(protocol)
    return ActuatorProfile(
        name=name or f"slot_{s}",
        slots=(s,),
        kind=kind,
        cfg=(
            CfgSlotSpec(
                bus=int(bus),
                protocol=proto,
                motor_id=int(motor_id) & 0xFF,
                master_id=int(master_id) & 0xFF,
                enabled=bool(enabled),
            ),
        ),
    )


def neck_profile(*, name: str = "neck") -> ServoProfile:
    return ServoProfile(
        name=name,
        entries=(
            ServoEntry(
                slot=NECK_PITCH_SERVO_SLOT,
                dxl_id=NECK_PITCH_DXL_ID,
                model=SERVO_MODEL_XL430,
            ),
            ServoEntry(
                slot=NECK_YAW_SERVO_SLOT,
                dxl_id=NECK_YAW_DXL_ID,
                model=SERVO_MODEL_XL430,
            ),
        ),
    )


__all__ = [
    "ActuatorProfile",
    "CfgSlotSpec",
    "ServoEntry",
    "ServoProfile",
    "arm_profile",
    "lift_profile",
    "neck_profile",
    "parse_protocol",
    "parse_slots_spec",
    "single_profile",
    "wheel_profile",
]
