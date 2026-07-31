"""Assembly — compose peripheral-homogeneous profiles with overlap checks."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Tuple

from .profile import Profile
from .typed_profiles import (
    ActuatorProfile,
    ServoProfile,
    arm_profile,
    base_wheel_profile,
    lift_profile,
    neck_profile,
    torso_profile,
    wheel_profile,
)


@dataclass(frozen=True)
class Assembly:
    """Named composition of typed profiles for HostProxy / controls.

    SDK config substrate (not pcb_lab-owned). Actuator and servo profiles
    stay separate — never one mixed map.
    """

    name: str
    actuators: Mapping[str, ActuatorProfile] = field(default_factory=dict)
    servos: Mapping[str, ServoProfile] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # frozen dataclass: validate via object.__setattr__ already done; just call
        self.validate()

    def validate(self) -> None:
        act_seen: Dict[int, str] = {}
        for name, prof in self.actuators.items():
            if name != prof.name:
                raise ValueError(
                    f"actuator key {name!r} != profile.name {prof.name!r}"
                )
            for slot in prof.slots:
                if slot in act_seen:
                    raise ValueError(
                        f"actuator slot {slot} overlaps: "
                        f"{act_seen[slot]!r} and {name!r}"
                    )
                act_seen[slot] = name

        servo_seen: Dict[int, str] = {}
        for name, prof in self.servos.items():
            if name != prof.name:
                raise ValueError(
                    f"servo key {name!r} != profile.name {prof.name!r}"
                )
            for slot in prof.slots:
                if slot in servo_seen:
                    raise ValueError(
                        f"servo slot {slot} overlaps: "
                        f"{servo_seen[slot]!r} and {name!r}"
                    )
                servo_seen[slot] = name

    def actuator(self, name: str) -> ActuatorProfile:
        try:
            return self.actuators[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.actuators))
            raise KeyError(f"unknown actuator {name!r}; known: {known}") from exc

    def servo(self, name: str) -> ServoProfile:
        try:
            return self.servos[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.servos))
            raise KeyError(f"unknown servo {name!r}; known: {known}") from exc

    def all_actuator_slots(self) -> Tuple[int, ...]:
        seen: List[int] = []
        for prof in self.actuators.values():
            for s in prof.slots:
                if s not in seen:
                    seen.append(s)
        return tuple(seen)

    def to_demux_profile(self) -> Profile:
        """Actuator-only demux map for HostProxy (legacy ``Profile``)."""
        return Profile(
            name=self.name,
            components={name: prof.slots for name, prof in self.actuators.items()},
        )


def yam_product_assembly() -> Assembly:
    """Stock YAM product demux: arms + base wheels + torso, neck servos."""
    return Assembly(
        name="yam_product",
        actuators={
            "left_arm": arm_profile("yam", side="left"),
            "right_arm": arm_profile("yam", side="right"),
            "base_wheel_1": base_wheel_profile(1),
            "base_wheel_2": base_wheel_profile(2),
            "base_wheel_3": base_wheel_profile(3),
            "torso": torso_profile(),
        },
        servos={"neck": neck_profile()},
    )


def bench_continuous_assembly() -> Assembly:
    """Continuous / bus56 bench: spare-slot base + product base_product."""
    return Assembly(
        name="yam_bench_continuous",
        actuators={
            "left_arm": arm_profile("yam", side="left"),
            "right_arm": arm_profile("yam", side="right"),
            "base": wheel_profile(name="base", bench=True),
            "base_product": wheel_profile(name="base_product", bench=False),
            "lift": lift_profile(),
        },
        servos={"neck": neck_profile()},
    )


def assembly_from_name(name: str) -> Assembly:
    key = (name or "product").strip().lower()
    if key in ("product", "yam", "yam_product"):
        return yam_product_assembly()
    if key in ("bench", "continuous", "yam_bench_continuous"):
        return bench_continuous_assembly()
    raise ValueError(f"unknown assembly {name!r}; use product|bench")


def assembly_put_actuator(asm: Assembly, profile: ActuatorProfile) -> Assembly:
    """Return a new Assembly with ``profile`` inserted/replaced by name."""
    acts = dict(asm.actuators)
    acts[profile.name] = profile
    return Assembly(name=asm.name, actuators=acts, servos=dict(asm.servos))


def assembly_remove_actuator(asm: Assembly, name: str) -> Assembly:
    acts = dict(asm.actuators)
    if name not in acts:
        raise KeyError(f"unknown actuator {name!r}")
    del acts[name]
    return Assembly(name=asm.name, actuators=acts, servos=dict(asm.servos))


def assembly_put_servo(asm: Assembly, profile: ServoProfile) -> Assembly:
    servos = dict(asm.servos)
    servos[profile.name] = profile
    return Assembly(name=asm.name, actuators=dict(asm.actuators), servos=servos)


__all__ = [
    "Assembly",
    "assembly_from_name",
    "assembly_put_actuator",
    "assembly_put_servo",
    "assembly_remove_actuator",
    "bench_continuous_assembly",
    "yam_product_assembly",
]
