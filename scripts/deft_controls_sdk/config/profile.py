"""Host demux Profile (actuator-only) + slot constants.

Canonical product identity is ``Assembly`` (``config.assembly``). ``Profile``
remains the HostProxy demux shim: name → ordered actuator slots.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Tuple

# --- product slot map (platform truth) ---------------------------------------

LEFT_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 7))
RIGHT_ARM_SLOTS: Tuple[int, ...] = tuple(range(7, 14))
BASE_STEER_SLOTS: Dict[str, int] = {"BwC": 14, "BwR": 15, "BwL": 16}
BASE_DRIVE_SLOTS: Dict[str, int] = {"BpC": 17, "BpR": 18, "BpL": 19}
BASE_SLOTS: Tuple[int, ...] = (14, 15, 16, 17, 18, 19)
# Product demux: each wheel module = (steer, drive) for C / R / L.
BASE_WHEEL_1_SLOTS: Tuple[int, ...] = (14, 17)  # Center
BASE_WHEEL_2_SLOTS: Tuple[int, ...] = (15, 18)  # Right
BASE_WHEEL_3_SLOTS: Tuple[int, ...] = (16, 19)  # Left
BASE_WHEEL_SLOTS: Dict[str, Tuple[int, ...]] = {
    "base_wheel_1": BASE_WHEEL_1_SLOTS,
    "base_wheel_2": BASE_WHEEL_2_SLOTS,
    "base_wheel_3": BASE_WHEEL_3_SLOTS,
}
LIFT_SLOT = 20
TORSO_SLOT = LIFT_SLOT  # product section name for slot 20
SPARE_SLOTS: Tuple[int, ...] = (21, 22, 23, 24, 25)
# Continuous / bus56 lab: CH5+CH6 motors on spare slots (not product base 14–19).
BENCH_BASE_SLOTS: Tuple[int, ...] = (22, 23, 24, 25)
NECK_PITCH_SERVO_SLOT = 0
NECK_YAW_SERVO_SLOT = 1

# Canonical product actuator section names (HostProxy demux / ROS).
PRODUCT_ACTUATOR_SECTIONS: Tuple[str, ...] = (
    "left_arm",
    "right_arm",
    "base_wheel_1",
    "base_wheel_2",
    "base_wheel_3",
    "torso",
)


@dataclass(frozen=True)
class Profile:
    """Actuator-only demux map for HostProxy (not a mixed peripheral profile)."""

    name: str
    components: Mapping[str, Tuple[int, ...]]

    def slots(self, component: str) -> Tuple[int, ...]:
        try:
            return self.components[component]
        except KeyError as exc:
            known = ", ".join(sorted(self.components))
            raise KeyError(f"unknown component {component!r}; known: {known}") from exc

    def all_slots(self) -> Tuple[int, ...]:
        seen: List[int] = []
        for slots in self.components.values():
            for s in slots:
                if s not in seen:
                    seen.append(s)
        return tuple(seen)


def yam_product_profile() -> Profile:
    """Back-compat demux shim — prefer ``yam_product_assembly()``."""
    from .assembly import yam_product_assembly

    return yam_product_assembly().to_demux_profile()


def bench_continuous_profile() -> Profile:
    """Back-compat demux shim — prefer ``bench_continuous_assembly()``."""
    from .assembly import bench_continuous_assembly

    return bench_continuous_assembly().to_demux_profile()
