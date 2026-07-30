"""Host demux profiles — name → ordered actuator slots (identity, not motion)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Tuple

# --- product slot map (platform truth) ---------------------------------------

LEFT_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 7))
RIGHT_ARM_SLOTS: Tuple[int, ...] = tuple(range(7, 14))
BASE_STEER_SLOTS: Dict[str, int] = {"BwC": 14, "BwR": 15, "BwL": 16}
BASE_DRIVE_SLOTS: Dict[str, int] = {"BpC": 17, "BpR": 18, "BpL": 19}
BASE_SLOTS: Tuple[int, ...] = (14, 15, 16, 17, 18, 19)
LIFT_SLOT = 20
SPARE_SLOTS: Tuple[int, ...] = (21, 22, 23, 24, 25)
# Continuous / bus56 lab: CH5+CH6 motors on spare slots (not product base 14–19).
BENCH_BASE_SLOTS: Tuple[int, ...] = (22, 23, 24, 25)
NECK_PITCH_SERVO_SLOT = 0
NECK_YAW_SERVO_SLOT = 1


@dataclass(frozen=True)
class Profile:
    """Named groups of actuator slots (host demux layer 1)."""

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
    return Profile(
        name="yam_product",
        components={
            "left_arm": LEFT_ARM_SLOTS,
            "right_arm": RIGHT_ARM_SLOTS,
            "base": BASE_SLOTS,
            "lift": (LIFT_SLOT,),
        },
    )


def bench_continuous_profile() -> Profile:
    """Host demux for continuous / bus56 bench (spare-slot base).

    ``base`` here is slots 22–25 (CFG IDs set by continuous BASE_ROWS).
    Product drivetrain map stays available as ``base_product`` (14–19).
    """
    return Profile(
        name="yam_bench_continuous",
        components={
            "left_arm": LEFT_ARM_SLOTS,
            "right_arm": RIGHT_ARM_SLOTS,
            "base": BENCH_BASE_SLOTS,
            "base_product": BASE_SLOTS,
            "lift": (LIFT_SLOT,),
        },
    )
