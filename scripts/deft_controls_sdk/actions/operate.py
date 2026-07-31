"""Operate helpers — spin (wheel jog) and arm cruise via ``TeleopEngine``.

General plant actions (not suite-specific). The Assembly workshop TUI calls
these; ``debug_dashboard`` can later drive the same ``TeleopEngine`` from
mouse UI without owning a second cruise implementation.
"""
from __future__ import annotations

from typing import Callable, Dict, Mapping, Optional, Sequence

from .teleop import SlotSpec, TeleopEngine, build_actuator_specs


HubGetter = Callable[[], object]
FeedbackGetter = Callable[[], Dict[int, dict]]


def make_teleop_engine(
    hub_getter: HubGetter,
    *,
    feedback_getter: Optional[FeedbackGetter] = None,
    hz: float = 25.0,
) -> TeleopEngine:
    """Construct a shared cruise engine (dashboard-compatible API)."""
    return TeleopEngine(
        hub_getter=hub_getter,
        feedback_getter=feedback_getter,
        hz=hz,
    )


def feedback_positions_from_proxy(proxy: object) -> Dict[int, dict]:
    """``{slot: {position, velocity}}`` from a HostProxy / hub-shaped object."""
    fb_fn = getattr(proxy, "latest_feedback", None)
    if fb_fn is None:
        hub = getattr(proxy, "hub", None)
        fb_fn = getattr(hub, "latest_feedback", None) if hub is not None else None
    if fb_fn is None:
        return {}
    fb = fb_fn()
    if fb is None:
        return {}
    out: Dict[int, dict] = {}
    # FeedbackImage exposes .actuator(slot) → state with position/velocity
    for slot in range(26):
        st = fb.actuator(slot) if hasattr(fb, "actuator") else None
        if st is None:
            continue
        out[slot] = {
            "position": float(getattr(st, "position", 0.0) or 0.0),
            "velocity": float(getattr(st, "velocity", 0.0) or 0.0),
        }
    return out


def seed_for_slot(proxy: object, slot: int) -> float:
    samples = feedback_positions_from_proxy(proxy)
    sample = samples.get(int(slot))
    if sample is None:
        return 0.0
    return float(sample["position"])


def spin_jog(
    engine: TeleopEngine,
    *,
    slots: Sequence[int],
    specs: Mapping[int, SlotSpec],
    seeds: Mapping[int, float],
    direction: int,
    cruise: float,
) -> None:
    """Engage wheel/base jog toward lo/hi (or seed-relative Damiao window)."""
    for slot in slots:
        s = int(slot)
        spec = specs.get(s)
        if spec is None:
            raise ValueError(f"no SlotSpec for slot {s}")
        engine.jog_actuator(
            s,
            spec=spec,
            seed=float(seeds.get(s, 0.0)),
            direction=int(direction),
            cruise=float(cruise),
        )


def move_arm_cruise(
    engine: TeleopEngine,
    *,
    slots: Sequence[int],
    specs: Mapping[int, SlotSpec],
    seeds: Mapping[int, float],
    targets: Mapping[int, float],
    cruise: float,
) -> None:
    """Engage verified arm slots toward absolute targets (mouse teleop core)."""
    for slot in slots:
        s = int(slot)
        if s not in targets:
            continue
        spec = specs.get(s)
        if spec is None:
            raise ValueError(f"no SlotSpec for slot {s}")
        engine.engage_actuator(
            s,
            spec=spec,
            seed=float(seeds.get(s, 0.0)),
            target=float(targets[s]),
            cruise=float(cruise),
        )


def stop_slots(engine: TeleopEngine, slots: Sequence[int]) -> None:
    for slot in slots:
        engine.stop_actuator(int(slot))


def specs_for_cfg_map(cfg_map: str = "bench") -> Dict[int, SlotSpec]:
    return build_actuator_specs(cfg_map)


__all__ = [
    "feedback_positions_from_proxy",
    "make_teleop_engine",
    "move_arm_cruise",
    "seed_for_slot",
    "specs_for_cfg_map",
    "spin_jog",
    "stop_slots",
]
