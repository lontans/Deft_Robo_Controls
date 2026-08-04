"""Operate helpers — cruise/jog via ``TeleopEngine``.

Plant CMDH helpers (not CFG/discover). Lifecycle reminder::

    proxy.arm_plant()
    engine = make_teleop_engine(...)
    spin_jog(...) / neck_cruise(...) / move_arm_cruise(...)
    stop_slots(...) / stop_servos(...)
    engine.stop()
    proxy.disarm_plant()
"""
from __future__ import annotations

from typing import Callable, Dict, Mapping, Optional, Sequence

from deft_controls_sdk.config.profile import NECK_PITCH_SERVO_SLOT, NECK_YAW_SERVO_SLOT

from .arm_brace import ENGAGE_KP, J2_KP_SCALE
from .gravity_comp import GravityComp, I2RT_GRAVITY_SCALE, try_gravity_comp
from .teleop import SlotSpec, TeleopEngine, build_actuator_specs, build_servo_specs


HubGetter = Callable[[], object]
FeedbackGetter = Callable[[], Dict[int, dict]]


def make_teleop_engine(
    hub_getter: HubGetter,
    *,
    feedback_getter: Optional[FeedbackGetter] = None,
    hz: float = 60.0,
    brace_left_arm: bool = True,
    gravity: bool = True,
    gravity_scale: float = I2RT_GRAVITY_SCALE,
    gravity_comp: Optional[GravityComp] = None,
    arm_kp_scale: float = ENGAGE_KP,
    j2_kp_scale: float = J2_KP_SCALE,
) -> TeleopEngine:
    """Construct a shared cruise engine (actuators + neck servos).

    Left-arm teleop defaults match continuous ``--mouse``: brace all 7 joints,
    ENGAGE_KP / J2 boost, optional MuJoCo gravity FF (silently off if unavailable).
    """
    gc = gravity_comp
    if gc is None and gravity:
        gc = try_gravity_comp(scale=gravity_scale)
    return TeleopEngine(
        hub_getter=hub_getter,
        feedback_getter=feedback_getter,
        hz=hz,
        brace_left_arm=brace_left_arm,
        gravity_comp=gc,
        arm_kp_scale=arm_kp_scale,
        j2_kp_scale=j2_kp_scale,
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


def neck_cruise(
    engine: TeleopEngine,
    *,
    pitch: Optional[float] = None,
    yaw: Optional[float] = None,
    cruise: Optional[float] = None,
    specs: Optional[Mapping[int, SlotSpec]] = None,
) -> None:
    """Cruise neck servos to native DXL positions (same engine as wheel/arm).

    ``pitch`` / ``yaw`` are native step positions (e.g. 2048 center).
    Omit a joint to leave it alone. Requires ``proxy.arm_plant()`` first.
    """
    servo_specs = dict(specs) if specs is not None else build_servo_specs()
    targets = {
        NECK_PITCH_SERVO_SLOT: pitch,
        NECK_YAW_SERVO_SLOT: yaw,
    }
    for slot, target in targets.items():
        if target is None:
            continue
        spec = servo_specs.get(int(slot))
        if spec is None:
            raise ValueError(f"no SlotSpec for neck slot {slot}")
        rate = float(cruise) if cruise is not None else float(spec.cruise_default)
        engine.engage_servo(
            int(slot),
            spec=spec,
            target=float(target),
            cruise=rate,
        )


def stop_slots(engine: TeleopEngine, slots: Sequence[int]) -> None:
    for slot in slots:
        engine.stop_actuator(int(slot))


def stop_servos(
    engine: TeleopEngine,
    slots: Optional[Sequence[int]] = None,
) -> None:
    """Freeze neck (or given servo slots) in place."""
    if slots is None:
        slots = (NECK_PITCH_SERVO_SLOT, NECK_YAW_SERVO_SLOT)
    for slot in slots:
        engine.stop_servo(int(slot))


def specs_for_cfg_map(cfg_map: str = "bench") -> Dict[int, SlotSpec]:
    return build_actuator_specs(cfg_map)


__all__ = [
    "feedback_positions_from_proxy",
    "make_teleop_engine",
    "move_arm_cruise",
    "neck_cruise",
    "seed_for_slot",
    "specs_for_cfg_map",
    "spin_jog",
    "stop_servos",
    "stop_slots",
]
