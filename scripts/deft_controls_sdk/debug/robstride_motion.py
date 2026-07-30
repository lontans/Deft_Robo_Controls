"""Shared RobStride plant-motion helpers used by bringup / continuous / base lab.

Not limited to RS02 — MIT rail helpers for the RobStride family (RS01/RS02/…).
"""
from __future__ import annotations

import time
from typing import Optional

from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.debug.metrics import drain_latest
from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import parse_actuator_feedback, parse_feedback_header

PROTO_ROBSTRIDE = 1

# MIT encode range in firmware (robstride.h RS02_P_MIN/MAX — family rail).
ROBSTRIDE_P_MIN = -12.57
ROBSTRIDE_P_MAX = 12.57
ROBSTRIDE_P_SPAN = ROBSTRIDE_P_MAX - ROBSTRIDE_P_MIN

# Back-compat aliases (older continuous / scripts).
RS02_P_MIN = ROBSTRIDE_P_MIN
RS02_P_MAX = ROBSTRIDE_P_MAX
RS02_P_SPAN = ROBSTRIDE_P_SPAN


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def robstride_near(a: float, b: float, eps: float = 0.40) -> bool:
    """True if poses match linearly or as opposite ends of the MIT rail."""
    d = abs(float(a) - float(b))
    return d <= eps or abs(d - ROBSTRIDE_P_SPAN) <= eps


def robstride_resolve_start(
    probe_pos: Optional[float], plant_fb: Optional[float]
) -> Optional[float]:
    """Prefer probe when plant FB is stale across the ±12.57 rail."""
    if probe_pos is None and plant_fb is None:
        return None
    if probe_pos is None:
        return float(plant_fb)  # type: ignore[arg-type]
    if plant_fb is None:
        return float(probe_pos)
    if robstride_near(probe_pos, plant_fb):
        return float(plant_fb)
    return float(probe_pos)


def robstride_plan_angle(
    start: float,
    want_angle: float,
    *,
    margin: float = 0.20,
) -> float:
    """Signed travel that stays inside the MIT range (flip/shrink if needed)."""
    travel = abs(float(want_angle))
    if travel < 1e-6:
        return 0.0
    prefer = 1.0 if want_angle >= 0.0 else -1.0
    lo = ROBSTRIDE_P_MIN + margin
    hi = ROBSTRIDE_P_MAX - margin
    room_pos = max(0.0, hi - float(start))
    room_neg = max(0.0, float(start) - lo)

    def _fit(sign: float) -> float:
        room = room_pos if sign > 0.0 else room_neg
        return sign * min(travel, room)

    primary = _fit(prefer)
    alternate = _fit(-prefer)
    if abs(primary) >= travel - 1e-3:
        return primary
    if abs(alternate) > abs(primary) + 1e-6:
        return alternate
    return primary


def quiet_all_slots(hub: ControlsPcbHub) -> int:
    """Disable every enabled CFG slot (RAM). Stops ghost MIT at p=0 on other buses."""
    n = 0
    table = hub.debug.cfg_get_table()
    for s, row in enumerate(table):
        if not bool(row.get("enabled", False)):
            continue
        hub.debug.cfg_set_slot(
            slot=s,
            bus=int(row.get("bus", 1)),
            protocol=int(row.get("protocol", PROTO_ROBSTRIDE)),
            motor_id=int(row.get("motor_id", 1)),
            enabled=False,
            persist=False,
        )
        n += 1
    return n


def seed_idle_at_fb(hub: ControlsPcbHub, slot: int, pos: float) -> None:
    """Hold last FB pose with kp=0 — never leave plant desire at p=0 while shaft elsewhere."""
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    desires[slot] = ActuatorDesire(position=pos, velocity=0.0, kp=0.0, kd=0.0)
    _conn(hub).set_actuators(desires, send=True)


def sample_position(
    hub: ControlsPcbHub, slot: int, *, timeout_s: float = 1.0
) -> Optional[float]:
    deadline = time.monotonic() + timeout_s
    _conn(hub).send_once()
    while time.monotonic() < deadline:
        raw = drain_latest(hub)
        if raw is None:
            time.sleep(0.005)
            continue
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        act = parse_actuator_feedback(raw, slot)
        if act is not None:
            return float(act["position"])
        time.sleep(0.005)
    return None


# Legacy names (yam_continuous / older imports).
rs02_near = robstride_near
rs02_resolve_start = robstride_resolve_start
rs02_plan_angle = robstride_plan_angle
