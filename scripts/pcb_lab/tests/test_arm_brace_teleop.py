"""Offline: left-arm brace writer + TeleopEngine brace path."""
from __future__ import annotations

import os
import sys
import time

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.actions.arm_brace import (  # noqa: E402
    ENGAGE_KP,
    J2_KP_SCALE,
    lead_clamp,
    write_left_arm,
)
from deft_controls_sdk.actions.operate import make_teleop_engine  # noqa: E402
from deft_controls_sdk.actions.teleop import build_actuator_specs  # noqa: E402
from deft_controls_sdk.config.actuator import DEFAULT_ARM_KP, LEFT_ARM_SLOTS  # noqa: E402
from deft_controls_sdk.link import ActuatorDesire  # noqa: E402


class _FakeHub:
    def __init__(self) -> None:
        self.actuators: dict = {}

    def set_actuator(self, slot, desire, *, send=False):
        self.actuators[int(slot)] = desire

    def set_actuators(self, desires, *, send=False):
        for s, d in desires.items():
            self.actuators[int(s)] = d


def test_lead_clamp_limits_command():
    assert lead_clamp(1.0, 0.0) == 0.25
    assert abs(lead_clamp(0.1, 0.0) - 0.1) < 1e-9
    assert lead_clamp(-1.0, 0.0, j2=True) == -0.32


def test_write_left_arm_braces_all_seven():
    hub = _FakeHub()
    q = [0.0, -3.0, 2.0, 0.0, 0.0, 0.0, 1.5]
    write_left_arm(hub, q, kp_scale=ENGAGE_KP, j2_kp_scale=J2_KP_SCALE)
    assert set(hub.actuators) == set(LEFT_ARM_SLOTS)
    # J2 uses boost > ENGAGE_KP
    j2 = hub.actuators[LEFT_ARM_SLOTS[1]]
    assert abs(j2.kp - DEFAULT_ARM_KP[1] * J2_KP_SCALE) < 1e-6
    j1 = hub.actuators[LEFT_ARM_SLOTS[0]]
    assert abs(j1.kp - DEFAULT_ARM_KP[0] * ENGAGE_KP) < 1e-6


def test_teleop_engine_braces_siblings_when_one_arm_joint_engaged():
    hub = _FakeHub()
    fb = {
        s: {"position": float(i) * 0.1, "velocity": 0.0}
        for i, s in enumerate(LEFT_ARM_SLOTS)
    }
    engine = make_teleop_engine(
        lambda: hub,
        feedback_getter=lambda: fb,
        hz=100.0,
        brace_left_arm=True,
        gravity=False,
    )
    specs = build_actuator_specs("bench")
    slot0 = LEFT_ARM_SLOTS[0]
    engine.engage_actuator(
        slot0,
        spec=specs[slot0],
        seed=0.0,
        target=0.2,
        cruise=0.5,
    )
    time.sleep(0.08)
    engine.stop()
    # All 7 left-arm slots written (brace), not only slot 0
    assert set(LEFT_ARM_SLOTS).issubset(set(hub.actuators))
    for s in LEFT_ARM_SLOTS:
        assert isinstance(hub.actuators[s], ActuatorDesire)
        assert hub.actuators[s].kp > 0.0
