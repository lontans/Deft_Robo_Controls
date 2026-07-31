"""Offline tests for typed profiles + Assembly."""
from __future__ import annotations

import os
import sys

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.actions import ActuatorAction  # noqa: E402
from deft_controls_sdk.config import (  # noqa: E402
    Assembly,
    ActuatorProfile,
    BASE_SLOTS,
    BENCH_BASE_SLOTS,
    LEFT_ARM_SLOTS,
    PROTO_ROBSTRIDE,
    RIGHT_ARM_SLOTS,
    arm_profile,
    bench_continuous_assembly,
    bench_continuous_profile,
    parse_slots_spec,
    single_profile,
    wheel_profile,
    yam_product_assembly,
    yam_product_profile,
)
from deft_controls_sdk.link import ActuatorDesire  # noqa: E402


def test_parse_slots_spec():
    assert parse_slots_spec("0-6", ()) == LEFT_ARM_SLOTS
    assert parse_slots_spec("22,23", ()) == (22, 23)
    assert parse_slots_spec(None, BASE_SLOTS) == BASE_SLOTS


def test_arm_and_wheel_factories():
    left = arm_profile("yam", side="left")
    assert left.name == "left_arm"
    assert left.slots == LEFT_ARM_SLOTS
    assert left.kind == "joint"
    assert left.cfg and left.cfg[0] is not None

    custom = arm_profile("yam", side="left", slots="0,1,2")
    assert custom.slots == (0, 1, 2)
    assert custom.cfg == ()

    base = wheel_profile(name="base")
    assert base.slots == BASE_SLOTS
    assert base.kind == "wheel"

    bench = wheel_profile(name="base", bench=True)
    assert bench.slots == BENCH_BASE_SLOTS


def test_single_profile_as_cfg_row():
    p = single_profile(22, protocol="robstride", motor_id=0x70, bus=5, kind="wheel")
    row = p.as_cfg_row()
    assert row == {
        "slot": 22,
        "bus": 5,
        "protocol": PROTO_ROBSTRIDE,
        "motor_id": 0x70,
        "master_id": 0,
        "enabled": True,
    }


def test_yam_assembly_and_demux_shim():
    asm = yam_product_assembly()
    assert set(asm.actuators) == {"left_arm", "right_arm", "base", "lift"}
    assert "neck" in asm.servos
    demux = asm.to_demux_profile()
    legacy = yam_product_profile()
    assert demux.name == legacy.name
    assert demux.components == legacy.components
    assert demux.slots("left_arm") == LEFT_ARM_SLOTS
    assert demux.slots("right_arm") == RIGHT_ARM_SLOTS


def test_bench_assembly_demux():
    asm = bench_continuous_assembly()
    demux = asm.to_demux_profile()
    assert demux.slots("base") == BENCH_BASE_SLOTS
    assert demux.slots("base_product") == BASE_SLOTS
    assert demux.components == bench_continuous_profile().components


def test_assembly_overlap_raises():
    a = arm_profile("yam", side="left")
    b = ActuatorProfile(name="dup", slots=(3, 4), kind="joint")
    with pytest.raises(ValueError, match="overlaps"):
        Assembly(name="bad", actuators={"left_arm": a, "dup": b})


def test_actuator_action_from_actuator_profile():
    class _Sink:
        def __init__(self) -> None:
            self.actuators = {}

        def set_actuators(self, desires, *, send=False):
            self.actuators.update(desires)

        def latest_feedback(self):
            return None

    sink = _Sink()
    prof = single_profile(22, protocol="rs", motor_id=0x70, bus=5, kind="wheel")
    action = ActuatorAction.from_actuator_profile(sink, prof)
    assert action.kind == "wheel"
    action.hold([0.25], send=False)
    assert sink.actuators[22].position == pytest.approx(0.25)
    nudged = action.nudge(delta=0.05, send=False)
    assert nudged[0] == pytest.approx(0.05)  # no FB → from zeros + delta
    assert isinstance(sink.actuators[22], ActuatorDesire)
