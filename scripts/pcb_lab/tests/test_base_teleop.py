"""Offline tests: SlotSpec.role/wheel_module + build_base_wheel_specs (no hardware)."""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.actions.base_teleop import build_base_wheel_specs  # noqa: E402
from deft_controls_sdk.actions.teleop import BASE_BENCH_ROWS, build_actuator_specs  # noqa: E402
from deft_controls_sdk.config.profile import (  # noqa: E402
    BASE_DRIVE_SLOTS,
    BASE_STEER_SLOTS,
    BASE_WHEEL_SLOTS,
)


def test_arm_and_neck_specs_default_to_position_role():
    specs = build_actuator_specs("bench")
    assert specs[0].role == "position"  # left arm
    assert specs[0].wheel_module is None
    assert specs[7].role == "position"  # right arm

    from deft_controls_sdk.actions.teleop import build_servo_specs

    servo_specs = build_servo_specs()
    assert all(s.role == "position" for s in servo_specs.values())


def test_bench_base_slots_are_all_drive_role():
    specs = build_actuator_specs("bench")
    for slot, _label in BASE_BENCH_ROWS:
        assert specs[slot].group == "base"
        assert specs[slot].role == "drive"
        assert specs[slot].wheel_module is None  # bench slots aren't paired into wheel modules


def test_product_steer_and_drive_slots_get_correct_role_and_wheel_module():
    specs = build_actuator_specs("product")
    for label, slot in BASE_STEER_SLOTS.items():
        spec = specs[slot]
        assert spec.group == "base"
        assert spec.role == "steer", f"{label} (slot {slot}) expected role=steer"
    for label, slot in BASE_DRIVE_SLOTS.items():
        spec = specs[slot]
        assert spec.group == "base"
        assert spec.role == "drive", f"{label} (slot {slot}) expected role=drive"

    # Cross-check wheel_module against config.profile.BASE_WHEEL_SLOTS directly.
    for wheel_name, slots in BASE_WHEEL_SLOTS.items():
        for slot in slots:
            assert specs[slot].wheel_module == wheel_name


def test_build_base_wheel_specs_bench_filters_to_base_group():
    specs = build_base_wheel_specs("bench")
    assert set(specs.keys()) == {slot for slot, _label in BASE_BENCH_ROWS}
    assert all(s.group == "base" for s in specs.values())
    assert all(s.role == "drive" for s in specs.values())


def test_build_base_wheel_specs_product_filters_to_base_group():
    specs = build_base_wheel_specs("product")
    expected_slots = set(BASE_STEER_SLOTS.values()) | set(BASE_DRIVE_SLOTS.values())
    assert set(specs.keys()) == expected_slots
    assert all(s.group == "base" for s in specs.values())
    # No arm/neck slots leaked through.
    assert 0 not in specs and 7 not in specs
