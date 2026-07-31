"""Offline tests for deft_controls_sdk.ros — pure conversion + no-rclpy import.

bridges.py / topics.py name constants must be importable and testable with
no ROS install; node.py legitimately needs rclpy (see test_ros_node.py).
"""
from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.ros import bridges, topics  # noqa: E402


def test_ros_package_import_does_not_require_rclpy():
    assert "rclpy" not in sys.modules


def test_import_deft_controls_sdk_does_not_require_rclpy():
    import deft_controls_sdk  # noqa: F401

    assert "rclpy" not in sys.modules


def test_positions_from_command_float64_multi_array():
    msg = SimpleNamespace(data=[0.1, 0.2, 0.3])
    assert bridges.positions_from_command(msg) == pytest.approx([0.1, 0.2, 0.3])


def test_positions_from_command_joint_state():
    msg = SimpleNamespace(position=[1.0, 2.0], name=["j0", "j1"])
    assert bridges.positions_from_command(msg) == pytest.approx([1.0, 2.0])


def test_positions_from_command_rejects_unknown_shape():
    with pytest.raises(TypeError):
        bridges.positions_from_command(SimpleNamespace(foo=1))


def test_joint_names():
    assert bridges.joint_names(3) == ["j0", "j1", "j2"]


def test_led_command_from_msg_normalizes():
    assert bridges.led_command_from_msg(SimpleNamespace(data=" Idle ")) == "idle"


def test_neck_positions_by_name():
    msg = SimpleNamespace(name=["neck_yaw", "neck_pitch"], position=[20, 10])
    assert bridges.neck_positions_from_joint_state(msg) == (10, 20)


def test_neck_positions_positional_fallback():
    msg = SimpleNamespace(name=[], position=[5, 6])
    assert bridges.neck_positions_from_joint_state(msg) == (5, 6)


def test_neck_positions_requires_two_values():
    with pytest.raises(ValueError):
        bridges.neck_positions_from_joint_state(SimpleNamespace(name=[], position=[1]))


def test_topic_names():
    assert topics.command_topic("left_arm") == "actuators/left_arm/command"
    assert topics.state_topic("base") == "actuators/base/state"
    assert topics.LED_COMMAND_TOPIC == "led/command"
    assert topics.SERVO_NECK_COMMAND_TOPIC == "servo/neck/command"
