"""ControlsPcbHostNode tests against a fake HostProxy — skipped if rclpy is absent.

Same fake-hub pattern as test_host_proxy.py; only the ROS glue (topic wiring,
command -> ActuatorAction/LedAction/ServoAction dispatch) is under test here,
not real COM or a spinning executor.
"""
from __future__ import annotations

import os
import sys

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

rclpy = pytest.importorskip("rclpy")

from deft_controls_sdk.config import LEFT_ARM_SLOTS, yam_product_profile  # noqa: E402
from deft_controls_sdk.host_proxy import HostProxy  # noqa: E402
from deft_controls_sdk.ros.node import ControlsPcbHostNode  # noqa: E402
from pcb_lab.tests.test_host_proxy import _FakeHub  # noqa: E402


@pytest.fixture()
def rclpy_ctx():
    rclpy.init(args=None)
    try:
        yield
    finally:
        rclpy.shutdown()


def _wrapped_proxy() -> HostProxy:
    return HostProxy.wrap(_FakeHub(), profile=yam_product_profile())


def test_default_components_match_profile(rclpy_ctx):
    proxy = _wrapped_proxy()
    node = ControlsPcbHostNode(proxy=proxy)
    try:
        assert node._components == ["left_arm", "right_arm", "base", "lift"]
        assert set(node._state_pubs) == set(node._components)
    finally:
        node.destroy_node()


def test_components_allow_list_filters(rclpy_ctx):
    proxy = _wrapped_proxy()
    node = ControlsPcbHostNode(proxy=proxy, components=["left_arm", "not_a_component"])
    try:
        assert node._components == ["left_arm"]
    finally:
        node.destroy_node()


def test_actuator_command_calls_hold(rclpy_ctx):
    from std_msgs.msg import Float64MultiArray

    proxy = _wrapped_proxy()
    node = ControlsPcbHostNode(proxy=proxy, components=["left_arm"])
    try:
        msg = Float64MultiArray()
        msg.data = [0.1 * i for i in range(7)]
        node._make_actuator_command_cb("left_arm")(msg)
        for i, slot in enumerate(LEFT_ARM_SLOTS):
            desire = proxy.hub._connection.actuators[slot]
            assert desire.position == pytest.approx(0.1 * i)
    finally:
        node.destroy_node()


def test_led_command_applies_preset(rclpy_ctx):
    from std_msgs.msg import String

    proxy = _wrapped_proxy()
    node = ControlsPcbHostNode(proxy=proxy, components=[])
    try:
        node._on_led_command(String(data="idle"))
        assert proxy.hub.led_desire is not None
    finally:
        node.destroy_node()


def test_servo_command_dispatches_pitch_and_yaw(rclpy_ctx):
    from sensor_msgs.msg import JointState

    proxy = _wrapped_proxy()
    node = ControlsPcbHostNode(proxy=proxy, components=[])
    calls = []
    proxy.set_servo = lambda slot, desire, *, send=False: calls.append((slot, desire.native_step_position))
    try:
        msg = JointState()
        msg.name = ["neck_pitch", "neck_yaw"]
        msg.position = [10.0, 20.0]
        node._on_servo_command(msg)
        assert (0, 10) in calls
        assert (1, 20) in calls
    finally:
        node.destroy_node()


def test_soft_dfu_mode_rejected(rclpy_ctx):
    proxy = _wrapped_proxy()
    with pytest.raises(ValueError):
        ControlsPcbHostNode(proxy=proxy, mode="soft_dfu")


def test_timer_publishes_feedback_none_positions_skipped(rclpy_ctx):
    proxy = _wrapped_proxy()
    node = ControlsPcbHostNode(proxy=proxy, components=["left_arm"])
    try:
        # _FakeHub reports no feedback yet -> positions() is None -> no publish,
        # and the timer callback must not raise.
        node._on_timer()
    finally:
        node.destroy_node()
