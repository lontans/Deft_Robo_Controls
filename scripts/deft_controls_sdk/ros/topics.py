"""Topic name constants + QoS helpers for ``ControlsPcbHostNode``.

Namespaced by product *section* name (``left_arm``, ``base_wheel_1``, …), not
bus number — matches ``HostProxy.set_section`` / ``docs/integration.md``.

Name constants have no ``rclpy`` dependency; the QoS helpers import ``rclpy``
lazily so importing this module alone still works without ROS installed.
"""
from __future__ import annotations

LED_COMMAND_TOPIC = "led/command"
SERVO_NECK_COMMAND_TOPIC = "servo/neck/command"
PDU_STATUS_TOPIC = "pdu/status"


def command_topic(component: str) -> str:
    return f"actuators/{component}/command"


def state_topic(component: str) -> str:
    return f"actuators/{component}/state"


def command_qos():
    """Best-effort, keep-last-1 — teleop commands, stale ones are worthless."""
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def state_qos():
    """Best-effort, keep-last-1 — feedback publish, same shape as command_qos."""
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


__all__ = [
    "LED_COMMAND_TOPIC",
    "PDU_STATUS_TOPIC",
    "SERVO_NECK_COMMAND_TOPIC",
    "command_qos",
    "command_topic",
    "state_qos",
    "state_topic",
]
