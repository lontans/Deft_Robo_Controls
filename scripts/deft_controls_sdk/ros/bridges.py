"""Pure msg <-> plant-value conversion helpers — no rclpy, no COM.

Takes/returns plain Python values (and duck-typed message-shaped objects) so
this module is importable and unit-testable without ROS installed. ``node.py``
is the only place that touches real ``rclpy`` / ``std_msgs`` / ``sensor_msgs``
types.
"""
from __future__ import annotations

from typing import Any, List, Tuple


def positions_from_command(msg: Any) -> List[float]:
    """``std_msgs/Float64MultiArray.data`` or ``sensor_msgs/JointState.position``.

    Duck-types on whichever attribute is present so a command topic can be
    swapped between the two message types without touching this function.
    """
    data = getattr(msg, "position", None)
    if data is None:
        data = getattr(msg, "data", None)
    if data is None:
        raise TypeError(
            f"{type(msg).__name__} has neither .position nor .data — "
            "expected Float64MultiArray or JointState"
        )
    return [float(v) for v in data]


def joint_names(n: int) -> List[str]:
    """``j0..jN`` slot-index names for a published ``JointState``."""
    return [f"j{i}" for i in range(n)]


def led_command_from_msg(msg: Any) -> str:
    """``std_msgs/String.data`` -> normalized LED preset name."""
    return str(msg.data).strip().lower()


def neck_positions_from_joint_state(
    msg: Any,
    *,
    pitch_name: str = "neck_pitch",
    yaw_name: str = "neck_yaw",
) -> Tuple[int, int]:
    """(pitch, yaw) native DXL step positions from a 2-element JointState.

    Matches by ``name`` when both names are present; otherwise falls back to
    positional order ``[pitch, yaw]``.
    """
    names = list(getattr(msg, "name", None) or [])
    positions = list(getattr(msg, "position", None) or [])
    if names and len(names) == len(positions):
        by_name = dict(zip(names, positions))
        if pitch_name in by_name and yaw_name in by_name:
            return int(by_name[pitch_name]), int(by_name[yaw_name])
    if len(positions) >= 2:
        return int(positions[0]), int(positions[1])
    raise ValueError(
        f"expected 2 positions (pitch, yaw) for neck servo command, got {positions!r}"
    )


__all__ = [
    "joint_names",
    "led_command_from_msg",
    "neck_positions_from_joint_state",
    "positions_from_command",
]
