"""Dynamixel neck helpers (2-DOF) for PCB servo slots."""
from __future__ import annotations

from deft_controls_sdk.link import ServoDesire
from deft_controls_sdk.vbeta.session import PcbRobotSession
from deft_controls_sdk.vbeta.slots import (
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
)

# XL-class approx: 4096 steps / 360 deg, center 2048 ≈ 0 deg
_STEPS_PER_DEG = 4096.0 / 360.0
_CENTER = 2048


def deg_to_steps(deg: float) -> int:
    return int(round(_CENTER + float(deg) * _STEPS_PER_DEG))


def steps_to_deg(steps: int) -> float:
    return (int(steps) - _CENTER) / _STEPS_PER_DEG


class PcbNeckDriver:
    """Pitch/yaw in degrees → host servo slots 0/1."""

    def __init__(
        self,
        session: PcbRobotSession,
        *,
        pitch_offset_deg: float = 30.0,
        pitch_id: int = NECK_PITCH_DXL_ID,
        yaw_id: int = NECK_YAW_DXL_ID,
    ) -> None:
        self._session = session
        self.pitch_offset_deg = float(pitch_offset_deg)
        self.pitch_id = int(pitch_id)
        self.yaw_id = int(yaw_id)

    def go_to(self, pitch_deg: float, yaw_deg: float) -> None:
        pitch_cmd = float(pitch_deg) + self.pitch_offset_deg
        self._session.set_servo(
            NECK_PITCH_SERVO_SLOT,
            ServoDesire(
                servo_id=self.pitch_id,
                native_step_position=deg_to_steps(pitch_cmd),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )
        self._session.set_servo(
            NECK_YAW_SERVO_SLOT,
            ServoDesire(
                servo_id=self.yaw_id,
                native_step_position=deg_to_steps(yaw_deg),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )

    def disable(self) -> None:
        self._session.set_servo(
            NECK_PITCH_SERVO_SLOT, ServoDesire(servo_id=0), send=False
        )
        self._session.set_servo(
            NECK_YAW_SERVO_SLOT, ServoDesire(servo_id=0), send=False
        )
