"""ServoAction — plant ServoDesire helpers for neck DXL (normal behaviour)."""
from __future__ import annotations

from typing import Optional

from deft_controls_sdk.config.servo import (
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
)
from deft_controls_sdk.link import ServoDesire

from .sink import ServoSink


class ServoAction:
    """Neck (or arbitrary) DXL plant commands."""

    def __init__(self, sink: ServoSink) -> None:
        self._sink = sink

    def set(
        self,
        slot: int,
        *,
        servo_id: int,
        position: int = 2048,
        speed: int = 0,
        torque_enable: bool = True,
        operating_mode: int = 3,
        send: bool = False,
    ) -> ServoDesire:
        desire = ServoDesire(
            servo_id=int(servo_id) & 0xFF,
            native_step_position=int(position),
            native_speed_unit=int(speed),
            torque_enable=bool(torque_enable),
            operating_mode=int(operating_mode),
        )
        self._sink.set_servo(int(slot), desire, send=send)
        return desire

    def clear(self, slot: int, *, send: bool = False) -> ServoDesire:
        desire = ServoDesire(servo_id=0)
        self._sink.set_servo(int(slot), desire, send=send)
        return desire

    def neck_center(
        self,
        *,
        pitch_id: int = NECK_PITCH_DXL_ID,
        yaw_id: int = NECK_YAW_DXL_ID,
        pitch_slot: int = NECK_PITCH_SERVO_SLOT,
        yaw_slot: int = NECK_YAW_SERVO_SLOT,
        position: int = 2048,
        send: bool = False,
    ) -> None:
        self.set(pitch_slot, servo_id=pitch_id, position=position, send=False)
        self.set(yaw_slot, servo_id=yaw_id, position=position, send=send)

    def neck_clear(
        self,
        *,
        pitch_slot: int = NECK_PITCH_SERVO_SLOT,
        yaw_slot: int = NECK_YAW_SERVO_SLOT,
        send: bool = False,
    ) -> None:
        self.clear(pitch_slot, send=False)
        self.clear(yaw_slot, send=send)


__all__ = ["ServoAction"]
