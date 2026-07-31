"""ServoAction — plant ServoDesire helpers for neck DXL (normal behaviour)."""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from deft_controls_sdk.config.servo import (
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
)
from deft_controls_sdk.link import ServoDesire

from .plant import PlantAction
from .sink import ServoSink

if TYPE_CHECKING:
    from deft_controls_sdk.config.typed_profiles import ServoProfile


class ServoAction(PlantAction):
    """Neck (or arbitrary) DXL plant commands."""

    def __init__(
        self,
        sink: ServoSink,
        *,
        servo_profile: Optional["ServoProfile"] = None,
    ) -> None:
        super().__init__(sink)
        self._servo_profile = servo_profile

    @classmethod
    def from_servo_profile(
        cls,
        sink: ServoSink,
        profile: "ServoProfile",
    ) -> "ServoAction":
        return cls(sink, servo_profile=profile)

    @property
    def servo_profile(self) -> Optional["ServoProfile"]:
        return self._servo_profile

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
        if self._servo_profile is not None and len(self._servo_profile.entries) >= 2:
            e0, e1 = self._servo_profile.entries[0], self._servo_profile.entries[1]
            pitch_slot, pitch_id = e0.slot, e0.dxl_id
            yaw_slot, yaw_id = e1.slot, e1.dxl_id
        self.set(pitch_slot, servo_id=pitch_id, position=position, send=False)
        self.set(yaw_slot, servo_id=yaw_id, position=position, send=send)

    def neck_clear(
        self,
        *,
        pitch_slot: int = NECK_PITCH_SERVO_SLOT,
        yaw_slot: int = NECK_YAW_SERVO_SLOT,
        send: bool = False,
    ) -> None:
        if self._servo_profile is not None and len(self._servo_profile.entries) >= 2:
            pitch_slot = self._servo_profile.entries[0].slot
            yaw_slot = self._servo_profile.entries[1].slot
        self.clear(pitch_slot, send=False)
        self.clear(yaw_slot, send=send)

    def center_profile(self, *, position: int = 2048, send: bool = False) -> None:
        """Center all entries in bound ``ServoProfile`` (or default neck)."""
        self.neck_center(position=position, send=send)

    def clear_profile(self, *, send: bool = False) -> None:
        self.neck_clear(send=send)


__all__ = ["ServoAction"]
