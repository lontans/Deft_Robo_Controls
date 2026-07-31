"""ServoAction — plant ServoDesire helpers for neck DXL (normal behaviour)."""
from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Optional

from deft_controls_sdk.config.servo import (
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
)
from deft_controls_sdk.link import ServoDesire

from .mounted import MountedAction
from .plant import PlantAction
from .sink import ServoSink

if TYPE_CHECKING:
    from deft_controls_sdk.config.typed_profiles import ServoProfile


class ServoAction(PlantAction):
    """Neck (or arbitrary) DXL plant commands.

    Prefer::

        a.mount(a.servo().neck_center())
        a.apply()
    """

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

    def _emit(self, mounted: MountedAction, *, send: bool) -> MountedAction:
        if self._actions is not None:
            if send:
                self._actions.mount(mounted)
                self._actions.apply()
            return mounted
        for slot, desire in mounted.servos.items():
            # Only the last write may TX when unbound (matches prior neck_*).
            self._sink.set_servo(int(slot), desire, send=False)
        if send and mounted.servos:
            # Re-touch last slot with send=True for one TX frame.
            last_slot = list(mounted.servos)[-1]
            self._sink.set_servo(last_slot, mounted.servos[last_slot], send=True)
        return mounted

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
    ) -> MountedAction:
        desire = ServoDesire(
            servo_id=int(servo_id) & 0xFF,
            native_step_position=int(position),
            native_speed_unit=int(speed),
            torque_enable=bool(torque_enable),
            operating_mode=int(operating_mode),
        )
        mounted = MountedAction.from_servos(
            f"servo.set[{int(slot)}]",
            {int(slot): desire},
            meta={"op": "set", "slot": int(slot)},
        )
        return self._emit(mounted, send=send)

    def clear(self, slot: int, *, send: bool = False) -> MountedAction:
        desire = ServoDesire(servo_id=0)
        return self._emit(
            MountedAction.from_servos(
                f"servo.clear[{int(slot)}]",
                {int(slot): desire},
                meta={"op": "clear", "slot": int(slot)},
            ),
            send=send,
        )

    def _neck_slots(
        self,
        *,
        pitch_id: int,
        yaw_id: int,
        pitch_slot: int,
        yaw_slot: int,
    ) -> tuple[int, int, int, int]:
        if self._servo_profile is not None and len(self._servo_profile.entries) >= 2:
            e0, e1 = self._servo_profile.entries[0], self._servo_profile.entries[1]
            return e0.slot, e0.dxl_id, e1.slot, e1.dxl_id
        return pitch_slot, pitch_id, yaw_slot, yaw_id

    def neck_center(
        self,
        *,
        pitch_id: int = NECK_PITCH_DXL_ID,
        yaw_id: int = NECK_YAW_DXL_ID,
        pitch_slot: int = NECK_PITCH_SERVO_SLOT,
        yaw_slot: int = NECK_YAW_SERVO_SLOT,
        position: int = 2048,
        send: bool = False,
    ) -> MountedAction:
        pitch_slot, pitch_id, yaw_slot, yaw_id = self._neck_slots(
            pitch_id=pitch_id,
            yaw_id=yaw_id,
            pitch_slot=pitch_slot,
            yaw_slot=yaw_slot,
        )
        batch: Dict[int, ServoDesire] = {
            int(pitch_slot): ServoDesire(
                servo_id=int(pitch_id) & 0xFF,
                native_step_position=int(position),
                torque_enable=True,
                operating_mode=3,
            ),
            int(yaw_slot): ServoDesire(
                servo_id=int(yaw_id) & 0xFF,
                native_step_position=int(position),
                torque_enable=True,
                operating_mode=3,
            ),
        }
        return self._emit(
            MountedAction.from_servos(
                "servo.neck_center",
                batch,
                meta={"op": "neck_center", "position": int(position)},
            ),
            send=send,
        )

    def neck_clear(
        self,
        *,
        pitch_slot: int = NECK_PITCH_SERVO_SLOT,
        yaw_slot: int = NECK_YAW_SERVO_SLOT,
        send: bool = False,
    ) -> MountedAction:
        if self._servo_profile is not None and len(self._servo_profile.entries) >= 2:
            pitch_slot = self._servo_profile.entries[0].slot
            yaw_slot = self._servo_profile.entries[1].slot
        batch = {
            int(pitch_slot): ServoDesire(servo_id=0),
            int(yaw_slot): ServoDesire(servo_id=0),
        }
        return self._emit(
            MountedAction.from_servos(
                "servo.neck_clear", batch, meta={"op": "neck_clear"}
            ),
            send=send,
        )

    def center_profile(self, *, position: int = 2048, send: bool = False) -> MountedAction:
        """Center all entries in bound ``ServoProfile`` (or default neck)."""
        return self.neck_center(position=position, send=send)

    def clear_profile(self, *, send: bool = False) -> MountedAction:
        return self.neck_clear(send=send)


__all__ = ["ServoAction"]
