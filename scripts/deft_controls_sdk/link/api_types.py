"""Typed host objects — desires out, feedback in (not raw wire helpers)."""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Mapping, Optional, Union

from .exceptions import InvalidFrameError, InvalidSlotError
from .exchange import (
    ACTUATOR_COUNT,
    HOST_COMMAND_MAGIC,
    HOST_EXCHANGE_ACTUATOR_SLOTS,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    parse_actuator_feedback,
    parse_feedback_header,
    patch_actuator_desire,
    patch_led_command,
    patch_servo_command,
    patch_system_mcu_state,
    patch_system_rx_sim,
    patch_system_rx_sim_mask,
)


class McuState(IntEnum):
    """system.mcu_state (host_system_command_t bits 1-3)."""

    NORMAL = 0
    RECOVERY = 1
    DIAG_ONLY = 2
    ESTOP = 3


@dataclass(frozen=True)
class ActuatorDesire:
    """Raw MIT desire for one actuator slot — position (rad), velocity (rad/s), kp, kd,
    torque (Nm). An all-zero desire is a valid idle/no-torque command.
    """

    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


IDLE = ActuatorDesire()


@dataclass(frozen=True)
class ServoDesire:
    """One DXL host command (6 B). servo_id==0 clears the slot / ends session when both clear."""

    servo_id: int = 0
    native_step_position: int = 0
    native_speed_unit: int = 0
    torque_enable: bool = True
    led_control: bool = False
    operating_mode: int = 3  # position control


@dataclass(frozen=True)
class LedDesire:
    """SK9822 host command (2 B). mode 0=OFF, 1=TEST chase, 2=FLASH. led_count 0 ⇒ max (300)."""

    mode: int = 0
    master_brightness: int = 8
    led_count: int = 0


def validate_slot(slot: int) -> None:
    if not (0 <= slot < ACTUATOR_COUNT):
        raise InvalidSlotError(
            f"slot must be 0..{ACTUATOR_COUNT - 1} "
            f"(ACTUATOR_COUNT={ACTUATOR_COUNT}, wire={HOST_EXCHANGE_ACTUATOR_SLOTS}); "
            f"got {slot}."
        )


class CommandImage:
    """Mutable builder for one 672 B command frame."""

    def __init__(self, seq: int = 0, mcu_state: McuState = McuState.NORMAL) -> None:
        self._buf = bytearray(IMAGE_BYTES)
        struct.pack_into(
            "<IHHI", self._buf, 0, HOST_COMMAND_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, seq & 0xFFFFFFFF
        )
        patch_system_mcu_state(self._buf, int(mcu_state))
        self._desires: Dict[int, ActuatorDesire] = {}
        self._servos: Dict[int, ServoDesire] = {}
        self._led: Optional[LedDesire] = None
        self._rx_sim_mask = 0

    @property
    def seq(self) -> int:
        seq, = struct.unpack_from("<I", self._buf, 8)
        return seq

    def set_seq(self, seq: int) -> "CommandImage":
        struct.pack_into("<I", self._buf, 8, seq & 0xFFFFFFFF)
        return self

    def set_mcu_state(self, state: McuState) -> "CommandImage":
        patch_system_mcu_state(self._buf, int(state))
        return self

    def set_rx_sim(self, enable: bool) -> "CommandImage":
        """True → ACTUATOR rx_sim only (bit0). Prefer set_rx_sim_mask for children."""
        return self.set_rx_sim_mask(0x1 if enable else 0)

    def set_rx_sim_mask(self, mask: int) -> "CommandImage":
        """system.reserved bits0..3: ACTUATOR|SERVO|LED|PDU."""
        self._rx_sim_mask = int(mask) & 0xF
        patch_system_rx_sim_mask(self._buf, self._rx_sim_mask)
        return self

    def set_actuator(self, slot: int, desire: ActuatorDesire) -> "CommandImage":
        validate_slot(slot)
        patch_actuator_desire(
            self._buf, desire.position, desire.velocity, desire.kp, desire.kd, desire.torque, slot=slot
        )
        self._desires[slot] = desire
        return self

    def set_actuators(self, desires: Mapping[int, ActuatorDesire]) -> "CommandImage":
        for slot, desire in desires.items():
            self.set_actuator(slot, desire)
        return self

    def set_servo(self, slot: int, desire: ServoDesire) -> "CommandImage":
        if slot not in (0, 1):
            raise InvalidSlotError(f"servo slot must be 0..1, got {slot}")
        patch_servo_command(
            self._buf,
            slot,
            servo_id=desire.servo_id,
            native_step_position=desire.native_step_position,
            native_speed_unit=desire.native_speed_unit,
            torque_enable=desire.torque_enable,
            led_control=desire.led_control,
            operating_mode=desire.operating_mode,
        )
        self._servos[slot] = desire
        return self

    def set_servos(self, desires: Mapping[int, ServoDesire]) -> "CommandImage":
        for slot, desire in desires.items():
            self.set_servo(slot, desire)
        return self

    def set_led(self, desire: LedDesire) -> "CommandImage":
        patch_led_command(
            self._buf,
            mode=desire.mode,
            master_brightness=desire.master_brightness,
            led_count=desire.led_count,
        )
        self._led = desire
        return self

    def desire(self, slot: int) -> ActuatorDesire:
        validate_slot(slot)
        return self._desires.get(slot, IDLE)

    def to_bytes(self) -> bytes:
        return bytes(self._buf)

    def __bytes__(self) -> bytes:
        return self.to_bytes()

    def __len__(self) -> int:
        return len(self._buf)


class PlantBlockReason(IntEnum):
    """diag.h plant_block_reason_t — why actuator_apply_desire() is not driving CAN."""

    NONE = 0
    BENCH_SESSION = 1
    PROBE_BUSY = 2
    QUIET_PERIOD = 3
    DIAG_ONLY = 4
    HOST_STALE = 5
    SERVO_SESSION = 6


@dataclass(frozen=True)
class FeedbackState:
    """One actuator_feedback[] slot."""

    position: float
    velocity: float
    torque: float
    temperature: float
    fault: int


class FeedbackImage:
    """Parsed 672 B feedback frame — raises InvalidFrameError on bad magic/size."""

    __slots__ = ("_raw", "_header", "_slots")

    def __init__(self, raw: bytes) -> None:
        if len(raw) != IMAGE_BYTES:
            raise InvalidFrameError(f"expected {IMAGE_BYTES} B frame, got {len(raw)}")
        header = parse_feedback_header(raw)
        if header is None:
            raise InvalidFrameError("bad magic/layout_version/byte_size in feedback frame")
        self._raw = raw
        self._header = header
        slots: List[Optional[FeedbackState]] = []
        for slot in range(ACTUATOR_COUNT):
            act = parse_actuator_feedback(raw, slot)
            slots.append(
                FeedbackState(act["position"], act["velocity"], act["torque"], act["temperature"], act["fault"])
                if act is not None
                else None
            )
        self._slots = slots

    @property
    def raw(self) -> bytes:
        return self._raw

    @property
    def tick(self) -> int:
        """TIM6 control_tick_count, 12-bit, wraps at 4096."""
        return self._header["tick"]

    @property
    def ack_seq(self) -> int:
        """8-bit echo of the mounted command's header.seq (seq & 0xFF)."""
        return self._header["last_cmd_seq"]

    @property
    def mcu_state(self) -> int:
        return self._header["mcu_state"]

    @property
    def plant_block(self) -> Union[PlantBlockReason, int]:
        """PlantBlockReason, or the raw int if firmware reports an unmapped code."""
        raw = self._header["plant_block"]
        try:
            return PlantBlockReason(raw)
        except ValueError:
            return raw

    def actuator(self, slot: int) -> Optional[FeedbackState]:
        if not (0 <= slot < ACTUATOR_COUNT):
            raise ValueError(f"slot must be 0..{ACTUATOR_COUNT - 1}, got {slot}")
        return self._slots[slot]

    @property
    def actuators(self) -> List[Optional[FeedbackState]]:
        return list(self._slots)
