"""562 B command image builder — actuator_commands[] + mcu_state only.

No pdu, servo, or LED writes happen here on purpose: this is the "normal operation"
surface described in docs/controls_hub-controller.md. Use controls_pcb_host directly
for RS2/DM bench probes, servo control, or LED control.

Wire layout is v1 (docs/host-exchange-v1.md); offsets are imported from
controls_pcb_host.protocol, the existing mirror of App/Inc/host/host_exchange_schema.h —
this module does not redefine them.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Mapping

from controls_pcb_host.commands import patch_actuator_desire, patch_system_mcu_state
from controls_pcb_host.protocol import (
    ACTUATOR_COUNT,
    HOST_COMMAND_MAGIC,
    HOST_EXCHANGE_ACTUATOR_SLOTS,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
)

from .exceptions import InvalidSlotError


class McuState(IntEnum):
    """system.mcu_state (host_system_command_t bits 1-3)."""

    NORMAL = 0
    RECOVERY = 1
    DIAG_ONLY = 2
    ESTOP = 3


@dataclass(frozen=True)
class ActuatorDesire:
    """Raw MIT desire for one actuator slot — position (rad), velocity (rad/s), kp, kd,
    torque (Nm). No implicit defaults beyond IEEE-754 zero: an all-zero desire is a
    real, valid command (idle/no-torque), not "leave unchanged".

    Firmware copies these fields verbatim from the wire image into live state every
    dispatch (App/Src/plant/actuator.c: actuator_command_mount / actuator_apply_desire)
    — there is no host-invisible clamping, ramping, or unit conversion at this layer.
    Protocol plugins (robstride.c, damiao.c) clamp to motor-specific min/max only when
    packing the CAN frame.
    """

    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


IDLE = ActuatorDesire()


def validate_slot(slot: int) -> None:
    if not (0 <= slot < ACTUATOR_COUNT):
        raise InvalidSlotError(
            f"slot must be 0..{ACTUATOR_COUNT - 1} (firmware ACTUATOR_COUNT); got {slot}. "
            f"The wire image carries {HOST_EXCHANGE_ACTUATOR_SLOTS} actuator slots but "
            f"actuator_command_mount() only reads the first {ACTUATOR_COUNT} — the rest "
            f"are dead on the wire."
        )


class CommandImage:
    """Mutable builder for one 562 B command frame.

    Every ``set_actuator``/``set_actuators`` call is a full overwrite of that slot's
    desire on the wire — firmware has no per-slot "unchanged" sentinel, so any slot not
    set on a given image is sent as an all-zero (idle) desire. ``PlantSession`` handles
    resending the full held state every frame; use ``CommandImage`` directly only if you
    are managing that bookkeeping yourself.
    """

    def __init__(self, seq: int = 0, mcu_state: McuState = McuState.NORMAL) -> None:
        self._buf = bytearray(IMAGE_BYTES)
        struct.pack_into(
            "<IHHI", self._buf, 0, HOST_COMMAND_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, seq & 0xFFFFFFFF
        )
        patch_system_mcu_state(self._buf, int(mcu_state))
        self._desires: Dict[int, ActuatorDesire] = {}

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

    def desire(self, slot: int) -> ActuatorDesire:
        validate_slot(slot)
        return self._desires.get(slot, IDLE)

    def to_bytes(self) -> bytes:
        return bytes(self._buf)

    def __bytes__(self) -> bytes:
        return self.to_bytes()

    def __len__(self) -> int:
        return len(self._buf)
