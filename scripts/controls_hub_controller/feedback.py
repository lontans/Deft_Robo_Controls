"""562 B feedback image parser — actuator_feedback[] + plant_block/tick/ack_seq only.

Reuses controls_pcb_host.feedback for header/slot bit math (single source of truth for
the sys_word layout) and wraps the result in typed, read-only objects.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Union

from controls_pcb_host.feedback import parse_actuator_feedback, parse_feedback_header
from controls_pcb_host.protocol import ACTUATOR_COUNT, IMAGE_BYTES

from .exceptions import InvalidFrameError


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
    """One actuator_feedback[] slot: position (rad), velocity (rad/s), torque (Nm),
    temperature (deg C), fault (raw uint32 flags — protocol-specific)."""

    position: float
    velocity: float
    torque: float
    temperature: float
    fault: int


class FeedbackImage:
    """Parsed 562 B feedback frame — raises InvalidFrameError on bad magic/size."""

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
