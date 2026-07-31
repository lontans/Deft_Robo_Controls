"""PlantSink — minimal protocols for plant desire TX + feedback readback."""
from __future__ import annotations

from typing import Mapping, Optional, Protocol, runtime_checkable

from deft_controls_sdk.link import ActuatorDesire, FeedbackImage, LedDesire, ServoDesire


@runtime_checkable
class PlantSink(Protocol):
    """HostProxy and ControlsPcbHub both satisfy this for ActuatorAction."""

    def set_actuators(
        self, desires: Mapping[int, ActuatorDesire], *, send: bool = False
    ) -> None: ...

    def latest_feedback(self) -> Optional[FeedbackImage]: ...


@runtime_checkable
class LedSink(Protocol):
    def set_led(self, desire: LedDesire, *, send: bool = False) -> None: ...


@runtime_checkable
class ServoSink(Protocol):
    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = False) -> None: ...
