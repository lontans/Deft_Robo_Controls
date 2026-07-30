"""actions — plant-mode behaviour (CMDH desires): components, LED, servos.

Debug board RPC lives in ``debug``; when a debug tool needs normal behaviour
it should call into this package.
"""
from __future__ import annotations

from .component import ComponentAction
from .led import (
    LedAction,
    led_follow,
    led_idle,
    led_off,
    led_pdu,
    set_led,
)
from .servo import ServoAction
from .sink import LedSink, PlantSink, ServoSink

# Back-compat alias used by HostProxy / package exports
ComponentView = ComponentAction

__all__ = [
    "ComponentAction",
    "ComponentView",
    "LedAction",
    "LedSink",
    "PlantSink",
    "ServoAction",
    "ServoSink",
    "led_follow",
    "led_idle",
    "led_off",
    "led_pdu",
    "set_led",
]
