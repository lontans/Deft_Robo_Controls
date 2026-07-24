"""SK9822 LED helpers on PcbRobotSession / ControlsPcbHub."""
from __future__ import annotations

from typing import Protocol

from deft_controls_sdk.link import (
    LED_MODE_BLINK_RED_FAST,
    LED_MODE_BLINK_YELLOW_SLOW,
    LED_MODE_FLASH,
    LED_MODE_IDLE_CORNFLOWER,
    LED_MODE_OFF,
    LED_MODE_SOLID_GREEN,
    LED_MODE_SOLID_RED,
    LED_MODE_SOLID_YELLOW,
    LED_MODE_TEST,
    LedDesire,
)

# Re-export named modes next to the helpers (canonical defs in api_types).
__all__ = [
    "LED_MODE_OFF",
    "LED_MODE_TEST",
    "LED_MODE_FLASH",
    "LED_MODE_SOLID_GREEN",
    "LED_MODE_SOLID_YELLOW",
    "LED_MODE_SOLID_RED",
    "LED_MODE_BLINK_YELLOW_SLOW",
    "LED_MODE_BLINK_RED_FAST",
    "LED_MODE_IDLE_CORNFLOWER",
    "set_led",
    "led_off",
    "led_flash",
    "led_test",
    "led_solid_green",
    "led_solid_yellow",
    "led_solid_red",
    "led_caution",
    "led_fault",
    "led_idle",
]


class _LedSink(Protocol):
    def set_led(self, desire: LedDesire, *, send: bool = False) -> None: ...


def set_led(
    sink: _LedSink,
    mode: int,
    brightness: int = 8,
    count: int = 0,
    *,
    send: bool = False,
) -> LedDesire:
    """mode: LED_MODE_* (0=OFF … 8=IDLE_CORNFLOWER). count 0 ⇒ firmware max (300)."""
    desire = LedDesire(
        mode=int(mode) & 0x1F,
        master_brightness=max(0, min(31, int(brightness))),
        led_count=max(0, int(count)),
    )
    sink.set_led(desire, send=send)
    return desire


def led_off(sink: _LedSink, *, send: bool = False) -> LedDesire:
    return set_led(sink, LED_MODE_OFF, brightness=0, count=0, send=send)


def led_flash(sink: _LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    return set_led(sink, LED_MODE_FLASH, brightness=brightness, send=send)


def led_test(sink: _LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    return set_led(sink, LED_MODE_TEST, brightness=brightness, send=send)


def led_solid_green(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    return set_led(sink, LED_MODE_SOLID_GREEN, brightness=brightness, send=send)


def led_solid_yellow(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    return set_led(sink, LED_MODE_SOLID_YELLOW, brightness=brightness, send=send)


def led_solid_red(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    return set_led(sink, LED_MODE_SOLID_RED, brightness=brightness, send=send)


def led_caution(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    """Slow yellow blink (factory caution)."""
    return set_led(sink, LED_MODE_BLINK_YELLOW_SLOW, brightness=brightness, send=send)


def led_fault(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    """Fast red blink (estop / fault attention)."""
    return set_led(sink, LED_MODE_BLINK_RED_FAST, brightness=brightness, send=send)


def led_idle(
    sink: _LedSink, brightness: int = 12, *, send: bool = False
) -> LedDesire:
    """Cornflower idle blink (#6495ED, 500 on / 500 off)."""
    return set_led(sink, LED_MODE_IDLE_CORNFLOWER, brightness=brightness, send=send)
