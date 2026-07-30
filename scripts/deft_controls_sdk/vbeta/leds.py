"""SK9822 LED helpers — thin re-export of ``actions.led`` (+ pattern constants)."""
from __future__ import annotations

from deft_controls_sdk.actions.led import (
    LedAction,
    led_follow,
    led_idle,
    led_off,
    led_pdu,
    set_led,
)
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
)


def led_flash(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).flash(brightness=brightness, send=send)


def led_test(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).test_chase(brightness=brightness, send=send)


def led_solid_green(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).solid_green(brightness=brightness, send=send)


def led_solid_yellow(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).solid_yellow(brightness=brightness, send=send)


def led_solid_red(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).solid_red(brightness=brightness, send=send)


def led_caution(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).caution(brightness=brightness, send=send)


def led_fault(sink, brightness: int = 8, *, send: bool = False):
    return LedAction(sink).fault(brightness=brightness, send=send)


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
    "LedAction",
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
    "led_pdu",
    "led_follow",
]
