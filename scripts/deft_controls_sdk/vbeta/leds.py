"""SK9822 LED helpers on PcbRobotSession / ControlsPcbHub."""
from __future__ import annotations

from typing import Protocol, Union

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

# Re-export named patterns next to the helpers (canonical defs in api_types).
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
    "led_pdu",
    "led_follow",
]


class _LedSink(Protocol):
    def set_led(self, desire: LedDesire, *, send: bool = False) -> None: ...


def set_led(
    sink: _LedSink,
    mode: Union[str, int],
    brightness: int = 8,
    count: int = 0,
    *,
    pattern: int = 0,
    send: bool = False,
) -> LedDesire:
    """``mode``: policy ``debug|pdu|follow``, or legacy int pattern (→ debug)."""
    if isinstance(mode, int):
        desire = LedDesire(
            mode=int(mode),
            master_brightness=brightness,
            led_count=count,
        )
    else:
        desire = LedDesire(
            mode=str(mode),
            pattern=int(pattern),
            master_brightness=brightness,
            led_count=count,
        )
    sink.set_led(desire, send=send)
    return desire


def led_off(sink: _LedSink, *, send: bool = False) -> LedDesire:
    """Alias of follow (no host override) — not strip-black."""
    return set_led(sink, "follow", brightness=0, count=0, send=send)


def led_follow(sink: _LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    return set_led(sink, "follow", brightness=brightness, send=send)


def led_pdu(sink: _LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    return set_led(sink, "pdu", brightness=brightness, send=send)


def led_flash(sink: _LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    return set_led(
        sink, "debug", pattern=LED_MODE_FLASH, brightness=brightness, send=send
    )


def led_test(sink: _LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    return set_led(
        sink, "debug", pattern=LED_MODE_TEST, brightness=brightness, send=send
    )


def led_solid_green(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    return set_led(
        sink, "debug", pattern=LED_MODE_SOLID_GREEN, brightness=brightness, send=send
    )


def led_solid_yellow(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    return set_led(
        sink, "debug", pattern=LED_MODE_SOLID_YELLOW, brightness=brightness, send=send
    )


def led_solid_red(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    return set_led(
        sink, "debug", pattern=LED_MODE_SOLID_RED, brightness=brightness, send=send
    )


def led_caution(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    """Slow yellow blink (factory caution)."""
    return set_led(
        sink,
        "debug",
        pattern=LED_MODE_BLINK_YELLOW_SLOW,
        brightness=brightness,
        send=send,
    )


def led_fault(
    sink: _LedSink, brightness: int = 8, *, send: bool = False
) -> LedDesire:
    """Fast red blink (estop / fault attention)."""
    return set_led(
        sink,
        "debug",
        pattern=LED_MODE_BLINK_RED_FAST,
        brightness=brightness,
        send=send,
    )


def led_idle(
    sink: _LedSink, brightness: int = 12, *, send: bool = False
) -> LedDesire:
    """Cornflower idle blink (#6495ED, 500 on / 500 off) via debug pattern."""
    return set_led(
        sink,
        "debug",
        pattern=LED_MODE_IDLE_CORNFLOWER,
        brightness=brightness,
        send=send,
    )
