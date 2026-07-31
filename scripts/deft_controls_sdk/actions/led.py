"""LedAction — plant LedDesire helpers (normal behaviour, not debug RPC)."""
from __future__ import annotations

from typing import Union

from deft_controls_sdk.config.led import LED_PRESETS, LedPreset
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

from .mounted import MountedAction
from .plant import PlantAction
from .sink import LedSink


class LedAction(PlantAction):
    """Board LED strip commands via plant ``LedDesire``.

    Prefer::

        a.mount(a.led().solid_green())
        a.apply()
    """

    def __init__(self, sink: LedSink) -> None:
        super().__init__(sink)

    def _emit(self, mounted: MountedAction, *, send: bool) -> MountedAction:
        if self._actions is not None:
            if send:
                self._actions.mount(mounted)
                self._actions.apply()
            return mounted
        if mounted.led is not None:
            self._sink.set_led(mounted.led, send=bool(send))
        return mounted

    def set(
        self,
        mode: Union[str, int],
        brightness: int = 8,
        count: int = 0,
        *,
        pattern: int = 0,
        send: bool = False,
    ) -> MountedAction:
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
        return self._emit(
            MountedAction.from_led(
                f"led.set[{mode}]", desire, meta={"op": "set", "mode": mode}
            ),
            send=send,
        )

    def apply_preset(
        self, preset: Union[str, LedPreset], *, send: bool = True
    ) -> MountedAction:
        if isinstance(preset, str):
            key = preset.strip().lower()
            if key not in LED_PRESETS:
                known = ", ".join(sorted(LED_PRESETS))
                raise ValueError(f"unknown LED preset {preset!r}; known: {known}")
            preset = LED_PRESETS[key]
        return self.set(
            preset.policy,
            brightness=preset.brightness,
            pattern=int(preset.pattern),
            send=send,
        )

    def follow(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set("follow", brightness=brightness, send=send)

    def pdu(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set("pdu", brightness=brightness, send=send)

    def idle(self, brightness: int = 12, *, send: bool = False) -> MountedAction:
        return self.set(
            "debug",
            pattern=LED_MODE_IDLE_CORNFLOWER,
            brightness=brightness,
            send=send,
        )

    def off(self, *, send: bool = False) -> MountedAction:
        return self.set("follow", brightness=0, send=send)

    def flash(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set("debug", pattern=LED_MODE_FLASH, brightness=brightness, send=send)

    def test_chase(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set("debug", pattern=LED_MODE_TEST, brightness=brightness, send=send)

    def solid_green(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set(
            "debug", pattern=LED_MODE_SOLID_GREEN, brightness=brightness, send=send
        )

    def solid_yellow(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set(
            "debug", pattern=LED_MODE_SOLID_YELLOW, brightness=brightness, send=send
        )

    def solid_red(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set(
            "debug", pattern=LED_MODE_SOLID_RED, brightness=brightness, send=send
        )

    def caution(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set(
            "debug",
            pattern=LED_MODE_BLINK_YELLOW_SLOW,
            brightness=brightness,
            send=send,
        )

    def fault(self, brightness: int = 8, *, send: bool = False) -> MountedAction:
        return self.set(
            "debug",
            pattern=LED_MODE_BLINK_RED_FAST,
            brightness=brightness,
            send=send,
        )


# Module-level helpers matching former vbeta.leds API
def set_led(
    sink: LedSink,
    mode: Union[str, int],
    brightness: int = 8,
    count: int = 0,
    *,
    pattern: int = 0,
    send: bool = False,
) -> LedDesire:
    mounted = LedAction(sink).set(
        mode, brightness=brightness, count=count, pattern=pattern, send=send
    )
    assert mounted.led is not None
    return mounted.led


def led_follow(sink: LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    mounted = LedAction(sink).follow(brightness=brightness, send=send)
    assert mounted.led is not None
    return mounted.led


def led_pdu(sink: LedSink, brightness: int = 8, *, send: bool = False) -> LedDesire:
    mounted = LedAction(sink).pdu(brightness=brightness, send=send)
    assert mounted.led is not None
    return mounted.led


def led_idle(sink: LedSink, brightness: int = 12, *, send: bool = False) -> LedDesire:
    mounted = LedAction(sink).idle(brightness=brightness, send=send)
    assert mounted.led is not None
    return mounted.led


def led_off(sink: LedSink, *, send: bool = False) -> LedDesire:
    mounted = LedAction(sink).off(send=send)
    assert mounted.led is not None
    return mounted.led


__all__ = [
    "LedAction",
    "set_led",
    "led_follow",
    "led_pdu",
    "led_idle",
    "led_off",
    "LED_MODE_OFF",
    "LED_MODE_TEST",
    "LED_MODE_FLASH",
    "LED_MODE_SOLID_GREEN",
    "LED_MODE_SOLID_YELLOW",
    "LED_MODE_SOLID_RED",
    "LED_MODE_BLINK_YELLOW_SLOW",
    "LED_MODE_BLINK_RED_FAST",
    "LED_MODE_IDLE_CORNFLOWER",
]
