"""LED / PDU-listen panel — thin wrappers over LedAction / LED presets.

No new business logic: adapts ``LedAction`` / ``apply_led_preset`` / the
``HostProxy.listen_pdu`` property to the shared panel calling convention
(``fn(proxy, **params) -> dict``).
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from deft_controls_sdk.config.led import LED_PRESETS
from deft_controls_sdk.debug.suite.presets import apply_led_preset
from deft_controls_sdk.link import LedDesire

if TYPE_CHECKING:
    from deft_controls_sdk.host_proxy import HostProxy as HostProxyType

__all__ = [
    "set_led_panel",
    "apply_led_preset_panel",
    "set_listen_pdu_panel",
]


def set_led_panel(
    proxy: "HostProxyType",
    *,
    mode: str,
    brightness: int = 8,
    pattern: int = 0,
    count: int = 0,
) -> dict:
    """Thin wrap over ``proxy.set_led`` — direct ``LedDesire`` policy set."""
    desire = LedDesire(
        mode=mode,
        pattern=int(pattern),
        master_brightness=int(brightness),
        led_count=int(count),
    )
    proxy.set_led(desire, send=True)
    return {
        "applied": True,
        "mode": mode,
        "brightness": int(brightness),
        "pattern": int(pattern),
        "count": int(count),
    }


def apply_led_preset_panel(proxy: "HostProxyType", *, preset_name: str) -> dict:
    """Look up ``preset_name`` in ``LED_PRESETS`` and apply it via ``apply_led_preset``.

    Raises ``ValueError`` (naming the known presets) if ``preset_name`` is
    unknown. Requires ``mode="debug"`` (reads NVM periph before applying).
    """
    key = str(preset_name).strip().lower()
    if key not in LED_PRESETS:
        known = ", ".join(sorted(LED_PRESETS))
        raise ValueError(f"unknown LED preset {preset_name!r}; known: {known}")
    preset = LED_PRESETS[key]
    periph = dict(proxy.hub.debug.cfg_get_periph())
    apply_led_preset(proxy, preset, periph)
    return {
        "applied": True,
        "preset": preset.name,
        "policy": preset.policy,
        "brightness": int(preset.brightness),
        "pattern": int(preset.pattern),
    }


def set_listen_pdu_panel(proxy: "HostProxyType", *, enabled: bool) -> dict:
    """Thin wrap over the ``proxy.listen_pdu`` property setter."""
    proxy.listen_pdu = bool(enabled)
    return {"listen_pdu": bool(proxy.listen_pdu)}
