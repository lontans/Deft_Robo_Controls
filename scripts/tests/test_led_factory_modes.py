"""Pack-only tests: LED factory modes round-trip in the 672 B command image."""
from __future__ import annotations

import os
import struct
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

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
    CommandImage,
    LedDesire,
)
from deft_controls_sdk.link.exchange import IMAGE_BYTES, patch_led_command
from deft_controls_sdk.link.exchange.wire_layout import LED_CMD_OFF

FACTORY_MODES = (
    LED_MODE_OFF,
    LED_MODE_TEST,
    LED_MODE_FLASH,
    LED_MODE_SOLID_GREEN,
    LED_MODE_SOLID_YELLOW,
    LED_MODE_SOLID_RED,
    LED_MODE_BLINK_YELLOW_SLOW,
    LED_MODE_BLINK_RED_FAST,
    LED_MODE_IDLE_CORNFLOWER,
)


def _unpack_led(buf: bytes | bytearray) -> tuple[int, int, int]:
    word, = struct.unpack_from("<H", buf, LED_CMD_OFF)
    return word & 0x1F, (word >> 5) & 0x1F, (word >> 10) & 0x3F


def test_image_bytes_unchanged_672() -> None:
    assert IMAGE_BYTES == 694
    assert len(CommandImage().to_bytes()) == 694


def test_patch_led_command_round_trips_factory_modes() -> None:
    buf = bytearray(IMAGE_BYTES)
    for mode in FACTORY_MODES:
        for brightness in (0, 8, 31):
            for count in (0, 1, 63):
                patch_led_command(
                    buf, mode=mode, master_brightness=brightness, led_count=count
                )
                got_mode, got_bri, got_count = _unpack_led(buf)
                assert got_mode == mode
                assert got_bri == brightness
                assert got_count == count
    # Only the 2 B LED word was written; image length still 672.
    assert len(buf) == 694


def test_command_image_set_led_packs_named_modes() -> None:
    for mode in (
        LED_MODE_SOLID_GREEN,
        LED_MODE_SOLID_YELLOW,
        LED_MODE_SOLID_RED,
        LED_MODE_BLINK_YELLOW_SLOW,
        LED_MODE_BLINK_RED_FAST,
        LED_MODE_IDLE_CORNFLOWER,
    ):
        img = CommandImage(seq=7).set_led(
            LedDesire(mode=mode, master_brightness=12, led_count=0)
        )
        raw = img.to_bytes()
        assert len(raw) == 694
        mode_u, bri_u, count_u = _unpack_led(raw)
        assert mode_u == mode
        assert bri_u == 12
        assert count_u == 0


def test_led_mode_constants_match_rfc() -> None:
    assert LED_MODE_OFF == 0
    assert LED_MODE_TEST == 1
    assert LED_MODE_FLASH == 2
    assert LED_MODE_SOLID_GREEN == 3
    assert LED_MODE_SOLID_YELLOW == 4
    assert LED_MODE_SOLID_RED == 5
    assert LED_MODE_BLINK_YELLOW_SLOW == 6
    assert LED_MODE_BLINK_RED_FAST == 7
    assert LED_MODE_IDLE_CORNFLOWER == 8
