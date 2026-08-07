"""Tests for deft_controls_sdk.debug.panels.system.led_panel — no hardware.

Duck-typed fakes only (no real COM port, no real HostProxy/ControlsPcbHub).
"""
from __future__ import annotations

import os
import sys
from typing import Any, Dict, List

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import deft_controls_sdk.debug.panels.system.led_panel as led_panel
from deft_controls_sdk.debug.panels.system.led_panel import (
    apply_led_preset_panel,
    set_led_panel,
    set_listen_pdu_panel,
)


# -- fakes ------------------------------------------------------------------


class _FakeDebugAPI:
    def __init__(self) -> None:
        self.periph: Dict[str, Any] = {
            "listen_pdu": False,
            "flags": 0,
            "servos": [],
            "led": {"default_count": 300, "default_mode": 8, "default_brightness": 8},
        }

    def cfg_get_periph(self, *, timeout_s: float = 1.5) -> dict:
        return dict(self.periph)


class _FakeHub:
    def __init__(self) -> None:
        self.debug = _FakeDebugAPI()


class _FakeProxy:
    """Satisfies both the LedSink protocol (``set_led``) and ``.hub``/``.listen_pdu``."""

    def __init__(self) -> None:
        self.hub = _FakeHub()
        self.led_calls: List[tuple] = []
        self._listen_pdu = False

    def set_led(self, desire: Any, *, send: bool = False) -> None:
        self.led_calls.append((desire, send))

    @property
    def listen_pdu(self) -> bool:
        return self._listen_pdu

    @listen_pdu.setter
    def listen_pdu(self, enabled: bool) -> None:
        self._listen_pdu = bool(enabled)


class _RaisingLedProxy(_FakeProxy):
    def set_led(self, desire: Any, *, send: bool = False) -> None:
        raise ValueError(f"unknown led mode {desire.mode!r}")


class _RaisingPduProxy(_FakeProxy):
    @property
    def listen_pdu(self) -> bool:
        return self._listen_pdu

    @listen_pdu.setter
    def listen_pdu(self, enabled: bool) -> None:
        raise RuntimeError("no hub attached")


# -- set_led_panel ----------------------------------------------------------------


def test_set_led_panel_happy() -> None:
    proxy = _FakeProxy()
    out = set_led_panel(proxy, mode="debug", brightness=10, pattern=3, count=50)
    assert out == {
        "applied": True,
        "mode": "debug",
        "brightness": 10,
        "pattern": 3,
        "count": 50,
    }
    assert len(proxy.led_calls) == 1
    desire, send = proxy.led_calls[0]
    assert send is True
    assert desire.mode == "debug"
    assert desire.pattern == 3
    assert desire.master_brightness == 10
    assert desire.led_count == 50


def test_set_led_panel_error_propagates() -> None:
    """An invalid mode is rejected before it ever reaches the sink — LedDesire
    validates ``mode`` in ``__post_init__`` (link/api_types.py)."""
    proxy = _RaisingLedProxy()
    with pytest.raises(ValueError, match="LedDesire.mode must be one of"):
        set_led_panel(proxy, mode="bogus")
    assert not proxy.led_calls  # never reached the sink


# -- apply_led_preset_panel ---------------------------------------------------------


def test_apply_led_preset_panel_happy(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: Dict[str, Any] = {}

    def _fake_apply_led_preset(proxy, preset, periph):
        calls["proxy"] = proxy
        calls["preset"] = preset
        calls["periph"] = periph

    monkeypatch.setattr(led_panel, "apply_led_preset", _fake_apply_led_preset)

    proxy = _FakeProxy()
    out = apply_led_preset_panel(proxy, preset_name="PDU")  # case-insensitive lookup
    assert out["applied"] is True
    assert out["preset"] == "pdu"
    assert out["policy"] == "pdu"
    assert calls["preset"].name == "pdu"
    assert calls["periph"]["listen_pdu"] is False


def test_apply_led_preset_panel_unknown_preset_raises() -> None:
    proxy = _FakeProxy()
    with pytest.raises(ValueError, match="unknown LED preset"):
        apply_led_preset_panel(proxy, preset_name="does_not_exist")


# -- set_listen_pdu_panel --------------------------------------------------------------


def test_set_listen_pdu_panel_happy() -> None:
    proxy = _FakeProxy()
    out = set_listen_pdu_panel(proxy, enabled=True)
    assert out == {"listen_pdu": True}
    assert proxy.listen_pdu is True


def test_set_listen_pdu_panel_error_propagates() -> None:
    proxy = _RaisingPduProxy()
    with pytest.raises(RuntimeError, match="no hub attached"):
        set_listen_pdu_panel(proxy, enabled=True)
