"""Offline HostProxy / Profile tests (no COM)."""
from __future__ import annotations

import os
import sys
from types import SimpleNamespace
from typing import Dict, Optional

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.host_proxy import (  # noqa: E402
    LEFT_ARM_SLOTS,
    ComponentView,
    HostProxy,
    Profile,
    yam_product_profile,
)
from deft_controls_sdk.link import ActuatorDesire  # noqa: E402
from deft_controls_sdk.vbeta.session import PcbRobotSession  # noqa: E402


class _FakeConn:
    def __init__(self) -> None:
        self.actuators: Dict[int, ActuatorDesire] = {}
        self._latest_fb_raw = None

    def set_actuators(self, desires, *, send: bool = False) -> None:
        self.actuators.update(desires)

    def poll_feedback(self):
        return None


class _FakeHub:
    def __init__(self) -> None:
        self._connection = _FakeConn()
        self.is_streaming = True
        self.port = "FAKE"
        self.debug = SimpleNamespace(cfg_get_table=lambda: [None] * 26)

    def set_actuator(self, slot, desire, *, send: bool = False) -> None:
        self._connection.actuators[slot] = desire

    def set_servo(self, *a, **k) -> None:
        pass

    def set_led(self, *a, **k) -> None:
        pass

    def send_once(self) -> None:
        pass

    def soft_kill_park_if_requested(self, *, send: bool = False) -> bool:
        return False

    def close(self) -> None:
        pass

    def start_streaming(self, hz: float = 40.0, **k) -> None:
        self.is_streaming = True

    def stop_streaming(self) -> None:
        self.is_streaming = False

    def set_mcu_state(self, *a, **k) -> None:
        pass


def test_yam_product_profile_components():
    p = yam_product_profile()
    assert p.name == "yam_product"
    assert p.slots("left_arm") == LEFT_ARM_SLOTS
    assert len(p.slots("base")) == 6
    with pytest.raises(KeyError):
        p.slots("nope")


def test_component_hold_writes_slots():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    view = proxy.component("left_arm")
    view.hold([0.1 * i for i in range(7)], kp=9.0, kd=0.4, send=False)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d = hub._connection.actuators[slot]
        assert d.position == pytest.approx(0.1 * i)
        assert d.kp == pytest.approx(9.0)
        assert d.kd == pytest.approx(0.4)


def test_session_wraps_host_proxy():
    hub = _FakeHub()
    session = PcbRobotSession.wrap(hub)
    assert isinstance(session.proxy, HostProxy)
    session.set_actuator(0, ActuatorDesire(position=1.0, kp=1.0), send=False)
    assert hub._connection.actuators[0].position == pytest.approx(1.0)


def test_doctor_offline():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub)
    report = proxy.doctor()
    assert report["profile"] == "yam_product"
    assert report["cfg_ok"] is True
    assert report["streaming"] is True
