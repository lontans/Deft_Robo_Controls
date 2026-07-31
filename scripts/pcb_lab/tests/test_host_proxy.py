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

from deft_controls_sdk.actions import (  # noqa: E402
    ActuatorAction,
    LedAction,
    PlantAction,
    PduLinkAction,
    ServoAction,
)
from deft_controls_sdk.config import (  # noqa: E402
    BENCH_BASE_SLOTS,
    LEFT_ARM_SLOTS,
    Profile,
    bench_continuous_profile,
    yam_product_profile,
)
from deft_controls_sdk.host_proxy import HostProxy  # noqa: E402
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
        from deft_controls_sdk.link import McuState

        self._connection = _FakeConn()
        self.is_streaming = True
        self.port = "FAKE"
        self.debug = SimpleNamespace(cfg_get_table=lambda: [None] * 26)
        self._mcu_state = McuState.NORMAL
        self._plant_apply = False
        self._led_desire = None
        self._listen_pdu = False
        self._stream_hz = 200.0
        self._telemetry_hz = 200.0

    def set_actuator(self, slot, desire, *, send: bool = False) -> None:
        self._connection.actuators[slot] = desire

    def set_servo(self, *a, **k) -> None:
        pass

    def set_led(self, desire, *, send: bool = False) -> None:
        self._led_desire = desire

    def send_once(self) -> None:
        pass

    def soft_kill_park_if_requested(self, *, send: bool = False) -> bool:
        return False

    def close(self) -> None:
        pass

    def start_streaming(self, hz: float = 200.0, **k) -> None:
        self.is_streaming = True
        self._stream_hz = float(hz)
        if "telemetry_hz" in k:
            self._telemetry_hz = float(k["telemetry_hz"])

    def stop_streaming(self) -> None:
        self.is_streaming = False

    def set_mcu_state(self, state, *, send: bool = False) -> None:
        self._mcu_state = state

    def set_plant_apply(self, enable: bool, *, send: bool = False) -> None:
        self._plant_apply = bool(enable)

    @property
    def mcu_state(self):
        return self._mcu_state

    @property
    def plant_apply(self):
        return self._plant_apply

    @property
    def led_desire(self):
        return self._led_desire

    @property
    def listen_pdu(self):
        return self._listen_pdu

    @listen_pdu.setter
    def listen_pdu(self, enabled: bool) -> None:
        self._listen_pdu = bool(enabled)

    @property
    def stream_hz(self):
        return self._stream_hz

    @property
    def telemetry_hz(self):
        return self._telemetry_hz

    def pdb_status(self, raw=None):
        return None


def test_yam_product_profile_components():
    p = yam_product_profile()
    assert p.name == "yam_product"
    assert p.slots("left_arm") == LEFT_ARM_SLOTS
    assert len(p.slots("base")) == 6
    with pytest.raises(KeyError):
        p.slots("nope")


def test_bench_continuous_profile_base_on_spare_slots():
    p = bench_continuous_profile()
    assert p.name == "yam_bench_continuous"
    assert p.slots("base") == BENCH_BASE_SLOTS
    assert len(p.slots("base_product")) == 6


def test_demux_report_offline():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=bench_continuous_profile())
    report = proxy.demux_report()
    assert report["profile"] == "yam_bench_continuous"
    assert report["by_component"]["base"][0]["slot"] == 22
    assert report["cfg_ok"] is True


def test_actuator_hold_writes_slots():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    view = proxy.actuators("left_arm")
    assert isinstance(view, ActuatorAction)
    assert isinstance(view, PlantAction)
    assert view.kind == "joint"
    assert proxy.component("left_arm").slots == view.slots
    view.hold([0.1 * i for i in range(7)], kp=9.0, kd=0.4, send=False)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d = hub._connection.actuators[slot]
        assert d.position == pytest.approx(0.1 * i)
        assert d.kp == pytest.approx(9.0)
        assert d.kd == pytest.approx(0.4)


def test_actuator_kind_defaults_joint_vs_wheel():
    from deft_controls_sdk.config import DEFAULT_ARM_KP, DEFAULT_WHEEL_KP, DEFAULT_WHEEL_KD

    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    arm = proxy.actuators("left_arm")
    assert arm.kind == "joint"
    arm.hold([0.0] * 7, send=False)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        assert hub._connection.actuators[slot].kp == pytest.approx(DEFAULT_ARM_KP[i])

    base = proxy.actuators("base")
    assert base.kind == "wheel"
    base.hold([0.1] * 6, send=False)
    for slot in yam_product_profile().slots("base"):
        d = hub._connection.actuators[slot]
        assert d.kp == pytest.approx(DEFAULT_WHEEL_KP)
        assert d.kd == pytest.approx(DEFAULT_WHEEL_KD)

    wheel = ActuatorAction.from_slots(proxy, (22,), name="ch5", kind="wheel")
    assert wheel.kind == "wheel"
    wheel.hold([0.2], send=False)
    assert hub._connection.actuators[22].kp == pytest.approx(DEFAULT_WHEEL_KP)


def test_actuator_action_hub_sink():
    """Same ActuatorAction type works with ControlsPcbHub-shaped sink."""
    hub = _FakeHub()

    def set_actuators(desires, *, send: bool = False) -> None:
        hub._connection.set_actuators(desires, send=send)

    def latest_feedback():
        return None

    hub.set_actuators = set_actuators  # type: ignore[method-assign]
    hub.latest_feedback = latest_feedback  # type: ignore[method-assign]
    action = ActuatorAction(hub, yam_product_profile(), "base")
    action.blank(send=False)
    assert set(hub._connection.actuators) == set(yam_product_profile().slots("base"))


def test_actuator_from_slots_single():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    one = ActuatorAction.from_slots(proxy, (22,), name="slot_22", kind="wheel")
    one.hold([0.5], kp=5.0, kd=0.2, send=False)
    assert hub._connection.actuators[22].position == pytest.approx(0.5)


def test_action_hierarchy_siblings():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    assert isinstance(proxy.actuators("base"), ActuatorAction)
    assert isinstance(proxy.led(), LedAction)
    assert isinstance(proxy.servo(), ServoAction)
    assert isinstance(proxy.pdu_link(), PduLinkAction)
    assert all(
        isinstance(x, PlantAction)
        for x in (proxy.actuators("base"), proxy.led(), proxy.servo(), proxy.pdu_link())
    )


def test_lab_robot_actuators_matches_proxy():
    from pcb_lab.lab import LabRobot

    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    lab = LabRobot(proxy)
    a = lab.actuators("left_arm")
    b = lab.proxy.actuators("left_arm")
    c = lab.component("left_arm")
    assert type(a) is type(b) is type(c) is ActuatorAction
    assert a.slots == b.slots == c.slots == LEFT_ARM_SLOTS


def test_telemetry_package_exports_cache():
    from deft_controls_sdk import TelemetryCache as TopCache
    from deft_controls_sdk.telemetry import TelemetryCache

    assert TopCache is TelemetryCache


def test_session_wraps_host_proxy():
    hub = _FakeHub()
    session = PcbRobotSession.wrap(hub)
    assert isinstance(session.proxy, HostProxy)
    session.set_actuator(0, ActuatorDesire(position=1.0, kp=1.0), send=False)
    assert hub._connection.actuators[0].position == pytest.approx(1.0)


def test_doctor_offline():
    from deft_controls_sdk.link import LedDesire
    from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER

    hub = _FakeHub()
    hub.set_led(
        LedDesire(
            mode="debug",
            pattern=LED_MODE_IDLE_CORNFLOWER,
            master_brightness=8,
        )
    )
    proxy = HostProxy.wrap(hub)
    report = proxy.doctor()
    assert report["profile"] == "yam_product"
    assert report["cfg_ok"] is True
    assert report["streaming"] is True
    assert report["mcu"]["host_command_name"] == "NORMAL"
    assert report["led"]["effective_mode_name"] == "idle_cornflower"
    assert report["led"]["source"] == "host_debug"
    assert report["led"]["host_policy"] == "debug"
    assert report["listen_pdu"] is False
    assert report["stream"]["hz"] == 200.0
    assert report["pdb"]["listening"] is False


def test_infer_led_from_pdb_kill():
    from deft_controls_sdk.link.api_types import (
        LED_MODE_BLINK_RED_FAST,
        LED_MODE_IDLE_CORNFLOWER,
        infer_effective_led,
        led_mode_from_pdb_kill,
    )

    assert led_mode_from_pdb_kill(kill_state=0, estop_sense=1) == LED_MODE_IDLE_CORNFLOWER
    assert led_mode_from_pdb_kill(kill_state=3, estop_sense=1) == LED_MODE_BLINK_RED_FAST
    led = infer_effective_led(
        host_led_mode="follow",
        listen_pdu=True,
        kill_state=1,
        estop_sense=1,
    )
    assert led["source"] == "pdu_traffic_light"
    assert led["effective_mode_name"] == "blink_yellow_slow"
