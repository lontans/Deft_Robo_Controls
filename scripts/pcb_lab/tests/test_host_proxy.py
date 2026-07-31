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
    Actions,
    ActuatorAction,
    LedAction,
    MountedAction,
    PlantAction,
    PduLinkAction,
    ServoAction,
)
from deft_controls_sdk.config import (  # noqa: E402
    BENCH_BASE_SLOTS,
    LEFT_ARM_SLOTS,
    Profile,
    bench_continuous_profile,
    yam_product_assembly,
    yam_product_profile,
)
from deft_controls_sdk.host_proxy import HostProxy  # noqa: E402
from deft_controls_sdk.link import ActuatorDesire  # noqa: E402


class _FakeFb:
    """Minimal FeedbackImage stand-in for hold()/nudge() sampling."""

    def __init__(self) -> None:
        self.positions: Dict[int, float] = {}

    def actuator(self, slot: int):
        if int(slot) not in self.positions:
            return None
        return SimpleNamespace(position=float(self.positions[int(slot)]))


class _FakeConn:
    def __init__(self) -> None:
        self.actuators: Dict[int, ActuatorDesire] = {}
        self.servos: Dict[int, object] = {}
        self._latest_fb_raw = None
        self.send_count = 0
        self.fb = _FakeFb()

    def set_actuators(self, desires, *, send: bool = False) -> None:
        self.actuators.update(desires)
        if send:
            self.send_count += 1

    def poll_feedback(self):
        return self.fb


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
        self.link_mode = "debug"
        self.send_once_count = 0

    def set_actuator(self, slot, desire, *, send: bool = False) -> None:
        self._connection.actuators[slot] = desire

    def set_servo(self, slot, desire, *, send: bool = False) -> None:
        self._connection.servos[int(slot)] = desire

    def set_led(self, desire, *, send: bool = False) -> None:
        self._led_desire = desire

    def send_once(self) -> None:
        self.send_once_count += 1

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
    from deft_controls_sdk.config import (
        BASE_WHEEL_1_SLOTS,
        PRODUCT_ACTUATOR_SECTIONS,
        TORSO_SLOT,
    )

    p = yam_product_profile()
    assert p.name == "yam_product"
    assert p.slots("left_arm") == LEFT_ARM_SLOTS
    assert p.slots("base_wheel_1") == BASE_WHEEL_1_SLOTS
    assert p.slots("torso") == (TORSO_SLOT,)
    assert set(p.components) == set(PRODUCT_ACTUATOR_SECTIONS)
    with pytest.raises(KeyError):
        p.slots("base")
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
    assert proxy.component("left_arm").slots == view.slots
    view.hold([0.1 * i for i in range(7)], kp=9.0, kd=0.4, send=False)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d = hub._connection.actuators[slot]
        assert d.position == pytest.approx(0.1 * i)
        assert d.kp == pytest.approx(9.0)
        assert d.kd == pytest.approx(0.4)


def test_actuator_hold_uses_neutral_defaults_or_explicit_gains():
    from deft_controls_sdk.config import DEFAULT_HOLD_KP, DEFAULT_HOLD_KD

    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    arm = proxy.actuators("left_arm")
    arm.hold([0.0] * 7, send=False)
    for slot in LEFT_ARM_SLOTS:
        assert hub._connection.actuators[slot].kp == pytest.approx(DEFAULT_HOLD_KP)
        assert hub._connection.actuators[slot].kd == pytest.approx(DEFAULT_HOLD_KD)

    wheel1 = proxy.actuators("base_wheel_1")
    wheel1.hold([0.1, 0.2], kp=40.0, kd=2.5, send=False)
    for slot in yam_product_profile().slots("base_wheel_1"):
        d = hub._connection.actuators[slot]
        assert d.kp == pytest.approx(40.0)
        assert d.kd == pytest.approx(2.5)

    wheel = ActuatorAction.from_slots(proxy, (22,), name="ch5")
    wheel.hold([0.2], send=False)
    assert hub._connection.actuators[22].kp == pytest.approx(DEFAULT_HOLD_KP)


def test_set_section_demuxes_ordered_desires():
    from deft_controls_sdk.config import BASE_WHEEL_1_SLOTS
    from deft_controls_sdk.link import ActuatorDesire

    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, assembly=yam_product_assembly())
    desires = [
        ActuatorDesire(position=0.3, velocity=0.0, kp=10.0, kd=1.0, torque=0.1),
        ActuatorDesire(position=0.0, velocity=1.5, kp=0.0, kd=2.0, torque=0.0),
    ]
    proxy.set_section("base_wheel_1", desires, send=False)
    assert hub._connection.actuators[BASE_WHEEL_1_SLOTS[0]].position == pytest.approx(0.3)
    assert hub._connection.actuators[BASE_WHEEL_1_SLOTS[1]].velocity == pytest.approx(1.5)
    with pytest.raises(ValueError, match="expects 2"):
        proxy.set_section("base_wheel_1", desires[:1], send=False)


def test_actuator_action_hub_sink():
    """Same ActuatorAction type works with ControlsPcbHub-shaped sink."""
    hub = _FakeHub()

    def set_actuators(desires, *, send: bool = False) -> None:
        hub._connection.set_actuators(desires, send=send)

    def latest_feedback():
        return None

    hub.set_actuators = set_actuators  # type: ignore[method-assign]
    hub.latest_feedback = latest_feedback  # type: ignore[method-assign]
    action = ActuatorAction(hub, yam_product_profile(), "base_wheel_1")
    action.blank(send=False)
    assert set(hub._connection.actuators) == set(
        yam_product_profile().slots("base_wheel_1")
    )


def test_actuator_from_slots_single():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    one = ActuatorAction.from_slots(proxy, (22,), name="slot_22")
    one.hold([0.5], kp=5.0, kd=0.2, send=False)
    assert hub._connection.actuators[22].position == pytest.approx(0.5)


def test_action_hierarchy_siblings():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    assert isinstance(proxy.actuators("base_wheel_1"), ActuatorAction)
    assert isinstance(proxy.led(), LedAction)
    assert isinstance(proxy.servo(), ServoAction)
    assert isinstance(proxy.pdu_link(), PduLinkAction)
    assert all(
        isinstance(x, PlantAction)
        for x in (
            proxy.actuators("base_wheel_1"),
            proxy.led(),
            proxy.servo(),
            proxy.pdu_link(),
        )
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


def test_arm_disarm_commit_and_assembly_bind():
    from deft_controls_sdk.config import assembly_from_name

    hub = _FakeHub()
    asm = assembly_from_name("bench")
    proxy = HostProxy.wrap(hub, assembly=asm)
    assert proxy.mode == "debug"
    assert proxy.assembly is asm
    assert proxy.armed is False

    proxy.arm_plant()
    assert proxy.armed is True
    assert hub.plant_apply is True

    proxy.disarm_plant()
    assert proxy.armed is False

    # Named group from assembly (spare-slot base on bench)
    base = proxy.actuators("base")
    assert base.slots == BENCH_BASE_SLOTS

    hub.link_mode = "bandwidth"
    with pytest.raises(RuntimeError, match="mode='debug'"):
        proxy.require_debug("inventory")


def test_actions_mount_apply_clear_pending():
    """Preferred notebook path: mount → inspect → apply → clear."""
    hub = _FakeHub()
    hub._connection.fb.positions[22] = 0.1
    hub._connection.fb.positions[23] = 0.2
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    a = proxy.actions
    assert isinstance(a, Actions)

    wheels = a.actuator(slots=(22, 23))
    neck = a.servo()
    assert wheels.actions is a
    assert neck.actions is a

    # Default hold() samples FB — stay put, no move
    hold = wheels.hold()
    assert isinstance(hold, MountedAction)
    assert hold.meta["sampled"] is True
    assert hold.meta["positions"] == [0.1, 0.2]
    # Bound helpers do not TX until apply
    assert 22 not in hub._connection.actuators

    a.mount(hold)
    a.mount(neck.neck_center())
    assert a.pending_count == 2
    assert a.pending[0]["label"].endswith(".hold")
    assert a.pending[1]["kind"] == "servo"
    assert hub.send_once_count == 0

    a.apply()
    assert a.pending_count == 0
    assert hub.send_once_count == 1
    assert hub._connection.actuators[22].position == pytest.approx(0.1)
    assert hub._connection.actuators[23].position == pytest.approx(0.2)
    assert hub._connection.servos  # neck slots written

    a.mount(wheels.nudge(index=0, delta=0.05))
    a.apply()
    assert hub._connection.actuators[22].position == pytest.approx(0.15)
    assert hub.send_once_count == 2

    a.clear()
    assert a.pending_count == 0
    assert hub._connection.actuators[22].position == pytest.approx(0.0)
    assert hub._connection.actuators[23].position == pytest.approx(0.0)
    for desire in hub._connection.servos.values():
        assert desire.servo_id == 0
    assert hub.send_once_count == 3


def test_hold_without_fb_raises():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    wheels = proxy.actions.actuator(slots=(22,))
    with pytest.raises(RuntimeError, match="no feedback"):
        wheels.hold()


def test_actions_bound_send_true_is_mount_apply():
    hub = _FakeHub()
    proxy = HostProxy.wrap(hub, profile=yam_product_profile())
    a = proxy.actions
    wheels = a.actuator(slots=(22,))
    wheels.hold([0.3], send=True)  # legacy shortcut
    assert hub._connection.actuators[22].position == pytest.approx(0.3)
    assert a.pending_count == 0
    assert hub.send_once_count >= 1


def test_telemetry_package_exports_cache():
    from deft_controls_sdk import TelemetryCache as TopCache
    from deft_controls_sdk.telemetry import TelemetryCache

    assert TopCache is TelemetryCache


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
