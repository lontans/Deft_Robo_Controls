"""Off-board contract tests for deft_controls_sdk.vbeta (no hardware)."""
from __future__ import annotations

import os
import struct
import sys
import time
from typing import Dict, Mapping, Optional

import numpy as np
import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.link import ActuatorDesire, LedDesire, ServoDesire
from deft_controls_sdk.link.api_types import FeedbackImage
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, IMAGE_BYTES
from deft_controls_sdk.link.exchange.wire_layout import (
    HOST_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
)
from deft_controls_sdk.vbeta import (
    BASE_DRIVE_SLOTS,
    BASE_STEER_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    PcbArmDriver,
    PcbNeckDriver,
    PcbPlatformClient,
    RIGHT_ARM_SLOTS,
    deg_to_steps,
    led_caution,
    led_fault,
    led_off,
    led_solid_green,
    led_solid_red,
    led_solid_yellow,
    set_led,
    yam_product_rows,
)
from deft_controls_sdk.vbeta.session import PcbRobotSession


class _FakeConn:
    def __init__(self, store: "FakeStore") -> None:
        self._store = store
        self._latest_fb_raw: Optional[bytes] = None

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = True) -> None:
        self._store.actuators.update(dict(desires))

    def poll_feedback(self) -> Optional[FeedbackImage]:
        self._latest_fb_raw = self._store.make_fb()
        return FeedbackImage(self._latest_fb_raw)


class FakeStore:
    def __init__(self) -> None:
        self.actuators: Dict[int, ActuatorDesire] = {}
        self.servos: Dict[int, ServoDesire] = {}
        self.led: Optional[LedDesire] = None
        self.is_streaming = True
        self.fb_pos = {s: 0.0 for s in range(ACTUATOR_COUNT)}
        self.fb_vel = {s: 0.0 for s in range(ACTUATOR_COUNT)}
        self._connection = _FakeConn(self)

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None:
        self.actuators[slot] = desire

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = True) -> None:
        self.servos[slot] = desire

    def set_led(self, desire: LedDesire, *, send: bool = True) -> None:
        self.led = desire

    def set_mcu_state(self, *a, **k) -> None:
        pass

    def send_once(self) -> None:
        pass

    def stop_streaming(self) -> None:
        self.is_streaming = False

    def close(self) -> None:
        pass

    def make_fb(self) -> bytes:
        buf = bytearray(IMAGE_BYTES)
        struct.pack_into(
            "<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 1
        )
        for slot in range(ACTUATOR_COUNT):
            off = 44 + slot * 22
            struct.pack_into(
                "<ffffIH",
                buf,
                off,
                float(self.fb_pos[slot]),
                float(self.fb_vel[slot]),
                0.0,
                25.0,
                0,
                0,
            )
        return bytes(buf)


def _session() -> tuple[PcbRobotSession, FakeStore]:
    store = FakeStore()
    session = PcbRobotSession.__new__(PcbRobotSession)
    session._hub = store  # type: ignore[assignment]
    session._owns_hub = False
    session._stream_hz = 40.0
    session._closed = False
    return session, store


def test_yam_product_rows_shape() -> None:
    rows = yam_product_rows()
    assert len(rows) == ACTUATOR_COUNT
    for slot in LEFT_ARM_SLOTS + RIGHT_ARM_SLOTS:
        bus, en, proto, mid, master = rows[slot]
        assert en and proto == 3 and mid >= 1 and master != 0
    assert rows[LIFT_SLOT][1] is False
    for slot in range(14, 20):
        bus, en, proto, mid, _master = rows[slot]
        assert en and proto == 1 and bus in (4, 5, 6)


def test_arm_goal_position_packing() -> None:
    session, store = _session()
    arm = PcbArmDriver(session, side="left", skip_home_on_connect=True)
    arm.is_connected = True
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)
    arm.write("Goal_Position", q)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d = store.actuators[slot]
        assert d.position == pytest.approx(float(q[i]))
        assert d.kp > 0


def test_arm_zero_torque() -> None:
    session, store = _session()
    arm = PcbArmDriver(session, side="right", skip_home_on_connect=True)
    arm.is_connected = True
    arm.write("Goal_Position", np.ones(7, dtype=np.float32))
    arm.write("Zero_Torque", True)
    for slot in RIGHT_ARM_SLOTS:
        d = store.actuators[slot]
        assert d.kp == 0.0 and d.position == 0.0


def test_arm_read_position() -> None:
    session, store = _session()
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        store.fb_pos[slot] = 1.0 + 0.1 * i
    arm = PcbArmDriver(session, side="left", skip_home_on_connect=True)
    arm.is_connected = True
    pos = arm.read("Position_Rad")
    assert pos.shape == (7,)
    assert float(pos[0]) == pytest.approx(1.0)
    assert float(pos[6]) == pytest.approx(1.6)


def test_platform_base_target_and_lift_stub() -> None:
    session, store = _session()
    plat = PcbPlatformClient(session, use_neck=False)
    plat.connect()
    plat.send_target_state(
        {"BwC": 0.5, "BwR": -0.5, "BwL": 0.0},
        {"BpC": 1.0, "BpR": 1.1, "BpL": 1.2},
    )
    assert store.actuators[BASE_STEER_SLOTS["BwC"]].position == pytest.approx(0.5)
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].velocity == pytest.approx(1.0)
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].kp == 0.0
    plat.send_command(("lift_cmd", 150.0))
    assert LIFT_SLOT not in store.actuators
    st = plat.get_state()
    assert st["lift_unimplemented"] == 1.0
    assert st["lift_height"] == 0.0
    assert st["target_bwc_angle"] == pytest.approx(0.5)


def test_platform_disable_drive_and_watchdog() -> None:
    session, store = _session()
    plat = PcbPlatformClient(session, use_neck=False, watchdog_s=0.05)
    plat.connect()
    plat.send_target_state(
        {"BwC": 0.2, "BwR": 0.2, "BwL": 0.2},
        {"BpC": 0.5, "BpR": 0.5, "BpL": 0.5},
    )
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].velocity == pytest.approx(0.5)

    plat.send_command(("disable_drive_current",))
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].kp == 0.0
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].velocity == 0.0
    # Steer still held
    assert store.actuators[BASE_STEER_SLOTS["BwC"]].position == pytest.approx(0.2)

    plat.send_command(("enable_drive_current",))
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].velocity == pytest.approx(0.5)

    # Watchdog: silence then get_state trips stop
    plat._last_cmd_t = time.monotonic() - 1.0  # noqa: SLF001
    plat.get_state()
    assert store.actuators[BASE_DRIVE_SLOTS["BpC"]].velocity == 0.0
    assert store.actuators[BASE_STEER_SLOTS["BwC"]].kp > 0


def test_base_cmd_stop_zeros_drive() -> None:
    session, store = _session()
    plat = PcbPlatformClient(session)
    plat.connect()
    plat.send_target_state(
        {"BwC": 0.0, "BwR": 0.0, "BwL": 0.0},
        {"BpC": 0.3, "BpR": 0.3, "BpL": 0.3},
    )
    plat.send_command(("base_cmd", 0.0, 0.0, 0.0))
    assert store.actuators[BASE_DRIVE_SLOTS["BpR"]].velocity == 0.0


def test_neck_and_led() -> None:
    session, store = _session()
    neck = PcbNeckDriver(session, pitch_offset_deg=30.0)
    neck.go_to(0.0, 10.0)
    assert store.servos[0].native_step_position == deg_to_steps(30.0)
    assert store.servos[1].native_step_position == deg_to_steps(10.0)
    set_led(session, 2, brightness=10)
    assert store.led is not None and store.led.mode == 2
    led_off(session)
    assert store.led.mode == 0


def test_led_factory_pattern_helpers() -> None:
    """Named factory/traffic-light helpers (docs/rfc-led-factory-patterns.md)
    resolve to the right mode + carry brightness through, same contract as
    the generic set_led/led_off path above."""
    session, store = _session()

    led_solid_green(session, brightness=20)
    assert store.led.mode == 3 and store.led.master_brightness == 20

    led_solid_yellow(session, brightness=12)
    assert store.led.mode == 4 and store.led.master_brightness == 12

    led_solid_red(session, brightness=31)
    assert store.led.mode == 5 and store.led.master_brightness == 31

    led_caution(session, brightness=8)
    assert store.led.mode == 6

    led_fault(session, brightness=8)
    assert store.led.mode == 7
