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
    SERVO0_FB_OFF,
    SERVO_SLOT_BYTES,
)
from deft_controls_sdk.pdb import KILL_NORMAL, PdbStatus
from deft_controls_sdk.vbeta import (
    BASE_DRIVE_SLOTS,
    BASE_STEER_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    RIG_RS02_BUS6_SLOT,
    PcbArmDriver,
    PcbNeckDriver,
    PcbPlatformClient,
    RIGHT_ARM_SLOTS,
    RigComponents,
    deg_to_steps,
    led_caution,
    led_fault,
    clamp_q7,
    led_idle,
    led_off,
    led_solid_green,
    led_solid_red,
    led_solid_yellow,
    neck_hold_present,
    pdb_poll,
    robstride_soft_hold,
    set_led,
    steps_to_deg,
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
    arm = PcbArmDriver(
        session, side="left", skip_home_on_connect=True, clamp_goals=False
    )
    arm.is_connected = True
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)
    arm.write("Goal_Position", q)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d = store.actuators[slot]
        assert d.position == pytest.approx(float(q[i]))
        assert d.kp > 0


def test_arm_goal_position_clamped_by_default() -> None:
    session, store = _session()
    arm = PcbArmDriver(session, side="left", skip_home_on_connect=True)
    arm.is_connected = True
    # Out-of-envelope command must land on the same soft window as clamp_q7
    # (MuJoCo ± left bench-clear motor frame when CLEAR_ACTIVE).
    q = np.array([0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.5], dtype=np.float32)
    expected = clamp_q7(q, "left")
    arm.write("Goal_Position", q)
    got = store.actuators[LEFT_ARM_SLOTS[1]].position
    assert got == pytest.approx(float(expected[1]))
    assert not np.allclose(expected, q)


def test_arm_zero_torque() -> None:
    session, store = _session()
    arm = PcbArmDriver(
        session, side="right", skip_home_on_connect=True, clamp_goals=False
    )
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


def test_platform_neck_cmd_no_double_offset() -> None:
    """PcbPlatformClient must forward neck pitch/yaw raw (ref: reference
    FeatherPlatformClient control loop calls neck.go_to(pitch, yaw) with no
    offset). YAMAIMobile.convert_vr_head_angles already bakes
    neck_pitch_offset_deg in before enqueueing "neck_cmd" — re-applying it
    here would double it."""
    session, store = _session()
    plat = PcbPlatformClient(session, use_neck=True)
    plat.connect()
    plat.send_command(("neck_cmd", 12.5, -7.0))
    assert store.servos[0].native_step_position == deg_to_steps(12.5)
    assert store.servos[1].native_step_position == deg_to_steps(-7.0)


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
    """Named factory/traffic-light helpers (docs/legacy/rfc/rfc-led-factory-patterns.md)
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

    led_idle(session, brightness=12)
    assert store.led.mode == 8 and store.led.master_brightness == 12


# -- Rig components (single Damiao arm + optional RS/neck/LED/PDU) -----------------


def _session_with_servo_positions(pitch_steps: int, yaw_steps: int) -> tuple[PcbRobotSession, FakeStore]:
    """Same fake session/store as `_session()`, but `make_fb()` also fills the
    servo feedback block so `neck_hold_present()` has something to read."""
    session, store = _session()
    base_make_fb = store.make_fb

    def make_fb_with_servos() -> bytes:
        buf = bytearray(base_make_fb())
        struct.pack_into("<hhH", buf, SERVO0_FB_OFF + 0 * SERVO_SLOT_BYTES, pitch_steps, 0, 0)
        struct.pack_into("<hhH", buf, SERVO0_FB_OFF + 1 * SERVO_SLOT_BYTES, yaw_steps, 0, 0)
        return bytes(buf)

    store.make_fb = make_fb_with_servos  # type: ignore[method-assign]
    return session, store


def test_robstride_soft_hold_uses_present_fb_position() -> None:
    session, store = _session()
    store.fb_pos[RIG_RS02_BUS6_SLOT] = 0.75
    desire = robstride_soft_hold(session)
    assert desire.position == pytest.approx(0.75)
    assert desire.kp > 0 and desire.kd > 0
    assert store.actuators[RIG_RS02_BUS6_SLOT].position == pytest.approx(0.75)


def test_robstride_soft_hold_defaults_to_zero_without_prior_fb() -> None:
    session, store = _session()
    desire = robstride_soft_hold(session)
    assert desire.position == pytest.approx(0.0)


def test_robstride_soft_hold_explicit_position_overrides_fb() -> None:
    session, store = _session()
    store.fb_pos[RIG_RS02_BUS6_SLOT] = 0.75
    desire = robstride_soft_hold(session, position=1.5)
    assert desire.position == pytest.approx(1.5)
    assert store.actuators[RIG_RS02_BUS6_SLOT].position == pytest.approx(1.5)


def test_neck_hold_present_reads_servo_fb_and_reissues_same_pose() -> None:
    pitch_steps = deg_to_steps(-10.0)
    yaw_steps = deg_to_steps(5.0)
    session, store = _session_with_servo_positions(pitch_steps, yaw_steps)
    neck = PcbNeckDriver(session)
    held = neck_hold_present(session, neck)
    assert held is not None
    pitch_deg, yaw_deg = held
    assert pitch_deg == pytest.approx(steps_to_deg(pitch_steps))
    assert yaw_deg == pytest.approx(steps_to_deg(yaw_steps))
    assert store.servos[0].native_step_position == pitch_steps
    assert store.servos[1].native_step_position == yaw_steps


def test_pdb_poll_delegates_to_hub_pdb_status() -> None:
    session, store = _session()
    canned = PdbStatus(
        kill_state=KILL_NORMAL,
        kill_reason=0,
        estop_sense=1,
        kill_state_name="normal",
        kill_reason_name="none",
        stale_failsafe=False,
    )
    store.pdb_status = lambda raw=None: canned  # type: ignore[method-assign]
    assert pdb_poll(session) is canned


def test_rig_components_tick_composes_attached_pieces() -> None:
    session, store = _session()
    store.fb_pos[RIG_RS02_BUS6_SLOT] = 0.3
    canned = PdbStatus(
        kill_state=KILL_NORMAL, kill_reason=0, estop_sense=1,
        kill_state_name="normal", kill_reason_name="none", stale_failsafe=False,
    )
    store.pdb_status = lambda raw=None: canned  # type: ignore[method-assign]

    rig = RigComponents(
        session=session,
        use_robstride=True,
        neck=PcbNeckDriver(session),
        use_led_idle=True,
        poll_pdb=True,
    )
    result = rig.tick()

    assert result.soft_kill_parked is False
    assert result.robstride_position == pytest.approx(0.3)
    assert result.neck_pose_deg is not None  # zero-steps default FB, still a valid hold
    assert result.pdb_status is canned
    assert store.led is not None and store.led.mode == 8


def test_rig_components_tick_skips_everything_when_soft_kill_parked() -> None:
    class _ParkedStore(FakeStore):
        def soft_kill_park_if_requested(self, *, send: bool = True) -> bool:
            return True

    store = _ParkedStore()
    session = PcbRobotSession.__new__(PcbRobotSession)
    session._hub = store  # type: ignore[assignment]
    session._owns_hub = False
    session._stream_hz = 40.0
    session._closed = False

    rig = RigComponents(
        session=session, use_robstride=True, neck=PcbNeckDriver(session), use_led_idle=True,
    )
    result = rig.tick()

    assert result.soft_kill_parked is True
    assert result.robstride_position is None
    assert result.neck_pose_deg is None
    assert RIG_RS02_BUS6_SLOT not in store.actuators
    assert store.led is None
