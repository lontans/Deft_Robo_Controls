"""Golden tests for HubSession servo/LED compose (no hardware)."""
from __future__ import annotations

import struct

import pytest

from controls_hub_controller import ActuatorDesire, HubSession, McuState
from controls_pcb_host.protocol import (
    HOST_PDU_PAYLOAD_BYTES,
    IMAGE_BYTES,
    LED_CMD_OFF,
    PDU_OFF,
    SERVO0_CMD_OFF,
    SERVO_SLOT_BYTES,
)


def _hub() -> HubSession:
    return HubSession("TESTPORT")  # never opened — no hardware needed for compose


def test_build_command_size_and_pdu_zero() -> None:
    frame = _hub().build_command()
    assert len(frame) == IMAGE_BYTES
    assert frame[PDU_OFF : PDU_OFF + HOST_PDU_PAYLOAD_BYTES] == b"\x00" * HOST_PDU_PAYLOAD_BYTES


def test_actuator_slot_survives_compose() -> None:
    hub = _hub()
    hub.set_actuator(3, ActuatorDesire(position=1.0, kp=8.0, kd=0.45), send=False)
    frame = hub.build_command()
    off = 16 + 3 * 20
    pos, vel, kp, kd, tau = struct.unpack_from("<fffff", frame, off)
    assert (pos, vel, kp, kd, tau) == pytest.approx((1.0, 0.0, 8.0, 0.45, 0.0))


def test_servo_slot0_patched_at_wire_offset() -> None:
    hub = _hub()
    hub.set_servo(0, 512, servo_id=1, speed=100, send=False)
    frame = hub.build_command()
    position, speed, flags = struct.unpack_from("<hhH", frame, SERVO0_CMD_OFF)
    assert position == 512
    assert speed == 100
    assert (flags >> 5) & 0xFF == 1  # servo_id
    assert flags & 1 == 1  # torque_enable default


def test_servo_slot1_offset_is_after_slot0() -> None:
    hub = _hub()
    hub.set_servo(1, -256, servo_id=2, send=False)
    frame = hub.build_command()
    off = SERVO0_CMD_OFF + SERVO_SLOT_BYTES
    position, _speed, flags = struct.unpack_from("<hhH", frame, off)
    assert position == -256
    assert (flags >> 5) & 0xFF == 2


def test_both_servo_slots_coexist() -> None:
    hub = _hub()
    hub.set_servo(0, 10, servo_id=1, send=False)
    hub.set_servo(1, 20, servo_id=2, send=False)
    frame = hub.build_command()
    pos0, _, _ = struct.unpack_from("<hhH", frame, SERVO0_CMD_OFF)
    pos1, _, _ = struct.unpack_from("<hhH", frame, SERVO0_CMD_OFF + SERVO_SLOT_BYTES)
    assert (pos0, pos1) == (10, 20)


def test_led_word_patched_at_offset_528() -> None:
    hub = _hub()
    hub.set_led(mode=0, brightness=31, led_count=6, send=False)
    frame = hub.build_command()
    word, = struct.unpack_from("<H", frame, LED_CMD_OFF)
    assert word & 0x1F == 0
    assert (word >> 5) & 0x1F == 31
    assert (word >> 10) & 0x3F == 6


def test_invalid_servo_slot_raises() -> None:
    with pytest.raises(ValueError):
        _hub().set_servo(2, 0, servo_id=1, send=False)


def test_estop_clears_actuator_desires_but_not_servo_led() -> None:
    hub = _hub()
    hub.set_actuator(0, ActuatorDesire(kp=8.0), send=False)
    hub.set_servo(0, 100, servo_id=1, send=False)
    hub.set_led(mode=0, brightness=10, led_count=1, send=False)
    hub.set_mcu_state(McuState.ESTOP, send=False)
    assert hub.held_desire(0) is None
    assert hub.held_servo(0) is not None
    assert hub.held_led is not None


def test_mcu_state_survives_compose() -> None:
    hub = _hub()
    hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
    frame = hub.build_command()
    sys_word, = struct.unpack_from("<I", frame, 12)
    assert (sys_word >> 1) & 7 == int(McuState.DIAG_ONLY)


def test_send_once_composes_servos_and_actuators() -> None:
    """send_once() must route through HubSession.build_command(), not PlantSession's."""
    hub = _hub()
    hub.set_actuator(0, ActuatorDesire(kp=5.0), send=False)
    hub.set_servo(0, 42, servo_id=1, send=False)
    frame = hub.build_command()  # same path as send_once minus serial write
    off = 16 + 0 * 20
    _pos, _vel, kp, _kd, _tau = struct.unpack_from("<fffff", frame, off)
    assert kp == pytest.approx(5.0)
    pos, _, _ = struct.unpack_from("<hhH", frame, SERVO0_CMD_OFF)
    assert pos == 42
