"""Golden tests for controls_hub_controller (no hardware)."""
from __future__ import annotations

import struct

import pytest

from controls_hub_controller import (
    ActuatorDesire,
    CommandImage,
    FeedbackImage,
    InvalidFrameError,
    InvalidSlotError,
    McuState,
    PlantBlockReason,
)
from controls_pcb_host.protocol import (
    ACTUATOR_COUNT,
    HOST_COMMAND_MAGIC,
    HOST_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
)


def test_command_image_size_and_header() -> None:
    img = CommandImage(seq=42, mcu_state=McuState.NORMAL)
    buf = img.to_bytes()
    assert len(buf) == IMAGE_BYTES
    magic, lv, size, seq = struct.unpack_from("<IHHI", buf, 0)
    assert magic == HOST_COMMAND_MAGIC
    assert lv == HOST_LAYOUT_VERSION
    assert size == IMAGE_BYTES
    assert seq == 42


def test_command_image_mcu_state_roundtrip() -> None:
    for state in McuState:
        img = CommandImage(mcu_state=state)
        sys_word, = struct.unpack_from("<I", img.to_bytes(), 12)
        assert (sys_word >> 1) & 7 == int(state)


def test_set_actuator_roundtrip_matches_wire_offsets() -> None:
    desire = ActuatorDesire(position=1.5, velocity=-0.25, kp=8.0, kd=0.45, torque=0.1)
    img = CommandImage().set_actuator(3, desire)
    buf = img.to_bytes()
    off = 16 + 3 * 20  # ACTUATOR0_OFF + slot * ACTUATOR_SLOT_BYTES
    pos, vel, kp, kd, tau = struct.unpack_from("<fffff", buf, off)
    assert (pos, vel, kp, kd, tau) == pytest.approx((1.5, -0.25, 8.0, 0.45, 0.1))
    assert img.desire(3) == desire


def test_unset_slots_are_zero_on_the_wire() -> None:
    img = CommandImage().set_actuator(0, ActuatorDesire(kp=8.0))
    buf = img.to_bytes()
    off = 16 + 1 * 20  # slot 1, never set
    assert struct.unpack_from("<fffff", buf, off) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_set_actuators_batch() -> None:
    d0 = ActuatorDesire(position=0.1)
    d1 = ActuatorDesire(position=-0.2)
    img = CommandImage().set_actuators({0: d0, 1: d1})
    assert img.desire(0) == d0
    assert img.desire(1) == d1


@pytest.mark.parametrize("slot", [-1, ACTUATOR_COUNT, ACTUATOR_COUNT + 5])
def test_invalid_slot_raises(slot: int) -> None:
    with pytest.raises(InvalidSlotError):
        CommandImage().set_actuator(slot, ActuatorDesire())


def _build_feedback_frame(*, seq_echo: int = 0, plant_block: int = 0, slot: int = 3,
                           position: float = 1.25, velocity: float = 0.0,
                           torque: float = 0.0, temperature: float = 25.0,
                           fault: int = 0) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0)
    sys_word = ((seq_echo & 0xFF) << 17) | ((plant_block & 0x7F) << 25)
    struct.pack_into("<I", buf, 12, sys_word)
    off = 16 + slot * 20
    struct.pack_into("<ffffI", buf, off, position, velocity, torque, temperature, fault)
    return bytes(buf)


def test_feedback_image_parses_ack_seq_and_plant_block() -> None:
    frame = _build_feedback_frame(seq_echo=0x2A, plant_block=int(PlantBlockReason.HOST_STALE))
    fb = FeedbackImage(frame)
    assert fb.ack_seq == 0x2A
    assert fb.plant_block == PlantBlockReason.HOST_STALE


def test_feedback_image_parses_actuator_slot() -> None:
    frame = _build_feedback_frame(slot=3, position=1.25, velocity=0.5, torque=0.2, temperature=31.0, fault=7)
    fb = FeedbackImage(frame)
    state = fb.actuator(3)
    assert state is not None
    assert (state.position, state.velocity, state.torque, state.temperature, state.fault) == pytest.approx(
        (1.25, 0.5, 0.2, 31.0, 7)
    )
    assert fb.actuator(0).position == 0.0  # untouched slot, still parsed as zero


def test_feedback_image_rejects_bad_size() -> None:
    with pytest.raises(InvalidFrameError):
        FeedbackImage(b"\x00" * 10)


def test_feedback_image_rejects_bad_magic() -> None:
    buf = bytearray(IMAGE_BYTES)
    with pytest.raises(InvalidFrameError):
        FeedbackImage(bytes(buf))


def test_ack_seq_tracks_command_seq_low_byte() -> None:
    cmd = CommandImage(seq=0x1FA)
    frame = _build_feedback_frame(seq_echo=cmd.seq & 0xFF)
    fb = FeedbackImage(frame)
    assert fb.ack_seq == cmd.seq & 0xFF
