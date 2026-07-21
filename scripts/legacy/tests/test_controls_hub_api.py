"""Golden tests for the ControlsHub façade (no hardware).

PlantSession is constructed but never opened — set_actuator(..., send=False)
and held_desire() don't touch serial, matching the pattern in
test_hub_session.py. Feedback plumbing is exercised by feeding a hand-built
562 B frame straight into ControlsHub's internal cache (poll_feedback goes
through PlantSession/FrameReader, which needs a live pump; here we test the
façade's own bookkeeping instead).
"""
from __future__ import annotations

import struct

import pytest

from controls_hub_api import ActuatorDesire, ControlsHub
from controls_hub_controller import FeedbackImage, PlantBlockReason, PlantSession
from controls_pcb_host.protocol import HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES


def _hub() -> ControlsHub:
    return ControlsHub(PlantSession("TESTPORT"))  # never opened — no hardware needed


def _feedback_frame(*, tick: int = 0, ack_seq: int = 0, mcu_state: int = 0, plant_block: int = 0) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0)
    sys_word = (tick & 0xFFF) | ((mcu_state & 0x7) << 13) | ((ack_seq & 0xFF) << 17) | ((plant_block & 0x7F) << 25)
    struct.pack_into("<I", buf, 12, sys_word)
    return bytes(buf)


def test_plant_set_actuator_delegates_to_session() -> None:
    hub = _hub()
    hub.plant.set_actuator(3, ActuatorDesire(position=1.0, kp=8.0, kd=0.45), send=False)
    held = hub.plant.held_desire(3)
    assert held is not None
    assert held.position == pytest.approx(1.0)
    assert held.kp == pytest.approx(8.0)


def test_plant_set_actuators_batch_delegates() -> None:
    hub = _hub()
    hub.plant.set_actuators({0: ActuatorDesire(kp=5.0), 1: ActuatorDesire(kp=6.0)}, send=False)
    assert hub.plant.held_desire(0).kp == pytest.approx(5.0)
    assert hub.plant.held_desire(1).kp == pytest.approx(6.0)


def test_health_snapshot_before_any_feedback() -> None:
    hub = _hub()
    snap = hub.health.snapshot()
    assert snap.connected is False  # never opened
    assert snap.tick is None
    assert snap.plant_block is None
    assert snap.fb_hz is None


def test_health_snapshot_reflects_recorded_feedback() -> None:
    hub = _hub()
    fb = FeedbackImage(_feedback_frame(tick=42, ack_seq=7, mcu_state=0, plant_block=5))
    hub._record_feedback(fb)  # exercise the same path plant.poll_feedback() uses
    snap = hub.health.snapshot()
    assert snap.tick == 42
    assert snap.ack_seq == 7
    assert snap.plant_block == PlantBlockReason.HOST_STALE


def test_health_fb_hz_computed_from_multiple_samples() -> None:
    hub = _hub()
    fb = FeedbackImage(_feedback_frame())
    for _ in range(5):
        hub._record_feedback(fb)
    snap = hub.health.snapshot()
    assert snap.fb_hz is not None
    assert snap.fb_hz >= 0.0


def test_debug_discover_not_implemented_points_at_cli() -> None:
    hub = _hub()
    with pytest.raises(NotImplementedError, match="control_hub.py discover"):
        hub.debug.discover()


def test_context_manager_closes_session() -> None:
    with _hub() as hub:
        assert hub.plant is not None
    assert hub._session.is_open is False
