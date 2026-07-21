"""Golden tests for FaultRecorder (black box) — no hardware."""
from __future__ import annotations

import json
import os
import sys
import time

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.telemetry import TelemetryCache
from deft_controls_sdk.telemetry.recorder import FaultRecorder


def _fb_kwargs(**overrides):
    base = dict(
        tick=0,
        ack_seq=0,
        mcu_state=0,
        plant_block=0,
        plant_block_name="none",
        pdu_tag=None,
        lap_ms=None,
        lap_max_ms=None,
        ticks_pending=None,
        svd_present=False,
        actuators=[],
    )
    base.update(overrides)
    return base


# -- FaultRecorder in isolation --------------------------------------------------------


def test_ring_buffer_is_bounded(tmp_path) -> None:
    rec = FaultRecorder(tmp_path, ring_size=5, post_fault_ticks=2)
    for i in range(20):
        rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})
    assert len(rec._ring) == 5  # never grows past ring_size


def test_fault_transition_triggers_dump_with_pre_and_post_context(tmp_path) -> None:
    rec = FaultRecorder(tmp_path, ring_size=10, post_fault_ticks=3, startup_grace_s=0.0)
    # 3 healthy ticks (pre-fault context)
    for i in range(3):
        rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})
    # fault appears
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.capturing_fault is True
    # post-fault ticks — dump flushes once remaining_ticks hits 0
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.capturing_fault is True  # still counting down
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.capturing_fault is False  # flushed
    assert rec.fault_count == 1
    assert rec.last_fault_path is not None
    assert rec.flush()
    assert rec.last_fault_path.is_file()

    lines = rec.last_fault_path.read_text(encoding="utf-8").strip().splitlines()
    # 3 pre-fault + 1 trigger tick + 3 post-fault ticks = 7 lines
    assert len(lines) == 7
    parsed = [json.loads(line) for line in lines]
    assert parsed[0]["plant_block"] == 0  # pre-fault context preserved
    assert parsed[3]["plant_block"] == 5  # trigger tick present
    assert parsed[-1]["plant_block"] == 5


def test_fault_does_not_refire_while_condition_persists(tmp_path) -> None:
    rec = FaultRecorder(tmp_path, ring_size=5, post_fault_ticks=2, startup_grace_s=0.0)
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})  # flush #1
    assert rec.fault_count == 1
    # still faulted — must not trigger a second dump on every subsequent tick
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.fault_count == 1
    # clears, then re-faults -> new incident
    rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.fault_count == 2


def test_actuator_fault_bit_also_triggers(tmp_path) -> None:
    rec = FaultRecorder(tmp_path, ring_size=5, post_fault_ticks=1, startup_grace_s=0.0)
    rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": [{"fault": 0}]})
    rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": [{"fault": 0x02}]})
    rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": [{"fault": 0x02}]})
    assert rec.fault_count == 1


def test_old_fault_files_are_pruned(tmp_path) -> None:
    rec = FaultRecorder(tmp_path, ring_size=2, post_fault_ticks=1, max_fault_files=3, startup_grace_s=0.0)
    for incident in range(6):
        rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})
        rec.observe({"plant_block": 7, "mcu_state": 0, "actuators": []})
        rec.observe({"plant_block": 7, "mcu_state": 0, "actuators": []})  # flush
    assert rec.flush()
    faults_dir = tmp_path / "faults"
    assert len(list(faults_dir.glob("fault_*.ndjson"))) == 3  # never more than max_fault_files


def test_manual_recording_writes_log_lines(tmp_path) -> None:
    rec = FaultRecorder(tmp_path)
    path = rec.start_recording()
    assert rec.is_recording is True
    for i in range(4):
        rec.log_line({"schema": "deft_fb_v1", "tick": i})
        rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})  # faults only
    rec.stop_recording()
    assert rec.flush()
    assert rec.is_recording is False
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 4


def test_startup_grace_suppresses_cold_start_false_positive(tmp_path) -> None:
    """Reproduces the real bench observation: the very first tick a recorder
    ever sees can legitimately read plant_block=HOST_STALE before streaming
    warms up. Without a grace window this always looked like a fresh fault."""
    rec = FaultRecorder(tmp_path, ring_size=5, post_fault_ticks=2, startup_grace_s=10.0)
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})  # first tick ever
    assert rec.capturing_fault is False
    assert rec.fault_count == 0
    rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})  # clears, as it did on the bench
    assert rec.fault_count == 0


def test_fault_still_triggers_once_grace_period_elapses(tmp_path) -> None:
    rec = FaultRecorder(tmp_path, ring_size=5, post_fault_ticks=1, startup_grace_s=0.05)
    rec.observe({"plant_block": 0, "mcu_state": 0, "actuators": []})  # healthy baseline
    time.sleep(0.1)  # cross the grace boundary
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})  # genuine transition now
    assert rec.capturing_fault is True
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.fault_count == 1


def test_zero_grace_period_preserves_immediate_trigger_behavior(tmp_path) -> None:
    """Pin the pre-grace-period behavior when explicitly disabled (grace=0)."""
    rec = FaultRecorder(tmp_path, ring_size=5, post_fault_ticks=1, startup_grace_s=0.0)
    rec.observe({"plant_block": 5, "mcu_state": 0, "actuators": []})
    assert rec.capturing_fault is True


def test_start_recording_is_idempotent(tmp_path) -> None:
    rec = FaultRecorder(tmp_path)
    path1 = rec.start_recording()
    path2 = rec.start_recording()
    assert path1 == path2
    rec.stop_recording()


# -- wired through TelemetryCache -------------------------------------------------------


def test_telemetry_cache_exposes_recording_status(tmp_path) -> None:
    from deft_controls_sdk.link.exchange import IMAGE_BYTES

    cache = TelemetryCache(session_dir=tmp_path, persist=True)
    cache.set_connected(True, port="COM5")
    assert cache.snapshot().recording is False
    cache.start_recording()
    # Recording lines require a wire image (compact deft_fb_v1), not fat SessionState.
    cache.update_from_feedback(**_fb_kwargs(tick=1), raw=b"\x00" * IMAGE_BYTES)
    snap = cache.snapshot()
    assert snap.recording is True
    assert snap.recording_bytes > 0
    cache.stop_recording()
    assert cache.snapshot().recording is False


def test_telemetry_cache_fault_dump_on_plant_block(tmp_path) -> None:
    recorder = FaultRecorder(tmp_path, startup_grace_s=0.0)
    cache = TelemetryCache(session_dir=tmp_path, persist=True, fault_recorder=recorder)
    cache.set_connected(True, port="COM5")
    cache.update_from_feedback(**_fb_kwargs(tick=1, plant_block=0))
    cache.update_from_feedback(**_fb_kwargs(tick=2, plant_block=1, plant_block_name="bench_session"))
    snap = cache.snapshot()
    assert snap.capturing_fault is True
    assert snap.fault_count == 0  # still capturing post-fault ticks


def test_telemetry_cache_default_grace_suppresses_connect_time_false_positive(tmp_path) -> None:
    """Without overriding the recorder, the default startup grace should absorb
    a plant_block/HOST_STALE reading on the very first tick after connect —
    this is the exact scenario that produced spurious fault dumps on the bench."""
    cache = TelemetryCache(session_dir=tmp_path, persist=True)
    cache.set_connected(True, port="COM5")
    cache.update_from_feedback(**_fb_kwargs(tick=1, plant_block=5, plant_block_name="host_stale"))
    assert cache.snapshot().capturing_fault is False
    assert cache.snapshot().fault_count == 0


def test_disconnect_stops_manual_recording(tmp_path) -> None:
    cache = TelemetryCache(session_dir=tmp_path, persist=True)
    cache.set_connected(True, port="COM5")
    cache.start_recording()
    assert cache.is_recording is True
    cache.set_connected(False)
    assert cache.is_recording is False


def test_update_from_feedback_does_not_block_on_slow_disk(tmp_path) -> None:
    """Control-path regress: even if disk is slow, update_from_feedback returns
    quickly (I/O is on the background writer)."""
    cache = TelemetryCache(session_dir=tmp_path, persist=True, state_hz=10.0)
    cache.set_connected(True, port="COM5")
    t0 = time.perf_counter()
    for i in range(100):
        cache.update_from_feedback(**_fb_kwargs(tick=i))
    elapsed = time.perf_counter() - t0
    # 100 sync write+rename cycles on a bad Windows path can take seconds;
    # RAM+enqueue should stay well under 250 ms even on a slow machine.
    assert elapsed < 0.25
    assert cache.flush()
    assert (tmp_path / "state.json").is_file()


def test_state_json_stays_flat_snapshot_not_a_growing_log(tmp_path) -> None:
    """The concern this whole feature was built to address without regressing:
    state.json must remain exactly one JSON object, never an appended log."""
    cache = TelemetryCache(session_dir=tmp_path, persist=True)
    cache.set_connected(True, port="COM5")
    for i in range(50):
        cache.update_from_feedback(**_fb_kwargs(tick=i))
    assert cache.flush()
    raw = (tmp_path / "state.json").read_text(encoding="utf-8")
    parsed = json.loads(raw)  # must parse as ONE object, not NDJSON
    assert isinstance(parsed, dict)
    assert parsed["tick"] == 49
