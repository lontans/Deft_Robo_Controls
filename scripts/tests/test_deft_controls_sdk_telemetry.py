"""Unit tests for TelemetryCache (no hardware)."""
from __future__ import annotations

import json
import os
import sys

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.telemetry import TelemetryCache


def test_atomic_state_json(tmp_path):
    cache = TelemetryCache(session_dir=tmp_path, persist=True)
    cache.set_connected(True, port="COM5", mode="idle")
    cache.update_from_feedback(
        tick=42,
        ack_seq=3,
        mcu_state=0,
        plant_block=0,
        plant_block_name="none",
        pdu_tag="S",
        lap_ms=1,
        lap_max_ms=2,
        ticks_pending=0,
        svd_present=True,
        actuators=[{"slot": 0, "position": 0.1, "velocity": 0.0, "torque": 0.0, "temperature": 30.0, "fault": 0}],
        mode="plant_stream",
    )
    assert cache.flush()
    path = tmp_path / "state.json"
    assert path.is_file()
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["schema"] == "deft_telemetry_v1"
    assert data["tick"] == 42
    assert data["grade"] == "green"
    assert data["port"] == "COM5"
    snap = cache.snapshot()
    assert snap.tick == 42
    assert snap.actuators[0]["position"] == pytest.approx(0.1)


def test_host_stale_context(tmp_path):
    cache = TelemetryCache(session_dir=tmp_path, persist=False)
    cache.set_connected(True, port="COM5")
    cache.update_from_feedback(
        tick=1,
        ack_seq=0,
        mcu_state=0,
        plant_block=5,
        plant_block_name="host_stale",
        pdu_tag="S",
        lap_ms=1,
        lap_max_ms=1,
        ticks_pending=0,
        svd_present=True,
        actuators=[],
    )
    snap = cache.snapshot()
    assert snap.grade == "yellow"
    assert any("HOST_STALE" in c for c in snap.context)


def test_mcp_fb_starve_vs_host_stall(tmp_path):
    """Low fb_hz with healthy send_ms is MCP USB starve, not a stuck host loop."""
    from deft_controls_sdk.telemetry.cache import _grade_and_context

    cache = TelemetryCache(session_dir=tmp_path, persist=False)
    cache.set_connected(True, port="COM5", mode="plant_stream")
    cache.update_from_feedback(
        tick=100,
        ack_seq=10,
        mcu_state=0,
        plant_block=0,
        plant_block_name="none",
        pdu_tag="S",
        lap_ms=0,
        lap_max_ms=1,
        ticks_pending=0,
        svd_present=True,
        actuators=[],
        mode="plant_stream",
    )
    cache.update_stream_timing(
        send_ms=1.8, poll_ms=0.01, publish_ms=0.0, loop_ms=1.9, tx_hz=50.0
    )
    # update_stream_timing does not re-grade (hot path); set fb_hz then grade.
    with cache._lock:
        cache._state.fb_hz = 4.0
        cache._state.age_s = 0.0
        _grade_and_context(cache._state)
    snap = cache.snapshot()
    assert snap.grade == "yellow"
    assert "sparse" in snap.summary.lower() or "fb=" in snap.summary.lower()
    assert any("fb_hz=4.0" in c for c in snap.context)
    assert snap.stream_tx_hz == pytest.approx(50.0)
