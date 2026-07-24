"""Unit tests for TelemetryCache (no hardware)."""
from __future__ import annotations

import json
import os
import struct
import sys

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.link import Connection, FeedbackImage
from deft_controls_sdk.link.exchange import IMAGE_BYTES
from deft_controls_sdk.link.exchange.wire_layout import (
    HOST_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    PDB_OFF,
    SYSTEM_FB_OFF,
)
from deft_controls_sdk.pdb import KILL_REASON_OVERCURRENT, KILL_SOFT_REQ, pack_feedback
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


def test_pdb_status_round_trips_through_snapshot(tmp_path) -> None:
    """update_from_feedback's optional pdb_status kwarg must survive to
    snapshot_dict() untouched — that's what the dashboard's PDU card reads."""
    cache = TelemetryCache(session_dir=tmp_path, persist=False)
    cache.set_connected(True, port="COM5")
    pdb_status = {
        "kill_state": 1,
        "kill_state_name": "soft_kill_req",
        "estop_sense": 1,
        "pdb": {"kill_state": 1, "estop_sense": 1, "contactor_state": 3},
        "pack_v_V": (48.0, 0.0, 0.0, 0.0),
    }
    cache.update_from_feedback(
        tick=1, ack_seq=0, mcu_state=0, plant_block=0, plant_block_name="none",
        pdu_tag="S", lap_ms=1, lap_max_ms=1, ticks_pending=0, svd_present=True,
        actuators=[], pdb_status=pdb_status,
    )
    assert cache.snapshot_dict()["pdb_status"] == pdb_status


def test_pdb_status_defaults_to_none() -> None:
    cache = TelemetryCache(session_dir=None, persist=False)
    cache.set_connected(True, port="COM5")
    cache.update_from_feedback(
        tick=1, ack_seq=0, mcu_state=0, plant_block=0, plant_block_name="none",
        pdu_tag="S", lap_ms=1, lap_max_ms=1, ticks_pending=0, svd_present=True,
        actuators=[],
    )
    assert cache.snapshot_dict()["pdb_status"] is None


def _build_plant_image_with_pdb_mirror() -> bytes:
    """Full IMAGE_BYTES plant feedback frame with a valid PDBF mirror at
    PDB_OFF — same construction as
    test_pdb_link_frames.py::test_parse_system_kill_and_pdb_status_from_usb_image,
    reused here to exercise Connection.publish_feedback's new pdb_status wiring
    end-to-end rather than pdb_status_from_frame() in isolation."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 1)
    buf[SYSTEM_FB_OFF + 14] = KILL_SOFT_REQ
    buf[SYSTEM_FB_OFF + 15] = KILL_REASON_OVERCURRENT
    buf[SYSTEM_FB_OFF + 16] = 1
    pdb = pack_feedback(
        seq=9,
        pack_v=(4800, 0, 0, 0),
        rail_v=(4800, 1900, 1200, 500),
        pack_i=(100, 0, 0, 0),
        rail_i=(10, 20, 30, 40),
        kill_state=KILL_SOFT_REQ,
        kill_reason=KILL_REASON_OVERCURRENT,
        estop_sense=1,
    )
    buf[PDB_OFF : PDB_OFF + len(pdb)] = pdb
    return bytes(buf)


def test_connection_publish_feedback_threads_pdb_status_to_telemetry(tmp_path) -> None:
    """publish_feedback() must compute PdbStatus from the raw plant image and
    hand it to TelemetryCache — no COM5, no real serial (Connection("TESTPORT")
    never opens a port until you call .open())."""
    conn = Connection("TESTPORT")
    cache = TelemetryCache(session_dir=tmp_path, persist=False)
    conn.attach_telemetry(cache)

    raw = _build_plant_image_with_pdb_mirror()
    conn.publish_feedback(FeedbackImage(raw))

    pdb_status = cache.snapshot_dict()["pdb_status"]
    assert pdb_status is not None
    assert pdb_status["kill_state_name"] == "soft_kill_req"
    assert pdb_status["pack_v_V"] == (48.0, 0.0, 0.0, 0.0)
    assert pdb_status["pdb"]["estop_sense"] == 1


def test_connection_publish_feedback_pdb_status_normal_before_any_pdb_traffic(tmp_path) -> None:
    """A valid plant image with all-zero system-kill bytes (no PDB UART
    traffic mirrored yet) must still decode cleanly to kill_state=NORMAL, not
    raise or leave pdb_status None — FeedbackImage() itself is what enforces
    the IMAGE_BYTES length pdb_status_from_frame() checks, so a too-short raw
    can never reach publish_feedback() in the first place."""
    conn = Connection("TESTPORT")
    cache = TelemetryCache(session_dir=tmp_path, persist=False)
    conn.attach_telemetry(cache)

    raw = bytearray(IMAGE_BYTES)
    struct.pack_into("<IHHI", raw, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 1)
    conn.publish_feedback(FeedbackImage(bytes(raw)))

    pdb_status = cache.snapshot_dict()["pdb_status"]
    assert pdb_status is not None
    assert pdb_status["kill_state_name"] == "normal"
