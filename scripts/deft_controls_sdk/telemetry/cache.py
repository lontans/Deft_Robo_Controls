"""TelemetryCache — in-memory + optional atomic state.json publish.

Fault-triggered black box + manual recording live in recorder.py
(FaultRecorder) — this module stays focused on "latest snapshot + grading."

Disk I/O (state.json replace, NDJSON append, fault dumps) is performed by
SessionDiskWriter on a background thread. The plant stream loop only updates
RAM and enqueues work — it must never block on write/flush/os.replace.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..link.exchange import ACTUATOR_COUNT
from .fb_log import build_fb_log_record
from .persist import SessionDiskWriter
from .recorder import FaultRecorder


def default_session_dir() -> Path:
    """Stable session dir shared by continuous + debug_dashboard.

    Prefer ``DEFT_SESSION_DIR``; else ``scripts/.deft_session`` (package-adjacent),
    not ``Path.cwd()`` — cwd differs when the dashboard is launched from home /
    IDE vs when ``yam_continuous_all`` runs from ``scripts/``, which made
    localhost UI look empty while state.json was updating elsewhere.
    """
    import os

    env = os.environ.get("DEFT_SESSION_DIR")
    if env:
        return Path(env).expanduser().resolve()
    # cache.py → telemetry → deft_controls_sdk → scripts
    scripts_root = Path(__file__).resolve().parents[2]
    return scripts_root / ".deft_session"


@dataclass
class SessionState:
    """Latest live snapshot. Scripts and debug_dashboard share this shape."""

    schema: str = "deft_telemetry_v1"
    updated_at: float = 0.0
    connected: bool = False
    port: Optional[str] = None
    mode: str = "idle"  # idle | plant_stream | debug_lease
    # Tier-1 health
    tick: Optional[int] = None
    ack_seq: Optional[int] = None
    mcu_state: Optional[int] = None
    plant_block: Optional[int] = None
    plant_block_name: Optional[str] = None
    fb_hz: Optional[float] = None
    age_s: Optional[float] = None
    # Stream-loop timing (ms) — host-side work per tick (NOT the 1/hz period).
    # At 50 Hz the period is ~20 ms; send/loop work is typically ~1–3 ms when
    # healthy. A low fb_hz with send_ms still ~2 ms is MCU USB TX starvation
    # (classic with non-blank MCP CH4–6), not a stuck host loop.
    stream_send_ms: Optional[float] = None
    stream_poll_ms: Optional[float] = None  # reader drain only (should stay <<1 ms)
    stream_credit_wait_ms: Optional[float] = None  # legacy field; stream no longer credit-paces
    stream_publish_ms: Optional[float] = None
    stream_loop_ms: Optional[float] = None
    stream_tx_hz: Optional[float] = None
    # Gaps between host plant writes (ms). Prefer p95 (~25 @ 40 Hz); max sticks
    # on rare OS spikes and made idle look broken at ~27–35.
    stream_tx_gap_p95_ms: Optional[float] = None
    stream_tx_gap_max_ms: Optional[float] = None
    # Host plant seq minus MCU last_cmd_seq (8-bit) at last FB sample.
    # High lag with healthy send_ms means USB FB is sparse (superloop lag),
    # not that Python failed to TX.
    stream_ack_lag: Optional[int] = None
    lap_ms: Optional[int] = None
    lap_max_ms: Optional[int] = None
    periph_lap_ms: Optional[int] = None
    periph_lap_max_ms: Optional[int] = None
    ticks_pending: Optional[int] = None
    svd_present: bool = False
    pdu_tag: Optional[str] = None
    # PDB (power distribution board) status mirrored from the plant feedback
    # image — see deft_controls_sdk.pdb.status.PdbStatus.to_dict(). None until
    # the first feedback frame with a parseable system-kill block arrives.
    pdb_status: Optional[Dict[str, Any]] = None
    grade: str = "red"  # green | yellow | red
    summary: str = "no data yet"
    context: List[str] = field(default_factory=list)
    # Compact actuator strip (all ACTUATOR_COUNT slots; None = no fb yet).
    # Defaults to ACTUATOR_COUNT Nones (not []) so a controller UI always has
    # a fixed number of rows to render, even before the first connect/feedback.
    actuators: List[Optional[Dict[str, Any]]] = field(default_factory=lambda: [None] * ACTUATOR_COUNT)
    # Black box (see recorder.FaultRecorder) — status only; the actual ring
    # buffer / NDJSON files never live in this dataclass, so state.json stays
    # a fixed-size snapshot no matter how many faults or how long a manual
    # recording has been running.
    fault_count: int = 0
    last_fault_path: Optional[str] = None
    capturing_fault: bool = False
    recording: bool = False
    recording_path: Optional[str] = None
    recording_bytes: int = 0
    recording_seconds: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


_PLANT_BLOCK_HINTS = {
    0: ("plant running", "actuator apply is live"),
    1: ("BENCH_SESSION", "debug/bench lease active — plant CAN apply gated until lease ends"),
    2: ("PROBE_BUSY", "a blocking probe is in progress"),
    3: ("QUIET_PERIOD", "post-session quiet window — wait or recover"),
    4: ("DIAG_ONLY", "mcu_state=DIAG_ONLY — plant will not apply"),
    5: ("HOST_STALE", "no fresh host command >500 ms — keep streaming (send_once / start_streaming)"),
    6: ("SERVO_SESSION", "servo host session is holding the plant path"),
}

_MCU_STATE_NAMES = {0: "NORMAL", 1: "RECOVERY", 2: "DIAG_ONLY", 3: "ESTOP"}


def _grade_and_context(state: SessionState) -> None:
    context: List[str] = []
    if not state.connected:
        state.grade = "red"
        state.summary = "disconnected"
        state.context = ["open ControlsPcbHub.connect(port) first"]
        return

    if state.age_s is not None and state.age_s > 1.0:
        state.grade = "red"
        state.summary = f"no feedback for {state.age_s:.1f}s"
        state.context = ["check USB cable / board power / that streaming is running"]
        return

    send_ms = state.stream_send_ms
    host_tx_ok = send_ms is not None and send_ms < 5.0
    fb_low = state.fb_hz is not None and state.fb_hz < 20.0
    fb_very_low = state.fb_hz is not None and state.fb_hz < 10.0

    ack_lag = state.stream_ack_lag
    # Pre-fix: plant mcp2518_try_send still called force_ready()/HAL_Delay and
    # stretched app_run laps; FB collapsed and ack_lag ≈ 0.5*tx_hz. After the
    # non-block plant TX fix, lag should stay near 0 with MCP holds too.
    if (
        state.fb_hz is not None
        and state.fb_hz < 5.0
        and host_tx_ok
        and (state.stream_tx_hz or 0) >= 20.0
    ):
        context.append(
            f"fb_hz={state.fb_hz:.1f} (raw USB FB) with host TX ok — MCU superloop "
            "not completing often enough to TX CDC feedback. Blank MCP to confirm; "
            "if only non-blank MCP trips this, plant MCP TX path is blocking again."
        )
    elif ack_lag is not None and ack_lag >= 26:
        context.append(
            f"ack_lag={ack_lag} - FB samples are sparse vs host TX. Often Apply "
            "left multiple CH4-6 slots ACTIVE (hosts TX can still show gap_p95~25). "
            "Use Idle all, then command one bus."
        )
    elif state.fb_hz is not None and state.fb_hz < 5.0:
        context.append(f"low fb_hz={state.fb_hz:.1f} - expect ~30-50 when streaming idle buses")

    if send_ms is not None and send_ms >= 10.0:
        context.append(
            f"host send_ms={send_ms:.1f} is high - serial write/flush contention "
            "(work time, not the 1/hz period)"
        )

    block = state.plant_block if state.plant_block is not None else -1
    title, hint = _PLANT_BLOCK_HINTS.get(block, (f"plant_block={block}", "unknown gate"))
    if block not in (0, None) and block != 0:
        context.append(f"{title}: {hint}")

    mcu = state.mcu_state
    if mcu is not None and mcu != 0:
        context.append(f"mcu_state={_MCU_STATE_NAMES.get(mcu, mcu)}")

    if state.svd_present and state.lap_max_ms is not None and state.lap_max_ms >= 3:
        context.append(f"superloop lap_max={state.lap_max_ms} ms (budget ~1–2 ms typical)")

    if not state.svd_present and state.pdu_tag and state.pdu_tag not in ("S", "0x00", None):
        context.append(f"timing unavailable — PDU tag={state.pdu_tag} (expected under DEBUG)")

    if block in (0, None) and (mcu in (0, None)) and (state.fb_hz is None or state.fb_hz >= 20):
        state.grade = "green"
        hz = f" fb_hz={state.fb_hz:.0f}" if state.fb_hz is not None else ""
        state.summary = f"OK tick={state.tick}{hz}"
    elif block not in (0, None):
        state.grade = "yellow"
        state.summary = title
    elif (
        state.fb_hz is not None
        and state.fb_hz < 5.0
        and host_tx_ok
    ):
        state.grade = "yellow"
        state.summary = f"USB FB sparse (fb={state.fb_hz:.1f}Hz)"
    elif ack_lag is not None and ack_lag >= 26:
        state.grade = "yellow"
        state.summary = f"ack lag {ack_lag} (sparse FB vs host TX)"
    elif fb_low and host_tx_ok:
        state.grade = "yellow"
        state.summary = "fb low (host TX ok)"
    elif fb_very_low:
        state.grade = "yellow"
        state.summary = "link weak"
    else:
        state.grade = "yellow"
        state.summary = "streaming"

    if state.grade == "green" and not context:
        context.append(hint if block == 0 else "link healthy")
    state.context = context


class TelemetryCache:
    """In-process latest state + optional atomic state.json for other readers.

    Dashboard `/api/state` reads RAM via snapshot_dict() — it never needs the
    on-disk mirror. state.json is for other processes and is published at a
    throttled rate on a background thread (see SessionDiskWriter).
    """

    def __init__(
        self,
        session_dir: Optional[os.PathLike[str] | str] = None,
        *,
        persist: bool = True,
        fault_recorder: Optional[FaultRecorder] = None,
        state_hz: float = 10.0,
    ) -> None:
        self._lock = threading.Lock()
        self._state = SessionState()
        self._fb_times: List[float] = []
        self._fb_frame_mark: Optional[int] = None
        self._fb_frame_mark_t: Optional[float] = None
        self._persist = persist
        self.session_dir = Path(session_dir) if session_dir is not None else default_session_dir()
        self.state_path = self.session_dir / "state.json"
        if persist:
            self.session_dir.mkdir(parents=True, exist_ok=True)
        self._disk = SessionDiskWriter(state_hz=state_hz)
        self._recorder = (
            fault_recorder
            if fault_recorder is not None
            else FaultRecorder(self.session_dir, disk=self._disk)
        )
        # If caller passed a recorder that already owns a disk writer, we still
        # keep ours for state.json; if they share ours, close once.
        self._owns_recorder_disk = fault_recorder is None

    @property
    def is_recording(self) -> bool:
        return self._recorder.is_recording

    def start_recording(self) -> Path:
        """Start compact NDJSON feedback logging (``deft_fb_v1`` + optional raw).

        While recording, each new feedback image (from the telemetry side thread
        or ``log_feedback``) appends one line on the background disk writer.
        This is independent of fault dumps and of ``state.json`` (UI mirror).
        """
        self.session_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            path = self._recorder.start_recording()
            self._apply_recorder_status_unlocked()
            if self._persist:
                self._publish_unlocked()
        return path

    def log_feedback(
        self,
        raw: bytes,
        *,
        host: Optional[Dict[str, Any]] = None,
        include_raw: bool = True,
    ) -> None:
        """Append one compact feedback record when you ask for it.

        Safe from scripts: ``json.dumps`` of a small dict + enqueue to the disk
        thread — not a full SessionState grade/asdict/state.json rewrite.
        If a manual recording is open, the line goes there; otherwise this is
        a no-op unless you open recording first (keeps "when requested" explicit).
        """
        if not self._recorder.is_recording:
            return
        if host is None:
            with self._lock:
                host = {
                    "ack_lag": self._state.stream_ack_lag,
                    "tx_hz": self._state.stream_tx_hz,
                    "send_ms": self._state.stream_send_ms,
                    "gap_p95_ms": self._state.stream_tx_gap_p95_ms,
                }
        rec = build_fb_log_record(raw, host=host, include_raw=include_raw)
        self._recorder.log_line(rec)
        with self._lock:
            self._apply_recorder_status_unlocked()

    def stop_recording(self) -> None:
        with self._lock:
            self._recorder.stop_recording()
            self._apply_recorder_status_unlocked()
            self._publish_unlocked()
        # Ensure the close reaches disk before callers read the file.
        self._disk.flush()

    def flush(self, timeout_s: float = 2.0) -> bool:
        """Wait for pending state.json / NDJSON / fault I/O (tests, shutdown)."""
        return self._disk.flush(timeout_s=timeout_s)

    def close(self) -> None:
        with self._lock:
            self._recorder.stop_recording()
        if self._owns_recorder_disk:
            self._recorder.close()
        else:
            self._disk.close()

    def _apply_recorder_status_unlocked(self) -> None:
        status = self._recorder.recording_status()
        self._state.recording = status["recording"]
        self._state.recording_path = status["recording_path"]
        self._state.recording_bytes = status["recording_bytes"]
        self._state.recording_seconds = status["recording_seconds"]
        self._state.fault_count = self._recorder.fault_count
        self._state.last_fault_path = str(self._recorder.last_fault_path) if self._recorder.last_fault_path else None
        self._state.capturing_fault = self._recorder.capturing_fault

    def snapshot(self) -> SessionState:
        with self._lock:
            # return a shallow copy via dict round-trip for safety
            d = self._state.to_dict()
        return SessionState(**d)

    def snapshot_dict(self) -> Dict[str, Any]:
        with self._lock:
            return self._state.to_dict()

    def set_connected(self, connected: bool, port: Optional[str] = None, mode: Optional[str] = None) -> None:
        with self._lock:
            self._state.connected = connected
            if port is not None:
                self._state.port = port
            if mode is not None:
                self._state.mode = mode
            if not connected:
                self._state.mode = "idle"
                # Enqueue close only — never flush/disk-wait under this lock
                # (stream + /api/state share it).
                self._recorder.stop_recording()
            self._apply_recorder_status_unlocked()
            _grade_and_context(self._state)
            self._publish_unlocked()

    def note_feedback_rx(self) -> None:
        """Mark that at least one FB frame was drained (keeps age_s fresh)."""
        now = time.time()
        with self._lock:
            self._state.age_s = 0.0
            self._state.updated_at = now

    def note_reader_frames(self, total_frames: int) -> None:
        """Drive ``fb_hz`` from FrameReader.total_frames (true USB FB rate).

        Idle/blank-MCP is ~500–1000 Hz (MCU floods). Counting once per host TX
        tick capped the UI at stream_hz (~40) and hid that. Non-blank MCP on
        current FW collapses raw USB FB to ~2 Hz (CDC FB TX starve).
        """
        now = time.time()
        with self._lock:
            if self._fb_frame_mark is None or self._fb_frame_mark_t is None:
                self._fb_frame_mark = total_frames
                self._fb_frame_mark_t = now
                return
            dt = now - self._fb_frame_mark_t
            # 1 s window so ~2 Hz MCP FB does not flicker to 0 between frames.
            if dt < 1.0:
                return
            df = total_frames - self._fb_frame_mark
            if df >= 0 and dt > 0:
                self._state.fb_hz = df / dt
            self._fb_frame_mark = total_frames
            self._fb_frame_mark_t = now

    def update_from_feedback(
        self,
        *,
        tick: int,
        ack_seq: int,
        mcu_state: int,
        plant_block: int,
        plant_block_name: str,
        pdu_tag: Optional[str],
        lap_ms: Optional[int],
        lap_max_ms: Optional[int],
        periph_lap_ms: Optional[int] = None,
        periph_lap_max_ms: Optional[int] = None,
        ticks_pending: Optional[int],
        svd_present: bool,
        actuators: List[Optional[Dict[str, Any]]],
        mode: str = "plant_stream",
        raw: Optional[bytes] = None,
        pdb_status: Optional[Dict[str, Any]] = None,
    ) -> None:
        now = time.time()
        with self._lock:
            # fb_hz is owned by note_reader_frames() — not by UI publish rate.
            self._state.updated_at = now
            self._state.mode = mode
            self._state.tick = tick
            self._state.ack_seq = ack_seq
            self._state.mcu_state = mcu_state
            self._state.plant_block = plant_block
            self._state.plant_block_name = plant_block_name
            self._state.age_s = 0.0
            self._state.lap_ms = lap_ms
            self._state.lap_max_ms = lap_max_ms
            self._state.periph_lap_ms = periph_lap_ms
            self._state.periph_lap_max_ms = periph_lap_max_ms
            self._state.ticks_pending = ticks_pending
            self._state.svd_present = svd_present
            self._state.pdu_tag = pdu_tag
            self._state.pdb_status = pdb_status
            self._state.actuators = actuators
            _grade_and_context(self._state)
            snap = self._state.to_dict()
            persist = self._persist
            state_path = self.state_path
            host_meta = {
                "ack_lag": self._state.stream_ack_lag,
                "tx_hz": self._state.stream_tx_hz,
                "send_ms": self._state.stream_send_ms,
                "gap_p95_ms": self._state.stream_tx_gap_p95_ms,
            }

        # Fault ring uses a slim snap (no raw). Manual recording uses compact
        # deft_fb_v1 (+ raw_b64) — not another copy of the whole UI state.
        self._recorder.observe(
            {
                "plant_block": plant_block,
                "mcu_state": mcu_state,
                "actuators": actuators,
                "tick": tick,
                "ack_seq": ack_seq,
            }
        )
        if raw is not None and self._recorder.is_recording:
            self._recorder.log_line(
                build_fb_log_record(raw, host=host_meta, include_raw=True)
            )
        with self._lock:
            self._apply_recorder_status_unlocked()
        if persist:
            self._disk.publish_state(state_path, snap)

    def update_stream_timing(
        self,
        *,
        send_ms: float,
        poll_ms: float,
        publish_ms: float,
        loop_ms: float,
        tx_hz: Optional[float] = None,
        tx_gap_p95_ms: Optional[float] = None,
        tx_gap_max_ms: Optional[float] = None,
        ack_lag: Optional[int] = None,
        credit_wait_ms: Optional[float] = None,
    ) -> None:
        """Host stream-loop segment times (ms). Does not touch disk."""
        with self._lock:
            self._state.stream_send_ms = send_ms
            self._state.stream_poll_ms = poll_ms
            self._state.stream_publish_ms = publish_ms
            self._state.stream_loop_ms = loop_ms
            if credit_wait_ms is not None:
                self._state.stream_credit_wait_ms = credit_wait_ms
            if tx_hz is not None:
                self._state.stream_tx_hz = tx_hz
            if tx_gap_p95_ms is not None:
                self._state.stream_tx_gap_p95_ms = tx_gap_p95_ms
            if tx_gap_max_ms is not None:
                self._state.stream_tx_gap_max_ms = tx_gap_max_ms
            if ack_lag is not None:
                self._state.stream_ack_lag = ack_lag
            # Do not re-grade every plant TX tick — /api/state + publish already
            # grade; grading under this lock at 40 Hz contended with the GUI poll.

    def touch_age(self) -> None:
        """Refresh age_s in RAM when no new frame arrived.

        Does **not** write state.json — under MCP, FB is sparse and the stream
        loop would otherwise flush disk every telemetry_hz on stale age, hitching
        plant TX (GUI-only load; bare scripts use persist=False).
        """
        with self._lock:
            if self._state.updated_at:
                self._state.age_s = time.time() - self._state.updated_at

    def set_error(self, message: str) -> None:
        with self._lock:
            self._state.grade = "red"
            self._state.summary = message
            self._state.context = [message]
            self._publish_unlocked()

    def _publish_unlocked(self) -> None:
        """Enqueue a coalesced state.json publish — never touches disk here."""
        if not self._persist:
            return
        self._disk.publish_state(self.state_path, self._state.to_dict())
