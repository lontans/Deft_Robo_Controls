"""FaultRecorder — bounded ring buffer + fault-triggered incident dumps, plus
an optional manual continuous recording. Both write NDJSON (one line per
snapshot) under the session dir.

Disk I/O is never done on the caller thread: open/append/close/fault-dump go
through SessionDiskWriter so the plant stream loop cannot stall on flush or
antivirus/OneDrive rename latency. See persist.py.
"""
from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

from .persist import SessionDiskWriter

_MCU_STATE_ESTOP = 3


@dataclass
class _PendingFault:
    lines: List[Dict[str, Any]]
    remaining_ticks: int
    reason: str
    path: Path


class FaultRecorder:
    def __init__(
        self,
        session_dir: Path,
        *,
        ring_size: int = 250,
        post_fault_ticks: int = 100,
        max_fault_files: int = 20,
        startup_grace_s: float = 3.0,
        disk: Optional[SessionDiskWriter] = None,
    ) -> None:
        self._session_dir = Path(session_dir)
        self._ring: Deque[Dict[str, Any]] = deque(maxlen=ring_size)
        self._post_fault_ticks = post_fault_ticks
        self._max_fault_files = max_fault_files
        self._startup_grace_s = startup_grace_s
        self._first_observed_monotonic: Optional[float] = None

        self._pending: Optional[_PendingFault] = None
        self._last_plant_block: Optional[int] = None
        self._last_mcu_state: Optional[int] = None
        self._last_fault_bits = 0
        self.fault_count = 0
        self.last_fault_path: Optional[Path] = None

        self._disk = disk if disk is not None else SessionDiskWriter()
        self._owns_disk = disk is None
        self._recording = False
        self._record_path: Optional[Path] = None
        self._record_started_at: Optional[float] = None
        self._record_bytes = 0  # optimistic (enqueued) byte count for UI

    # -- manual recording -------------------------------------------------------------

    @property
    def is_recording(self) -> bool:
        return self._recording

    def start_recording(self) -> Path:
        """Open an NDJSON file for compact feedback lines (``log_line``).

        Callers should feed ``deft_fb_v1`` records via ``log_line`` /
        ``TelemetryCache.log_feedback`` — not full ``SessionState`` dumps.
        """
        if self._recording:
            assert self._record_path is not None
            return self._record_path  # already recording — idempotent, not an error
        recordings_dir = self._session_dir / "recordings"
        recordings_dir.mkdir(parents=True, exist_ok=True)
        path = recordings_dir / f"record_{time.strftime('%Y%m%dT%H%M%S')}.ndjson"
        self._disk.open_recording(path)
        self._recording = True
        self._record_path = path
        self._record_started_at = time.monotonic()
        self._record_bytes = 0
        return path

    def stop_recording(self) -> None:
        if not self._recording:
            return
        self._disk.close_recording()
        self._recording = False
        self._record_path = None
        self._record_started_at = None
        self._record_bytes = 0

    def recording_status(self) -> Dict[str, Any]:
        return {
            "recording": self.is_recording,
            "recording_path": str(self._record_path) if self._record_path else None,
            "recording_bytes": self._record_bytes,
            "recording_seconds": (
                time.monotonic() - self._record_started_at if self._record_started_at is not None else None
            ),
        }

    def flush(self, timeout_s: float = 2.0) -> bool:
        """Wait for background disk work (tests / orderly shutdown)."""
        return self._disk.flush(timeout_s=timeout_s)

    def clear(self) -> None:
        """Reset session fault bookkeeping — ring buffer, count, last-path.

        Does not touch already-written ``faults/fault_*.ndjson`` dump files on
        disk (those are historical incident evidence); this only resets the
        in-memory "faults this session" counter and pre-fault ring so a long
        bench session can start counting fresh without a board reconnect.
        Any in-progress post-fault capture is discarded rather than flushed.
        If the underlying condition (e.g. still ESTOP) has not cleared, the
        next observed tick will treat it as a new fault — clear does not
        suppress detection, only the bookkeeping.
        """
        self._ring.clear()
        self._pending = None
        self._last_plant_block = None
        self._last_mcu_state = None
        self._last_fault_bits = 0
        self.fault_count = 0
        self.last_fault_path = None

    def close(self) -> None:
        self.stop_recording()
        if self._owns_disk:
            self._disk.close()

    # -- fault-triggered ring buffer ---------------------------------------------------

    def log_line(self, record: Dict[str, Any]) -> None:
        """Append one NDJSON object to the manual recording (if open).

        Used for compact ``deft_fb_v1`` feedback lines — not the fat UI
        ``SessionState``. No-op when not recording.
        """
        if not self._recording:
            return
        line = json.dumps(record, separators=(",", ":"))
        self._record_bytes += self._disk.append_record(line)

    def observe(self, state_dict: Dict[str, Any]) -> None:
        """Feed one snapshot into the fault ring / incident detector only.

        Does **not** write the manual recording file — that is ``log_line`` /
        ``TelemetryCache.log_feedback`` so accurate logs stay compact wire
        captures instead of re-dumping the whole UI SessionState.
        """
        if self._first_observed_monotonic is None:
            self._first_observed_monotonic = time.monotonic()

        self._ring.append(state_dict)

        if self._pending is not None:
            # Already capturing a prior incident's post-fault window — this
            # tick belongs to that window. Checked *before* _check_fault_transition
            # so the tick that starts a new pending isn't also counted against it.
            self._pending.lines.append(state_dict)
            self._pending.remaining_ticks -= 1
            if self._pending.remaining_ticks <= 0:
                self._enqueue_pending_fault()

        self._check_fault_transition(state_dict)

    @property
    def capturing_fault(self) -> bool:
        return self._pending is not None

    def _in_startup_grace(self) -> bool:
        if self._first_observed_monotonic is None:
            return True
        return (time.monotonic() - self._first_observed_monotonic) < self._startup_grace_s

    def _check_fault_transition(self, state_dict: Dict[str, Any]) -> None:
        plant_block = state_dict.get("plant_block") or 0
        mcu_state = state_dict.get("mcu_state") or 0
        fault_bits = 0
        for act in state_dict.get("actuators") or []:
            if act is not None:
                fault_bits |= int(act.get("fault") or 0)

        # HOST_STALE/QUIET_PERIOD readings (and fb_hz ramping from cold) are
        # normal for the first ~1s after connect/start_streaming — see the
        # bench log that motivated this: tick 0 read plant_block=HOST_STALE
        # with fb_hz=None (the very first sample), cleared within 264 ms, and
        # was steady by +1.25s. Without a grace window, *every* connect would
        # trip a "fault" on its first observed tick — that's cold-start noise,
        # not a real transition, so it's suppressed here (baseline tracking
        # below still runs so detection is accurate once grace ends).
        in_grace = self._in_startup_grace()

        newly_faulted = (
            not in_grace
            and (
                (plant_block != 0 and (self._last_plant_block or 0) == 0)
                or (fault_bits != 0 and self._last_fault_bits == 0)
                or (mcu_state == _MCU_STATE_ESTOP and self._last_mcu_state != _MCU_STATE_ESTOP)
            )
        )

        self._last_plant_block = plant_block
        self._last_mcu_state = mcu_state
        self._last_fault_bits = fault_bits

        if newly_faulted and self._pending is None:
            if plant_block:
                reason = f"plant_block_{plant_block}"
            elif fault_bits:
                reason = f"actuator_fault_{fault_bits:x}"
            else:
                reason = "estop"
            faults_dir = self._session_dir / "faults"
            stamp = time.strftime("%Y%m%dT%H%M%S")
            path = faults_dir / f"fault_{stamp}_{self.fault_count:04d}_{reason}.ndjson"
            # Ring already holds this tick (appended at the top of observe()) —
            # everything in it right now is the pre-fault context.
            self._pending = _PendingFault(
                lines=list(self._ring),
                remaining_ticks=self._post_fault_ticks,
                reason=reason,
                path=path,
            )

    def _enqueue_pending_fault(self) -> None:
        pending = self._pending
        self._pending = None
        if pending is None:
            return
        self._disk.write_fault(pending.path, pending.lines, max_files=self._max_fault_files)
        self.fault_count += 1
        self.last_fault_path = pending.path
