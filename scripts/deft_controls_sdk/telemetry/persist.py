"""Background session disk I/O — never block the plant stream thread.

state.json, manual NDJSON recording, and fault dumps all go through this
writer. The control loop may enqueue (and coalesce) freely; a daemon thread
owns every open/write/flush/os.replace.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Union


@dataclass(frozen=True)
class _StateJob:
    path: Path
    payload: Dict[str, Any]


@dataclass(frozen=True)
class _RecordOpen:
    path: Path


@dataclass(frozen=True)
class _RecordLine:
    line: str  # includes trailing newline


@dataclass(frozen=True)
class _RecordClose:
    pass


@dataclass(frozen=True)
class _FaultDump:
    path: Path
    lines: List[Dict[str, Any]]
    max_files: int = 20


@dataclass(frozen=True)
class _FlushToken:
    event: threading.Event


_Job = Union[_StateJob, _RecordOpen, _RecordLine, _RecordClose, _FaultDump, _FlushToken]


class SessionDiskWriter:
    """Single background thread for all `.deft_session` file I/O.

    * ``publish_state`` — coalesced (latest wins) and rate-limited (~state_hz).
    * ``append_record`` / open / close — ordered, never dropped.
    * ``write_fault`` — one shot NDJSON dump.
    * ``flush`` — wait until previously enqueued work is done (tests / stop).
    """

    def __init__(self, *, state_hz: float = 10.0) -> None:
        self._state_hz = max(0.5, float(state_hz))
        self._state_period = 1.0 / self._state_hz
        self._q: queue.SimpleQueue[_Job] = queue.SimpleQueue()
        self._pending_state: Optional[_StateJob] = None
        self._pending_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._record_file = None
        self._record_bytes = 0
        self._dropped_record_lines = 0
        self._thread = threading.Thread(
            target=self._run,
            name="deft-session-disk",
            daemon=True,
        )
        self._thread.start()

    # -- hot-path API (non-blocking) ----------------------------------------------------

    def publish_state(self, path: Path, payload: Dict[str, Any]) -> None:
        """Schedule an atomic state.json replace. Coalesces: only the latest
        unpaid snapshot is kept, so a slow disk cannot backlog 50 Hz rewrites."""
        with self._pending_lock:
            self._pending_state = _StateJob(path=Path(path), payload=payload)
        self._wake.set()

    def open_recording(self, path: Path) -> None:
        self._q.put(_RecordOpen(path=Path(path)))
        self._wake.set()

    def append_record(self, line: str) -> int:
        """Enqueue one NDJSON line (caller supplies trailing newline).
        Returns byte length for optimistic recording_bytes accounting."""
        if not line.endswith("\n"):
            line = line + "\n"
        self._q.put(_RecordLine(line=line))
        self._wake.set()
        return len(line.encode("utf-8"))

    def close_recording(self) -> None:
        self._q.put(_RecordClose())
        self._wake.set()

    def write_fault(self, path: Path, lines: List[Dict[str, Any]], *, max_files: int = 20) -> None:
        self._q.put(_FaultDump(path=Path(path), lines=list(lines), max_files=max_files))
        self._wake.set()

    def flush(self, timeout_s: float = 2.0) -> bool:
        """Block until the writer has finished all work queued before this call
        (including any coalesced state snapshot pending right now)."""
        # Promote pending state onto the ordered queue so flush covers it.
        with self._pending_lock:
            pending = self._pending_state
            self._pending_state = None
        if pending is not None:
            self._q.put(pending)
        done = threading.Event()
        self._q.put(_FlushToken(event=done))
        self._wake.set()
        return done.wait(timeout=timeout_s)

    def close(self, timeout_s: float = 2.0) -> None:
        self.flush(timeout_s=timeout_s)
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=timeout_s)

    @property
    def record_bytes_written(self) -> int:
        """Bytes the writer has actually flushed to the record file."""
        return self._record_bytes

    @property
    def dropped_record_lines(self) -> int:
        return self._dropped_record_lines

    # -- worker ------------------------------------------------------------------------

    def _run(self) -> None:
        last_state_mono = 0.0
        while not self._stop.is_set():
            self._drain_queue()
            last_state_mono = self._maybe_write_coalesced_state(last_state_mono)
            # Sleep until wake, stop, or next state slot — keeps rate limit honest
            # without spinning when idle.
            wait = self._state_period
            with self._pending_lock:
                if self._pending_state is None and self._q.empty():
                    wait = 0.25
                elif self._pending_state is not None:
                    elapsed = time.monotonic() - last_state_mono
                    wait = max(0.0, self._state_period - elapsed)
            self._wake.wait(timeout=wait)
            self._wake.clear()
        self._drain_queue()
        self._maybe_write_coalesced_state(0.0, force=True)
        self._close_record_file()

    def _drain_queue(self) -> None:
        while True:
            try:
                job = self._q.get_nowait()
            except queue.Empty:
                return
            self._handle(job)

    def _maybe_write_coalesced_state(self, last_state_mono: float, *, force: bool = False) -> float:
        now = time.monotonic()
        if not force and (now - last_state_mono) < self._state_period:
            return last_state_mono
        with self._pending_lock:
            job = self._pending_state
            self._pending_state = None
        if job is None:
            return last_state_mono
        self._write_state(job)
        return now

    def _handle(self, job: _Job) -> None:
        if isinstance(job, _StateJob):
            self._write_state(job)
        elif isinstance(job, _RecordOpen):
            self._close_record_file()
            job.path.parent.mkdir(parents=True, exist_ok=True)
            self._record_file = job.path.open("w", encoding="utf-8")
            self._record_bytes = 0
        elif isinstance(job, _RecordLine):
            if self._record_file is None:
                self._dropped_record_lines += 1
                return
            self._record_file.write(job.line)
            self._record_file.flush()
            self._record_bytes += len(job.line.encode("utf-8"))
        elif isinstance(job, _RecordClose):
            self._close_record_file()
        elif isinstance(job, _FaultDump):
            job.path.parent.mkdir(parents=True, exist_ok=True)
            with job.path.open("w", encoding="utf-8") as f:
                for row in job.lines:
                    f.write(json.dumps(row) + "\n")
            self._prune_faults(job.path.parent, job.max_files)
        elif isinstance(job, _FlushToken):
            job.event.set()

    @staticmethod
    def _prune_faults(faults_dir: Path, max_files: int) -> None:
        if max_files <= 0 or not faults_dir.is_dir():
            return
        files = sorted(faults_dir.glob("fault_*.ndjson"), key=lambda p: p.stat().st_mtime)
        while len(files) > max_files:
            files.pop(0).unlink(missing_ok=True)

    @staticmethod
    def _write_state(job: _StateJob) -> None:
        job.path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(job.payload, indent=2)
        tmp = job.path.with_suffix(".json.tmp")
        tmp.write_text(payload, encoding="utf-8")
        os.replace(tmp, job.path)

    def _close_record_file(self) -> None:
        if self._record_file is not None:
            try:
                self._record_file.close()
            except Exception:
                pass
        self._record_file = None
