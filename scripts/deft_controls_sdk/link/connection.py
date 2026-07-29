"""Connection — owns the USB/UART link, held actuator desires, and the optional
background plant-stream/telemetry-publish loop.

No teleop policy or bench plugins (see deft_controls_sdk/debug/ for DEBUG-mode
ops, which borrow this same connection under a lease rather than opening a second
port). Streaming and telemetry-publish (formerly a separate LinkSession) live here
too — both are just behaviors of "the one thing that owns the socket," and DEBUG
ops need to publish through the same TelemetryCache the streaming loop uses, so
splitting them into a second class bought purity without buying an independent
seam anyone used.
"""
from __future__ import annotations

import sys
import threading
import time
from typing import TYPE_CHECKING, Callable, Dict, List, Mapping, Optional

from .exceptions import FeedbackTimeoutError, NotConnectedError
from .exchange import (
    ACTUATOR_COUNT,
    DEFAULT_BAUD,
    IMAGE_BYTES,
    PDU_OFF,
    PLANT_BLOCK_NAMES,
    FrameReader,
    SerialRxPump,
    describe_open_port,
    open_serial,
    parse_feedback_header,
)
from .api_types import ActuatorDesire, CommandImage, FeedbackImage, LedDesire, McuState, ServoDesire, validate_slot

if TYPE_CHECKING:
    from deft_controls_sdk.telemetry import TelemetryCache

HOST_STALE_MS = 500
"""ACTUATOR_HOST_STALE_MS — App/Src/plant/diag/diag_gates.c."""

# Leave this much of each tick for a busy-wait. Plain time.sleep on Windows
# often overshoots; a single overshoot past the deadline becomes a 40–50 ms
# host_tx_gap (and sticky gap_max) even on idle with no MCP.
_SPIN_RESERVE_S = 0.0025
# Cap each coarse sleep so one scheduler overshoot cannot eat a whole period.
_SLEEP_CHUNK_S = 0.002


def _windows_timer_period(enable: bool) -> None:
    """Request 1 ms scheduler resolution while the plant stream runs."""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        if enable:
            ctypes.windll.winmm.timeBeginPeriod(1)
        else:
            ctypes.windll.winmm.timeEndPeriod(1)
    except Exception:
        pass


def _boost_current_thread_priority() -> None:
    """Prefer the plant stream over UI/GC noise on Windows."""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        # THREAD_PRIORITY_ABOVE_NORMAL = 1
        ctypes.windll.kernel32.SetThreadPriority(
            ctypes.windll.kernel32.GetCurrentThread(),
            1,
        )
    except Exception:
        pass


class Connection:
    """Sole serial owner: held desires (pdu=0 plant path) + optional background
    streaming loop that publishes TelemetryCache."""

    def __init__(self, port: str, *, baud: int = DEFAULT_BAUD) -> None:
        self.port = port
        self.baud = baud
        self._ser = None
        self._reader = FrameReader()
        self._pump: Optional[SerialRxPump] = None
        self._seq_lock = threading.Lock()
        self._seq = 0
        self._state_lock = threading.Lock()
        """Guards _desires/_mcu_state — the controller dashboard's HTTP handler
        threads (one per request) can now call set_actuator()/set_mcu_state()
        concurrently with the background streaming thread's build_command(),
        which didn't happen before there was a second writer. Without this,
        set_actuators() mutating _desires mid-iteration in another thread's
        build_command() can raise 'dictionary changed size during iteration'."""
        self._write_lock = threading.Lock()
        """Guards the actual ser.write() — two 672 B frames written from
        different threads without this can interleave on the wire and corrupt
        the frame the MCU sees."""
        self._mcu_state = McuState.NORMAL
        self._rx_sim_mask = 0
        self._desires: Dict[int, ActuatorDesire] = {}
        self._servos: Dict[int, ServoDesire] = {}
        self._led: Optional[LedDesire] = None
        self._next_tick: Optional[float] = None
        self._telemetry: Optional["TelemetryCache"] = None
        self._stream_thread: Optional[threading.Thread] = None
        self._telemetry_thread: Optional[threading.Thread] = None
        self._stream_stop = threading.Event()
        self._stream_hz = 40.0
        # Telemetry runs on a *side* thread — never inside the plant TX loop.
        # Legacy teleop was send→sleep only; coupling TelemetryCache into that
        # path is what made the Windows GUI bringup feel high-latency.
        self._telemetry_hz = 10.0
        self._latest_fb_raw: Optional[bytes] = None
        self._latest_fb_pub_id: Optional[int] = None
        self._last_ack_seq: Optional[int] = None
        self._last_sent_seq: Optional[int] = None
        self._last_plant_send_mono: float = 0.0
        self._unacked: int = 0
        self._timer_period_held = False
        # Hot-path stats for the telemetry thread (replaced as a whole dict).
        self._hot_stats: Dict[str, object] = {}
        self._hot_fb_gen: int = 0  # bumps when _latest_fb_raw is replaced
        # Optional hub hook (e.g. soft_kill_park_if_requested) before each plant TX.
        self._pre_plant_send = None

    @classmethod
    def connect(cls, port: str, *, baud: int = DEFAULT_BAUD) -> "Connection":
        sess = cls(port, baud=baud)
        sess.open()
        return sess

    def open(self) -> None:
        describe_open_port(self.port)
        self._ser = open_serial(self.port, self.baud)
        time.sleep(0.3)
        self._reader.clear()
        self._pump = SerialRxPump(self._ser, self._reader)
        self._pump.__enter__()

    def close(self) -> None:
        self.stop_streaming()
        if self._telemetry is not None:
            self._telemetry.set_connected(False)
        if self._pump is not None:
            self._pump.__exit__()
            self._pump = None
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "Connection":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    @property
    def is_streaming(self) -> bool:
        return self._stream_thread is not None and self._stream_thread.is_alive()

    def _require_serial(self):
        if self._ser is None:
            raise NotConnectedError("Connection is not open — call open()/connect() first")
        return self._ser

    def _next_seq(self) -> int:
        with self._seq_lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return s

    def next_seq(self) -> int:
        """Public accessor — DEBUG-mode bench frames (deft_controls_sdk/debug/)
        need their own sequence numbers on the same wire, same counter as plant."""
        return self._next_seq()

    def exchange_raw(
        self,
        frame: bytes,
        parse_fn: Callable[[bytes], Optional[dict]],
        *,
        timeout_s: float,
        predicate: Optional[Callable[[dict], bool]] = None,
    ) -> Optional[dict]:
        """Write a raw (bench/DEBUG) frame, then poll incoming frames through
        parse_fn until one parses and (if given) satisfies predicate, or timeout.

        Protocol-agnostic on purpose: Connection stays plant-only and never
        imports RS2/DM/CFG parsing — deft_controls_sdk/debug/ supplies
        parse_fn/predicate per bench op. Mirrors the drain-then-poll pattern
        used by every bench probe in legacy (wait_probe_response / _wait_cfg_response).
        """
        self.write_raw(frame, drain=True)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self._reader.pop()
            while raw is not None:
                parsed = parse_fn(raw)
                if parsed is not None and (predicate is None or predicate(parsed)):
                    return parsed
                raw = self._reader.pop()
            time.sleep(0.005)
        return None

    def set_mcu_state(self, state: McuState, *, send: bool = True) -> None:
        state = McuState(state)
        with self._state_lock:
            self._mcu_state = state
            if state in (McuState.RECOVERY, McuState.ESTOP):
                self._desires.clear()
                self._servos.clear()
                self._led = LedDesire(mode=0, master_brightness=0, led_count=0)
        if send and self._ser is not None:
            self.send_once()

    def set_rx_sim(self, enable: bool) -> None:
        """Enable ACTUATOR rx_sim only (bit0). Use set_rx_sim_mask for fine control."""
        self.set_rx_sim_mask(0x1 if enable else 0)

    def set_rx_sim_mask(self, mask: int) -> None:
        """bits0..3: ACTUATOR|SERVO|LED|PDU. Soft-servo (bit1) off for real DXL."""
        with self._state_lock:
            self._rx_sim_mask = int(mask) & 0xF

    @property
    def mcu_state(self) -> McuState:
        return self._mcu_state

    def recover(self) -> None:
        self.set_mcu_state(McuState.RECOVERY)
        time.sleep(0.05)
        self.set_mcu_state(McuState.NORMAL)

    def build_command(self) -> CommandImage:
        with self._state_lock:
            mcu_state = self._mcu_state
            desires = dict(self._desires)  # snapshot — don't hold the lock during CommandImage packing
            servos = dict(self._servos)
            led = self._led
            rx_sim_mask = getattr(self, "_rx_sim_mask", 0)
        img = CommandImage(seq=self._next_seq(), mcu_state=mcu_state)
        img.set_actuators(desires)
        if servos:
            img.set_servos(servos)
        if led is not None:
            img.set_led(led)
        img.set_rx_sim_mask(rx_sim_mask)
        return img

    def write_command(self, image: CommandImage) -> None:
        self.write_raw(image.to_bytes())

    def write_raw(self, frame: bytes, *, drain: bool = True) -> None:
        if len(frame) != IMAGE_BYTES:
            raise ValueError(f"command frame must be {IMAGE_BYTES} bytes, got {len(frame)}")
        ser = self._require_serial()
        with self._write_lock:
            ser.write(frame)
            # Legacy PcbSession.write_command always flush()s after every plant
            # frame. Skipping flush made Windows CDC coalesce/burst OUT traffic
            # so the board saw irregular command arrival (ACT LED blink→freeze).
            # drain=False is only for experiments; plant + bench default on.
            if drain:
                ser.flush()

    @property
    def reader(self) -> FrameReader:
        return self._reader

    def send_once(self, *, drain: bool = True) -> None:
        img = self.build_command()
        self.write_raw(img.to_bytes(), drain=drain)
        self._last_sent_seq = img.seq
        self._last_plant_send_mono = time.monotonic()
        # Do not recompute ack_lag here: under MCP, FB is sparse and a frozen
        # last_ack would make lag climb every TX tick even while the MCU drains
        # at 40 Hz. Lag is only meaningful at FB sample times.

    def _recompute_unacked(self) -> None:
        """Derive in-flight from seq vs ack (8-bit) at last FB sample."""
        if self._last_sent_seq is None or self._last_ack_seq is None:
            return
        lag = (self._last_sent_seq - self._last_ack_seq) & 0xFF
        # Values near 255 mean ack is ahead (wrap / stale) — treat as 0.
        self._unacked = 0 if lag > 128 else lag

    def _ack_in_flight(self) -> Optional[int]:
        """Last observed (host_seq - last_cmd_seq) at a feedback sample."""
        if self._last_sent_seq is None or self._last_ack_seq is None:
            return None
        return self._unacked

    def _note_feedback_raw(self, raw: bytes) -> None:
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("last_cmd_seq") is None:
            return
        self._last_ack_seq = int(hdr["last_cmd_seq"]) & 0xFF
        self._recompute_unacked()

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None:
        validate_slot(slot)
        with self._state_lock:
            self._desires[slot] = desire
        if send:
            self.send_once()

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = True) -> None:
        for slot in desires:
            validate_slot(slot)
        with self._state_lock:
            self._desires.update(desires)
        if send:
            self.send_once()

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = True) -> None:
        if slot not in (0, 1):
            raise ValueError(f"servo slot must be 0..1, got {slot}")
        with self._state_lock:
            self._servos[slot] = desire
        if send:
            self.send_once()

    def set_servos(self, desires: Mapping[int, ServoDesire], *, send: bool = True) -> None:
        for slot in desires:
            if slot not in (0, 1):
                raise ValueError(f"servo slot must be 0..1, got {slot}")
        with self._state_lock:
            self._servos.update(desires)
        if send:
            self.send_once()

    def clear_servos(self, *, send: bool = True) -> None:
        with self._state_lock:
            self._servos.clear()
        if send:
            self.send_once()

    def set_led(self, desire: LedDesire, *, send: bool = True) -> None:
        with self._state_lock:
            self._led = desire
        if send:
            self.send_once()

    def clear_led(self, *, send: bool = True) -> None:
        with self._state_lock:
            self._led = LedDesire(mode=0, master_brightness=0, led_count=0)
        if send:
            self.send_once()

    def held_desire(self, slot: int) -> Optional[ActuatorDesire]:
        with self._state_lock:
            return self._desires.get(slot)

    def held_desires(self) -> Dict[int, ActuatorDesire]:
        """Snapshot all held desires in one lock (dashboard /api/state)."""
        with self._state_lock:
            return dict(self._desires)

    def held_servo(self, slot: int) -> Optional[ServoDesire]:
        with self._state_lock:
            return self._servos.get(slot)

    def held_servos(self) -> Dict[int, ServoDesire]:
        """Snapshot all held DXL servo desires in one lock (mirrors held_desires())."""
        with self._state_lock:
            return dict(self._servos)

    @staticmethod
    def _is_plant_feedback(raw: bytes) -> bool:
        hdr = parse_feedback_header(raw)
        return hdr is not None and not hdr.get("is_debug")

    def _drain_latest_plant_feedback(self) -> Optional[bytes]:
        """Newest HBHF only — DBGF (DEBUG mailbox replies) do not update plant FB."""
        latest: Optional[bytes] = None
        frame = self._reader.pop()
        while frame is not None:
            if self._is_plant_feedback(frame):
                latest = frame
            frame = self._reader.pop()
        return latest

    def poll_feedback(self) -> Optional[FeedbackImage]:
        latest = self._drain_latest_plant_feedback()
        return FeedbackImage(latest) if latest is not None else None

    def read_feedback(self, *, timeout_s: float = 1.0, latest: bool = True) -> FeedbackImage:
        self._require_serial()
        deadline = time.monotonic() + timeout_s
        frame: Optional[bytes] = None
        while time.monotonic() < deadline:
            got = self._reader.pop()
            if got is None:
                time.sleep(0.002)
                continue
            if not self._is_plant_feedback(got):
                continue
            frame = got
            if not latest:
                break
            nxt = self._reader.pop()
            while nxt is not None:
                if self._is_plant_feedback(nxt):
                    frame = nxt
                nxt = self._reader.pop()
        if frame is None:
            raise FeedbackTimeoutError(f"no feedback frame within {timeout_s}s")
        return FeedbackImage(frame)

    def sleep_until_next_tick(self, hz: float) -> None:
        """Absolute phase lock until the next plant tick.

        Linux / Jetson: plain ``time.sleep`` to the deadline is usually enough
        (1 ms timer). Windows: coarse sleep in small chunks + short spin — the
        OS sleep alone overshoots enough to invent 40–50 ms TX gaps. That
        Windows path is a host workaround, not something the Jetson needs.
        """
        period = 1.0 / hz
        now = time.perf_counter()
        if self._next_tick is None:
            self._next_tick = now + period
            return
        target = self._next_tick
        if now - target > period:
            self._next_tick = time.perf_counter() + period
            return
        if sys.platform != "win32":
            remaining = target - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            while time.perf_counter() < target:
                pass
            self._next_tick = target + period
            return
        while True:
            remaining = target - time.perf_counter()
            if remaining <= 0:
                break
            if remaining > _SPIN_RESERVE_S:
                time.sleep(min(remaining - _SPIN_RESERVE_S, _SLEEP_CHUNK_S))
            else:
                while time.perf_counter() < target:
                    pass
                break
        self._next_tick = target + period

    def run_at_hz(
        self,
        hz: float,
        callback: Callable[[Optional[FeedbackImage]], Optional[Mapping[int, ActuatorDesire]]],
        *,
        stop: Optional[Callable[[], bool]] = None,
    ) -> None:
        self._next_tick = None
        while stop is None or not stop():
            fb = self.poll_feedback()
            desires = callback(fb)
            if desires:
                self.set_actuators(desires, send=False)
            self.send_once()
            self.sleep_until_next_tick(hz)

    # -- background streaming + telemetry publish (formerly LinkSession) -----------

    def attach_telemetry(self, telemetry: "TelemetryCache") -> None:
        """Wire a TelemetryCache so start_streaming()/send_once()/DEBUG ops can
        publish state for the dashboard. Optional — Connection works without one."""
        self._telemetry = telemetry
        telemetry.set_connected(True, port=self.port, mode="idle")

    def publish_feedback(self, fb: FeedbackImage, *, mode: str = "plant_stream") -> None:
        if self._telemetry is None:
            return
        raw = fb.raw
        hdr = parse_feedback_header(raw) or {}
        try:
            plant_block = int(fb.plant_block)
        except (TypeError, ValueError):
            plant_block = int(hdr.get("plant_block") or 0)

        actuators: List[Optional[dict]] = []
        for i in range(ACTUATOR_COUNT):
            act = fb.actuator(i)
            if act is None:
                actuators.append(None)
            else:
                actuators.append(
                    {
                        "slot": i,
                        "position": act.position,
                        "velocity": act.velocity,
                        "torque": act.torque,
                        "temperature": act.temperature,
                        "fault": act.fault,
                    }
                )

        pdu = raw[PDU_OFF : PDU_OFF + 32] if len(raw) >= PDU_OFF + 32 else b""
        svd_present = len(pdu) >= 3 and pdu[:3] == b"SVD"

        # Local import: deft_controls_sdk.pdb sits below deft_controls_sdk.link
        # in the dependency graph but connection.py is imported very early
        # (deft_controls_sdk/__init__.py -> controls_pcb_hub -> link), so keep
        # this off the module-level import list to dodge any import-order
        # fragility rather than proving there isn't one.
        from deft_controls_sdk.pdb.status import pdb_status_from_frame

        pdb_status = pdb_status_from_frame(raw)
        self._telemetry.update_from_feedback(
            tick=fb.tick,
            ack_seq=fb.ack_seq,
            mcu_state=fb.mcu_state,
            plant_block=plant_block,
            plant_block_name=hdr.get("plant_block_name") or PLANT_BLOCK_NAMES.get(plant_block, str(plant_block)),
            pdu_tag=hdr.get("pdu_tag"),
            lap_ms=hdr.get("act_lap_ms", hdr.get("lap_ms")),
            lap_max_ms=hdr.get("act_lap_peak_ms", hdr.get("lap_max_ms")),
            periph_lap_ms=hdr.get("periph_lap_ms"),
            periph_lap_max_ms=hdr.get(
                "periph_lap_peak_ms", hdr.get("periph_lap_max_ms")
            ),
            ticks_pending=hdr.get("ticks_pending"),
            svd_present=svd_present,
            actuators=actuators,
            mode=mode,
            raw=raw,
            pdb_status=pdb_status.to_dict() if pdb_status is not None else None,
        )

    def set_pre_plant_send(self, fn) -> None:
        """Register a no-arg callback invoked on the plant stream before each TX."""
        self._pre_plant_send = fn

    def start_streaming(self, hz: float = 40.0, *, telemetry_hz: float = 10.0) -> None:
        """Background plant stream — resends held desires, polls feedback.

        ``hz`` is the plant TX / HOST_STALE cadence (legacy teleop default 40).
        Plant loop is send→sleep only. ``telemetry_hz`` is a *separate* thread
        that samples latest FB + hot stats into TelemetryCache / UI — it must
        not run on the plant thread (that coupling caused Windows GUI lag).

        Does **not** auto-recover: RECOVERY mounts plant_recovery_all on the MCU
        (non-blocking MCP enqueue+flush; plant tick drains remainder). Use hub.recover()
        explicitly when needed.
        """
        if self._stream_thread is not None and self._stream_thread.is_alive():
            return
        self._stream_hz = hz
        self._telemetry_hz = max(1.0, float(telemetry_hz))
        self._latest_fb_raw = None
        self._latest_fb_pub_id = None
        self._hot_fb_gen = 0
        self._hot_stats = {}
        self._last_ack_seq = None
        self._last_sent_seq = None
        self._unacked = 0
        self._last_plant_send_mono = 0.0
        self._stream_stop.clear()
        # Clear HOST_STALE without RECOVERY — a few normal plant frames.
        for _ in range(3):
            try:
                self.send_once()
            except Exception:
                break
            time.sleep(0.02)
        if not self._timer_period_held:
            _windows_timer_period(True)
            self._timer_period_held = True
        self._stream_thread = threading.Thread(
            target=self._stream_loop, name="deft-connection-stream", daemon=True
        )
        self._stream_thread.start()
        if self._telemetry is not None:
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_loop, name="deft-connection-telemetry", daemon=True
            )
            self._telemetry_thread.start()

    def stop_streaming(self) -> None:
        self._stream_stop.set()
        t = self._stream_thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._stream_thread = None
        tt = self._telemetry_thread
        if tt is not None and tt.is_alive():
            tt.join(timeout=2.0)
        self._telemetry_thread = None
        if self._timer_period_held:
            _windows_timer_period(False)
            self._timer_period_held = False

    def _stream_loop(self) -> None:
        """Plant TX hot path — intentionally no TelemetryCache calls.

        drain newest FB → send_once → sleep. Same shape as legacy teleop.
        Stats are published by ``_telemetry_loop`` on another thread.
        """
        self._next_tick = None
        _boost_current_thread_priority()
        tx_times: list[float] = []
        tx_gap_events: list[tuple[float, float]] = []
        last_send_at: Optional[float] = None
        while not self._stream_stop.is_set():
            t_loop = time.perf_counter()
            send_ms = poll_ms = 0.0
            try:
                t0 = time.perf_counter()
                latest = self._drain_latest_plant_feedback()
                if latest is not None:
                    self._note_feedback_raw(latest)
                    self._latest_fb_raw = latest
                    self._hot_fb_gen += 1
                poll_ms = (time.perf_counter() - t0) * 1000.0

                hook = self._pre_plant_send
                if hook is not None:
                    try:
                        hook()
                    except Exception:
                        pass

                t0 = time.perf_counter()
                self.send_once(drain=True)
                send_ms = (time.perf_counter() - t0) * 1000.0
                now_send = time.perf_counter()
                if last_send_at is not None:
                    tx_gap_events.append((now_send, (now_send - last_send_at) * 1000.0))
                last_send_at = now_send
                tx_times.append(now_send)
                if len(tx_times) > 30:
                    tx_times.pop(0)
            except Exception as exc:
                tel = self._telemetry
                if tel is not None:
                    tel.set_error(f"stream error: {exc}")

            loop_ms = (time.perf_counter() - t_loop) * 1000.0
            tx_hz: Optional[float] = None
            if len(tx_times) >= 2:
                span = tx_times[-1] - tx_times[0]
                if span > 0:
                    tx_hz = (len(tx_times) - 1) / span
            gap_p95 = gap_max = None
            if tx_gap_events:
                cutoff = time.perf_counter() - 0.35
                tx_gap_events = [e for e in tx_gap_events if e[0] >= cutoff]
                recent = [g for _, g in tx_gap_events]
                if recent:
                    ordered = sorted(recent)
                    gap_max = ordered[-1]
                    gap_p95 = ordered[int(0.95 * (len(ordered) - 1))]
            # Whole-dict replace — telemetry thread reads without blocking plant.
            self._hot_stats = {
                "send_ms": send_ms,
                "poll_ms": poll_ms,
                "loop_ms": loop_ms,
                "tx_hz": tx_hz,
                "gap_p95": gap_p95,
                "gap_max": gap_max,
                "ack_lag": self._ack_in_flight(),
                "fb_frames": self._reader.total_frames,
                "fb_gen": self._hot_fb_gen,
            }
            self.sleep_until_next_tick(self._stream_hz)

    def _telemetry_loop(self) -> None:
        """Side thread: sample hot stats + latest FB into TelemetryCache.

        Never called from the plant TX thread. Safe for Jetson/Linux and for a
        Windows GUI — disk/json/HTTP cannot stretch plant gaps.
        """
        tel = self._telemetry
        if tel is None:
            return
        period = 1.0 / self._telemetry_hz
        last_fb_gen = -1
        next_due = time.perf_counter()
        while not self._stream_stop.is_set():
            now = time.perf_counter()
            if now < next_due:
                time.sleep(min(0.005, next_due - now))
                continue
            next_due = now + period
            t0 = time.perf_counter()
            try:
                stats = self._hot_stats
                fb_frames = int(stats.get("fb_frames") or self._reader.total_frames)
                tel.note_reader_frames(fb_frames)
                fb_gen = int(stats.get("fb_gen") or 0)
                raw = self._latest_fb_raw
                if raw is not None and fb_gen != last_fb_gen:
                    tel.note_feedback_rx()
                    self.publish_feedback(FeedbackImage(raw), mode="plant_stream")
                    last_fb_gen = fb_gen
                    self._latest_fb_pub_id = fb_gen
                else:
                    tel.touch_age()
                publish_ms = (time.perf_counter() - t0) * 1000.0
                tel.update_stream_timing(
                    send_ms=float(stats.get("send_ms") or 0.0),
                    poll_ms=float(stats.get("poll_ms") or 0.0),
                    publish_ms=publish_ms,
                    loop_ms=float(stats.get("loop_ms") or 0.0),
                    tx_hz=stats.get("tx_hz") if isinstance(stats.get("tx_hz"), (int, float)) else None,
                    tx_gap_p95_ms=stats.get("gap_p95") if isinstance(stats.get("gap_p95"), (int, float)) else None,
                    tx_gap_max_ms=stats.get("gap_max") if isinstance(stats.get("gap_max"), (int, float)) else None,
                    ack_lag=stats.get("ack_lag") if isinstance(stats.get("ack_lag"), int) else None,
                    credit_wait_ms=0.0,
                )
            except Exception as exc:
                tel.set_error(f"telemetry error: {exc}")
