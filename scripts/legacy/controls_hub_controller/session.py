"""PlantSession — normal-operation motor control over USB CDC / UART.

Thin, honest mapping to the 562 B host_exchange_schema.h contract: open a session,
set mcu_state=NORMAL, write actuator_commands[]. No ramping, homing, arrow-key, or
"backdrivable idle" logic lives here — see docs/controls_hub-controller.md and the
non-goals section of the design doc. Application-level trajectory/safety policy
belongs above this layer, not inside it.

Hold-last-command semantics (App/Src/plant/actuator.c, control_loop.c):
  - The MCU applies actuator_desire_live[] at 500 Hz (TIM6) and re-applies it on
    every tick until a new command image is mounted.
  - BUT actuator_command_mount() copies all ACTUATOR_COUNT slots from every fresh
    command image unconditionally (no per-slot change detection) — any slot you do
    not include in a given frame is mounted as an all-zero desire. PlantSession
    resends every currently-held desire on every send_once()/run_at_hz() tick so
    callers don't have to reason about this; if you build CommandImage yourself,
    you must include every slot you care about on every frame.

Stale watchdog (diag_gates.c, ACTUATOR_HOST_STALE_MS = 500):
  - If no valid command image has been dispatched in the last 500 ms AND no enabled
    slot's live desire is non-idle (kp/kd/|velocity|/|torque| > 0.01), plant_block
    becomes HOST_STALE and actuator_apply_desire() stops driving CAN. Streaming
    callers must send at a rate that keeps them under 500 ms between frames.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Dict, Mapping, Optional

from controls_pcb_host.protocol import DEFAULT_BAUD, ACTUATOR_COUNT, IMAGE_BYTES, SESSION_BEGIN, SESSION_END
from controls_pcb_host.protocol.can_bus import is_rs02_plant_bus
from controls_pcb_host.transport import FrameReader, SerialRxPump, describe_open_port, open_serial

from .config import slot_config

from .command import ActuatorDesire, CommandImage, McuState, validate_slot
from .exceptions import FeedbackTimeoutError, NotConnectedError
from .feedback import FeedbackImage

HOST_STALE_MS = 500
"""ACTUATOR_HOST_STALE_MS — App/Src/plant/diag/diag_gates.c."""


class PlantSession:
    def __init__(self, port: str, *, baud: int = DEFAULT_BAUD) -> None:
        self.port = port
        self.baud = baud
        self._ser = None
        self._reader = FrameReader()
        self._pump: Optional[SerialRxPump] = None
        self._seq_lock = threading.Lock()
        self._seq = 0
        self._mcu_state = McuState.NORMAL
        self._desires: Dict[int, ActuatorDesire] = {}
        self._next_tick: Optional[float] = None

    @classmethod
    def connect(cls, port: str, *, baud: int = DEFAULT_BAUD) -> "PlantSession":
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
        if self._pump is not None:
            self._pump.__exit__()
            self._pump = None
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "PlantSession":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        """True once open()/connect() has succeeded and close() has not been called."""
        return self._ser is not None

    def _require_serial(self):
        if self._ser is None:
            raise NotConnectedError("PlantSession is not connected — call open()/connect() first")
        return self._ser

    def _next_seq(self) -> int:
        with self._seq_lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return s

    # -- mcu_state -----------------------------------------------------------------

    def set_mcu_state(self, state: McuState, *, send: bool = True) -> None:
        """Set system.mcu_state. Sends immediately by default (send=True) — a state
        change is meant to take effect now, not wait for the next unrelated
        send_once()/run_at_hz() tick. (Previously this only staged the state locally
        and callers had to remember an explicit send_once(); see
        docs/controls_hub-controller.md Implementation review.) Pass send=False to
        stage the state and let a caller-composed frame carry it instead."""
        state = McuState(state)
        self._mcu_state = state
        if state in (McuState.RECOVERY, McuState.ESTOP):
            # plant_recovery_all() clears live desires on the MCU when RECOVERY/ESTOP
            # is mounted; drop the host-held mirror too so a stale desire doesn't
            # reappear the instant NORMAL resumes.
            self._desires.clear()
        if send and self._ser is not None:
            self.send_once()

    @property
    def mcu_state(self) -> McuState:
        return self._mcu_state

    def recover(self) -> None:
        """RECOVERY then NORMAL — matches plant_recovery_all() intent (reset/disable
        actuators, clear desires). Blocks ~50 ms so the MCU has a tick to process
        RECOVERY before NORMAL resumes. Both transitions send immediately
        (set_mcu_state's default send=True)."""
        self.set_mcu_state(McuState.RECOVERY)
        time.sleep(0.05)
        self.set_mcu_state(McuState.NORMAL)

    def arm_actuator_slots(self, slots: Optional[list[int]] = None) -> None:
        """RS2 enable-only per slot (brief pdu backdoor), then return to NORMAL streaming.

        Operational path uses actuator_commands[] with pdu=0. After recover() or a
        prior teleop exit, drives are often disabled — same as manual
        ``control_hub probe --bus N``. Does not change held desires or mcu_state.
        """
        from controls_pcb_host.plugins import robstride as rs02

        self._require_serial()
        if slots is None:
            slots = list(range(ACTUATOR_COUNT))
        for slot in slots:
            cfg = slot_config(slot)
            if cfg.protocol_name != "robstride" or not cfg.enabled:
                continue
            if not is_rs02_plant_bus(cfg.bus):
                continue
            rs02.enable_for_plant(self, cfg.bus, cfg.motor_id)
        # Clear any bench-session gate; next send_once() is pure plant path.
        self.send_once()

    # -- command building / sending --------------------------------------------------

    def build_command(self) -> CommandImage:
        """Build the next command image from currently-held desires + mcu_state."""
        img = CommandImage(seq=self._next_seq(), mcu_state=self._mcu_state)
        img.set_actuators(self._desires)
        return img

    def write_command(self, image: CommandImage) -> None:
        """Send a raw, caller-built 562 B command image as-is (bypasses the held
        desire dict — use send_once()/set_actuator() for the normal flow)."""
        self.write_raw(image.to_bytes())

    def write_raw(self, frame: bytes) -> None:
        """Send a pre-built 562 B frame (bench pdu probes use this path)."""
        if len(frame) != IMAGE_BYTES:
            raise ValueError(f"command frame must be {IMAGE_BYTES} bytes, got {len(frame)}")
        ser = self._require_serial()
        ser.write(frame)
        ser.flush()

    @property
    def reader(self) -> FrameReader:
        return self._reader

    def rs2_session_begin(self, bus: int = 1) -> None:
        from controls_pcb_host.plugins import robstride as rs02

        rs02.send_diag(self, 0, SESSION_BEGIN, timeout_s=2.0, bus=bus)

    def rs2_session_end(self, bus: int = 1) -> None:
        from controls_pcb_host.plugins import robstride as rs02

        rs02.send_diag(self, 0, SESSION_END, timeout_s=2.0, bus=bus)

    def send_once(self) -> None:
        """Send exactly one command image built from current desires/mcu_state."""
        self.write_command(self.build_command())

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None:
        validate_slot(slot)
        self._desires[slot] = desire
        if send:
            self.send_once()

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = True) -> None:
        for slot in desires:
            validate_slot(slot)
        self._desires.update(desires)
        if send:
            self.send_once()

    def held_desire(self, slot: int) -> Optional[ActuatorDesire]:
        return self._desires.get(slot)

    # -- feedback ---------------------------------------------------------------------

    def poll_feedback(self) -> Optional[FeedbackImage]:
        """Non-blocking: latest buffered feedback frame, or None if none arrived
        since the last poll/read."""
        latest: Optional[bytes] = None
        frame = self._reader.pop()
        while frame is not None:
            latest = frame
            frame = self._reader.pop()
        return FeedbackImage(latest) if latest is not None else None

    def read_feedback(self, *, timeout_s: float = 1.0, latest: bool = True) -> FeedbackImage:
        """Blocking: wait up to timeout_s for a feedback frame. latest=True (default)
        drains the buffer and returns the newest frame; latest=False returns the
        oldest buffered frame (FIFO)."""
        self._require_serial()
        deadline = time.monotonic() + timeout_s
        frame: Optional[bytes] = None
        while time.monotonic() < deadline:
            got = self._reader.pop()
            if got is None:
                time.sleep(0.002)
                continue
            frame = got
            if not latest:
                break
            nxt = self._reader.pop()
            while nxt is not None:
                frame = nxt
                nxt = self._reader.pop()
        if frame is None:
            raise FeedbackTimeoutError(f"no feedback frame within {timeout_s}s")
        return FeedbackImage(frame)

    # -- streaming ----------------------------------------------------------------

    def sleep_until_next_tick(self, hz: float) -> None:
        """App-owns-the-loop pacing helper (usage example B). Call once per
        iteration; sleeps just enough to hold a steady `hz` cadence. No smoothing —
        if the caller falls more than one period behind, the schedule resets rather
        than bursting to catch up."""
        period = 1.0 / hz
        now = time.perf_counter()
        if self._next_tick is None or self._next_tick <= now - period:
            self._next_tick = now + period
            return
        sleep_s = self._next_tick - now
        if sleep_s > 0:
            time.sleep(sleep_s)
        self._next_tick += period

    def run_at_hz(
        self,
        hz: float,
        callback: Callable[[Optional[FeedbackImage]], Optional[Mapping[int, ActuatorDesire]]],
        *,
        stop: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Streaming loop helper. Each tick: poll the latest feedback (or None),
        pass it to callback, apply whatever desire mapping callback returns (or hold
        current desires if it returns None/empty), then send_once(). callback owns
        all trajectory logic — this loop applies zero smoothing or ramping."""
        self._next_tick = None
        while stop is None or not stop():
            fb = self.poll_feedback()
            desires = callback(fb)
            if desires:
                self.set_actuators(desires, send=False)
            self.send_once()
            self.sleep_until_next_tick(hz)
