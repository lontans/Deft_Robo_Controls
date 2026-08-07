"""ControlsPcbHub — the one import surface for software.

    from deft_controls_sdk import ControlsPcbHub

    with ControlsPcbHub.connect() as hub:  # or connect("COM5") / serial=
        hub.start_streaming()
        print(hub.telemetry.snapshot())

Owns COM via link.Connection. Publishes TelemetryCache for scripts and
debug_dashboard. Never imports debug_dashboard.

hub.debug.* is DEBUG mode — see docs/host-contract.md. Same Connection; plant apply may
be gated (plant_block=BENCH_SESSION) while a lease is held.
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Mapping, Optional, Union

from deft_controls_sdk.debug import DebugAPI, find_cdc_port
from deft_controls_sdk.link import ActuatorDesire, Connection, FeedbackImage, LedDesire, McuState, ServoDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD
from deft_controls_sdk.pdb import KILL_SOFT_REQ, PdbStatus, pdb_status_from_frame
from deft_controls_sdk.pdb import limits as pdb_limits
from deft_controls_sdk.telemetry import TelemetryCache, default_session_dir


class ControlsPcbHub:
    def __init__(self, connection: Connection, telemetry: TelemetryCache) -> None:
        # When False: ignore PDB kill for soft-kill hooks + doctor LED inference.
        # USB FB may still show HARD+COMMS_LOSS (MCU stale failsafe); host policy
        # treats PDB as not listening. Product with a live PDU should set True.
        self._listen_pdu = False
        self._connection = connection
        self._telemetry = telemetry
        self._debug = DebugAPI(connection, telemetry)
        # Dashboard soft_kill state machine (debug_dashboard/state.py) — see
        # soft_kill_freeze(). Purely host-side bookkeeping; the wire only ever
        # sees NORMAL + plant_apply=1 while frozen (see soft_kill_freeze docstring).
        self._soft_kill: bool = False

    @classmethod
    def connect(
        cls,
        port: Optional[str] = None,
        *,
        serial: Optional[str] = None,
        baud: int = DEFAULT_BAUD,
        session_dir: Optional[Union[str, os.PathLike[str]]] = None,
        persist_telemetry: bool = False,
        telemetry: Optional[TelemetryCache] = None,
        mode: str = "bandwidth",
    ) -> "ControlsPcbHub":
        """Connect and attach telemetry.

        ``mode``: ``bandwidth`` (stm32_mode=0, no debug-lanes / no hub.debug RPC)
        or ``debug`` (stm32_mode=1). Change mode only by disconnect + reconnect.
        ``port`` omitted → auto-pick STM32 USB CDC.
        """
        if not port:
            port = find_cdc_port(serial=serial)
        elif serial is not None:
            # Explicit port wins; serial only used for auto-pick.
            pass
        connection = Connection.connect(port, baud=baud, mode=mode)
        if telemetry is None:
            telemetry = TelemetryCache(session_dir=session_dir or default_session_dir(), persist=persist_telemetry)
        connection.attach_telemetry(telemetry)
        return cls(connection, telemetry)

    @property
    def stm32_mode(self) -> int:
        return self._connection.stm32_mode

    @property
    def link_mode(self) -> str:
        return self._connection.link_mode_name

    def close(self) -> None:
        self._connection.close()

    def __enter__(self) -> "ControlsPcbHub":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def telemetry(self) -> TelemetryCache:
        return self._telemetry

    @property
    def debug(self) -> DebugAPI:
        """DEBUG mode: discover / calibrate / config, under a bench lease.
        See deft_controls_sdk/debug/README or DebugAPI's docstring."""
        return self._debug

    @property
    def port(self) -> str:
        return self._connection.port

    @property
    def state_path(self) -> Path:
        return self.telemetry.state_path

    @property
    def listen_pdu(self) -> bool:
        """If False, host ignores PDB kill for soft-kill + status LED policy."""
        return self._listen_pdu

    @listen_pdu.setter
    def listen_pdu(self, enabled: bool) -> None:
        self._listen_pdu = bool(enabled)

    @property
    def stream_hz(self) -> Optional[float]:
        hz = getattr(self._connection, "_stream_hz", None)
        return float(hz) if hz is not None else None

    @property
    def telemetry_hz(self) -> Optional[float]:
        hz = getattr(self._connection, "_telemetry_hz", None)
        return float(hz) if hz is not None else None

    def set_auto_soft_kill(self, enabled: bool) -> None:
        """Enable/disable plant-TX soft-kill freeze hooks (PDU SOFT_KILL_REQ + V/I).

        Product teleop leaves this on. The debug dashboard's Follow/idle path
        turns it off so Connect does not latch a frozen hold from a residual
        PDU soft-kill or host V/I check before the operator enters Active.
        No-op when ``listen_pdu`` is False (bench without a PDU peer).

        The auto-trip path (real overcurrent/undervoltage, or an upstream PDU
        asserting a kill request) now hold-at-torque freezes
        (:meth:`soft_kill_freeze`) rather than hard-ESTOP-parking
        (:meth:`soft_kill_park`) — see debug_dashboard/state.py's soft_kill
        state machine. ``soft_kill_park`` remains available separately for an
        explicit hard-ESTOP (Advanced panel button).
        """
        if enabled and self._listen_pdu:

            def _pre_plant_send() -> None:
                self.soft_kill_park_if_requested(send=False)
                self.soft_kill_park_if_bad_vi(send=False)

            self._connection.set_pre_plant_send(_pre_plant_send)
        else:
            self._connection.set_pre_plant_send(None)

    def start_streaming(
        self,
        hz: float = 200.0,
        *,
        telemetry_hz: float = 200.0,
        auto_soft_kill: Optional[bool] = None,
    ) -> None:
        """Background plant stream — keeps HOST_STALE clear and feeds telemetry.

        Plant TX runs at ``hz`` on its own thread (send→sleep, legacy-shaped).
        TelemetryCache / UI publish runs on a *side* thread at ``telemetry_hz``
        so dashboard disk/json cannot stretch plant TX gaps.

        When ``auto_soft_kill`` is True: auto-parks via
        :meth:`soft_kill_park_if_requested` when USB kill is ``SOFT_KILL_REQ``,
        and via :meth:`soft_kill_park_if_bad_vi` against ``pdb.limits``.
        Default follows ``listen_pdu``. Pass ``auto_soft_kill=False`` for
        read-only observe streams (dashboard).
        """
        if auto_soft_kill is None:
            auto_soft_kill = self._listen_pdu
        self.set_auto_soft_kill(bool(auto_soft_kill))
        self._connection.start_streaming(hz=hz, telemetry_hz=telemetry_hz)

    def log_feedback(self, raw: Optional[bytes] = None, *, include_raw: bool = True) -> None:
        """Append one compact feedback record to an open recording (opt-in).

        If ``raw`` is omitted, uses the latest feedback image held by the
        connection (from the plant stream). No-op unless
        ``telemetry.start_recording()`` was called.
        """
        if raw is None:
            raw = self._connection._latest_fb_raw
        if raw is None:
            return
        self._telemetry.log_feedback(raw, include_raw=include_raw)

    def stop_streaming(self) -> None:
        self._connection.stop_streaming()

    def recover(self) -> None:
        self._connection.recover()

    def set_rx_sim(self, enable: bool) -> None:
        """Bench: ACTUATOR rx_sim only (synthetic RS02 FB into CAN rings)."""
        self._connection.set_rx_sim(enable)

    def set_rx_sim_mask(self, mask: int) -> None:
        """bits0..3: ACTUATOR|SERVO|LED|PDU."""
        self._connection.set_rx_sim_mask(mask)

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None:
        self._connection.set_actuator(slot, desire, send=send)

    def set_actuators(
        self, desires: Mapping[int, ActuatorDesire], *, send: bool = True
    ) -> None:
        """Batch plant desires (PlantSink / ActuatorAction)."""
        self._connection.set_actuators(desires, send=send)

    def latest_feedback(self) -> Optional[FeedbackImage]:
        """Latest plant FB image (PlantSink / ActuatorAction)."""
        fb = self._connection.poll_feedback()
        if fb is not None:
            return fb
        raw = self._connection._latest_fb_raw  # noqa: SLF001
        return FeedbackImage(raw) if raw is not None else None

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = True) -> None:
        self._connection.set_servo(slot, desire, send=send)

    def clear_servos(self, *, send: bool = True) -> None:
        self._connection.clear_servos(send=send)

    def set_led(self, desire: LedDesire, *, send: bool = True) -> None:
        self._connection.set_led(desire, send=send)

    def held_desire(self, slot: int) -> Optional[ActuatorDesire]:
        """Currently-held desire for one slot — what the background stream is
        actually resending, not just what the last set_actuator() call asked
        for (a controller UI needs this to show *commanded* state distinctly
        from measured feedback and from an unapplied input box)."""
        return self._connection.held_desire(slot)

    def held_desires(self) -> dict:
        return self._connection.held_desires()

    def held_servo(self, slot: int) -> Optional[ServoDesire]:
        return self._connection.held_servo(slot)

    def held_servos(self) -> dict:
        return self._connection.held_servos()

    @property
    def is_streaming(self) -> bool:
        return self._connection.is_streaming

    def send_once(self) -> None:
        self._connection.send_once()
        fb = self._connection.poll_feedback()
        if fb is not None:
            self._connection.publish_feedback(fb)

    def refresh_feedback(
        self,
        *,
        slots: Optional[list[int]] = None,
        seconds: float = 0.5,
        hz: float = 200.0,
    ) -> Optional[FeedbackImage]:
        """Pump the 694 B plant stream until actuator FB in HBHF is fresh.

        After CFG / DEBUG probe, plant ``actuator_state`` in the feedback image
        can still be zeros until the MCU has exchanged CAN with the drive.
        Seed idle-anchored desires first (``kp=0``, ``position=pose`` — never
        blank ``p=0`` while the shaft is elsewhere); firmware then parareads
        on FDCAN and MCP, and this call streams long enough for replies to
        land in the payload.

        Call before reading ``FeedbackImage.actuator(slot).position`` for
        metrics / teleop. ``slots`` is validated only (held image is what TX).

        Returns the latest ``FeedbackImage`` (or ``None`` if none arrived).
        """
        if slots is None:
            slots = list(range(ACTUATOR_COUNT))
        for s in slots:
            if not (0 <= int(s) < ACTUATOR_COUNT):
                raise ValueError(f"slot must be 0..{ACTUATOR_COUNT - 1}, got {s}")

        dt = 1.0 / max(hz, 1.0)
        t_end = time.perf_counter() + max(0.1, float(seconds))
        next_t = time.perf_counter()
        last_fb: Optional[FeedbackImage] = None
        while time.perf_counter() < t_end:
            self._connection.send_once()
            fb = self._connection.poll_feedback()
            if fb is not None:
                last_fb = fb
                self._connection.publish_feedback(fb)
            next_t += dt
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()
        return last_fb

    def set_mcu_state(self, state: McuState, *, send: bool = True) -> None:
        self._connection.set_mcu_state(state, send=send)

    def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
        """Arm plant apply (True) or observe-gate (False) via system wire bit11."""
        self._connection.set_plant_apply(enable, send=send)

    @property
    def mcu_state(self) -> McuState:
        """Host-commanded ``system.mcu_state`` (NORMAL / RECOVERY / ESTOP / …)."""
        return self._connection.mcu_state

    @property
    def plant_apply(self) -> bool:
        """Host-commanded plant_apply arm (False = observe, no actuator mount)."""
        return self._connection.plant_apply

    @property
    def led_desire(self) -> Optional[LedDesire]:
        """Host-commanded LED mode held for plant TX (None if never set)."""
        return self._connection.led_desire

    def pdb_status(self, raw: Optional[bytes] = None) -> Optional[PdbStatus]:
        """Typed USB PDB status: system kill bytes + optional ``pdb[64]`` mirror.

        Uses ``raw`` when given; otherwise drains the newest plant FB from the
        RX reader (falls back to the stream cache). Stale peer ⇒
        ``kill_state=HARD`` / ``kill_reason=COMMS_LOSS`` (MCU).
        """
        if raw is None:
            latest = self._connection._drain_latest_plant_feedback()  # noqa: SLF001
            if latest is not None:
                self._connection._latest_fb_raw = latest  # noqa: SLF001
                raw = latest
            else:
                raw = self._connection._latest_fb_raw  # noqa: SLF001
        if raw is None:
            return None
        return pdb_status_from_frame(raw)

    @property
    def soft_kill(self) -> bool:
        """True while frozen (hold-at-torque) — see :meth:`soft_kill_freeze`."""
        return self._soft_kill

    def soft_kill_freeze(self, *, send: bool = True) -> None:
        """Freeze: stop updating held desires, stay NORMAL + plant_apply=1.

        Unlike :meth:`soft_kill_park`, this does NOT blank desires and does
        NOT call ``set_mcu_state`` — the point is to hold whatever position
        (kp/kd/position) is *currently* held, at full torque, rather than
        de-energize. Firmware ``plant_recovery_all()`` de-energizes the wire
        the instant ``mcu_state`` becomes RECOVERY/ESTOP, and
        ``Connection.set_mcu_state`` clears all held desires the moment that
        happens too (see link/connection.py) — so hold-at-torque is
        structurally incompatible with ever calling
        ``set_mcu_state(RECOVERY/ESTOP)`` while frozen. This method only
        arms ``plant_apply=1`` + confirms ``mcu_state`` is NORMAL (never
        touches RECOVERY/ESTOP) and flips :attr:`soft_kill` True — callers
        (debug_dashboard/state.py) are responsible for actually not calling
        ``set_actuator``/etc. again while frozen; this method alone does not
        prevent new commands from being issued, it only marks the intent and
        makes sure the wire stays in a state where "stop touching desires"
        is a valid freeze. See debug_dashboard/state.py for the enforcement
        side (the dashboard state machine raises before reaching this hub
        when :attr:`soft_kill` is True and a motion call comes in).

        # TODO(soft_kill_freeze divergence handling): a future improvement
        # should detect when a frozen target is unreachable (large sustained
        # feedback/target divergence — e.g. a stalled/jammed/overloaded joint
        # fighting to hold an unreachable position) and back off/adjust the
        # held command rather than keep forcing it indefinitely.
        # actions/teleop.py's existing HOLD_FLAG_RAD/HOLD_KD_BOOST_* constants
        # (in TeleopEngine._tick_actuators' "arrived — hold" branch, which
        # already flags large tracking error and reactively boosts kd) are
        # the natural extension point for this later. Not implemented now.
        """
        self._soft_kill = True
        self._connection.set_mcu_state(McuState.NORMAL, send=False)
        self._connection.set_plant_apply(True, send=send)

    def soft_kill_unfreeze(self) -> None:
        """Clear the frozen flag — callers may resume updating held desires.

        Does not itself change anything on the wire (no send) — the next
        motion command simply stops being blocked.
        """
        self._soft_kill = False

    def soft_kill_park(self, *, send: bool = True) -> PdbStatus | None:
        """Product soft-kill park: clear desires/servos and latch ``McuState.ESTOP``.

        Firmware ``plant_recovery_all()`` then acks ``SOFT_KILL_READY`` on the
        PDB UART when the peer is still in ``SOFT_KILL_REQ``. Call when
        ``pdb_status().soft_kill_req`` (or proactively before kill proves).
        """
        blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        self._connection.set_actuators(blank, send=False)
        self._connection.clear_servos(send=False)
        self.set_led(LedDesire(mode="follow", master_brightness=0, led_count=0), send=False)
        self.set_mcu_state(McuState.ESTOP, send=send)
        if send:
            # One extra poll so callers can read post-park kill/LED immediately.
            self.send_once()
        return self.pdb_status()

    def soft_kill_park_if_requested(self, *, send: bool = True) -> bool:
        """If USB kill is ``SOFT_KILL_REQ``, run :meth:`soft_kill_freeze`.

        Returns True when a freeze was performed. No-op when ``listen_pdu``
        is False. Was ``soft_kill_park`` (hard-ESTOP) before the soft_kill
        state machine — auto-trip now hold-at-torque freezes instead, see
        :meth:`soft_kill_freeze`.
        """
        if not self._listen_pdu:
            return False
        status = self.pdb_status()
        if status is None or status.kill_state != KILL_SOFT_REQ:
            return False
        self.soft_kill_freeze(send=send)
        return True

    def soft_kill_park_if_bad_vi(self, *, send: bool = True) -> bool:
        """Host-side belt-and-suspenders: independently re-check PDU V/I
        against the shared thresholds (:mod:`deft_controls_sdk.pdb.limits`,
        mirrored from firmware ``App/Inc/host/pdb_vi_limits.h``) and freeze
        even on firmware that doesn't yet overlay ``SOFT_KILL_REQ`` itself.

        Only acts when the peer reports ``NORMAL`` — stale (fails safe to
        ``HARD_ESTOP``/``COMMS_LOSS`` already), ``SOFT_KILL_REQ``/``READY``,
        and ``HARD_ESTOP`` are all already handled by the existing kill-state
        paths. Returns True when a freeze was performed.
        No-op when ``listen_pdu`` is False. Was ``soft_kill_park`` (hard-ESTOP)
        before the soft_kill state machine — see :meth:`soft_kill_freeze`.
        """
        if not self._listen_pdu:
            return False
        status = self.pdb_status()
        if status is None or not status.normal:
            return False
        check = pdb_limits.check_status(status)
        if check is None or not check.violated:
            return False
        self.soft_kill_freeze(send=send)
        return True
