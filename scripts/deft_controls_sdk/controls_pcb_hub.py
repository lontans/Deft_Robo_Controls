"""ControlsPcbHub — the one import surface for software.

    from deft_controls_sdk import ControlsPcbHub

    with ControlsPcbHub.connect() as hub:  # or connect("COM5") / serial=
        hub.start_streaming()
        print(hub.telemetry.snapshot())

Owns COM via link.Connection. Publishes TelemetryCache for scripts and
debug_dashboard. Never imports debug_dashboard.

hub.debug.* is DEBUG mode — see docs/api.md. Same Connection; plant apply may
be gated (plant_block=BENCH_SESSION) while a lease is held.
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Optional, Union

from deft_controls_sdk.bench import DebugAPI, find_cdc_port
from deft_controls_sdk.link import ActuatorDesire, Connection, FeedbackImage, LedDesire, McuState, ServoDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD
from deft_controls_sdk.telemetry import TelemetryCache, default_session_dir


class ControlsPcbHub:
    def __init__(self, connection: Connection, telemetry: TelemetryCache) -> None:
        self._connection = connection
        self._telemetry = telemetry
        self._debug = DebugAPI(connection, telemetry)

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
    ) -> "ControlsPcbHub":
        """Connect and attach telemetry.

        ``port`` omitted → auto-pick STM32 USB CDC (0483:5740), optionally
        filtered by USB ``serial``. ``persist_telemetry`` defaults **False**.
        """
        if not port:
            port = find_cdc_port(serial=serial)
        elif serial is not None:
            # Explicit port wins; serial only used for auto-pick.
            pass
        connection = Connection.connect(port, baud=baud)
        if telemetry is None:
            telemetry = TelemetryCache(session_dir=session_dir or default_session_dir(), persist=persist_telemetry)
        connection.attach_telemetry(telemetry)
        return cls(connection, telemetry)

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
        See deft_controls_sdk/bench/README or DebugAPI's docstring."""
        return self._debug

    @property
    def port(self) -> str:
        return self._connection.port

    @property
    def state_path(self) -> Path:
        return self.telemetry.state_path

    def start_streaming(self, hz: float = 40.0, *, telemetry_hz: float = 10.0) -> None:
        """Background plant stream — keeps HOST_STALE clear and feeds telemetry.

        Plant TX runs at ``hz`` on its own thread (send→sleep, legacy-shaped).
        TelemetryCache / UI publish runs on a *side* thread at ``telemetry_hz``
        so dashboard disk/json cannot stretch plant TX gaps.
        """
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

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = True) -> None:
        self._connection.set_servo(slot, desire, send=send)

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
        hz: float = 40.0,
    ) -> Optional[FeedbackImage]:
        """Pump the 672 B plant stream until actuator FB in HBHF is fresh.

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
