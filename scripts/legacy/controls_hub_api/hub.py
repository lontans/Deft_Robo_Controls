"""ControlsHub — top-level host façade (plant / debug / health).

Sole intended entry point for new application code: pick PLANT vs DEBUG,
never a `pdu` tag. Wraps `controls_hub_controller.PlantSession` (single COM
owner) with three scoped views:

    hub.plant   — actuator_commands[] + mcu_state, pdu always zero (500 Hz path)
    hub.debug   — bench/diag lease; plant apply may be gated while held
    hub.health  — read-only snapshot derived from the last feedback frame

See docs/plan-host-api-streamline.md for the target mental model and
docs/architecture.md §Host API modes for the mode/lease diagram this
implements a thin slice of.
"""
from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Generator, List, Mapping, Optional, Type, Union

from controls_hub_controller import (
    ActuatorDesire,
    FeedbackImage,
    PlantBlockReason,
    PlantSession,
)

_FB_HISTORY = 20
"""Feedback timestamps kept for fb_hz estimation — small ring, not a log."""


@dataclass(frozen=True)
class HealthSnapshot:
    """Read-only view derived from the last feedback frame seen by hub.plant.

    Never opens a second COM connection and never polls independently — it
    is a cache of whatever hub.plant.poll_feedback()/read_feedback()/
    run_at_hz() already observed.
    """

    connected: bool
    tick: Optional[int]
    ack_seq: Optional[int]
    mcu_state: Optional[int]
    plant_block: Optional[Union[PlantBlockReason, int]]
    fb_hz: Optional[float]
    age_s: Optional[float]


class PlantAPI:
    """hub.plant.* — normal-operation motor control. Thin wrapper over
    PlantSession (or HubSession); pdu is never written."""

    def __init__(self, hub: "ControlsHub") -> None:
        self._hub = hub

    @property
    def _session(self) -> PlantSession:
        return self._hub._session

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None:
        self._session.set_actuator(slot, desire, send=send)

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = True) -> None:
        self._session.set_actuators(desires, send=send)

    def send_once(self) -> None:
        self._session.send_once()

    def held_desire(self, slot: int) -> Optional[ActuatorDesire]:
        return self._session.held_desire(slot)

    def poll_feedback(self) -> Optional[FeedbackImage]:
        """Non-blocking: latest buffered feedback frame, or None."""
        fb = self._session.poll_feedback()
        if fb is not None:
            self._hub._record_feedback(fb)
        return fb

    def read_feedback(self, *, timeout_s: float = 1.0, latest: bool = True) -> FeedbackImage:
        """Blocking: wait up to timeout_s for a feedback frame."""
        fb = self._session.read_feedback(timeout_s=timeout_s, latest=latest)
        self._hub._record_feedback(fb)
        return fb

    def run_at_hz(
        self,
        hz: float,
        callback: Callable[[Optional[FeedbackImage]], Optional[Mapping[int, ActuatorDesire]]],
        *,
        stop: Optional[Callable[[], bool]] = None,
    ) -> None:
        """Streaming loop helper — see PlantSession.run_at_hz. Feedback frames
        observed here also update hub.health's cache."""

        def _wrapped(fb: Optional[FeedbackImage]) -> Optional[Mapping[int, ActuatorDesire]]:
            if fb is not None:
                self._hub._record_feedback(fb)
            return callback(fb)

        self._session.run_at_hz(hz, _wrapped, stop=stop)

    def sleep_until_next_tick(self, hz: float) -> None:
        self._session.sleep_until_next_tick(hz)

    def recover(self) -> None:
        self._session.recover()


class DebugAPI:
    """hub.debug.* — bench/diag lease on the same COM connection as
    hub.plant. Holding a lease documents (and, for RS2, actually triggers)
    plant apply being gated — check hub.health.snapshot().plant_block while
    a lease is held.

    Richer bench flows (discover / calibrate / config) are not wrapped here
    yet; they stay on the `control_hub.py` CLI (PcbSession), a separate
    session class that must not be opened concurrently against the same
    port as an open ControlsHub.
    """

    def __init__(self, hub: "ControlsHub") -> None:
        self._hub = hub

    @contextmanager
    def lease(self, *, bus: int = 1) -> Generator[None, None, None]:
        """RS2 session_begin/end bracket. While held, plant_runtime gates
        actuator_apply_desire (plant_block=BENCH_SESSION) — see
        docs/architecture.md "Plant runtime gates"."""
        session = self._hub._session
        session.rs2_session_begin(bus=bus)
        try:
            yield
        finally:
            session.rs2_session_end(bus=bus)

    def discover(self, *args: object, **kwargs: object) -> None:
        raise NotImplementedError(
            "hub.debug.discover is not wrapped yet — use "
            "`python scripts/control_hub.py discover ...` (PcbSession), not "
            "concurrently with an open ControlsHub on the same port."
        )

    def calibrate(self, *args: object, **kwargs: object) -> None:
        raise NotImplementedError(
            "hub.debug.calibrate is not wrapped yet — use "
            "`python scripts/control_hub.py calibrate ...` (PcbSession), not "
            "concurrently with an open ControlsHub on the same port."
        )

    def config_show(self, *args: object, **kwargs: object) -> None:
        raise NotImplementedError(
            "hub.debug.config_show is not wrapped yet — use "
            "`python scripts/control_hub.py config show ...` (PcbSession)."
        )


class HealthAPI:
    """hub.health.* — read-only. Derived from the last feedback frame that
    passed through hub.plant; never polls or opens a second COM connection."""

    def __init__(self, hub: "ControlsHub") -> None:
        self._hub = hub

    def snapshot(self) -> HealthSnapshot:
        return self._hub._health_snapshot()


class ControlsHub:
    """Sole intended entry point for application code. Owns one serial link
    (via PlantSession or a subclass such as HubSession) split into three
    scoped views: plant / debug / health. See module docstring and
    docs/plan-host-api-streamline.md.

        from controls_hub_api import ControlsHub, ActuatorDesire

        with ControlsHub.connect("COM5") as hub:
            hub.plant.set_actuator(3, ActuatorDesire(position=0.0, kp=8.0, kd=0.45))
            print(hub.health.snapshot())

    Only one process may hold the serial port at a time (Windows raises
    Access Denied on a second open) — do not also run controls_hub_dashboard.py
    against the same port while a ControlsHub is connected.
    """

    def __init__(self, session: PlantSession) -> None:
        self._session = session
        self._plant = PlantAPI(self)
        self._debug = DebugAPI(self)
        self._health = HealthAPI(self)
        self._last_feedback: Optional[FeedbackImage] = None
        self._last_feedback_at: Optional[float] = None
        self._fb_times: List[float] = []

    @classmethod
    def connect(cls, port: str, *, session_cls: Type[PlantSession] = PlantSession, **kwargs: object) -> "ControlsHub":
        session = session_cls.connect(port, **kwargs)  # type: ignore[arg-type]
        return cls(session)

    def close(self) -> None:
        self._session.close()

    def __enter__(self) -> "ControlsHub":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def plant(self) -> PlantAPI:
        return self._plant

    @property
    def debug(self) -> DebugAPI:
        return self._debug

    @property
    def health(self) -> HealthAPI:
        return self._health

    # -- internal: shared feedback cache backing HealthAPI ------------------

    def _record_feedback(self, fb: FeedbackImage) -> None:
        now = time.monotonic()
        self._last_feedback = fb
        self._last_feedback_at = now
        self._fb_times.append(now)
        if len(self._fb_times) > _FB_HISTORY:
            self._fb_times.pop(0)

    def _health_snapshot(self) -> HealthSnapshot:
        fb = self._last_feedback
        fb_hz: Optional[float] = None
        if len(self._fb_times) >= 2:
            span = self._fb_times[-1] - self._fb_times[0]
            if span > 0:
                fb_hz = (len(self._fb_times) - 1) / span
        age_s = time.monotonic() - self._last_feedback_at if self._last_feedback_at is not None else None
        return HealthSnapshot(
            connected=self._session.is_open,
            tick=fb.tick if fb is not None else None,
            ack_seq=fb.ack_seq if fb is not None else None,
            mcu_state=fb.mcu_state if fb is not None else None,
            plant_block=fb.plant_block if fb is not None else None,
            fb_hz=fb_hz,
            age_s=age_s,
        )
