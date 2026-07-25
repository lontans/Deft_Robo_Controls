"""Sole ControlsPcbHub owner for vbeta drivers."""
from __future__ import annotations

import time
from typing import Mapping, Optional

from deft_controls_sdk import ControlsPcbHub, LedDesire, McuState
from deft_controls_sdk.link import ActuatorDesire, FeedbackImage, ServoDesire
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD
from deft_controls_sdk.vbeta.cfg import ensure_yam_product_cfg


class PcbRobotSession:
    """One process, one COM — arms / platform / neck / LEDs share this hub."""

    def __init__(self, hub: ControlsPcbHub, *, owns_hub: bool = True) -> None:
        self._hub = hub
        self._owns_hub = owns_hub
        self._stream_hz = 40.0
        self._closed = False

    @classmethod
    def connect(
        cls,
        port: Optional[str] = None,
        *,
        serial: Optional[str] = None,
        baud: int = DEFAULT_BAUD,
        stream_hz: float = 40.0,
        apply_yam_cfg: bool = False,
        force_cfg: bool = False,
        idle_first: bool = False,
        persist_telemetry: bool = False,
    ) -> "PcbRobotSession":
        hub = ControlsPcbHub.connect(
            port, serial=serial, baud=baud, persist_telemetry=persist_telemetry
        )
        session = cls(hub, owns_hub=True)
        session._stream_hz = float(stream_hz)
        if idle_first:
            # Gate plant CAN + blank desires + cornflower before CFG/MIT.
            blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
            session.set_actuators(blank, send=False)
            hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=False,
            )
            hub.send_once()
        else:
            hub.recover()
        if apply_yam_cfg:
            ensure_yam_product_cfg(hub, force=force_cfg)
        hub.start_streaming(hz=session._stream_hz)
        return session

    @classmethod
    def wrap(cls, hub: ControlsPcbHub, *, stream_hz: float = 40.0) -> "PcbRobotSession":
        """Use an existing hub (tests / caller already owns COM)."""
        session = cls(hub, owns_hub=False)
        session._stream_hz = float(stream_hz)
        if not hub.is_streaming:
            hub.start_streaming(hz=session._stream_hz)
        return session

    @property
    def hub(self) -> ControlsPcbHub:
        return self._hub

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            # Leave plant gated + cornflower idle (not NORMAL/LED-off — strip
            # would fall back to PDB red on UART4).
            blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            self.set_actuators(blank, send=False)
            self._hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
            self._hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=False,
            )
            self._hub.send_once()
            # Brief stream so the idle image is applied, then stop; g_cmd_live holds.
            if not self._hub.is_streaming:
                self._hub.start_streaming(hz=5.0)
            time.sleep(0.25)
            self._hub.stop_streaming()
        finally:
            if self._owns_hub:
                self._hub.close()

    def __enter__(self) -> "PcbRobotSession":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = False) -> None:
        self._hub.set_actuator(slot, desire, send=send)

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = False) -> None:
        conn = self._hub._connection  # noqa: SLF001 — batch hold update
        conn.set_actuators(desires, send=send)

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = False) -> None:
        self._hub.set_servo(slot, desire, send=send)

    def set_led(self, desire: LedDesire, *, send: bool = False) -> None:
        self._hub.set_led(desire, send=send)

    def service_soft_kill(self) -> bool:
        """If peer requested soft-kill, park via hub (Track B API)."""
        fn = getattr(self._hub, "soft_kill_park_if_requested", None)
        if fn is None:
            return False
        return bool(fn(send=False))

    def send_once(self) -> None:
        self.service_soft_kill()
        self._hub.send_once()

    def poll_feedback(self) -> Optional[FeedbackImage]:
        return self._hub._connection.poll_feedback()  # noqa: SLF001

    def latest_feedback(self) -> Optional[FeedbackImage]:
        fb = self.poll_feedback()
        if fb is not None:
            return fb
        raw = self._hub._connection._latest_fb_raw  # noqa: SLF001
        return FeedbackImage(raw) if raw is not None else None

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)
