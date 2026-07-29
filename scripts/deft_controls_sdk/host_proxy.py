"""HostProxy — platform demux on top of ControlsPcbHub.

One COM owner. Apps (pcb_lab, vbeta, i2rt bridge) talk in *components*
(left_arm, base, …), not raw slot indexes. Does not import vbeta.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.link import ActuatorDesire, FeedbackImage, LedDesire, McuState, ServoDesire
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD

# --- product slot map (platform truth; vbeta.slots re-exports) ---------------

LEFT_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 7))
RIGHT_ARM_SLOTS: Tuple[int, ...] = tuple(range(7, 14))
BASE_STEER_SLOTS: Dict[str, int] = {"BwC": 14, "BwR": 15, "BwL": 16}
BASE_DRIVE_SLOTS: Dict[str, int] = {"BpC": 17, "BpR": 18, "BpL": 19}
BASE_SLOTS: Tuple[int, ...] = (14, 15, 16, 17, 18, 19)
LIFT_SLOT = 20
SPARE_SLOTS: Tuple[int, ...] = (21, 22, 23, 24, 25)
NECK_PITCH_SERVO_SLOT = 0
NECK_YAW_SERVO_SLOT = 1


@dataclass(frozen=True)
class Profile:
    """Named groups of actuator slots."""

    name: str
    components: Mapping[str, Tuple[int, ...]]

    def slots(self, component: str) -> Tuple[int, ...]:
        try:
            return self.components[component]
        except KeyError as exc:
            known = ", ".join(sorted(self.components))
            raise KeyError(f"unknown component {component!r}; known: {known}") from exc

    def all_slots(self) -> Tuple[int, ...]:
        seen: List[int] = []
        for slots in self.components.values():
            for s in slots:
                if s not in seen:
                    seen.append(s)
        return tuple(seen)


def yam_product_profile() -> Profile:
    return Profile(
        name="yam_product",
        components={
            "left_arm": LEFT_ARM_SLOTS,
            "right_arm": RIGHT_ARM_SLOTS,
            "base": BASE_SLOTS,
            "lift": (LIFT_SLOT,),
        },
    )


class ComponentView:
    """MIT desire/FB for one named component (ordered slots)."""

    def __init__(self, proxy: "HostProxy", name: str) -> None:
        self._proxy = proxy
        self.name = name
        self.slots = proxy.profile.slots(name)

    def set_desires(self, desires: Sequence[ActuatorDesire], *, send: bool = False) -> None:
        if len(desires) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} desires, got {len(desires)}"
            )
        batch = {slot: desire for slot, desire in zip(self.slots, desires)}
        self._proxy.set_actuators(batch, send=send)

    def blank(self, *, send: bool = False) -> None:
        self.set_desires([ActuatorDesire() for _ in self.slots], send=send)

    def hold(
        self,
        positions: Sequence[float],
        *,
        kp: float = 8.0,
        kd: float = 0.5,
        send: bool = False,
    ) -> None:
        if len(positions) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} positions, got {len(positions)}"
            )
        desires = [
            ActuatorDesire(position=float(p), kp=float(kp), kd=float(kd)) for p in positions
        ]
        self.set_desires(desires, send=send)

    def positions(self) -> Optional[List[float]]:
        """Latest FB positions for this component, or None if no feedback yet."""
        fb = self._proxy.latest_feedback()
        if fb is None:
            return None
        out: List[float] = []
        for slot in self.slots:
            st = fb.actuator(slot)
            out.append(float(st.position) if st is not None else 0.0)
        return out


class HostProxy:
    """One process, one COM — component MIT API over ControlsPcbHub."""

    def __init__(
        self,
        hub: ControlsPcbHub,
        *,
        profile: Optional[Profile] = None,
        owns_hub: bool = True,
    ) -> None:
        self._hub = hub
        self._owns_hub = owns_hub
        self._profile = profile or yam_product_profile()
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
        profile: Optional[Profile] = None,
        idle_first: bool = False,
        persist_telemetry: bool = False,
        apply_yam_cfg: bool = False,
        force_cfg: bool = False,
    ) -> "HostProxy":
        hub = ControlsPcbHub.connect(
            port, serial=serial, baud=baud, persist_telemetry=persist_telemetry
        )
        proxy = cls(hub, profile=profile, owns_hub=True)
        proxy._stream_hz = float(stream_hz)
        if idle_first:
            blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
            proxy.set_actuators(blank, send=False)
            hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=False,
            )
            hub.send_once()
        else:
            hub.recover()
        if apply_yam_cfg:
            # Late import: CFG helpers live in vbeta; optional product path only.
            from deft_controls_sdk.vbeta.cfg import ensure_yam_product_cfg

            ensure_yam_product_cfg(hub, force=force_cfg)
        hub.start_streaming(hz=proxy._stream_hz)
        return proxy

    @classmethod
    def wrap(
        cls,
        hub: ControlsPcbHub,
        *,
        stream_hz: float = 40.0,
        profile: Optional[Profile] = None,
    ) -> "HostProxy":
        """Use an existing hub (tests / caller already owns COM)."""
        proxy = cls(hub, profile=profile, owns_hub=False)
        proxy._stream_hz = float(stream_hz)
        if not hub.is_streaming:
            hub.start_streaming(hz=proxy._stream_hz)
        return proxy

    @property
    def hub(self) -> ControlsPcbHub:
        return self._hub

    @property
    def profile(self) -> Profile:
        return self._profile

    def component(self, name: str) -> ComponentView:
        return ComponentView(self, name)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            self.set_actuators(blank, send=False)
            self._hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
            self._hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=False,
            )
            self._hub.send_once()
            if not self._hub.is_streaming:
                self._hub.start_streaming(hz=5.0)
            time.sleep(0.25)
            self._hub.stop_streaming()
        finally:
            if self._owns_hub:
                self._hub.close()

    def __enter__(self) -> "HostProxy":
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

    def doctor(self) -> Dict[str, object]:
        """Offline-friendly health snapshot (CFG table if DEBUG allows)."""
        report: Dict[str, object] = {
            "profile": self._profile.name,
            "components": {k: list(v) for k, v in self._profile.components.items()},
            "streaming": bool(self._hub.is_streaming),
            "port": self._hub.port,
        }
        try:
            table = self._hub.debug.cfg_get_table()
            report["cfg_slots"] = len(table) if table is not None else 0
            report["cfg_ok"] = table is not None
        except Exception as exc:  # noqa: BLE001 — doctor must not raise
            report["cfg_ok"] = False
            report["cfg_error"] = str(exc)
        fb = self.latest_feedback()
        report["has_feedback"] = fb is not None
        return report
