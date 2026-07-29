"""Sole COM session for vbeta drivers — thin wrap of HostProxy.

YAM-shaped drivers (PcbArmDriver, …) keep taking PcbRobotSession.
Platform demux lives in deft_controls_sdk.host_proxy.HostProxy.
"""
from __future__ import annotations

from typing import Mapping, Optional

from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.host_proxy import HostProxy, Profile, yam_product_profile
from deft_controls_sdk.link import ActuatorDesire, FeedbackImage, LedDesire, ServoDesire
from deft_controls_sdk.link.exchange import DEFAULT_BAUD


class PcbRobotSession:
    """One process, one COM — arms / platform / neck / LEDs share HostProxy."""

    def __init__(self, proxy: HostProxy) -> None:
        self._proxy = proxy

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
        proxy = HostProxy.connect(
            port,
            serial=serial,
            baud=baud,
            stream_hz=stream_hz,
            profile=yam_product_profile(),
            idle_first=idle_first,
            persist_telemetry=persist_telemetry,
            apply_yam_cfg=apply_yam_cfg,
            force_cfg=force_cfg,
        )
        return cls(proxy)

    @classmethod
    def wrap(cls, hub: ControlsPcbHub, *, stream_hz: float = 40.0) -> "PcbRobotSession":
        """Use an existing hub (tests / caller already owns COM)."""
        return cls(HostProxy.wrap(hub, stream_hz=stream_hz, profile=yam_product_profile()))

    @classmethod
    def from_proxy(cls, proxy: HostProxy) -> "PcbRobotSession":
        return cls(proxy)

    @property
    def proxy(self) -> HostProxy:
        return self._proxy

    @property
    def hub(self) -> ControlsPcbHub:
        return self._proxy.hub

    @property
    def profile(self) -> Profile:
        return self._proxy.profile

    def close(self) -> None:
        self._proxy.close()

    def __enter__(self) -> "PcbRobotSession":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = False) -> None:
        self._proxy.set_actuator(slot, desire, send=send)

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = False) -> None:
        self._proxy.set_actuators(desires, send=send)

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = False) -> None:
        self._proxy.set_servo(slot, desire, send=send)

    def set_led(self, desire: LedDesire, *, send: bool = False) -> None:
        self._proxy.set_led(desire, send=send)

    def service_soft_kill(self) -> bool:
        return self._proxy.service_soft_kill()

    def send_once(self) -> None:
        self._proxy.send_once()

    def poll_feedback(self) -> Optional[FeedbackImage]:
        return self._proxy.poll_feedback()

    def latest_feedback(self) -> Optional[FeedbackImage]:
        return self._proxy.latest_feedback()

    def sleep(self, seconds: float) -> None:
        self._proxy.sleep(seconds)
