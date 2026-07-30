"""ComponentAction — plant MIT desires for one named profile component."""
from __future__ import annotations

from typing import List, Optional, Sequence, TYPE_CHECKING

from deft_controls_sdk.link import ActuatorDesire

from .sink import PlantSink

if TYPE_CHECKING:
    from deft_controls_sdk.config import Profile


class ComponentAction:
    """Runtime plant commands for a named component (slots from ``Profile``).

    Desire presets:
    - ``hold`` — stiffen at positions (kp/kd > 0)
    - ``blank`` — all-zero desires (typically limp / no torque)
    """

    def __init__(self, sink: PlantSink, profile: "Profile", name: str) -> None:
        self._sink = sink
        self._profile = profile
        self.name = name
        self.slots = profile.slots(name)

    @property
    def profile(self) -> "Profile":
        return self._profile

    @property
    def sink(self) -> PlantSink:
        return self._sink

    def set_desires(self, desires: Sequence[ActuatorDesire], *, send: bool = False) -> None:
        if len(desires) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} desires, got {len(desires)}"
            )
        batch = {slot: desire for slot, desire in zip(self.slots, desires)}
        self._sink.set_actuators(batch, send=send)

    def blank(self, *, send: bool = False) -> None:
        """Clear desires on this component (all-zero ActuatorDesire)."""
        self.set_desires([ActuatorDesire() for _ in self.slots], send=send)

    def hold(
        self,
        positions: Sequence[float],
        *,
        kp: float = 8.0,
        kd: float = 0.5,
        send: bool = False,
    ) -> None:
        """Stiffen at ``positions`` (plant MIT desire preset)."""
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
        fb = self._sink.latest_feedback()
        if fb is None:
            return None
        out: List[float] = []
        for slot in self.slots:
            st = fb.actuator(slot)
            out.append(float(st.position) if st is not None else 0.0)
        return out
