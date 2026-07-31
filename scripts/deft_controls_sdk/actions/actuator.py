"""ActuatorAction — plant MIT desires for one ordered slot group."""
from __future__ import annotations

from typing import List, Optional, Sequence, TYPE_CHECKING, Tuple, Union

from deft_controls_sdk.config.actuator import (
    ActuatorKind,
    kind_for_component,
    resolve_gain_vectors,
)
from deft_controls_sdk.link import ActuatorDesire

from .plant import PlantAction
from .sink import PlantSink

if TYPE_CHECKING:
    from deft_controls_sdk.config import Profile
    from deft_controls_sdk.config.typed_profiles import ActuatorProfile

GainIn = Union[float, Sequence[float], None]


class ActuatorAction(PlantAction):
    """Runtime plant commands for an ordered actuator slot list.

    Prefer ``from_actuator_profile`` (typed section/single). Demux
    ``Profile`` component ctor and ``from_slots`` remain for HostProxy.

    ``kind`` selects default MIT gains (joint vs wheel).
    """

    def __init__(
        self,
        sink: PlantSink,
        profile: "Profile",
        name: str,
        *,
        kind: Optional[ActuatorKind] = None,
    ) -> None:
        """Demux ``Profile`` component constructor (name → slots)."""
        super().__init__(sink)
        self._demux_profile: Optional["Profile"] = profile
        self._actuator_profile: Optional["ActuatorProfile"] = None
        self.name = name
        self.slots: Tuple[int, ...] = profile.slots(name)
        self.kind: ActuatorKind = kind if kind is not None else kind_for_component(name)

    @classmethod
    def from_actuator_profile(
        cls,
        sink: PlantSink,
        profile: "ActuatorProfile",
    ) -> "ActuatorAction":
        """Typed section/single profile (slots + kind from ``ActuatorProfile``)."""
        obj = cls.__new__(cls)
        PlantAction.__init__(obj, sink)
        obj._demux_profile = None
        obj._actuator_profile = profile
        obj.name = profile.name
        obj.slots = tuple(profile.slots)
        obj.kind = profile.kind
        return obj

    @classmethod
    def from_slots(
        cls,
        sink: PlantSink,
        slots: Sequence[int],
        *,
        name: str = "slots",
        kind: ActuatorKind = "joint",
    ) -> "ActuatorAction":
        """Ad-hoc / bus / single-actuator group (no Profile required)."""
        obj = cls.__new__(cls)
        PlantAction.__init__(obj, sink)
        obj._demux_profile = None
        obj._actuator_profile = None
        obj.name = str(name)
        obj.slots = tuple(int(s) for s in slots)
        obj.kind = kind
        return obj

    @classmethod
    def from_profile(
        cls,
        sink: PlantSink,
        profile: "Profile",
        name: str,
        *,
        kind: Optional[ActuatorKind] = None,
    ) -> "ActuatorAction":
        return cls(sink, profile, name, kind=kind)

    @property
    def profile(self) -> Optional["Profile"]:
        """Demux Profile if constructed that way; else None."""
        return self._demux_profile

    @property
    def actuator_profile(self) -> Optional["ActuatorProfile"]:
        return self._actuator_profile

    def set_desires(self, desires: Sequence[ActuatorDesire], *, send: bool = False) -> None:
        if len(desires) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} desires, got {len(desires)}"
            )
        batch = {slot: desire for slot, desire in zip(self.slots, desires)}
        self._sink.set_actuators(batch, send=send)

    def blank(self, *, send: bool = False) -> None:
        """Clear desires on this group (all-zero ActuatorDesire)."""
        self.set_desires([ActuatorDesire() for _ in self.slots], send=send)

    def hold(
        self,
        positions: Sequence[float],
        *,
        kp: GainIn = None,
        kd: GainIn = None,
        torque: GainIn = None,
        send: bool = False,
    ) -> None:
        """Stiffen at ``positions`` using kind defaults unless gains are passed."""
        n = len(self.slots)
        if len(positions) != n:
            raise ValueError(
                f"{self.name}: expected {n} positions, got {len(positions)}"
            )
        kps, kds, tqs = resolve_gain_vectors(
            self.kind, n, kp=kp, kd=kd, torque=torque
        )
        desires = [
            ActuatorDesire(
                position=float(p),
                kp=float(kps[i]),
                kd=float(kds[i]),
                torque=float(tqs[i]),
            )
            for i, p in enumerate(positions)
        ]
        self.set_desires(desires, send=send)

    def nudge(
        self,
        *,
        index: int = 0,
        delta: float = 0.05,
        kp: GainIn = None,
        kd: GainIn = None,
        torque: GainIn = None,
        send: bool = False,
    ) -> List[float]:
        """Step one index by ``delta`` rad from FB (or zeros) and hold."""
        pos = list(self.positions() or [0.0] * len(self.slots))
        if not (0 <= index < len(pos)):
            raise ValueError(f"index {index} out of range 0..{len(pos) - 1}")
        pos[index] = float(pos[index]) + float(delta)
        self.hold(pos, kp=kp, kd=kd, torque=torque, send=send)
        return pos

    def positions(self) -> Optional[List[float]]:
        """Latest FB positions for this group, or None if no feedback yet."""
        fb = self._sink.latest_feedback()
        if fb is None:
            return None
        out: List[float] = []
        for slot in self.slots:
            st = fb.actuator(slot)
            out.append(float(st.position) if st is not None else 0.0)
        return out


__all__ = ["ActuatorAction"]
