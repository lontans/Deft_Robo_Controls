"""ActuatorAction — plant MIT desires for one ordered slot group."""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, TYPE_CHECKING, Tuple, Union

from deft_controls_sdk.config.actuator import resolve_hold_gains
from deft_controls_sdk.link import ActuatorDesire

from .mounted import MountedAction
from .plant import PlantAction
from .sink import PlantSink

if TYPE_CHECKING:
    from deft_controls_sdk.config import Profile
    from deft_controls_sdk.config.typed_profiles import ActuatorProfile

GainIn = Union[float, Sequence[float], None]


class ActuatorAction(PlantAction):
    """Runtime plant commands for an ordered actuator slot list.

    Prefer ``proxy.actions.actuator(slots=…)`` then::

        a.mount(wheels.hold(pos, kp=20.0, kd=1.0))
        a.apply()

    Gains are not a profile/board concept — pass ``kp``/``kd`` (or accept
    neutral hold defaults). ``send=True`` is a legacy shortcut.
    """

    def __init__(
        self,
        sink: PlantSink,
        profile: "Profile",
        name: str,
    ) -> None:
        """Demux ``Profile`` component constructor (name → slots)."""
        super().__init__(sink)
        self._demux_profile: Optional["Profile"] = profile
        self._actuator_profile: Optional["ActuatorProfile"] = None
        self.name = name
        self.slots: Tuple[int, ...] = profile.slots(name)

    @classmethod
    def from_actuator_profile(
        cls,
        sink: PlantSink,
        profile: "ActuatorProfile",
    ) -> "ActuatorAction":
        """Typed section/single profile (slots from ``ActuatorProfile``)."""
        obj = cls.__new__(cls)
        PlantAction.__init__(obj, sink)
        obj._demux_profile = None
        obj._actuator_profile = profile
        obj.name = profile.name
        obj.slots = tuple(profile.slots)
        return obj

    @classmethod
    def from_slots(
        cls,
        sink: PlantSink,
        slots: Sequence[int],
        *,
        name: str = "slots",
    ) -> "ActuatorAction":
        """Ad-hoc / bus / single-actuator group (no Profile required)."""
        obj = cls.__new__(cls)
        PlantAction.__init__(obj, sink)
        obj._demux_profile = None
        obj._actuator_profile = None
        obj.name = str(name)
        obj.slots = tuple(int(s) for s in slots)
        return obj

    @classmethod
    def from_profile(
        cls,
        sink: PlantSink,
        profile: "Profile",
        name: str,
    ) -> "ActuatorAction":
        return cls(sink, profile, name)

    @property
    def profile(self) -> Optional["Profile"]:
        """Demux Profile if constructed that way; else None."""
        return self._demux_profile

    @property
    def actuator_profile(self) -> Optional["ActuatorProfile"]:
        return self._actuator_profile

    def _build_hold_desires(
        self,
        positions: Sequence[float],
        *,
        kp: GainIn = None,
        kd: GainIn = None,
        torque: GainIn = None,
    ) -> Dict[int, ActuatorDesire]:
        n = len(self.slots)
        if len(positions) != n:
            raise ValueError(
                f"{self.name}: expected {n} positions, got {len(positions)}"
            )
        kps, kds, tqs = resolve_hold_gains(n, kp=kp, kd=kd, torque=torque)
        return {
            int(slot): ActuatorDesire(
                position=float(positions[i]),
                kp=float(kps[i]),
                kd=float(kds[i]),
                torque=float(tqs[i]),
            )
            for i, slot in enumerate(self.slots)
        }

    def _emit(self, mounted: MountedAction, *, send: bool) -> MountedAction:
        """Bound → return patch (or mount+apply if send). Unbound → sink write."""
        if self._actions is not None:
            if send:
                self._actions.mount(mounted)
                self._actions.apply()
            return mounted
        if mounted.actuators:
            self._sink.set_actuators(mounted.actuators, send=bool(send))
        return mounted

    def set_desires(
        self, desires: Sequence[ActuatorDesire], *, send: bool = False
    ) -> MountedAction:
        if len(desires) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} desires, got {len(desires)}"
            )
        batch = {slot: desire for slot, desire in zip(self.slots, desires)}
        return self._emit(
            MountedAction.from_actuators(f"{self.name}.set_desires", batch),
            send=send,
        )

    def blank(self, *, send: bool = False) -> MountedAction:
        """Clear desires on this group (all-zero ActuatorDesire)."""
        batch = {s: ActuatorDesire() for s in self.slots}
        return self._emit(
            MountedAction.from_actuators(
                f"{self.name}.blank", batch, meta={"op": "blank"}
            ),
            send=send,
        )

    def hold(
        self,
        positions: Optional[Sequence[float]] = None,
        *,
        kp: GainIn = None,
        kd: GainIn = None,
        torque: GainIn = None,
        send: bool = False,
    ) -> MountedAction:
        """Hold actuators in place (default) or at explicit ``positions``.

        With no ``positions``, samples latest FB and holds there — does **not**
        command a move. Raises if feedback is not available yet.

        Preferred::

            a.mount(wheels.hold())                 # stay put
            a.mount(wheels.hold([0.1, 0.2]))       # explicit setpoints
            a.apply()

        Omitting ``kp``/``kd`` uses neutral hold defaults.
        ``send=True`` is legacy (immediate TX / batch apply).
        """
        sampled_from_fb = positions is None
        if sampled_from_fb:
            sampled = self.positions()
            if sampled is None:
                raise RuntimeError(
                    f"{self.name}.hold(): no feedback yet — cannot sample position "
                    f"to hold in place (pass positions= explicitly if needed)"
                )
            positions = sampled
        pos_list = [float(p) for p in positions]
        batch = self._build_hold_desires(
            pos_list, kp=kp, kd=kd, torque=torque
        )
        return self._emit(
            MountedAction.from_actuators(
                f"{self.name}.hold",
                batch,
                meta={
                    "op": "hold",
                    "positions": pos_list,
                    "sampled": sampled_from_fb,
                },
            ),
            send=send,
        )

    def nudge(
        self,
        *,
        index: int = 0,
        delta: float = 0.05,
        kp: GainIn = None,
        kd: GainIn = None,
        torque: GainIn = None,
        send: bool = False,
    ) -> MountedAction:
        """Step one index by ``delta`` rad from current FB, then hold."""
        pos = self.positions()
        if pos is None:
            raise RuntimeError(
                f"{self.name}.nudge(): no feedback yet — cannot sample position"
            )
        pos = list(pos)
        if not (0 <= index < len(pos)):
            raise ValueError(f"index {index} out of range 0..{len(pos) - 1}")
        pos[index] = float(pos[index]) + float(delta)
        mounted = self.hold(pos, kp=kp, kd=kd, torque=torque, send=send)
        mounted.meta["op"] = "nudge"
        mounted.meta["index"] = int(index)
        mounted.meta["delta"] = float(delta)
        mounted.meta["sampled"] = False
        mounted.label = f"{self.name}.nudge"
        return mounted

    def positions(self) -> Optional[List[float]]:
        """Latest FB positions for this group, or None if any slot lacks FB.

        Incomplete samples return ``None`` so ``hold()`` cannot silently
        command zeros when a slot has not reported yet.
        """
        fb = self._sink.latest_feedback()
        if fb is None:
            return None
        out: List[float] = []
        for slot in self.slots:
            st = fb.actuator(slot)
            if st is None:
                return None
            out.append(float(st.position))
        return out


__all__ = ["ActuatorAction"]
