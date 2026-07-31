"""Actions — session façade: mount → apply → clear.

Notebook workflow::

    a = proxy.actions
    wheels = a.actuator(slots=(22, 23))
    a.mount(wheels.hold([0.0, 0.0]))
    a.mount(a.servo().neck_center())
    print(a.pending)   # inspect
    a.apply()          # commit once
    a.clear()          # blank touched groups + empty list
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence

from deft_controls_sdk.link import ActuatorDesire, ServoDesire

from .actuator import ActuatorAction
from .led import LedAction
from .mounted import MountedAction, merge_mounted
from .pdu_link import PduLinkAction
from .servo import ServoAction

if TYPE_CHECKING:
    from deft_controls_sdk.host_proxy import HostProxy


class Actions:
    """Plant command batch bound to one ``HostProxy`` session."""

    def __init__(self, proxy: "HostProxy") -> None:
        self._proxy = proxy
        self._pending: List[MountedAction] = []
        self._touched_actuators: set[int] = set()
        self._touched_servos: set[int] = set()
        self._touched_led: bool = False

    # -- factories ------------------------------------------------------------------

    def actuator(
        self,
        *,
        slots: Optional[Sequence[int]] = None,
        name: Optional[str] = None,
    ) -> ActuatorAction:
        """Build an ``ActuatorAction`` linked to this batch.

        ``slots=`` selects by bus index (optional ``name=`` is only a label).
        ``name=`` alone resolves an assembly/profile group.
        """
        if slots is not None:
            label = str(name) if name is not None else "slots"
            view = ActuatorAction.from_slots(self._proxy, slots, name=label)
        elif name is not None:
            view = self._proxy.actuators(str(name))
        else:
            raise ValueError("actuator() needs name= or slots=")
        view.bind_actions(self)
        return view

    def servo(self, name: str = "neck") -> ServoAction:
        view = self._proxy.servo(name)
        view.bind_actions(self)
        return view

    def led(self) -> LedAction:
        view = LedAction(self._proxy)
        view.bind_actions(self)
        return view

    def pdu(self) -> PduLinkAction:
        return PduLinkAction(self._proxy)

    # -- batch ----------------------------------------------------------------------

    @property
    def pending(self) -> List[Dict[str, Any]]:
        """Describe mounted (not yet applied) patches."""
        return [m.describe() for m in self._pending]

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def __len__(self) -> int:
        return len(self._pending)

    def __iter__(self):
        return iter(self._pending)

    def mounted(self) -> List[MountedAction]:
        """Raw pending patches (mutable list copy)."""
        return list(self._pending)

    def mount(self, item: Optional[MountedAction]) -> MountedAction:
        """Append a staged patch. ``hold`` / ``neck_center`` return these."""
        if item is None:
            raise TypeError("mount() expected a MountedAction (got None)")
        if not isinstance(item, MountedAction):
            raise TypeError(
                f"mount() expected MountedAction, got {type(item).__name__}"
            )
        self._pending.append(item)
        return item

    def apply(self) -> MountedAction:
        """Write all pending patches to the hub held map and ``commit()`` once."""
        merged = merge_mounted(self._pending)
        proxy = self._proxy
        if merged.actuators:
            proxy.set_actuators(merged.actuators, send=False)
            self._touched_actuators.update(merged.actuators)
        for slot, desire in merged.servos.items():
            proxy.set_servo(int(slot), desire, send=False)
            self._touched_servos.add(int(slot))
        if merged.led is not None:
            proxy.set_led(merged.led, send=False)
            self._touched_led = True
        proxy.commit()
        self._pending.clear()
        return merged

    def clear(self) -> None:
        """Blank touched actuator/servo groups, drop LED to follow/off path, empty pending."""
        proxy = self._proxy
        if self._touched_actuators:
            blank = {
                s: ActuatorDesire() for s in sorted(self._touched_actuators)
            }
            proxy.set_actuators(blank, send=False)
        for slot in sorted(self._touched_servos):
            proxy.set_servo(int(slot), ServoDesire(servo_id=0), send=False)
        if self._touched_led:
            from deft_controls_sdk.link import LedDesire

            proxy.set_led(LedDesire(mode="follow", master_brightness=0), send=False)
        if self._touched_actuators or self._touched_servos or self._touched_led:
            proxy.commit()
        self._pending.clear()
        self._touched_actuators.clear()
        self._touched_servos.clear()
        self._touched_led = False

    def discard(self) -> None:
        """Drop pending mounts without applying or blanking the plant."""
        self._pending.clear()


__all__ = ["Actions"]
