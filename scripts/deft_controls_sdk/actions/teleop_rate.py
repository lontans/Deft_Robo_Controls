"""Convenience wrapper around :meth:`TeleopEngine.set_actuator_rate`.

Kept small on purpose — this is plumbing for the future HTTP/UI layer (slot/spec lookup
+ seed), not new teleop logic. The real primitive lives on ``TeleopEngine`` itself.
"""
from __future__ import annotations

from typing import Callable, Dict

from .teleop import SlotSpec, TeleopEngine


def rate_teleop_actuator(
    engine: TeleopEngine,
    slot: int,
    *,
    specs: Dict[int, SlotSpec],
    seed_fn: Callable[[int], float],
    rate: float,
    timeout_s: float = 0.25,
) -> None:
    """Look up ``slot``'s :class:`SlotSpec`, seed from ``seed_fn(slot)`` (current
    position), and engage/refresh rate-mode teleop on ``engine``.

    Raises ``ValueError`` if ``slot`` isn't in ``specs`` at all — that's a genuine caller
    bug, there's nothing to rate-teleop.

    Gating choice: this does *not* additionally raise when ``spec.role == "position"``.
    ``role`` is a UI-facing hint (arm/neck joints are ``"position"``, base slots are
    ``"steer"``/``"drive"``), not a mechanical restriction — ``set_actuator_rate`` clamps
    to ``[lo, hi]`` exactly like target-mode already does, so rate-teleporting a
    ``"position"`` slot (e.g. a slow rate-nudge on an arm joint) is not unsafe, just an
    unusual UI choice. Blocking it here would take away a legitimate use without adding
    real protection. Instead, matching how ``jog_actuator`` vs ``engage_actuator`` gating
    already lives at the caller layer today (not inside ``build_actuator_specs``/
    ``SlotSpec``), the HTTP/UI layer is expected to only offer rate controls for the
    slots it wants continuous-teleop for (typically ``"drive"``, optionally ``"steer"``).
    """
    spec = specs.get(slot)
    if spec is None:
        raise ValueError(f"slot {slot} not in specs")
    seed = seed_fn(slot)
    engine.set_actuator_rate(slot, spec=spec, seed=seed, rate=rate, timeout_s=timeout_s)


__all__ = ["rate_teleop_actuator"]
