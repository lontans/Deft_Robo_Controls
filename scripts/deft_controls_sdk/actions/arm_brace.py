"""Left-arm full-image brace writer (continuous / mouse teleop parity).

Dashboard single-slot teleop used to rewrite only the engaged joint and leave
siblings at idle-anchor (kp=0) — that sags under load. Continuous
``_write_plant`` braces all 7 every tick with ENGAGE_KP / J2 boost + optional
gravity FF. This module is that plant write, shared by ``TeleopEngine``.
"""
from __future__ import annotations

from typing import Dict, Optional, Sequence, TYPE_CHECKING

from deft_controls_sdk.config.actuator import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    LEFT_ARM_SLOTS,
)
from deft_controls_sdk.link import ActuatorDesire

if TYPE_CHECKING:
    from .gravity_comp import GravityComp

# Mouse / continuous teleop schedule (pcb_lab/legacy/mouse_arm_teleop.py).
ENGAGE_KP = 0.85
J2_KP_SCALE = 1.40
J2_INDEX = 1
MAX_LEAD = 0.25  # rad — command vs FB lead clamp
J2_MAX_LEAD = 0.32


def lead_clamp(cmd: float, fb: float, *, j2: bool = False) -> float:
    lead = J2_MAX_LEAD if j2 else MAX_LEAD
    d = float(cmd) - float(fb)
    if abs(d) > lead:
        return float(fb) + (lead if d > 0 else -lead)
    return float(cmd)


def write_left_arm(
    hub: object,
    q7: Sequence[float],
    *,
    dq7: Optional[Sequence[float]] = None,
    kp_scale: float = ENGAGE_KP,
    j2_kp_scale: float = J2_KP_SCALE,
    kd: Sequence[float] = DEFAULT_ARM_KD,
    gravity_comp: Optional["GravityComp"] = None,
    send: bool = False,
) -> Dict[int, ActuatorDesire]:
    """Brace all left-arm slots. Never leave siblings blank/kp=0.

    ``kp_scale`` is clipped to [0, 1] except J2, which uses ``j2_kp_scale``
    (may be >1 for lift authority — same as continuous ``_write_plant``).
    """
    if len(q7) != 7:
        raise ValueError(f"expected 7 arm positions, got {len(q7)}")
    dq = list(dq7) if dq7 is not None else [0.0] * 7
    if len(dq) != 7:
        raise ValueError(f"expected 7 arm velocities, got {len(dq)}")

    scale = float(max(0.0, min(1.0, kp_scale)))
    torque = [0.0] * 7
    if gravity_comp is not None:
        try:
            t6 = gravity_comp.compute(q7[:6])
            for i in range(6):
                torque[i] = float(t6[i])
        except Exception:
            pass

    batch: Dict[int, ActuatorDesire] = {}
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        s = float(j2_kp_scale) if i == J2_INDEX else scale
        batch[int(slot)] = ActuatorDesire(
            position=float(q7[i]),
            velocity=float(dq[i]),
            kp=float(DEFAULT_ARM_KP[i]) * s,
            kd=float(kd[i]),
            torque=float(torque[i]),
        )
    hub.set_actuators(batch, send=send)
    return batch


__all__ = [
    "ENGAGE_KP",
    "J2_INDEX",
    "J2_KP_SCALE",
    "J2_MAX_LEAD",
    "MAX_LEAD",
    "lead_clamp",
    "write_left_arm",
]
