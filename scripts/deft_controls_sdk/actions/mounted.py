"""MountedAction — one staged plant patch (not yet applied to the wire)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from deft_controls_sdk.link import ActuatorDesire, LedDesire, ServoDesire


@dataclass
class MountedAction:
    """A batchable desire patch produced by Actuator/Servo/Led helpers."""

    kind: str  # "actuator" | "servo" | "led"
    label: str
    actuators: Dict[int, ActuatorDesire] = field(default_factory=dict)
    servos: Dict[int, ServoDesire] = field(default_factory=dict)
    led: Optional[LedDesire] = None
    meta: Dict[str, Any] = field(default_factory=dict)

    @property
    def actuator_slots(self) -> Tuple[int, ...]:
        return tuple(sorted(self.actuators))

    @property
    def servo_slots(self) -> Tuple[int, ...]:
        return tuple(sorted(self.servos))

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly summary for notebook inspection."""
        out: Dict[str, Any] = {"kind": self.kind, "label": self.label}
        if self.actuators:
            out["actuators"] = {
                int(s): {
                    "position": float(d.position),
                    "kp": float(d.kp),
                    "kd": float(d.kd),
                    "torque": float(d.torque),
                }
                for s, d in sorted(self.actuators.items())
            }
        if self.servos:
            out["servos"] = {
                int(s): {
                    "servo_id": int(d.servo_id),
                    "position": int(d.native_step_position),
                    "torque_enable": bool(d.torque_enable),
                }
                for s, d in sorted(self.servos.items())
            }
        if self.led is not None:
            out["led"] = {
                "mode": self.led.mode,
                "pattern": int(getattr(self.led, "pattern", 0) or 0),
                "brightness": int(self.led.master_brightness),
            }
        if self.meta:
            out["meta"] = dict(self.meta)
        return out

    @classmethod
    def from_actuators(
        cls,
        label: str,
        desires: Dict[int, ActuatorDesire],
        *,
        meta: Optional[Dict[str, Any]] = None,
    ) -> "MountedAction":
        return cls(
            kind="actuator",
            label=label,
            actuators={int(s): d for s, d in desires.items()},
            meta=dict(meta or {}),
        )

    @classmethod
    def from_servos(
        cls,
        label: str,
        desires: Dict[int, ServoDesire],
        *,
        meta: Optional[Dict[str, Any]] = None,
    ) -> "MountedAction":
        return cls(
            kind="servo",
            label=label,
            servos={int(s): d for s, d in desires.items()},
            meta=dict(meta or {}),
        )

    @classmethod
    def from_led(
        cls,
        label: str,
        desire: LedDesire,
        *,
        meta: Optional[Dict[str, Any]] = None,
    ) -> "MountedAction":
        return cls(kind="led", label=label, led=desire, meta=dict(meta or {}))


def merge_mounted(items: List[MountedAction]) -> MountedAction:
    """Collapse a pending list into one patch (later mounts win per slot)."""
    actuators: Dict[int, ActuatorDesire] = {}
    servos: Dict[int, ServoDesire] = {}
    led: Optional[LedDesire] = None
    labels: List[str] = []
    for item in items:
        labels.append(item.label)
        actuators.update(item.actuators)
        servos.update(item.servos)
        if item.led is not None:
            led = item.led
    return MountedAction(
        kind="batch",
        label="+".join(labels) if labels else "empty",
        actuators=actuators,
        servos=servos,
        led=led,
    )


__all__ = ["MountedAction", "merge_mounted"]
