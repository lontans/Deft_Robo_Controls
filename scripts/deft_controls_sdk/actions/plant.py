"""PlantAction — shared base for plant-mode behaviour (not debug RPC)."""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from .facade import Actions


class PlantAction:
    """Thin base for ActuatorAction / ServoAction / LedAction / PduLinkAction."""

    def __init__(self, sink: Any) -> None:
        self._sink = sink
        self._actions: Optional["Actions"] = None

    @property
    def sink(self) -> Any:
        return self._sink

    def bind_actions(self, actions: "Actions") -> None:
        """Link this helper to an ``Actions`` batch (mount/apply workflow)."""
        self._actions = actions

    @property
    def actions(self) -> Optional["Actions"]:
        return self._actions


__all__ = ["PlantAction"]
