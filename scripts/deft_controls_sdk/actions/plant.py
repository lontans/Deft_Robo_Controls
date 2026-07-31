"""PlantAction — shared base for plant-mode behaviour (not debug RPC)."""
from __future__ import annotations

from typing import Any


class PlantAction:
    """Thin base for ActuatorAction / ServoAction / LedAction / PduLinkAction."""

    def __init__(self, sink: Any) -> None:
        self._sink = sink

    @property
    def sink(self) -> Any:
        return self._sink


__all__ = ["PlantAction"]
