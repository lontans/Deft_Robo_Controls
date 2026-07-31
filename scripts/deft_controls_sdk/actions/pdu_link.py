"""PduLinkAction — listen_pdu / soft-kill / PDB status (plant policy, not MIT)."""
from __future__ import annotations

from typing import Any, Optional

from .plant import PlantAction


class PduLinkAction(PlantAction):
    """PDU kill-link host policy on a hub- or proxy-shaped sink."""

    def __init__(self, sink: Any) -> None:
        super().__init__(sink)

    @property
    def listen(self) -> bool:
        return bool(getattr(self._sink, "listen_pdu", False))

    @listen.setter
    def listen(self, enabled: bool) -> None:
        self._sink.listen_pdu = bool(enabled)

    def enable(self, enabled: bool = True) -> None:
        self.listen = bool(enabled)

    def status(self) -> Any:
        """Typed PDB status when the sink exposes ``pdb_status`` (hub), else None."""
        hub = getattr(self._sink, "hub", self._sink)
        fn = getattr(hub, "pdb_status", None)
        if fn is None:
            return None
        return fn()

    def park_if_requested(self, *, send: bool = True) -> bool:
        hub = getattr(self._sink, "hub", self._sink)
        fn = getattr(hub, "soft_kill_park_if_requested", None)
        if fn is None:
            svc = getattr(self._sink, "service_soft_kill", None)
            return bool(svc()) if svc is not None else False
        return bool(fn(send=send))


__all__ = ["PduLinkAction"]
