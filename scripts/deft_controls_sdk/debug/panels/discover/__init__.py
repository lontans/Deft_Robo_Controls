"""Dashboard-facing "panel" wrappers around existing discover / calibrate SDK
primitives (bench debug ops).

Each panel function follows the shared dashboard contract::

    def action_name(proxy: HostProxy, **params) -> dict: ...

- First positional arg is always a ``HostProxy``.
- Return is a plain JSON-serializable ``dict``.
- Blocking is fine — discover / calibrate are inherently slow bench ops.
- Failures raise plain ``Exception``/``ValueError``/``RuntimeError`` with a
  human-readable message; callers (the HTTP layer) are expected to catch and
  translate to an error response.
"""
from __future__ import annotations

from .calibrate_panel import calibrate_robstride_panel
from .discover_panel import discover_panel

__all__ = [
    "calibrate_robstride_panel",
    "discover_panel",
]
