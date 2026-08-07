"""Localhost controller + telemetry dashboard — thin entry point.

Run:

    python -m deft_controls_sdk.debug_dashboard
    # browser: http://127.0.0.1:8766  -> pick a port, click Connect

    python -m deft_controls_sdk.debug_dashboard --port COM5
    # same UI, auto-connects at launch

Connect opens ``mode="debug"`` and enters **Active** frozen (``soft_kill=True``
— hold whatever is currently held, at full torque; NORMAL + plant_apply=1;
nothing is blanked/de-energized). The operator must explicitly release
soft_kill before any motion command (Apply / teleop target / jog) is
accepted. Idle / Stop / Hard-ESTOP-park remain available regardless — see
``state.py``'s module docstring for the full Follow/Active x soft_kill state
machine.

HTTP defaults to **8766** so it does not collide with ``pdb_uart_sim.py
--control-port 8765``.

The real implementation lives in :mod:`state` (``AppState``) and
:mod:`routes` (HTTP route table, ``wire_panels()``, ``serve()``) — this
module only re-exports the two names ``__main__.py`` (and existing callers/
tests) import, so ``from deft_controls_sdk.debug_dashboard.app import
AppState, serve`` keeps working unchanged.
"""
from __future__ import annotations

from .routes import serve
from .state import AppState

__all__ = ["AppState", "serve"]
