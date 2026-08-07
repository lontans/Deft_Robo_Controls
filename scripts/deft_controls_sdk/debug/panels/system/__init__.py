"""System panels — thin, testable wrappers over existing SDK primitives.

Every panel function takes a ``HostProxy`` (or, for :func:`run_bandwidth_panel`,
a bare COM ``port`` string — bandwidth needs its own exclusive ``mode="bandwidth"``
connection) and returns a plain JSON-serializable ``dict``. No new business
logic lives here; see ``system_panel.py`` and ``led_panel.py`` docstrings for
the primitives each function wraps.
"""
from __future__ import annotations

from .led_panel import (
    apply_led_preset_panel,
    set_led_panel,
    set_listen_pdu_panel,
)
from .system_panel import (
    host_link_eval_panel,
    run_bandwidth_panel,
    run_inventory_panel,
    save_cfg_nvm_panel,
    set_cfg_periph_panel,
    set_cfg_slot_panel,
    show_cfg_panel,
)

__all__ = [
    "apply_led_preset_panel",
    "host_link_eval_panel",
    "run_bandwidth_panel",
    "run_inventory_panel",
    "save_cfg_nvm_panel",
    "set_cfg_periph_panel",
    "set_cfg_slot_panel",
    "set_led_panel",
    "set_listen_pdu_panel",
    "show_cfg_panel",
]
