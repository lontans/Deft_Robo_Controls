"""Plant firmware / CFG / LED debug suite (HostProxy + NVM).

Canonical entry::

    python -m deft_controls_sdk.debug.suite [--port COM5] scan|show|set|test …

Lab alias (identical argv)::

    python -m pcb_lab.debug …

``test`` domains own connect/mode (bandwidth vs debug). Suite tests must not
import ``vbeta.*`` — reuse suite presets / pcb_tui / hub.debug / debug.metrics.
"""
from __future__ import annotations

from .cli import main
from .pcb_tui import run_pcb_dashboard
from .proto import parse_protocol, protocol_name
from .show import (
    collect_bandwidth,
    collect_cfg,
    collect_status,
    format_banner,
    format_cfg_table,
    terminal_cols,
)

__all__ = [
    "main",
    "run_pcb_dashboard",
    "parse_protocol",
    "protocol_name",
    "collect_cfg",
    "collect_bandwidth",
    "collect_status",
    "format_cfg_table",
    "format_banner",
    "terminal_cols",
]
