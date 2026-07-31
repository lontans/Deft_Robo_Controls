"""Plant firmware / CFG / LED debug suite (HostProxy + NVM).

Canonical entry::

    python -m deft_controls_sdk.debug.suite [--port COM5] show|set|test …
    python -m pcb_lab.debug …   # identical CLI alias

Programmatic (notebooks): ``HostProxy`` + ``deft_controls_sdk.debug`` /
``actions`` — not this package. Suite is interactive CLI/TUI only.

``test`` domains own connect/mode (bandwidth vs debug). Suite uses
``config`` / ``actions`` / ``HostProxy`` only — no product-driver package.
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
