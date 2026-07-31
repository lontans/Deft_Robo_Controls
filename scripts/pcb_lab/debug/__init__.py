"""pcb_lab.debug — thin lab alias for ``deft_controls_sdk.debug.suite``.

    python -m pcb_lab.debug scan
    python -m pcb_lab.debug --port COM5 show --cfg
    python -m pcb_lab.debug --port COM5 show --pcb
    python -m pcb_lab.debug --port COM5 set --cfg
    python -m pcb_lab.debug test                 # Assembly workshop
    python -m pcb_lab.debug test --bandwidth
    python -m pcb_lab.debug test --actuators|--led|--servo|--pdu-link

Implementation lives in ``deft_controls_sdk.debug.suite``.
"""
from __future__ import annotations

from deft_controls_sdk.debug.suite import main
from deft_controls_sdk.debug.suite import pcb_tui as pcb_tui
from deft_controls_sdk.debug.suite import presets as presets
from deft_controls_sdk.debug.suite import proto as proto
from deft_controls_sdk.debug.suite import show as show
from deft_controls_sdk.debug.suite.cli import _build_parser, main as cli_main
from deft_controls_sdk.debug.suite.cfg_editor import run_cfg_editor
from deft_controls_sdk.debug.suite.pcb_tui import run_pcb_dashboard
from deft_controls_sdk.debug.suite.proto import parse_protocol, protocol_name
from deft_controls_sdk.debug.suite.show import (
    collect_bandwidth,
    collect_cfg,
    collect_status,
    format_banner,
    format_cfg_table,
    terminal_cols,
)

# Keep ``from pcb_lab.debug.cli import …`` working for lab.py / tests.
from deft_controls_sdk.debug.suite import cli as cli  # noqa: F401

__all__ = [
    "main",
    "cli_main",
    "cli",
    "_build_parser",
    "run_cfg_editor",
    "run_pcb_dashboard",
    "parse_protocol",
    "protocol_name",
    "collect_cfg",
    "collect_bandwidth",
    "collect_status",
    "format_cfg_table",
    "format_banner",
    "terminal_cols",
    "proto",
    "show",
    "presets",
    "pcb_tui",
]
