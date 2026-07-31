"""pcb_lab.debug — CLI alias of ``deft_controls_sdk.debug.suite``.

Only intended entry::

    python -m pcb_lab.debug {show|set|test}

For notebooks / scripts import the SDK directly::

    from deft_controls_sdk import HostProxy
    from deft_controls_sdk.debug import collect_cfg, run_inventory
    from deft_controls_sdk.debug.suite.cli import _build_parser  # tests only
"""
from __future__ import annotations

from deft_controls_sdk.debug.suite import main

__all__ = ["main"]
