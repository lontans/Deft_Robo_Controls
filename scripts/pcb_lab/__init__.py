"""pcb_lab — board toolkit CLI (Soft-DFU / scan / bandwidth).

    python -m pcb_lab
    python -m pcb_lab scan|status|leave|flash|images|build

Peripherals / CFG (CLI alias of ``deft_controls_sdk.debug.suite``)::

    python -m pcb_lab.debug {show|set|test}

**Programmatic API is the SDK** — do not import helpers from ``pcb_lab``::

    from deft_controls_sdk import HostProxy
    from deft_controls_sdk.actions import ActuatorAction
    from deft_controls_sdk.config import assembly_from_name
    from deft_controls_sdk.debug import as_hex, collect_cfg, run_inventory

``LabRobot`` remains a thin optional script façade over ``HostProxy``.
"""
from __future__ import annotations

from pcb_lab.lab import LabRobot, main

__all__ = ["LabRobot", "main"]
