"""pcb_lab — board toolkit (Soft-DFU / scan / health) + HostProxy doctor.

    python -m pcb_lab              # interactive menu
    python -m pcb_lab -h
    python -m pcb_lab scan|status|leave|flash|images|build
    python -m pcb_lab show defaults|health
    python -m pcb_lab doctor
    python -m pcb_lab.continuous --port COM5 --duration 20
    python -m pcb_lab.debug --port COM5 show --pcb

``pcb_lab.debug`` is a thin alias of ``deft_controls_sdk.debug.suite``
(always ``mode=debug``). Owns COM exclusively — disconnect the dashboard first.
"""
from __future__ import annotations

from pcb_lab.lab import LabRobot, main

__all__ = ["LabRobot", "main"]
