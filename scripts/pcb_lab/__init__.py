"""pcb_lab — plant lab app on HostProxy (not the YAM/vbeta product path).

    python -m pcb_lab doctor --port COM5
    python -m pcb_lab hold --component left_arm --port COM5
    python -m pcb_lab step --component left_arm --joint 0 --delta 0.05
    python -m pcb_lab blank --component left_arm

Owns COM exclusively — disconnect the debug dashboard first.
"""
from __future__ import annotations

from pcb_lab.lab import LabRobot, main

__all__ = ["LabRobot", "main"]
