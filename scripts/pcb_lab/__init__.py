"""pcb_lab — plant lab app on HostProxy (not the YAM/vbeta product path).

    python -m pcb_lab --port COM5 doctor
    python -m pcb_lab --port COM5 demux --profile bench
    python -m pcb_lab --port COM5 hold --component left_arm
    python -m pcb_lab.continuous --port COM5 --duration 20
    python -m pcb_lab.debug scan
    python -m pcb_lab.debug --port COM5 show --cfg --bandwidth --status
    python -m pcb_lab.debug --port COM5 set --cfg
    python -m pcb_lab.debug --port COM5 set --cfg --slot 22 --bus 5 \\
        --protocol robstride --motor-id 0x70 --persist

Owns COM exclusively — disconnect the debug dashboard first.
"""
from __future__ import annotations

from pcb_lab.lab import LabRobot, main

__all__ = ["LabRobot", "main"]
