"""pcb_lab.debug — scan ports / show status / set CFG (NVM).

    python -m pcb_lab.debug scan
    python -m pcb_lab.debug --port COM5 show --cfg --bandwidth --status
    python -m pcb_lab.debug --port COM5 show --pcb
    python -m pcb_lab.debug --port COM5 set --cfg
    python -m pcb_lab.debug --port COM5 set --cfg --slot 22 --bus 5 \\
        --protocol robstride --motor-id 0x70 --persist
    python -m pcb_lab.debug_dashboard --port COM5
"""
from __future__ import annotations

from pcb_lab.debug.cli import main

__all__ = ["main"]
