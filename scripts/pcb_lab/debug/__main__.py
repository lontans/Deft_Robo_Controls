"""python -m pcb_lab.debug … — alias for deft_controls_sdk.debug.suite."""
from __future__ import annotations

from deft_controls_sdk.debug.suite import main

if __name__ == "__main__":
    raise SystemExit(main())
