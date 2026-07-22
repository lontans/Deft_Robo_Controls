#!/usr/bin/env python3
"""One-shot USB soft-DFU flash entrypoint (Windows / Linux / Jetson).

    python scripts/soft_dfu_flash.py
    python scripts/soft_dfu_flash.py --serial 3167376F3435
    python scripts/soft_dfu_flash.py --image Debug/foo.elf
"""
from __future__ import annotations

import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.bench.soft_dfu import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main(["flash", *sys.argv[1:]]))
