#!/usr/bin/env python3
"""One-shot firmware flash (Windows / Linux / Jetson).

    python scripts/soft_dfu_flash.py
    python scripts/soft_dfu_flash.py --serial 3167376F3435
    python scripts/soft_dfu_flash.py --image Debug/foo.elf

Optional subcommands (advanced): scan | enter | leave | flash
"""
from __future__ import annotations

import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.bench.soft_dfu import main  # noqa: E402

_SUBCOMMANDS = frozenset({"flash", "enter", "leave", "scan"})


def _dispatch_argv(argv: list[str]) -> list[str]:
    """Default to ``flash`` so users run one script with optional flags only."""
    if not argv:
        return ["flash"]
    if argv[0] in _SUBCOMMANDS:
        return argv
    if argv[0] in ("-h", "--help"):
        return ["flash", "--help"]
    return ["flash", *argv]


if __name__ == "__main__":
    raise SystemExit(main(_dispatch_argv(sys.argv[1:])))
