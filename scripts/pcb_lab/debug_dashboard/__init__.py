"""``python -m pcb_lab.debug_dashboard`` — localhost plant + telemetry UI."""

from __future__ import annotations

__all__ = ["main"]


def main(argv: list[str] | None = None) -> int:
    from pcb_lab.debug_dashboard.__main__ import main as _main

    return int(_main(argv))
