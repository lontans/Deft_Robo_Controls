"""Deprecated — Assembly workshop removed from bare ``test``.

Bare ``pcb_lab.debug test`` now runs :func:`board_verify.run_board_verify`.
Assemblies / profiles stay in ``deft_controls_sdk.config`` for controls.
Product arm teleop stays in ``actions`` / controls Jupyter — not pcb_lab.
"""
from __future__ import annotations

import argparse
import warnings

from .board_verify import run_board_verify


def run_assembly_workshop(args: argparse.Namespace) -> int:
    warnings.warn(
        "run_assembly_workshop is deprecated; use board_verify.run_board_verify "
        "(bare pcb_lab.debug test).",
        DeprecationWarning,
        stacklevel=2,
    )
    return run_board_verify(args)


__all__ = ["run_assembly_workshop"]
