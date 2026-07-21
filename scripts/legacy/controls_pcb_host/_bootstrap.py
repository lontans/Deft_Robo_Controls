"""Ensure `scripts/` is on sys.path for legacy module imports."""
from __future__ import annotations

import os
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def ensure_scripts_path() -> str:
    if _SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, _SCRIPTS_DIR)
    return _SCRIPTS_DIR
