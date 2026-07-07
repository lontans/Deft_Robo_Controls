#!/usr/bin/env python3
"""Unified entrypoint: python scripts/controls_pcb_host.py --help"""
from __future__ import annotations

import os
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from controls_pcb_host.cli.main import main

if __name__ == "__main__":
    raise SystemExit(main())
