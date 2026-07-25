#!/usr/bin/env python3
"""Deprecated shim — use ``python vbeta_smoke.py arm …``."""
from vbeta_smoke_lib import arm_main

if __name__ == "__main__":
    raise SystemExit(arm_main())
