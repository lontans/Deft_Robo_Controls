#!/usr/bin/env python3
"""Deprecated shim — use ``python vbeta_smoke.py base …``."""
from vbeta_smoke_lib import base_main

if __name__ == "__main__":
    raise SystemExit(base_main())
