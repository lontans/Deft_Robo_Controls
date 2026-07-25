#!/usr/bin/env python3
"""Deprecated shim — use ``python vbeta_smoke.py neck …``."""
from vbeta_smoke_lib import neck_led_main

if __name__ == "__main__":
    raise SystemExit(neck_led_main())
