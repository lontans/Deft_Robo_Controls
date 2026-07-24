#!/usr/bin/env python3
"""Exclusive COM: YAM one-arm hold / jog smoke via PcbArmDriver (soft-clamped).

See ``vbeta_smoke_lib.py`` for --hold / --jog / Jetson port notes.
Do not run on hardware until the arm rig HW gate opens.
"""
from vbeta_smoke_lib import arm_main

if __name__ == "__main__":
    raise SystemExit(arm_main())
