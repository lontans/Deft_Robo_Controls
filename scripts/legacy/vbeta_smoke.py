#!/usr/bin/env python3
"""Unified vbeta HW smoke CLI (arm / base / neck-led).

    python vbeta_smoke.py arm --side left --hold --hold-s 3
    python vbeta_smoke.py arm --side left --jog --joint 0 --delta 0.05
    python vbeta_smoke.py base --creep 0.2
    python vbeta_smoke.py neck --hold-s 2

Owns COM exclusively — disconnect the debug dashboard first.
"""
from __future__ import annotations

import sys

from vbeta_smoke_lib import arm_main, base_main, neck_led_main


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    cmd = argv[0].strip().lower()
    rest = argv[1:]
    if cmd in ("arm", "a"):
        return arm_main(rest)
    if cmd in ("base", "b"):
        return base_main(rest)
    if cmd in ("neck", "neck-led", "led", "n"):
        return neck_led_main(rest)
    print(f"unknown smoke target {cmd!r}; use arm|base|neck", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
