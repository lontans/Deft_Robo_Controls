"""Deprecated alias — use ``python -m deft_controls_sdk.debug_dashboard``.

pcb_lab owns board USB/flash + ``pcb_lab.debug {show|set|test}`` only.
"""
from __future__ import annotations

import sys
import warnings


def main(argv: list[str] | None = None) -> int:
    warnings.warn(
        "pcb_lab.debug_dashboard is deprecated; "
        "use python -m deft_controls_sdk.debug_dashboard",
        DeprecationWarning,
        stacklevel=2,
    )
    print(
        "DEPRECATED: use  python -m deft_controls_sdk.debug_dashboard\n"
        "pcb_lab surface: python -m pcb_lab | python -m pcb_lab.debug {show|set|test}",
        file=sys.stderr,
    )
    from deft_controls_sdk.debug_dashboard.__main__ import main as _main

    return int(_main(argv))


if __name__ == "__main__":
    raise SystemExit(main())
