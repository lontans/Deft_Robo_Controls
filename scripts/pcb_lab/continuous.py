"""Deprecated — product cruise is not part of pcb_lab.

Board prove: ``python -m pcb_lab`` / ``python -m pcb_lab.debug test``.
Controls cruise / teleop: ``deft_controls_sdk.actions`` + notebooks / dashboard.

Legacy handoff (gitignored local script) remains loadable if present, but this
entry is no longer wired into ``python -m pcb_lab``.
"""
from __future__ import annotations

import importlib.util
import sys
import warnings
from pathlib import Path
from typing import Optional, Sequence


_LEGACY = Path(__file__).resolve().parent / "legacy" / "yam_continuous_all.py"


def _load_legacy_main():
    if not _LEGACY.is_file():
        raise FileNotFoundError(
            f"missing {_LEGACY}\n"
            "continuous is deprecated for pcb_lab. "
            "Use pcb_lab.debug test for HW prove, or controls scripts for cruise."
        )
    spec = importlib.util.spec_from_file_location("yam_continuous_all", _LEGACY)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {_LEGACY}")
    mod = importlib.util.module_from_spec(spec)
    scripts = Path(__file__).resolve().parent.parent
    legacy_dir = _LEGACY.parent
    for p in (str(scripts), str(legacy_dir)):
        if p not in sys.path:
            sys.path.insert(0, p)
    sys.modules["yam_continuous_all"] = mod
    spec.loader.exec_module(mod)
    return mod.main


def main(argv: Optional[Sequence[str]] = None) -> int:
    warnings.warn(
        "pcb_lab.continuous is deprecated — not part of the board-verify surface. "
        "Use python -m pcb_lab.debug test (HW) or deft_controls_sdk.actions (controls).",
        DeprecationWarning,
        stacklevel=2,
    )
    print(
        "DEPRECATED: pcb_lab.continuous is not a pcb_lab entrypoint.\n"
        "  board:  python -m pcb_lab / python -m pcb_lab.debug test\n"
        "  cruise: deft_controls_sdk.actions (TeleopEngine / spin_jog)\n",
        flush=True,
    )
    cont_main = _load_legacy_main()
    return int(cont_main(list(argv) if argv is not None else None))


if __name__ == "__main__":
    raise SystemExit(main())
