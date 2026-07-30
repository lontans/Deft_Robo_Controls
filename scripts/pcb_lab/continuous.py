"""Continuous cruise entry — HostProxy demux + legacy cruise loop.

Living surface (tracked). Full teleop/cruise logic still lives in
``pcb_lab/legacy/yam_continuous_all.py`` (gitignored local / Jetson sync).
That script already owns COM via ``PcbRobotSession`` → ``HostProxy``; this
module selects ``bench_continuous_profile()`` (base = spare slots 22–25)
before handing off.

Demux reminder (see ``host_proxy`` docstring):
  Profile  → which slots a component name means (host, at connect)
  CFG      → bus/protocol/motor_id per slot (MCU; continuous writes BASE_ROWS)

Explore without motion::

    python -m pcb_lab --port COM5 demux --profile bench
    python -m pcb_lab.continuous --help

Run cruise (dashboard must release COM)::

    python -m pcb_lab.continuous --port COM5 --duration 20
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import List, Optional, Sequence


_LEGACY = Path(__file__).resolve().parent / "legacy" / "yam_continuous_all.py"


def _load_legacy_main():
    if not _LEGACY.is_file():
        raise FileNotFoundError(
            f"missing {_LEGACY}\n"
            "Restore pcb_lab/legacy/yam_continuous_all.py locally "
            "(gitignored) before running continuous."
        )
    spec = importlib.util.spec_from_file_location("yam_continuous_all", _LEGACY)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {_LEGACY}")
    mod = importlib.util.module_from_spec(spec)
    # Legacy expects scripts/ and itself on path for sibling imports.
    scripts = Path(__file__).resolve().parent.parent
    legacy_dir = _LEGACY.parent
    for p in (str(scripts), str(legacy_dir)):
        if p not in sys.path:
            sys.path.insert(0, p)
    sys.modules["yam_continuous_all"] = mod
    spec.loader.exec_module(mod)
    return mod.main


def main(argv: Optional[Sequence[str]] = None) -> int:
    print(
        "continuous → HostProxy(bench_continuous_profile) → "
        "legacy yam_continuous_all\n"
        "  profile.base = slots 22-25 (CFG IDs from BASE_ROWS)\n"
        "  explore: python -m pcb_lab --port COM5 demux --profile bench",
        flush=True,
    )
    cont_main = _load_legacy_main()
    return int(cont_main(list(argv) if argv is not None else None))


if __name__ == "__main__":
    raise SystemExit(main())
