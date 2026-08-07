"""Shared helpers for talk-through demos (not a public SDK API)."""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional, Sequence


def _ensure_scripts_on_path() -> None:
    here = os.path.abspath(os.path.dirname(__file__))
    scripts = os.path.abspath(os.path.join(here, ".."))
    if scripts not in sys.path:
        sys.path.insert(0, scripts)


_ensure_scripts_on_path()


def step(n: int, title: str, detail: str = "") -> None:
    print()
    print(f"[{n}] {title}")
    if detail:
        print(f"    {detail}")


def pause(seconds: float, *, why: str = "") -> None:
    if why:
        print(f"    … {why} ({seconds:g}s)")
    time.sleep(float(seconds))


def add_port_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--port",
        default=None,
        help="CDC port (COM5 / /dev/ttyACM0). Omit to auto-pick STM32 0483:5740.",
    )
    p.add_argument(
        "--serial",
        default=None,
        help="USB serial if multiple boards are present",
    )


def connect_kwargs(args: argparse.Namespace) -> dict:
    return {"port": args.port, "serial": args.serial}


def say_done(name: str) -> None:
    print()
    print(f"OK — {name} finished; CDC released (with-block closed).")


def require_one_owner() -> None:
    print(
        "Note: only one process may own the CDC "
        "(close dashboard / pcb_lab.debug / other HostProxy first)."
    )


def summarize_section_fb(section: str, fb: Sequence[object], slots: Sequence[int]) -> None:
    live = sum(1 for x in fb if x is not None)
    print(f"    {section}: feedback {live}/{len(slots)}  slots={tuple(slots)}")


def optional_float(value: Optional[str]) -> Optional[float]:
    if value is None:
        return None
    return float(value)
