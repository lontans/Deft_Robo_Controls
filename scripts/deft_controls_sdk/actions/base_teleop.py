"""Base wheel-module ``SlotSpec`` helpers.

Design choice: delegate to (not fork/move) ``build_actuator_specs``'s existing base
branch, then filter. The bench-vs-product base construction logic (``BASE_BENCH_ROWS``
vs ``BASE_STEER_SLOTS``/``BASE_DRIVE_SLOTS`` + ``wheel_module`` lookup) already lives
there and is exercised by existing tests/callers; duplicating or relocating it here would
risk the two views drifting apart. This module is just the "give me only the base/wheel
slots" view a base-teleop caller wants.
"""
from __future__ import annotations

from typing import Dict

from .teleop import SlotSpec, build_actuator_specs


def build_base_wheel_specs(cfg_map: str = "bench") -> Dict[int, SlotSpec]:
    """Base-only view of :func:`build_actuator_specs` — filters to ``group == "base"``.

    On ``cfg_map="bench"`` this is the 4 ``BASE_BENCH_ROWS`` slots (all ``role="drive"``,
    no ``wheel_module``). On ``cfg_map="product"`` (anything else) this is the 6 product
    base slots (``role="steer"``/``"drive"``, each with its ``wheel_module``).
    """
    specs = build_actuator_specs(cfg_map)
    return {slot: spec for slot, spec in specs.items() if spec.group == "base"}


__all__ = ["build_base_wheel_specs"]
