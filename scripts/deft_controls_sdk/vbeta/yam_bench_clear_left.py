"""Left-arm motor-frame clear envelope for the current bench (bus 1 / slots 0–6).

Filled by ``scripts/yam_arm_clear_range.py`` after operator-supervised sweeps.
Until ``CLEAR_ACTIVE`` is True, ``yam_limits`` ignores this module.
"""
from __future__ import annotations

from typing import Optional, Tuple

# Set True when CLEAR_LO / CLEAR_HI are real measured values.
CLEAR_ACTIVE = False

# Motor-frame rad, arm-local J1..J7 (index 0 = J1).
CLEAR_LO: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
CLEAR_HI: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
HOME_Q: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

SOURCE = "unset — run scripts/yam_arm_clear_range.py"
INSET_RAD = 0.08
STEP_RAD = 0.03


def clear_q7() -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    """Return ``(lo, hi)`` when active, else None."""
    if not CLEAR_ACTIVE:
        return None
    if len(CLEAR_LO) != 7 or len(CLEAR_HI) != 7:
        return None
    return tuple(CLEAR_LO), tuple(CLEAR_HI)
