"""Left-arm motor-frame clear envelope (teleop min/max capture 2026-07-24)."""
from __future__ import annotations

from typing import Optional, Tuple

CLEAR_ACTIVE = True

CLEAR_LO: Tuple[float, ...] = (
    -1.4621,
    -4.5929,
    1.1277,
    -0.9361,
    -1.4037,
    -1.4190,
    1.2562,
)
CLEAR_HI: Tuple[float, ...] = (
    1.0345,
    -2.6826,
    3.0601,
    1.3488,
    1.0597,
    0.1792,
    2.6456,
)
HOME_Q: Tuple[float, ...] = (
    0.0521,
    -3.0577,
    3.0230,
    -0.9773,
    -0.0021,
    -1.1155,
    2.7229,
)
SOURCE = (
    "bench left CH1 teleop-minmax 2026-07-24 inset=0.08 "
    "(J2 fault on fast drop; matrix kept)"
)
INSET_RAD = 0.08


def clear_q7() -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    if not CLEAR_ACTIVE:
        return None
    return tuple(CLEAR_LO), tuple(CLEAR_HI)
