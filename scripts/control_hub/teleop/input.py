"""Non-blocking terminal input for teleop."""
from __future__ import annotations

import sys
from typing import Optional, Tuple

if sys.platform == "win32":
    import msvcrt

    def poll_key_nonblocking(extra: Optional[Tuple[str, ...]] = None) -> Optional[str]:
        allowed = frozenset(("q", "r"))
        if extra:
            allowed = allowed | frozenset(extra)
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):
            ch2 = msvcrt.getch()
            if ch2 == b"K":
                return "left"
            if ch2 == b"M":
                return "right"
            return None
        try:
            c = ch.decode("utf-8", errors="ignore").lower()
        except Exception:
            return None
        if c in allowed:
            return c
        return None

    def poll_arrow_direction() -> int:
        import ctypes

        user32 = ctypes.windll.user32
        left = bool(user32.GetAsyncKeyState(0x25) & 0x8000)
        right = bool(user32.GetAsyncKeyState(0x27) & 0x8000)
        if left and not right:
            return -1
        if right and not left:
            return 1
        return 0

else:
    import select
    import termios
    import tty

    def poll_key_nonblocking(extra: Optional[Tuple[str, ...]] = None) -> Optional[str]:
        allowed = frozenset(("q", "r", "left", "right", "l", "o"))
        if extra:
            allowed = allowed | frozenset(extra)
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            low = ch.lower()
            if low in allowed:
                return low
        return None

    def poll_arrow_direction() -> int:
        key = poll_key_nonblocking()
        if key in ("left", "l"):
            return -1
        if key in ("right", "o"):
            return 1
        return 0
