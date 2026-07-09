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
            if ch2 == b"H":
                return "up"
            if ch2 == b"P":
                return "down"
            return None
        try:
            c = ch.decode("utf-8", errors="ignore").lower()
        except Exception:
            return None
        if c in allowed:
            return c
        return None

    def poll_arrow_keys_raw() -> Tuple[bool, bool]:
        import ctypes

        user32 = ctypes.windll.user32
        left = bool(user32.GetAsyncKeyState(0x25) & 0x8000)
        right = bool(user32.GetAsyncKeyState(0x27) & 0x8000)
        return left, right

    def poll_arrow_direction() -> int:
        left, right = poll_arrow_keys_raw()
        if left and not right:
            return -1
        if right and not left:
            return 1
        return 0

    def poll_vertical_arrow() -> int:
        import ctypes

        user32 = ctypes.windll.user32
        up = bool(user32.GetAsyncKeyState(0x26) & 0x8000)
        down = bool(user32.GetAsyncKeyState(0x28) & 0x8000)
        if up and not down:
            return 1
        if down and not up:
            return -1
        return 0

    _vup_prev = False
    _vdown_prev = False

    def poll_vertical_arrow_pressed() -> int:
        """+1 on up press, -1 on down press, 0 otherwise (edge-triggered)."""
        global _vup_prev, _vdown_prev
        import ctypes

        user32 = ctypes.windll.user32
        up = bool(user32.GetAsyncKeyState(0x26) & 0x8000)
        down = bool(user32.GetAsyncKeyState(0x28) & 0x8000)
        edge = 0
        if up and not _vup_prev and not down:
            edge = 1
        elif down and not _vdown_prev and not up:
            edge = -1
        _vup_prev = up
        _vdown_prev = down
        return edge

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

    def poll_arrow_keys_raw() -> Tuple[bool, bool]:
        key = poll_key_nonblocking()
        return key in ("left", "l"), key in ("right", "o")

    def poll_arrow_direction() -> int:
        left, right = poll_arrow_keys_raw()
        if left and not right:
            return -1
        if right and not left:
            return 1
        return 0

    def poll_vertical_arrow() -> int:
        key = poll_key_nonblocking()
        if key == "up":
            return 1
        if key == "down":
            return -1
        return 0

    def poll_vertical_arrow_pressed() -> int:
        return poll_vertical_arrow()
