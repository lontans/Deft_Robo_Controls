"""SK9822 LED strip test."""
from __future__ import annotations

import time

from .. import commands as cmd
from ..session import PcbSession

DEFAULT_HZ = 40.0


def run_test(
    session: PcbSession,
    *,
    mode: int = 0,
    brightness: int = 8,
    led_count: int = 0,
    hz: float = DEFAULT_HZ,
) -> None:
    mode = max(0, min(31, mode))
    brightness = max(0, min(31, brightness))
    led_count = max(0, min(63, led_count))
    dt = 1.0 / max(hz, 1.0)
    print(
        f"LED: mode={mode} brightness={brightness}/31 "
        f"count={led_count} @ {hz:.0f} Hz  (Ctrl+C to stop)"
    )
    try:
        while True:
            frame = cmd.build_led_command(session.next_seq(), mode, brightness, led_count)
            session.write_command(frame)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nSending mode=off...")
        session.write_command(cmd.build_led_command(session.next_seq(), 1, 0, led_count))
