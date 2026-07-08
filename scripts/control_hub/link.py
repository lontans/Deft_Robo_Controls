"""USB link heal for plant / bench handoff."""
from __future__ import annotations

import time

from controls_pcb_host.feedback import parse_feedback_header
from controls_pcb_host.session import PcbSession


def heal_usb(session: PcbSession, *, rounds: int = 20) -> bool:
    session.reader.drain()
    for _ in range(rounds):
        session.send_plant()
        time.sleep(0.04)
        frame = session.reader.pop()
        while frame is not None:
            if parse_feedback_header(frame) is not None:
                print("  USB link OK (562 B plant feedback).")
                return True
            frame = session.reader.pop()
    return False


def release_stuck(session: PcbSession, *, rounds: int = 12) -> None:
    """Neutral plant frames — release bench gates without RS2 SESSION_END."""
    session.reader.drain()
    for _ in range(rounds):
        session.send_plant()
        time.sleep(0.04)
    session.reader.drain()
