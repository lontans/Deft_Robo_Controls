"""Pause plant TX around DEBUG ``exchange_raw`` (discover / CFG / cal)."""
from __future__ import annotations

from contextlib import contextmanager
from typing import Any, Iterator


@contextmanager
def pause_plant_stream(owner: Any) -> Iterator[None]:
    """Pause plant TX/FB drain around CFG / DEBUG ``exchange_raw`` calls.

    ``owner`` is a ``ControlsPcbHub`` or ``Connection`` (anything with
    ``is_streaming`` / ``stop_streaming`` / ``start_streaming``).

    The plant stream thread shares ``Connection.reader`` with bench probes.
    While it runs, discover/CFG replies can be starved — pause for the RPC,
    then restore streaming.
    """
    was = bool(owner.is_streaming)
    hz = 40.0
    conn = getattr(owner, "_connection", owner)
    try:
        hz = float(getattr(conn, "_stream_hz", 40.0) or 40.0)
    except Exception:
        pass
    if was:
        owner.stop_streaming()
    try:
        yield
    finally:
        if was:
            owner.start_streaming(hz=hz)
