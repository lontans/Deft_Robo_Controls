"""Golden tests for deft_controls_sdk/bench/soft_dfu.py (no hardware).

The board never replies to the DFU backdoor frame (see soft_dfu.py docstring —
it resets immediately), so unlike the other bench tests there is no responder
to feed back into the FrameReader. These tests only check what goes out on
the wire and the confirm=True safety gate — not board behavior, which is
unverified until flashed (see App/Inc/host/soft_dfu.h).
"""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.bench import DebugAPI
from deft_controls_sdk.bench.soft_dfu import enter_bootloader
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import IMAGE_BYTES, PDU_OFF


def _fake_connection(sent: list) -> Connection:
    conn = Connection("TESTPORT")
    conn._ser = object()  # is_open True; nothing actually opens a port

    def _write_raw(frame: bytes, *, drain: bool = False) -> None:
        sent.append(frame)

    conn.write_raw = _write_raw  # type: ignore[method-assign]
    return conn


def test_enter_bootloader_requires_confirm() -> None:
    sent: list = []
    conn = _fake_connection(sent)
    with pytest.raises(ValueError, match="confirm=True"):
        enter_bootloader(conn)
    assert sent == []  # must not touch the wire without confirm


def test_enter_bootloader_sends_dfu_tag_at_pdu_offset() -> None:
    sent: list = []
    conn = _fake_connection(sent)
    enter_bootloader(conn, confirm=True)
    assert len(sent) == 1
    frame = sent[0]
    assert len(frame) == IMAGE_BYTES
    assert frame[PDU_OFF : PDU_OFF + 4] == b"DFU!"


def test_debug_api_enter_bootloader_wires_through() -> None:
    """Same check via the public hub.debug.enter_bootloader() surface,
    since that's what docs/api.md documents as the real entry point."""
    sent: list = []
    conn = _fake_connection(sent)
    debug = DebugAPI(conn, None)
    with pytest.raises(ValueError, match="confirm=True"):
        debug.enter_bootloader()
    debug.enter_bootloader(confirm=True)
    assert len(sent) == 1
    assert sent[0][PDU_OFF : PDU_OFF + 4] == b"DFU!"
