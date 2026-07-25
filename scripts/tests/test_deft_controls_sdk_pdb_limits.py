"""Host-side PDU V/I soft-kill belt-and-suspenders (deft_controls_sdk.pdb.limits).

Pure/offline — no COM5, no motors. Injects the same bad pack_v / overcurrent
readings ``pdb_uart_sim.py`` can produce (via the shared ``pack_feedback()``
frame builder it also uses) and proves the *host-side* park path fires even
when the peer still reports ``kill_state=NORMAL`` on the USB mirror — i.e.
the case this module exists for: firmware that doesn't (yet) run Cursonier's
``pdb_vi_reject_reason()`` overlay. See docs/peripherals/pdu-uart-soft-kill.md.
"""
from __future__ import annotations

import os
import struct
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.link import McuState
from deft_controls_sdk.link.exchange import IMAGE_BYTES
from deft_controls_sdk.link.exchange.wire_layout import (
    HOST_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    PDB_OFF,
    SYSTEM_FB_OFF,
)
from deft_controls_sdk.pdb import (
    FRAME_BYTES,
    KILL_NORMAL,
    KILL_REASON_NONE,
    KILL_REASON_OVERCURRENT,
    KILL_REASON_UNDERVOLTAGE,
    check_status,
    pack_feedback,
    pdb_status_from_frame,
    pdb_vi_reject_reason,
)


def _image(
    *,
    pack_v=(4800, 0, 0, 0),
    rail_v=(4800, 1900, 1200, 500),
    pack_i=(100, 0, 0, 0),
    rail_i=(10, 20, 30, 40),
    contactor_state=0b1111,
) -> bytes:
    """One full ``IMAGE_BYTES`` plant feedback frame, ``kill_state=NORMAL``
    on the USB mirror — as a pre-Cursonier-flash board would still report
    even with bad V/I, which is the exact gap the host-side check covers —
    embedding a fresh PDBF built the same way ``pdb_uart_sim.py`` does."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 1
    )
    buf[SYSTEM_FB_OFF + 14] = KILL_NORMAL
    buf[SYSTEM_FB_OFF + 15] = KILL_REASON_NONE
    buf[SYSTEM_FB_OFF + 16] = 1
    pdb = pack_feedback(
        seq=1,
        pack_v=pack_v,
        rail_v=rail_v,
        pack_i=pack_i,
        rail_i=rail_i,
        contactor_state=contactor_state,
        kill_state=KILL_NORMAL,
        kill_reason=0,
        estop_sense=1,
    )
    buf[PDB_OFF : PDB_OFF + FRAME_BYTES] = pdb
    return bytes(buf)


# ---- pure pdb_vi_reject_reason() / check_status() -------------------------


def test_reject_reason_flags_pack_undervoltage():
    frame = _image(pack_v=(3900, 0, 0, 0))  # 39.0 V, below the 40 V floor
    pdb = pdb_status_from_frame(frame).pdb
    assert pdb_vi_reject_reason(pdb) == KILL_REASON_UNDERVOLTAGE


def test_reject_reason_flags_overcurrent():
    frame = _image(pack_i=(3100, 0, 0, 0))  # 31 A, above the 30 A ceiling
    pdb = pdb_status_from_frame(frame).pdb
    assert pdb_vi_reject_reason(pdb) == KILL_REASON_OVERCURRENT


def test_reject_reason_flags_rail48_overvoltage():
    frame = _image(rail_v=(5300, 1900, 1200, 500))  # 53.0 V, above 52 V
    pdb = pdb_status_from_frame(frame).pdb
    assert pdb_vi_reject_reason(pdb) == KILL_REASON_UNDERVOLTAGE


def test_reject_reason_ignores_unpopulated_pack_channels():
    # pack slots 1-3 are 0 V / 0 A -- unpopulated, must not trip UV.
    frame = _image(pack_v=(4800, 0, 0, 0))
    pdb = pdb_status_from_frame(frame).pdb
    assert pdb_vi_reject_reason(pdb) == KILL_REASON_NONE


def test_check_status_none_without_status():
    assert check_status(None) is None


def test_check_status_matches_reject_reason():
    status = pdb_status_from_frame(_image(pack_v=(3900, 0, 0, 0)))
    check = check_status(status)
    assert check is not None
    assert check.violated
    assert check.reason == KILL_REASON_UNDERVOLTAGE
    assert check.reason_name == "undervoltage"


# ---- hub.soft_kill_park_if_bad_vi() wiring --------------------------------


class _FakeConnection:
    """Minimal stand-in for ``deft_controls_sdk.link.Connection`` — just
    enough surface for ``ControlsPcbHub.pdb_status()``/``soft_kill_park()``
    to run without a real serial port, plant loop, or motors."""

    def __init__(self, frame: bytes) -> None:
        self._frame = frame
        self._latest_fb_raw = None
        self.actuators_sent = None
        self.servos_cleared = False
        self.mcu_state = None

    def _drain_latest_plant_feedback(self):
        return self._frame

    def set_actuators(self, desires, send=False):
        self.actuators_sent = desires

    def clear_servos(self, send=False):
        self.servos_cleared = True

    def set_led(self, desire, send=False):
        pass

    def set_mcu_state(self, state, send=True):
        self.mcu_state = state

    def send_once(self):
        pass


def _hub_with(frame: bytes) -> ControlsPcbHub:
    hub = ControlsPcbHub.__new__(ControlsPcbHub)
    hub._connection = _FakeConnection(frame)
    return hub


def test_soft_kill_park_if_bad_vi_parks_on_pack_undervoltage():
    hub = _hub_with(_image(pack_v=(3900, 0, 0, 0)))
    parked = hub.soft_kill_park_if_bad_vi(send=False)
    assert parked is True
    assert hub._connection.servos_cleared is True
    assert hub._connection.mcu_state == McuState.ESTOP
    assert hub._connection.actuators_sent is not None


def test_soft_kill_park_if_bad_vi_parks_on_overcurrent():
    hub = _hub_with(_image(pack_i=(3100, 0, 0, 0)))
    assert hub.soft_kill_park_if_bad_vi(send=False) is True


def test_soft_kill_park_if_bad_vi_noop_when_in_range():
    hub = _hub_with(_image())
    assert hub.soft_kill_park_if_bad_vi(send=False) is False
    assert hub._connection.mcu_state is None
    assert hub._connection.servos_cleared is False


def test_soft_kill_park_if_bad_vi_skips_when_peer_already_soft_kill_req():
    # kill_state != NORMAL -- soft_kill_park_if_requested() already owns
    # this path; the V/I check must not double-handle it.
    buf = bytearray(_image(pack_v=(3900, 0, 0, 0)))
    from deft_controls_sdk.pdb import KILL_SOFT_REQ

    buf[SYSTEM_FB_OFF + 14] = KILL_SOFT_REQ
    hub = _hub_with(bytes(buf))
    assert hub.soft_kill_park_if_bad_vi(send=False) is False
