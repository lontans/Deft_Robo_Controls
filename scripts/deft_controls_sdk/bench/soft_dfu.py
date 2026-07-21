"""Soft-DFU backdoor — resets the board into the ROM bootloader over USB CDC.

Firmware side: App/Src/host/soft_dfu.c / App/Inc/host/soft_dfu.h — read that
file's header comment before using this. UNVERIFIED WITHOUT HARDWARE: the
firmware has not been flashed or tested against real silicon yet (no ST-Link
available when this was written). Try this only once you can fall back to
the ST-Link if the board doesn't come back up as a DFU device.

The board never replies to this frame — it resets immediately on receipt, so
there is nothing to wait for. The connection will stop seeing feedback
frames until the board re-enumerates over USB (as a DFU device, not the
normal CDC port, if the jump worked).
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from deft_controls_sdk.link.exchange import PDU_OFF, build_plant_command

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection

_TAG = b"DFU!"  # must match SOFT_DFU_TAG0..3 in App/Inc/host/soft_dfu.h


def enter_bootloader(connection: "Connection", *, confirm: bool = False) -> None:
    """Fire-and-forget: sends the DFU backdoor frame, which resets the board.

    Requires confirm=True (deliberately not a default) since this interrupts
    whatever session is running and, until verified on hardware, might not
    come back up as a DFU device at all — see the module docstring.
    """
    if not confirm:
        raise ValueError(
            "enter_bootloader() requires confirm=True — this resets the board "
            "and is unverified without hardware, see soft_dfu.py docstring"
        )
    frame = bytearray(build_plant_command(connection.next_seq()))
    frame[PDU_OFF : PDU_OFF + len(_TAG)] = _TAG
    connection.write_raw(bytes(frame))
