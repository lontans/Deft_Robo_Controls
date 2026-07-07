"""Golden tests for 562 B image layout (no hardware)."""
from __future__ import annotations

import struct

from controls_pcb_host.commands import (
    build_dm_probe_command,
    build_plant_command,
    build_rs2_scan_command,
)
from controls_pcb_host.protocol import (
    HOST_COMMAND_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    PLANT_MCU_STATE_DIAG_ONLY,
    PLANT_MCU_STATE_NORMAL,
    RS2_TAG0,
)


def test_image_size() -> None:
    assert len(build_plant_command(1)) == IMAGE_BYTES
    assert len(build_rs2_scan_command(0x70, 0, 2, bus=4)) == IMAGE_BYTES
    assert len(build_dm_probe_command(6, 16, 3, bus=3)) == IMAGE_BYTES


def test_plant_header() -> None:
    buf = build_plant_command(42, mcu_state=PLANT_MCU_STATE_NORMAL)
    magic, lv, size, seq = struct.unpack_from("<IHHI", buf, 0)
    assert magic == HOST_COMMAND_MAGIC
    assert lv == HOST_LAYOUT_VERSION
    assert size == IMAGE_BYTES
    assert seq == 42
    sys_word, = struct.unpack_from("<I", buf, 12)
    assert (sys_word >> 1) & 7 == PLANT_MCU_STATE_NORMAL


def test_rs2_pdu_tag() -> None:
    buf = build_rs2_scan_command(0x74, 11, 1, bus=1)
    assert buf[PDU_OFF] == RS2_TAG0
    assert buf[PDU_OFF + 3] == 0x74
    assert buf[PDU_OFF + 4] == 11


def test_dm_diag_state() -> None:
    buf = build_dm_probe_command(6, 16, 1, bus=3)
    sys_word, = struct.unpack_from("<I", buf, 12)
    assert (sys_word >> 1) & 7 == PLANT_MCU_STATE_DIAG_ONLY
    assert buf[PDU_OFF : PDU_OFF + 3] == b"DM0"
