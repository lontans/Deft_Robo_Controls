"""Controls <-> PDB (Power Distribution Board) UART frame contract — 64 B.

Wire contract: docs/pdb-uart-v1.md. Bit-exact port of the controls-side C in
App/Inc/host/pdb_link.h / App/Src/host/pdb_link.c — do not diverge without
updating both sides and this module together.

Separate physical link (UART4, PC10/PC11) from the USB host_exchange image
(deft_controls_sdk.link.exchange) — different framing, different magics.
"""
from __future__ import annotations

import struct
from typing import Optional, Sequence, Tuple

FRAME_BYTES = 64

MAGIC_CMD = 0x43424450  # "PDBC" (Controls -> PDB)
MAGIC_FB = 0x46424450  # "PDBF" (PDB -> Controls)
VERSION = 1

OFF_MAGIC = 0
OFF_VERSION = 4
OFF_SEQ = 5
OFF_FLAGS = 6
OFF_PAYLOAD = 8
OFF_CRC = 62

# Command payload (Controls -> PDB), from OFF_PAYLOAD
CMD_OFF_RAIL_ENABLE = 8
CMD_OFF_KILL_REQ = 9
CMD_OFF_HEARTBEAT = 10

# Feedback payload (PDB -> Controls), from OFF_PAYLOAD
FB_OFF_PACK_V = 8  # 4x u16
FB_OFF_RAIL_V = 16  # 4x u16
FB_OFF_PACK_I = 24  # 4x u16
FB_OFF_RAIL_I = 32  # 4x u16
FB_OFF_CONTACTOR = 40
FB_OFF_KILL_STATE = 41
FB_OFF_KILL_REASON = 42
FB_OFF_ESTOP_SENSE = 43
FB_OFF_FAULT_FLAGS = 44
FB_OFF_HB_ECHO = 45

KILL_NORMAL = 0
KILL_SOFT_REQ = 1
KILL_SOFT_READY = 2
KILL_HARD_ESTOP = 3

KILL_STATE_NAMES = {
    KILL_NORMAL: "normal",
    KILL_SOFT_REQ: "soft_kill_req",
    KILL_SOFT_READY: "soft_kill_ready",
    KILL_HARD_ESTOP: "hard_estop",
}

KILL_REASON_NONE = 0
KILL_REASON_HOST = 1
KILL_REASON_UNDERVOLTAGE = 2
KILL_REASON_OVERCURRENT = 3
KILL_REASON_OVERTEMP = 4
KILL_REASON_COMMS_LOSS = 5
KILL_REASON_BUTTON = 6
KILL_REASON_OTHER = 7

KILL_REASON_NAMES = {
    KILL_REASON_NONE: "none",
    KILL_REASON_HOST: "host",
    KILL_REASON_UNDERVOLTAGE: "undervoltage",
    KILL_REASON_OVERCURRENT: "overcurrent",
    KILL_REASON_OVERTEMP: "overtemp",
    KILL_REASON_COMMS_LOSS: "comms_loss",
    KILL_REASON_BUTTON: "button",
    KILL_REASON_OTHER: "other",
}

# ~4 missed frames at the documented 20 Hz nominal PDB->Controls rate.
STALE_MS = 200
# Controls -> PDB command cadence (host_link.c side of this doc).
TX_PERIOD_MS = 20


def crc16(data: bytes) -> int:
    """CRC16-CCITT: poly 0x1021, init 0xFFFF, MSB-first, no reflect, no final
    XOR. Bit-exact to pdb_crc16() in App/Src/host/pdb_link.c — deliberately
    bit-banged (not table-driven) so it is trivial to reproduce on any
    toolchain without sharing a generated table."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _magic_at(buf: bytes, magic: int) -> bool:
    return len(buf) >= 4 and struct.unpack_from("<I", buf, OFF_MAGIC)[0] == magic


def _finish(buf: bytearray) -> bytes:
    crc = crc16(bytes(buf[:OFF_CRC]))
    struct.pack_into("<H", buf, OFF_CRC, crc)
    return bytes(buf)


def is_frame_valid(buf: bytes, expect_magic: int) -> bool:
    """Magic + version + CRC all pass. A failing frame is 'no frame this
    cycle' — never partially trusted (matches pdb_frame_valid() firmware
    semantics: a corrupt frame must not refresh link freshness)."""
    if len(buf) != FRAME_BYTES:
        return False
    if not _magic_at(buf, expect_magic):
        return False
    if buf[OFF_VERSION] != VERSION:
        return False
    crc = crc16(buf[:OFF_CRC])
    stored, = struct.unpack_from("<H", buf, OFF_CRC)
    return stored == crc


def pack_command(
    *,
    seq: int = 0,
    rail_enable_cmd: int = 0,
    kill_request: int = KILL_NORMAL,
    heartbeat: int = 0,
) -> bytes:
    """Build one 64 B PDBC command frame (Controls -> PDB)."""
    buf = bytearray(FRAME_BYTES)
    struct.pack_into("<IBBB", buf, OFF_MAGIC, MAGIC_CMD, VERSION, seq & 0xFF, 0)
    buf[CMD_OFF_RAIL_ENABLE] = rail_enable_cmd & 0xFF
    buf[CMD_OFF_KILL_REQ] = kill_request & 0xFF
    buf[CMD_OFF_HEARTBEAT] = heartbeat & 0xFF
    return _finish(buf)


def parse_command(buf: bytes) -> Optional[dict]:
    """Parse a PDBC command frame. Returns None on any magic/version/CRC
    failure (fail-safe: treat as no frame, never partially trust)."""
    if not is_frame_valid(buf, MAGIC_CMD):
        return None
    seq = buf[OFF_SEQ]
    return {
        "seq": seq,
        "rail_enable_cmd": buf[CMD_OFF_RAIL_ENABLE],
        "kill_request": buf[CMD_OFF_KILL_REQ],
        "heartbeat": buf[CMD_OFF_HEARTBEAT],
    }


def pack_feedback(
    *,
    seq: int = 0,
    pack_v: Sequence[int] = (0, 0, 0, 0),
    rail_v: Sequence[int] = (0, 0, 0, 0),
    pack_i: Sequence[int] = (0, 0, 0, 0),
    rail_i: Sequence[int] = (0, 0, 0, 0),
    contactor_state: int = 0,
    kill_state: int = KILL_NORMAL,
    kill_reason: int = KILL_REASON_NONE,
    estop_sense: int = 0,
    fault_flags: int = 0,
    heartbeat_echo: int = 0,
) -> bytes:
    """Build one 64 B PDBF feedback frame (PDB -> Controls).

    pack_v/rail_v/pack_i/rail_i are 4-element uint16 sequences. Scales are a
    placeholder pending real sensor resolution (see pdb-uart-v1.md):
    10 mV/count for the voltages, 10 mA/count for the currents.
    """
    if len(pack_v) != 4 or len(rail_v) != 4 or len(pack_i) != 4 or len(rail_i) != 4:
        raise ValueError("pack_v/rail_v/pack_i/rail_i must each have 4 elements")
    buf = bytearray(FRAME_BYTES)
    struct.pack_into("<IBBB", buf, OFF_MAGIC, MAGIC_FB, VERSION, seq & 0xFF, 0)
    struct.pack_into("<4H", buf, FB_OFF_PACK_V, *(v & 0xFFFF for v in pack_v))
    struct.pack_into("<4H", buf, FB_OFF_RAIL_V, *(v & 0xFFFF for v in rail_v))
    struct.pack_into("<4H", buf, FB_OFF_PACK_I, *(v & 0xFFFF for v in pack_i))
    struct.pack_into("<4H", buf, FB_OFF_RAIL_I, *(v & 0xFFFF for v in rail_i))
    buf[FB_OFF_CONTACTOR] = contactor_state & 0xFF
    buf[FB_OFF_KILL_STATE] = kill_state & 0xFF
    buf[FB_OFF_KILL_REASON] = kill_reason & 0xFF
    buf[FB_OFF_ESTOP_SENSE] = estop_sense & 0xFF
    buf[FB_OFF_FAULT_FLAGS] = fault_flags & 0xFF
    buf[FB_OFF_HB_ECHO] = heartbeat_echo & 0xFF
    return _finish(buf)


def parse_feedback(buf: bytes) -> Optional[dict]:
    """Parse a PDBF feedback frame. Returns None on any magic/version/CRC
    failure (fail-safe: treat as no frame, never partially trust)."""
    if not is_frame_valid(buf, MAGIC_FB):
        return None
    seq = buf[OFF_SEQ]
    pack_v: Tuple[int, int, int, int] = struct.unpack_from("<4H", buf, FB_OFF_PACK_V)
    rail_v: Tuple[int, int, int, int] = struct.unpack_from("<4H", buf, FB_OFF_RAIL_V)
    pack_i: Tuple[int, int, int, int] = struct.unpack_from("<4H", buf, FB_OFF_PACK_I)
    rail_i: Tuple[int, int, int, int] = struct.unpack_from("<4H", buf, FB_OFF_RAIL_I)
    return {
        "seq": seq,
        "pack_v": pack_v,
        "rail_v": rail_v,
        "pack_i": pack_i,
        "rail_i": rail_i,
        "contactor_state": buf[FB_OFF_CONTACTOR],
        "kill_state": buf[FB_OFF_KILL_STATE],
        "kill_reason": buf[FB_OFF_KILL_REASON],
        "estop_sense": buf[FB_OFF_ESTOP_SENSE],
        "fault_flags": buf[FB_OFF_FAULT_FLAGS],
        "heartbeat_echo": buf[FB_OFF_HB_ECHO],
    }
