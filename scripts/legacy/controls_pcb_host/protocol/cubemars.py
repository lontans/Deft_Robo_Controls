"""
CubeMars AK-series "Servo Mode" wire mirror — mirrors App/Src/plant/plugins/cubemars.c
(2026-07-10 workstreams, not yet merged into the live STM32 build — see that folder's
README). Source: External_Documentation/CubeMars/CubeMars_AK_Driver_Doc_Generalised.pdf §5.1-5.2.

Position-Speed Loop Mode (6) only — see cubemars.c for why the other 6 control modes
aren't implemented yet. Everything here is pure wire-format math for unit tests; CubeMars
runs over the internal CAN bus via actuator_commands[], not a host-visible PDU, so there's
no bench CLI script for it (unlike RS2/DM/thermo) — nothing to send from the host directly.

TODO(hardware, P0): position/speed scale factors below are the vendor doc's documented
wire units (deg*10000, eRPM/10) but whether ActuatorDesire's mechanical rad/rad*s^-1 map
1:1 onto those without a pole-pair conversion is UNCONFIRMED until real hardware is
available. Do not treat these constants as verified SI conversions.
"""
from __future__ import annotations

import struct
from enum import IntEnum
from typing import Optional, Tuple

POS_SCALE = 10000.0
SPEED_SCALE = 1.0 / 10.0
ACCEL_SCALE = 1.0 / 10.0


class CubemarsControlMode(IntEnum):
    DUTY = 0
    CURRENT = 1
    CURRENT_BRAKE = 2
    SPEED = 3
    POSITION = 4
    SET_ORIGIN = 5
    POS_SPEED = 6


def build_ext_id(mode: CubemarsControlMode, node_id: int) -> int:
    return ((int(mode) & 0x1FFFFF) << 8) | (node_id & 0xFF)


def parse_ext_id(ext_id: int) -> Tuple[CubemarsControlMode, int]:
    node_id = ext_id & 0xFF
    mode_raw = (ext_id >> 8) & 0x1FFFFF
    return CubemarsControlMode(mode_raw), node_id


def pack_position_speed(position: float, speed: float, accel: float = 0.0) -> bytes:
    """Position-Speed Loop Mode payload — int32 pos | int16 speed | int16 accel, big-endian."""
    pos_raw = int(position * POS_SCALE)
    speed_raw = int(speed * SPEED_SCALE)
    accel_raw = int(accel * ACCEL_SCALE)
    return struct.pack(">ihh", pos_raw, speed_raw, accel_raw)


def unpack_position_speed(payload: bytes) -> dict:
    pos_raw, speed_raw, accel_raw = struct.unpack(">ihh", payload[:8])
    return {
        "position": pos_raw / POS_SCALE,
        "speed": speed_raw / SPEED_SCALE,
        "accel": accel_raw / ACCEL_SCALE,
    }


def unpack_upload_frame(payload: bytes) -> Optional[dict]:
    """Periodic servo-mode broadcast: int16 pos*0.1deg, int16 speed*10eRPM,
    int16 current*0.01A, int8 temp, uint8 error. CAN ID this rides on is
    UNDOCUMENTED/unconfirmed — see cubemars.c's parse_rx TODO. This only
    exercises the documented payload layout, not ID matching."""
    if len(payload) < 8:
        return None
    pos_raw, speed_raw, cur_raw, temp_raw, err = struct.unpack(">hhhbB", payload[:8])
    return {
        "position": pos_raw * 0.1,
        "speed": speed_raw * 10.0,
        "current": cur_raw * 0.01,
        "temperature": float(temp_raw),
        "error": err,
    }
