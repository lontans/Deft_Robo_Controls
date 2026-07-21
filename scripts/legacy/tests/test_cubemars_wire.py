"""Golden tests for the CubeMars Servo Mode wire mirror (no hardware).

These validate internal encode/decode consistency and the documented payload
layout only — NOT correctness against a real captured frame, since no
CubeMars motor is available to verify against. See
scripts/controls_pcb_host/protocol/cubemars.py's module docstring.
"""
from __future__ import annotations

import pytest

from controls_pcb_host.protocol.cubemars import (
    CubemarsControlMode,
    build_ext_id,
    pack_position_speed,
    parse_ext_id,
    unpack_position_speed,
    unpack_upload_frame,
)


@pytest.mark.parametrize(
    "mode,node_id",
    [
        (CubemarsControlMode.POS_SPEED, 0x01),
        (CubemarsControlMode.CURRENT, 0x7F),
        (CubemarsControlMode.SET_ORIGIN, 0xFF),
    ],
)
def test_ext_id_roundtrip(mode: CubemarsControlMode, node_id: int) -> None:
    ext_id = build_ext_id(mode, node_id)
    parsed_mode, parsed_node = parse_ext_id(ext_id)
    assert parsed_mode == mode
    assert parsed_node == node_id


def test_ext_id_layout_matches_doc() -> None:
    # Can ID bits [28:8] = control mode, [7:0] = source node ID.
    ext_id = build_ext_id(CubemarsControlMode.POS_SPEED, 0x42)
    assert ext_id & 0xFF == 0x42
    assert (ext_id >> 8) & 0x1FFFFF == int(CubemarsControlMode.POS_SPEED)


def test_pack_position_speed_roundtrip() -> None:
    payload = pack_position_speed(position=45.0, speed=1000.0, accel=500.0)
    assert len(payload) == 8
    out = unpack_position_speed(payload)
    assert out == pytest.approx({"position": 45.0, "speed": 1000.0, "accel": 500.0}, abs=0.1)


def test_pack_position_speed_default_accel_zero() -> None:
    payload = pack_position_speed(position=0.0, speed=0.0)
    out = unpack_position_speed(payload)
    assert out["accel"] == 0.0


def test_unpack_upload_frame_shape() -> None:
    # Synthetic payload — CAN ID this rides on is unconfirmed (see module
    # docstring); this only exercises the documented byte layout.
    import struct

    payload = struct.pack(">hhhbB", 100, 500, 250, 30, 0)
    out = unpack_upload_frame(payload)
    assert out is not None
    assert out == pytest.approx(
        {"position": 10.0, "speed": 5000.0, "current": 2.5, "temperature": 30.0, "error": 0}
    )


def test_unpack_upload_frame_short_payload_returns_none() -> None:
    assert unpack_upload_frame(b"\x00" * 4) is None
