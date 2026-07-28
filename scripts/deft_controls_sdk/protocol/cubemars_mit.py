"""CubeMars AK-series MIT Power Mode wire helpers (PDF §5.3).

Mirrors the intended App/Src/plant/plugins/cubemars.c MIT path (see
docs/legacy/rfc/rfc-cubemars-mit-plant.md). Pack layout matches Damiao MIT
(damiao_pack_tx) — not the broken sample in the CubeMars PDF.

Legacy Servo Mode (ext-ID mode 6) lives in
scripts/legacy/controls_pcb_host/protocol/cubemars.py and is intentionally
not expanded here.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional, Tuple

# Shared across AK models (PDF §5.3).
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
P_MIN = -12.5
P_MAX = 12.5

CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_SET_ZERO = 0xFE


class CubemarsAkModel(IntEnum):
    AK10_9 = 0
    AK60_6 = 1
    AK70_10 = 2
    AK80_6 = 3
    AK80_9 = 4
    AK80_80 = 5


# (v_min, v_max, t_min, t_max) — PDF §5.3 per-module table.
_AK_LIMITS: Dict[CubemarsAkModel, Tuple[float, float, float, float]] = {
    CubemarsAkModel.AK10_9: (-50.0, 50.0, -65.0, 65.0),
    CubemarsAkModel.AK60_6: (-50.0, 50.0, -15.0, 15.0),
    CubemarsAkModel.AK70_10: (-50.0, 50.0, -25.0, 25.0),
    CubemarsAkModel.AK80_6: (-76.0, 76.0, -12.0, 12.0),
    CubemarsAkModel.AK80_9: (-50.0, 50.0, -18.0, 18.0),
    CubemarsAkModel.AK80_80: (-8.0, 8.0, -144.0, 144.0),
}

DEFAULT_MODEL = CubemarsAkModel.AK80_9


@dataclass(frozen=True)
class CubemarsLimits:
    p_min: float = P_MIN
    p_max: float = P_MAX
    v_min: float = -50.0
    v_max: float = 50.0
    t_min: float = -18.0
    t_max: float = 18.0
    kp_min: float = KP_MIN
    kp_max: float = KP_MAX
    kd_min: float = KD_MIN
    kd_max: float = KD_MAX


def limits_for_model(model: CubemarsAkModel = DEFAULT_MODEL) -> CubemarsLimits:
    v_min, v_max, t_min, t_max = _AK_LIMITS[CubemarsAkModel(model)]
    return CubemarsLimits(v_min=v_min, v_max=v_max, t_min=t_min, t_max=t_max)


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Damiao-correct map: uses (1<<bits)-1. Do not use the PDF sample formula."""
    if bits <= 0 or bits > 16:
        return 0
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    span = x_max - x_min
    max_val = (1 << bits) - 1
    return int((x - x_min) * (float(max_val) / span))


def uint_to_float(raw: int, x_min: float, x_max: float, bits: int) -> float:
    """Inverse of float_to_uint — same max_val ((1<<bits)-1), not 65535-only."""
    if bits <= 0 or bits > 16:
        return x_min
    max_val = (1 << bits) - 1
    return x_min + (float(raw) * (x_max - x_min) / float(max_val))


def pack_enable(motor_id: int) -> Tuple[int, bytes]:
    """Std-ID MIT enter-mode frame: FF*7 + 0xFC in D[7]."""
    data = bytes([0xFF] * 7 + [CMD_ENABLE])
    return motor_id & 0x7FF, data


def pack_disable(motor_id: int) -> Tuple[int, bytes]:
    data = bytes([0xFF] * 7 + [CMD_DISABLE])
    return motor_id & 0x7FF, data


def pack_set_zero(motor_id: int) -> Tuple[int, bytes]:
    data = bytes([0xFF] * 7 + [CMD_SET_ZERO])
    return motor_id & 0x7FF, data


def pack_mit(
    motor_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    *,
    model: CubemarsAkModel = DEFAULT_MODEL,
) -> Tuple[int, bytes]:
    """Pack MIT command — same byte layout as Damiao damiao_pack_tx."""
    lim = limits_for_model(model)
    p_u = float_to_uint(position, lim.p_min, lim.p_max, 16)
    v_u = float_to_uint(velocity, lim.v_min, lim.v_max, 12)
    kp_u = float_to_uint(kp, lim.kp_min, lim.kp_max, 12)
    kd_u = float_to_uint(kd, lim.kd_min, lim.kd_max, 12)
    t_u = float_to_uint(torque, lim.t_min, lim.t_max, 12)

    data = bytes(
        [
            (p_u >> 8) & 0xFF,
            p_u & 0xFF,
            (v_u >> 4) & 0xFF,
            ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            # Correct: torque high nibble — PDF sample wrongly reuses kp>>8 here.
            ((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F),
            t_u & 0xFF,
        ]
    )
    return motor_id & 0x7FF, data


def unpack_mit_tx(
    payload: bytes,
    *,
    model: CubemarsAkModel = DEFAULT_MODEL,
) -> Dict[str, float]:
    if len(payload) < 8:
        raise ValueError("MIT TX payload needs 8 bytes")
    lim = limits_for_model(model)
    p_u = (payload[0] << 8) | payload[1]
    v_u = (payload[2] << 4) | (payload[3] >> 4)
    kp_u = ((payload[3] & 0x0F) << 8) | payload[4]
    kd_u = (payload[5] << 4) | (payload[6] >> 4)
    t_u = ((payload[6] & 0x0F) << 8) | payload[7]
    return {
        "position": uint_to_float(p_u, lim.p_min, lim.p_max, 16),
        "velocity": uint_to_float(v_u, lim.v_min, lim.v_max, 12),
        "kp": uint_to_float(kp_u, lim.kp_min, lim.kp_max, 12),
        "kd": uint_to_float(kd_u, lim.kd_min, lim.kd_max, 12),
        "torque": uint_to_float(t_u, lim.t_min, lim.t_max, 12),
    }


def resolve_rx_can_id(motor_id: int, master_id: Optional[int] = None) -> int:
    """PDF: Identifier = 0x00 + Drive ID. master_id 0/None → motor_id."""
    if master_id is None or master_id == 0 or master_id == 0xFFFFFFFF:
        return motor_id & 0x7FF
    return master_id & 0x7FF


def unpack_mit_rx(
    payload: bytes,
    *,
    model: CubemarsAkModel = DEFAULT_MODEL,
    expected_motor_id: Optional[int] = None,
) -> Optional[Dict[str, float]]:
    """Unpack MIT feedback. Accepts dlc>=6; temp/err need 8 bytes.

    PDF table: D[0]=Drive ID, D[1..2]=p, D[3..5]=v|t, D[6]=temp, D[7]=err.
    When ``expected_motor_id`` is set, reject D[0] mismatches (shared-bus
    garbage) — mirrors App cubemars_parse_mit.
    """
    if len(payload) < 6:
        return None
    motor_id = payload[0] & 0xFF
    if expected_motor_id is not None and motor_id != (expected_motor_id & 0xFF):
        return None
    lim = limits_for_model(model)
    p_u = (payload[1] << 8) | payload[2]
    v_u = (payload[3] << 4) | (payload[4] >> 4)
    t_u = ((payload[4] & 0x0F) << 8) | payload[5]
    temp = float(payload[6]) if len(payload) >= 7 else 0.0
    err = float(payload[7]) if len(payload) >= 8 else 0.0
    return {
        "motor_id": float(motor_id),
        "position": uint_to_float(p_u, lim.p_min, lim.p_max, 16),
        "velocity": uint_to_float(v_u, lim.v_min, lim.v_max, 12),
        "torque": uint_to_float(t_u, lim.t_min, lim.t_max, 12),
        "temperature": temp,
        "fault": err,
    }
