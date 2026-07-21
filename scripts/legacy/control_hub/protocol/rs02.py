"""
RS02 private CAN protocol — constants and decode aligned with RS02_Firmware_Documentation.pdf.

Comm types live in ext_id bits 24–28; feedback mms in data16 bits 22–23:
  0=rest, 1=cali, 2=running (motor mode).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

# Communication types (ext_id mode field)
COMM_GET_ID = 0x00
COMM_MOTOR_CTRL = 0x01
COMM_MOTOR_FEEDBACK = 0x02
COMM_MOTOR_ENABLE = 0x03
COMM_MOTOR_RESET = 0x04
COMM_MOTOR_CALI = 0x05
COMM_MOTOR_ZERO = 0x06
COMM_PARAREAD = 0x11
COMM_PARAWRITE = 0x12
COMM_DATA_SAVE = 0x16

COMM_NAMES = {
    COMM_GET_ID: "get_id",
    COMM_MOTOR_CTRL: "motor_ctrl",
    COMM_MOTOR_FEEDBACK: "motor_feedback",
    COMM_MOTOR_ENABLE: "motor_in",
    COMM_MOTOR_RESET: "motor_reset",
    COMM_MOTOR_CALI: "motor_cali",
    COMM_MOTOR_ZERO: "motor_zero",
    COMM_PARAREAD: "pararead",
    COMM_PARAWRITE: "parawrite",
    COMM_DATA_SAVE: "data_save",
}

# Parameter indices (§4.1.13)
PARAM_RUN_MODE = 0x7005
PARAM_MECH_POS = 0x7019
PARAM_BUS_VOLT = 0x701C
PARAM_IQ_TEST = 0x702D  # §4.2.7 — set 1 before encoder/cogging cal

VBUS_MIN_V = 24.0
VBUS_MAX_V = 60.0
DEFAULT_CAL_LISTEN_S = 28.0
CALI_SKIP_RESET = 0x4000  # probe param bit 14 — host already reset; TX 0x05 only
CALI_LISTEN_ONLY = 0x8000  # probe param bit 15 — listen without reset/re-TX 0x05

MMS_LABELS: Tuple[str, ...] = ("rest", "cali", "running")


@dataclass(frozen=True)
class ExtIdInfo:
    raw: int
    id_byte: int
    data16: int
    mode: int
    motor_id: int
    status_byte: int
    faults: List[str]
    mode_status: int


_FAULT_MAP = (
    (0x01, "under_voltage"),
    (0x02, "drive_fault"),
    (0x04, "over_temp"),
    (0x08, "mag_encoder"),
    (0x10, "stall_overload"),
    (0x20, "uncalibrated"),
)


def decode_ext_id(ext_id: int) -> ExtIdInfo:
    id_byte = ext_id & 0xFF
    data16 = (ext_id >> 8) & 0xFFFF
    mode = (ext_id >> 24) & 0x1F
    motor_id = data16 & 0xFF
    status_byte = (data16 >> 8) & 0xFF
    faults = [name for mask, name in _FAULT_MAP if status_byte & mask]
    mode_status = (data16 >> 14) & 0x3
    return ExtIdInfo(
        raw=ext_id,
        id_byte=id_byte,
        data16=data16,
        mode=mode,
        motor_id=motor_id,
        status_byte=status_byte,
        faults=faults,
        mode_status=mode_status,
    )


def mms_label(mode_status: int) -> str:
    if mode_status < len(MMS_LABELS):
        return MMS_LABELS[mode_status]
    return f"mode{mode_status}"


def comm_label(comm_mode: int) -> str:
    name = COMM_NAMES.get(comm_mode)
    if name is None:
        return f"0x{comm_mode:02X}"
    return f"0x{comm_mode:02X} {name}"
