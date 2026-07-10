"""
Bench PDU tags and probe kinds — mirrors App/Inc/plant/plant_diag.h and DXL/UB tags.
"""
from __future__ import annotations

# RS2 bench PDU
RS2_TAG0 = ord("R")
RS2_TAG1 = ord("S")
RS2_TAG2 = ord("2")
RS2_RESP_TAG = ord("r")

PROBE_FULL = 0
PROBE_ENABLE_CTRL = 1
PROBE_CTRL_ONLY = 2
PROBE_PROMISC = 10
PROBE_RESET = 11
PROBE_ENABLE_ONLY = 12
PROBE_CTRL_FAST = 13
PROBE_PARAREAD = 14
PROBE_PROACTIVE = 15
PROBE_CALI = 16
PROBE_ZERO = 17
PROBE_DATA_SAVE = 18
PROBE_PARAWRITE = 19
PROBE_MCP_SMOKE = 20
PROBE_MCP_WAKE = 21
PROBE_MCP_DISABLE = 22

SESSION_BEGIN = 254
SESSION_END = 255

# Damiao DM0 PDU
DM_TAG0 = ord("D")
DM_TAG1 = ord("M")
DM_TAG2 = ord("0")
DM_RESP_TAG = ord("m")

DM_PROBE_MIT = 0
DM_PROBE_POS = 1
DM_PROBE_VEL = 2
DM_PROBE_ALL = 3
DM_PROBE_ENABLE = 11
DM_PROBE_DISABLE = 12
DM_PROBE_CLEAR_FAULT = 13
DM_PROBE_READ_PARAM = 14
DM_PROBE_DISCOVER = 15
DM_PROBE_REG_SCAN = 16
DM_PROBE_ID_SWEEP = 17
DM_PROBE_SET_ZERO = 18
DM_PROBE_PROMISC = 10

DM_REG_ESC_ID = 0x08
DM_REG_MST_ID = 0x07
DM_REG_CTRL_MODE = 0x0A
DM_MASTER_ANY = 0xFF

DM_FW_MARKER0 = ord("D")
DM_FW_MARKER1 = ord("1")

DM_FB_MAGIC = 0xDA000000
DM_FB_MAGIC_FOUND_OLD = 0xDB000000

DM_PROBE_KIND_NAMES = {
    DM_PROBE_MIT: "mit",
    DM_PROBE_POS: "pos",
    DM_PROBE_VEL: "vel",
    DM_PROBE_ALL: "all",
    DM_PROBE_ENABLE: "enable",
    DM_PROBE_DISABLE: "disable",
    DM_PROBE_CLEAR_FAULT: "clear_fault",
    DM_PROBE_READ_PARAM: "read_param",
    DM_PROBE_DISCOVER: "discover",
    DM_PROBE_REG_SCAN: "reg_scan",
    DM_PROBE_ID_SWEEP: "id_sweep",
    DM_PROBE_SET_ZERO: "set_zero",
    DM_PROBE_PROMISC: "promisc",
}

# Dynamixel bench PDU
DXL_TAG0 = ord("D")
DXL_TAG1 = ord("X")
DXL_TAG2 = ord("L")
DXL_RESP_TAG = ord("d")

PLANT_DXL_PROBE_SCAN = 1
PLANT_DXL_PROBE_PING = 2
PLANT_DXL_PROBE_FIND_BAUD = 3

# UART4 Damiao debug bridge
UB_TAG0 = ord("U")
UB_TAG1 = ord("B")
UB_RESP_TAG = ord("u")
UB_CMD_MAX_BYTES = 29
UB_FB_MAX_BYTES = 30

# Config PDU — mirrors App/Inc/plant/plant_config_nvm.h
CFG_TAG0 = ord("C")
CFG_TAG1 = ord("F")
CFG_TAG2 = ord("G")
CFG_RESP_TAG0 = ord("c")
CFG_RESP_TAG1 = ord("f")
CFG_RESP_TAG2 = ord("g")
CFG_RESP_TAG = CFG_RESP_TAG0  # first-byte tag for feedback routing

CFG_OP_GET = 1
CFG_OP_SET = 2
CFG_OP_SAVE = 3
CFG_OP_LOAD = 4
CFG_OP_DEFAULTS = 5

CFG_STATUS_OK = 0
CFG_STATUS_BAD_ARG = 1
CFG_STATUS_FLASH_ERR = 2
CFG_STATUS_BAD_CRC = 3

CFG_STATUS_NAMES = {
    CFG_STATUS_OK: "ok",
    CFG_STATUS_BAD_ARG: "bad_arg",
    CFG_STATUS_FLASH_ERR: "flash_err",
    CFG_STATUS_BAD_CRC: "bad_crc",
}

RS2_COMM_NAMES = {
    0x00: "get_id",
    0x01: "motor_ctrl",
    0x02: "motor_feedback",
    0x03: "motor_in",
    0x04: "motor_reset",
    0x05: "motor_cali",
    0x06: "motor_zero",
    0x07: "set_can_id",
    0x11: "pararead",
    0x12: "parawrite",
    0x15: "error_feedback",
    0x16: "data_save",
    0x17: "baud_rate",
    0x18: "proactive",
    0x19: "mode_set",
}
