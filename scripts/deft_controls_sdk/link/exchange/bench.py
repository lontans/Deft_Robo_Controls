"""Tagged-pdu (DEBUG mode) wire helpers — RS2 / DM0 / CFG bench PDU tags.

Mirrors `App/Inc/plant/plant_diag.h` and `plant_config_nvm.h`. Ported from
scripts/pcb_lab/legacy/controls_pcb_host/protocol/diag_pdu.py + commands.py +
feedback.py — kept byte-for-byte identical to the legacy encode/decode so a
DEBUG-mode probe behaves the same whether it goes through the SDK or the
legacy CLI. Plant-path pack/parse stays in pack.py/parse.py; this module is
only imported by `deft_controls_sdk/debug/`.
"""
from __future__ import annotations

import struct
from typing import Dict, Optional

from .pack import patch_actuator_desire, patch_system_mcu_state
from .parse import parse_actuator_feedback
from .wire_layout import (
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    MAX_CAN_BUS,
    MIN_CAN_BUS,
    PDU_OFF,
    PLANT_MCU_STATE_DIAG_ONLY,
    PLANT_MCU_STATE_NORMAL,
)

# -- CAN bus routing (pdu.data[11] = schematic branch 1..6) ---------------------------

PDU_BUS_OFF = PDU_OFF + 11

MCP_CAN_BUSES = frozenset({4, 5, 6})

_CAN_PINS: Dict[int, str] = {
    1: "PB8/9 FDCAN1",
    2: "PA8/PA15 FDCAN3",
    3: "PB12/13 FDCAN2",
    4: "PB11 MCP SPI-CAN",
    5: "PB1 MCP SPI-CAN",
    6: "PA4 MCP SPI-CAN",
}


def normalize_can_bus(bus: int) -> int:
    b = int(bus)
    if b < MIN_CAN_BUS or b > MAX_CAN_BUS:
        raise ValueError(f"CAN bus must be {MIN_CAN_BUS}..{MAX_CAN_BUS}, got {b}")
    return b


def is_mcp_bus(bus: int) -> bool:
    return normalize_can_bus(bus) in MCP_CAN_BUSES


def can_bus_label(bus: int) -> str:
    b = normalize_can_bus(bus)
    return f"CH{b} ({_CAN_PINS[b]})"


def build_debug_command(seq: int) -> bytearray:
    """Blank DEBUG command image (DBGC) — mailbox at PDU_OFF / pdb[0..31]."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI",
        buf,
        0,
        HOST_DEBUG_COMMAND_MAGIC,
        HOST_LAYOUT_VERSION,
        IMAGE_BYTES,
        seq & 0xFFFFFFFF,
    )
    return buf


# Internal alias used by RS2/DM/CFG builders in this module.
_blank_command = build_debug_command


def _patch_pdu_bus(buf: bytearray, bus: int) -> None:
    buf[PDU_BUS_OFF] = normalize_can_bus(bus) & 0xFF


# -- RS2 bench PDU (RobStride discover / probe / session) ----------------------------

RS2_TAG0, RS2_TAG1, RS2_TAG2 = ord("R"), ord("S"), ord("2")
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

SESSION_BEGIN = 254
SESSION_END = 255


def host_bus_mask(buses) -> int:
    """Bit0=host CH1 … bit5=CH6 (matches FW ``PLANT_DIAG_PDU_BUS_MASK``)."""
    mask = 0
    for b in buses:
        bi = int(b)
        if 1 <= bi <= 6:
            mask |= 1 << (bi - 1)
    return mask & 0x3F


def _rs2_mailbox(
    motor_id: int,
    probe_kind: int,
    bus: int,
    *,
    param_index: int = 0,
    param_raw_value: int = 0,
    bus_mask: int = 0,
) -> bytearray:
    mbox = bytearray(32)
    mbox[0] = RS2_TAG0
    mbox[1] = RS2_TAG1
    mbox[2] = RS2_TAG2
    mbox[3] = motor_id & 0xFF
    mbox[4] = probe_kind & 0xFF
    # SESSION_BEGIN: data[5]=bus_mask. Other kinds: param_index low (legacy).
    if bus_mask:
        mbox[5] = bus_mask & 0x3F
        mbox[6] = 0
    else:
        mbox[5] = param_index & 0xFF
        mbox[6] = (param_index >> 8) & 0xFF
    mbox[7] = param_raw_value & 0xFF
    mbox[8] = (param_raw_value >> 8) & 0xFF
    mbox[9] = (param_raw_value >> 16) & 0xFF
    mbox[10] = (param_raw_value >> 24) & 0xFF
    mbox[11] = normalize_can_bus(bus) & 0xFF
    return mbox


def build_rs2_scan_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    bus: int = 1,
    *,
    bus_mask: int = 0,
) -> bytes:
    """RS2 session/scan on debug lane 0 (legacy mailbox still accepted by FW).

    ``bus_mask``: optional multi-bus SESSION_BEGIN (bit0=CH1 … bit5=CH6).
    """
    from .debug_lanes import wrap_mailbox_as_debug_lanes
    from .wire_layout import DEBUG_LANE_RS

    return wrap_mailbox_as_debug_lanes(
        seq,
        _rs2_mailbox(motor_id, probe_kind, bus, bus_mask=bus_mask),
        DEBUG_LANE_RS,
    )


def build_rs2_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    param_index: int = 0,
    param_raw_value: int = 0,
    *,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    bus: int = 1,
) -> bytes:
    """RS2 probe on debug lane 0.

    ``param_index`` / ``param_raw_value`` → lane[5..10]. Optional MIT desire is
    packed as five floats at lane[12..31] (debug_lanes overlaps plant actuator[0],
    so desire cannot live at ACTUATOR0_OFF).
    """
    from .debug_lanes import wrap_mailbox_as_debug_lanes
    from .wire_layout import DEBUG_LANE_RS

    mbox = _rs2_mailbox(
        motor_id,
        probe_kind,
        bus,
        param_index=param_index,
        param_raw_value=param_raw_value,
    )
    if position != 0.0 or velocity != 0.0 or kp != 0.0 or kd != 0.0:
        struct.pack_into("<fffff", mbox, 12, position, velocity, kp, kd, 0.0)
    return wrap_mailbox_as_debug_lanes(seq, mbox, DEBUG_LANE_RS)


def parse_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    from .debug_lanes import extract_rs2_mailbox

    pdu = extract_rs2_mailbox(frame)
    if pdu[0] != RS2_RESP_TAG:
        # Dual-path: also accept legacy mailbox if lane empty
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        if pdu[0] != RS2_RESP_TAG:
            return None
    ext_id, = struct.unpack_from("<I", pdu, 4)
    temperature, position = struct.unpack_from("<ff", pdu, 16)
    bus_byte = int(pdu[27])
    bus = bus_byte if 1 <= bus_byte <= 6 else None
    return {
        "probe_id": pdu[1],
        "found": pdu[2] != 0,
        "comm_mode": pdu[3],
        "ext_id": ext_id,
        "can_data": bytes(pdu[8:16]),
        "temperature": temperature,
        "position": position,
        "discovered_id": pdu[24],
        "probe_kind": pdu[25],
        "raw_frames": pdu[26],
        "bus": bus,
    }


# -- Damiao DM0 bench PDU (discover) --------------------------------------------------

DM_TAG0, DM_TAG1, DM_TAG2 = ord("D"), ord("M"), ord("0")
DM_RESP_TAG = ord("m")

DM_PROBE_MIT = 0
DM_PROBE_ENABLE = 11
DM_PROBE_DISABLE = 12
DM_PROBE_CLEAR_FAULT = 13
DM_PROBE_REG_SCAN = 16
DM_PROBE_ID_SWEEP = 17

DM_REG_ESC_ID = 0x08
DM_MASTER_ANY = 0xFF

DM_FB_MAGIC = 0xDA000000
DM_FB_MAGIC_FOUND_OLD = 0xDB000000

DM_PROBE_KIND_NAMES = {
    DM_PROBE_MIT: "mit",
    DM_PROBE_ENABLE: "enable",
    DM_PROBE_DISABLE: "disable",
    DM_PROBE_CLEAR_FAULT: "clear_fault",
    DM_PROBE_REG_SCAN: "reg_scan",
    DM_PROBE_ID_SWEEP: "id_sweep",
}


def build_dm_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    *,
    bus: int = 1,
    master_id: int = DM_MASTER_ANY,
    listen_ms: int = 15,
    param_rid: int = DM_REG_ESC_ID,
    end_id: int = 0,
) -> bytes:
    from .debug_lanes import wrap_mailbox_as_debug_lanes
    from .wire_layout import DEBUG_LANE_DM

    mbox = bytearray(32)
    mbox[0] = DM_TAG0
    mbox[1] = DM_TAG1
    mbox[2] = DM_TAG2
    mbox[3] = motor_id & 0xFF
    mbox[4] = probe_kind & 0xFF
    mbox[5] = master_id & 0xFF
    mbox[6] = listen_ms & 0xFF
    mbox[7] = param_rid & 0xFF
    mbox[8] = end_id & 0xFF
    mbox[11] = normalize_can_bus(bus) & 0xFF
    return wrap_mailbox_as_debug_lanes(seq, mbox, DEBUG_LANE_DM)


def parse_dm_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    from .debug_lanes import extract_dm_mailbox

    pdu = extract_dm_mailbox(frame)
    if pdu[0] != DM_RESP_TAG:
        pdu = frame[PDU_OFF : PDU_OFF + 32]
        if pdu[0] != DM_RESP_TAG:
            return None
    rx_can_id, = struct.unpack_from("<I", pdu, 4)
    param_value, = struct.unpack_from("<I", pdu, 16)
    position, = struct.unpack_from("<f", pdu, 20)
    return {
        "probe_id": pdu[1],
        "found": pdu[2] != 0,
        "probe_kind": pdu[3],
        "rx_can_id": rx_can_id,
        "can_data": bytes(pdu[8:16]),
        "param_value": param_value,
        "position": position,
        "discovered_id": pdu[24],
        "master_id": pdu[25],
        "raw_frames": pdu[26],
        "err": pdu[27],
    }


def dm_fault_is_probe(fault: int) -> bool:
    top = fault & 0xFF000000
    return top in (DM_FB_MAGIC, DM_FB_MAGIC_FOUND_OLD)


def dm_fault_found(fault: int) -> bool:
    top = fault & 0xFF000000
    if top == DM_FB_MAGIC_FOUND_OLD:
        return True
    return bool((fault >> 23) & 1)


def parse_dm_from_actuator(frame: bytes, motor_id: int, slot: int = 2) -> Optional[dict]:
    """Damiao register-scan replies can also land in actuator_feedback[slot] (not just
    the pdu region) — see App/Src/plant/plugins/damiao.c. Same fault-word encoding as
    parse_dm_probe_pdu, decoded from the plant feedback slot instead."""
    act = parse_actuator_feedback(frame, slot)
    if act is None:
        return None
    fault = act["fault"]
    if not dm_fault_is_probe(fault):
        return None
    probed = int(act["velocity"]) & 0xFF
    if probed != (motor_id & 0xFF):
        return None
    esc_id = int(act["torque"]) & 0xFF
    return {
        "probe_id": probed,
        "found": dm_fault_found(fault),
        "probe_kind": DM_PROBE_REG_SCAN,
        "can_data": b"",
        "param_value": esc_id,
        "position": act["position"],
        "discovered_id": esc_id,
        "master_id": int(act["temperature"]) & 0xFF,
        "raw_frames": 0,
        "err": 0,
    }


def probe_kind_matches(resp_kind: int, expect_kind: Optional[int]) -> bool:
    if expect_kind is None:
        return True
    if expect_kind == resp_kind:
        return True
    if expect_kind == DM_PROBE_REG_SCAN and resp_kind in (16, 14):  # REG_SCAN / READ_PARAM
        return True
    return False


# -- CFG bench PDU (actuator table get/set/save) --------------------------------------

CFG_TAG0, CFG_TAG1, CFG_TAG2 = ord("C"), ord("F"), ord("G")
CFG_RESP_TAG0, CFG_RESP_TAG1, CFG_RESP_TAG2 = ord("c"), ord("f"), ord("g")

CFG_OP_GET = 1
CFG_OP_SET = 2
CFG_OP_SAVE = 3
CFG_OP_LOAD = 4
CFG_OP_DEFAULTS = 5
CFG_OP_GET_PERIPH = 6
CFG_OP_SET_PERIPH = 7

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

CFG_FLAG_LISTEN_PDU = 1 << 0
# Periph flags bits 1..2 — SPI3 accessory role (matches plant_config_nvm.h).
CFG_SPI3_ROLE_SHIFT = 1
CFG_SPI3_ROLE_MASK = 3 << CFG_SPI3_ROLE_SHIFT
SPI3_ROLE_LED = 0
SPI3_ROLE_THERMO = 1
SPI3_ROLE_NONE = 2
SPI3_ROLE_NAMES = {
    SPI3_ROLE_LED: "led",
    SPI3_ROLE_THERMO: "thermo",
    SPI3_ROLE_NONE: "none",
}


def _parse_spi3_role(value) -> int:
    if isinstance(value, str):
        key = value.strip().lower()
        for code, name in SPI3_ROLE_NAMES.items():
            if name == key or key == str(code):
                return code
        raise ValueError(f"unknown spi3_role {value!r}; expected led|thermo|none")
    return int(value) & 3


def build_cfg_command(
    op: int,
    seq: int,
    *,
    slot: int = 0,
    bus: int = 1,
    protocol: int = 0,
    motor_id: int = 0,
    master_id: int = 0,
    enabled: bool = True,
    mcu_state: int = PLANT_MCU_STATE_NORMAL,
    periph: Optional[dict] = None,
) -> bytes:
    """CFG on debug lane 7 (legacy pdb mailbox still accepted by FW)."""
    from .debug_lanes import wrap_mailbox_as_debug_lanes
    from .wire_layout import DEBUG_LANE_CFG

    mbox = bytearray(32)
    mbox[0] = CFG_TAG0
    mbox[1] = CFG_TAG1
    mbox[2] = CFG_TAG2
    mbox[3] = op & 0xFF
    mbox[4] = slot & 0xFF
    if op == CFG_OP_SET_PERIPH and periph is not None:
        flags = int(periph.get("flags", 0)) & 0xFF
        if "listen_pdu" in periph:
            if periph["listen_pdu"]:
                flags |= CFG_FLAG_LISTEN_PDU
            else:
                flags &= ~CFG_FLAG_LISTEN_PDU
        if "spi3_role" in periph:
            role = _parse_spi3_role(periph["spi3_role"])
            flags = (flags & ~CFG_SPI3_ROLE_MASK) | ((role << CFG_SPI3_ROLE_SHIFT) & CFG_SPI3_ROLE_MASK)
        mbox[8] = flags & 0xFF
        servos = periph.get("servos") or []
        for i in range(2):
            s = servos[i] if i < len(servos) else {}
            off = 9 + i * 8
            mbox[off + 0] = int(s.get("model", 0)) & 0xFF
            mbox[off + 1] = int(s.get("id", 0)) & 0xFF
            mbox[off + 2] = 1 if s.get("enabled", False) else 0
            pmin = int(s.get("pos_min", 0)) & 0xFFFF
            pmax = int(s.get("pos_max", 0)) & 0xFFFF
            mbox[off + 4] = pmin & 0xFF
            mbox[off + 5] = (pmin >> 8) & 0xFF
            mbox[off + 6] = pmax & 0xFF
            mbox[off + 7] = (pmax >> 8) & 0xFF
        led = periph.get("led") or {}
        off = 9 + 16
        count = int(led.get("default_count", 300)) & 0xFFFF
        mbox[off + 0] = count & 0xFF
        mbox[off + 1] = (count >> 8) & 0xFF
        mbox[off + 2] = int(led.get("default_mode", 8)) & 0xFF
        mbox[off + 3] = int(led.get("default_brightness", 8)) & 0xFF
    else:
        mbox[8] = bus & 0xFF
        mbox[9] = protocol & 0xFF
        mbox[10] = motor_id & 0xFF
        mbox[11] = master_id & 0xFF
        mbox[12] = 1 if enabled else 0
    _ = mcu_state  # debug-lanes RPC is observe (plant_apply off) on FW; kept for call-site compat
    return wrap_mailbox_as_debug_lanes(seq, mbox, DEBUG_LANE_CFG)


def parse_cfg_feedback(pdu: bytes) -> Optional[dict]:
    """Parse a CFG response (32 B DEBUG mailbox slice at PDU_OFF).

    Actuator GET: paginated 3 B slots. GET/SET_PERIPH: flags + 2×servo + LED.
    """
    if len(pdu) < 8:
        return None
    if pdu[0] != CFG_RESP_TAG0 or pdu[1] != CFG_RESP_TAG1 or pdu[2] != CFG_RESP_TAG2:
        return None
    op = pdu[3] & 0x7F
    status = pdu[4]
    if op in (CFG_OP_GET_PERIPH, CFG_OP_SET_PERIPH):
        flags = pdu[8]
        servos = []
        for i in range(2):
            off = 9 + i * 8
            servos.append(
                {
                    "slot": i,
                    "model": pdu[off],
                    "id": pdu[off + 1],
                    "enabled": bool(pdu[off + 2]),
                    "pos_min": pdu[off + 4] | (pdu[off + 5] << 8),
                    "pos_max": pdu[off + 6] | (pdu[off + 7] << 8),
                }
            )
        loff = 9 + 16
        spi3_role = (flags & CFG_SPI3_ROLE_MASK) >> CFG_SPI3_ROLE_SHIFT
        return {
            "op": op,
            "status": status,
            "status_name": CFG_STATUS_NAMES.get(status, f"unknown({status})"),
            "flags": flags,
            "listen_pdu": bool(flags & CFG_FLAG_LISTEN_PDU),
            "spi3_role": spi3_role,
            "spi3_role_name": SPI3_ROLE_NAMES.get(spi3_role, f"unknown({spi3_role})"),
            "servos": servos,
            "led": {
                "default_count": pdu[loff] | (pdu[loff + 1] << 8),
                "default_mode": pdu[loff + 2],
                "default_brightness": pdu[loff + 3],
            },
        }

    count = pdu[5]
    total_count = pdu[-2]
    start_slot = pdu[-1]
    max_fit = (len(pdu) - 6 - 2) // 3
    slots = []
    for i in range(min(count, max_fit)):
        off = 6 + i * 3
        bus = pdu[off]
        packed = pdu[off + 1]
        motor_id = pdu[off + 2]
        slots.append(
            {
                "slot": start_slot + i,
                "bus": bus,
                "protocol": packed & 0x7F,
                "motor_id": motor_id,
                "enabled": bool(packed & 0x80),
            }
        )
    return {
        "op": op,
        "status": status,
        "status_name": CFG_STATUS_NAMES.get(status, f"unknown({status})"),
        "slot_count": count,
        "total_count": total_count,
        "start_slot": start_slot,
        "slots": slots,
    }
