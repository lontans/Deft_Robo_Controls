#!/usr/bin/env python3
"""
RS-02 CAN bench tool over USB (562 B host image).

Modes:
  --discover   Find motor CAN ID (new motor / unknown address) — recommended
  --exercise   Probe one known motor ID many ways
  --param-scan Sweep 0x7005..0x702E (comm 0x11 pararead)
  (default)    Legacy ID scan (--promisc recommended)

Requires firmware with plant_diag (PDU tag RS2).

Examples:
  python scripts/rs02_can_scan.py --list-ports
  python scripts/rs02_can_scan.py --port COM9 --link-test
  python scripts/rs02_can_scan.py --port COM9 --discover --bus 4 --debug-link
  python scripts/rs02_can_scan.py --port COM9 --discover --discover-all
  python scripts/rs02_can_scan.py --port COM9 --discover --discover-quick
  python scripts/rs02_can_scan.py --port COM9 --discover --start 0x40 --end 0x80
  python scripts/rs02_can_scan.py --port COM9 --probe-id 0x7F
  python scripts/rs02_can_scan.py --port COM9 --mcp-smoke --bus 4
  python scripts/rs02_can_scan.py --port COM9 --mcp-smoke --bus 5
  python scripts/rs02_can_scan.py --port COM9 --mcp-smoke-all
  python scripts/rs02_can_scan.py --port COM9 --mcp-wake --bus 4
  python scripts/rs02_can_scan.py --port COM9 --ch4-discover
  python scripts/rs02_can_scan.py --port COM9 --discover --bus 4 --discover-deep --reset-first
  python scripts/rs02_can_scan.py --port COM9 --bench-cmds --bus 3 --target 0x74
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Set, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)

HOST_COMMAND_MAGIC = 0x434D4448
HOST_FEEDBACK_MAGIC = 0x46424848
HOST_LAYOUT_VERSION = 1
IMAGE_BYTES = 562
ACTUATOR0_OFF = 16
ACTUATOR_SLOT_BYTES = 20
HOST_EXCHANGE_ACTUATOR_SLOTS = 25
HOST_EXCHANGE_SERVO_SLOTS = 2
HOST_EXCHANGE_LED_SLOTS = 1
# plant_command.h — system.mcu_state @ command image byte offset 12
PLANT_MCU_STATE_NORMAL = 0
PLANT_MCU_STATE_RECOVERY = 1
PLANT_MCU_STATE_DIAG_ONLY = 2
PLANT_MCU_STATE_ESTOP = 3
SYSTEM_CMD_OFF = 12
SERVO_SLOT_BYTES = 6
SERVO0_CMD_OFF = ACTUATOR0_OFF + HOST_EXCHANGE_ACTUATOR_SLOTS * ACTUATOR_SLOT_BYTES
SERVO0_FB_OFF = SERVO0_CMD_OFF
PDU_OFF = 530
# RS2 bench PDU: pdu.data[11] = schematic bus 1..6 (CH1 FDCAN .. CH6 MCP2518).
PDU_BUS_OFF = PDU_OFF + 11
MIN_CAN_BUS = 1
MAX_CAN_BUS = 6

# Mirror App/Src/plant/plant_config.c — (schematic_bus, motor_id) per actuator slot.
PLANT_ACTUATOR_TABLE: Tuple[Tuple[int, int], ...] = (
    (1, 0x76),  # slot 0 CH1 FDCAN
    (1, 0x74),  # slot 1 CH1 FDCAN
    (2, 0x73),  # slot 2 CH2 FDCAN
    (4, 0x70),  # slot 3 CH4 MCP2518 SPI-CAN
)
PLANT_ACTUATOR_MOTOR_IDS: Tuple[int, ...] = tuple(mid for _, mid in PLANT_ACTUATOR_TABLE)

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
SESSION_BEGIN = 254
SESSION_END = 255

# RobStride private protocol: comm type lives in ext_id bits 24-28 (mode field).
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


def rs2_comm_label(comm_mode: int) -> str:
    name = RS2_COMM_NAMES.get(comm_mode)
    if name is None:
        return f"0x{comm_mode:02X}"
    return f"0x{comm_mode:02X} {name}"


def normalize_can_bus(bus: int) -> int:
    b = int(bus)
    if b < MIN_CAN_BUS or b > MAX_CAN_BUS:
        raise ValueError(f"CAN bus must be {MIN_CAN_BUS}..{MAX_CAN_BUS}, got {b}")
    return b


def can_bus_label(bus: int) -> str:
    b = normalize_can_bus(bus)
    pins = {
        1: "PB8/9 FDCAN1",
        2: "PA8/PA15 FDCAN3",
        3: "PB12/13 FDCAN2",
        4: "PB11 MCP SPI-CAN",
        5: "PB1 MCP SPI-CAN",
        6: "PA4 MCP SPI-CAN",
    }
    return f"CH{b} ({pins[b]})"


def probe_target_label(motor_id: int, bus: int) -> str:
    return f"0x{motor_id & 0xFF:02X} on {can_bus_label(bus)}"


def can_activity_led(bus: int) -> str:
    pins = {
        1: "PC7",
        2: "PC6",
        3: "PB15",
        4: "PB14",
        5: "PB2",
        6: "PC5",
    }
    b = normalize_can_bus(bus)
    return f"{pins[b]} (CH{b} ACT)"


def print_can_bus_note(bus: int) -> None:
    b = normalize_can_bus(bus)
    led = can_activity_led(b)
    if b in MCP_CAN_BUSES:
        print(
            f"Probe target: MCP2518 {can_bus_label(b)}; CAN activity → {led}."
        )
        print("MCP: longer per-probe listen; bus-off clears at RS2 session begin.")
    else:
        print(f"Probe target: {can_bus_label(b)}; CAN activity → {led}.")
    print(describe_actuator_can_sources())


def patch_pdu_bus(buf: bytearray, bus: int) -> None:
    buf[PDU_BUS_OFF] = normalize_can_bus(bus) & 0xFF


DEFAULT_BAUD = 115200
DEFAULT_TARGET = 0x70
STM32_VID = 0x0483

COMMON_IDS = (
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x10, 0x11, 0x20, 0x7F, 0xFD,
)

# Deep-wake first on these (0x7F→0x70, then 0x01→0x7F). Previously 0x30–0x6E were light-only.
_DISCOVER_PRIORITY_ORDER: List[int] = list(range(0x7F, 0x6F, -1)) + list(range(0x01, 0x80))
DISCOVER_PRIORITY_IDS: Tuple[int, ...] = tuple(dict.fromkeys(_DISCOVER_PRIORITY_ORDER))

# Pre-reset broadcast targets on MCP SPI-CAN rails (CH4–6).
MCP_RESET_IDS: Tuple[int, ...] = (0x7F, 0x7E, 0x7D, 0x7C, 0x7B, 0x70, 0x01)

MCP_CAN_BUSES = frozenset({4, 5, 6})

# Per probed ID: (label, probe_kind, timeout_s, repeat)
DISCOVER_PROBES_LIGHT: Tuple[Tuple[str, int, float, int], ...] = (
    ("enable", PROBE_ENABLE_ONLY, 0.55, 1),
    ("promisc", PROBE_PROMISC, 0.40, 1),
)

# Same wake path as --bench-cmds (proven on your 0x70 actuator).
DISCOVER_PROBES_DEEP: Tuple[Tuple[str, int, float, int], ...] = (
    ("reset", PROBE_RESET, 0.45, 1),
    ("enable", PROBE_ENABLE_ONLY, 0.55, 1),
    ("full", PROBE_FULL, 0.35, 1),
    ("ctrl", PROBE_CTRL_ONLY, 0.25, 8),
)

# name, probe_kind, timeout_s, repeat, pause_after_s
EXERCISE_SEQUENCE: Tuple[Tuple[str, int, float, int, float], ...] = (
    ("reset", PROBE_RESET, 0.45, 1, 0.20),
    ("full_init (run_mode+enable+ctrl)", PROBE_FULL, 0.35, 1, 0.12),
    ("enable+ctrl", PROBE_ENABLE_CTRL, 0.30, 1, 0.10),
    ("ctrl_only", PROBE_CTRL_ONLY, 0.28, 1, 0.08),
    ("ctrl_hold x5", PROBE_CTRL_ONLY, 0.25, 5, 0.05),
    ("promisc_full", PROBE_PROMISC, 0.35, 1, 0.10),
    ("ctrl_burst x15", PROBE_CTRL_ONLY, 0.20, 15, 0.03),
)

WAKE_SEQUENCE: Tuple[Tuple[str, int, float, int, float], ...] = (
    ("reset", PROBE_RESET, 0.45, 1, 0.30),
    ("enable_only (listen 200ms)", PROBE_ENABLE_ONLY, 0.55, 1, 0.20),
    ("full_init", PROBE_FULL, 0.35, 1, 0.15),
    ("ctrl_hold x10", PROBE_CTRL_ONLY, 0.25, 10, 0.05),
    ("ctrl_burst x30", PROBE_CTRL_ONLY, 0.15, 30, 0.03),
)

# RobStride section-4 runtime table (0x7005..0x702E). Names from MotorBridge / protocol docs.
RS02_PARAM_HINTS: Dict[int, Tuple[str, str]] = {
    0x7005: ("run_mode", "u8"),
    0x7006: ("iq_ref", "f32"),
    0x700A: ("spd_ref", "f32"),
    0x700B: ("limit_torque", "f32"),
    0x7010: ("cur_kp", "f32"),
    0x7011: ("cur_ki", "f32"),
    0x7014: ("cur_filter_gain", "f32"),
    0x7016: ("loc_ref", "f32"),
    0x7017: ("limit_spd", "f32"),
    0x7018: ("limit_cur", "f32"),
    0x7019: ("mechPos", "f32"),
    0x701A: ("iqf", "f32"),
    0x701B: ("mechVel", "f32"),
    0x701C: ("VBUS", "f32"),
    0x701E: ("loc_kp", "f32"),
    0x701F: ("spd_kp", "f32"),
    0x7020: ("spd_ki", "f32"),
    0x7021: ("spd_filter_gain", "f32"),
    0x7022: ("acc_rad", "f32"),
    0x7024: ("vel_max", "f32"),
    0x7025: ("acc_set", "f32"),
    0x7026: ("EPScan_time", "u16"),
    0x7028: ("canTimeout", "u32"),
    0x7029: ("zero_sta", "u8"),
    0x702A: ("damper", "u8"),
    0x702B: ("add_offset", "f32"),
    0x702C: ("alveolous_open", "u8"),
    0x702D: ("iq_test", "u8"),
    0x702E: ("dcc_set", "f32"),
}

DEFAULT_PARAM_START = 0x7005
DEFAULT_PARAM_END = 0x702E

ACTUATOR_PROFILES: Tuple[Tuple[str, float, float, float, float], ...] = (
    ("zero_kp50 (monitor default)", 0.0, 0.0, 50.0, 1.0),
    ("zero_kp5 (gentle probe)", 0.0, 0.0, 5.0, 0.2),
    ("zero_kp0_kd1", 0.0, 0.0, 0.0, 1.0),
    ("pos+0.05_kp50", 0.05, 0.0, 50.0, 1.0),
)


@dataclass
class ExtIdInfo:
    raw: int
    id_byte: int
    data16: int
    mode: int
    motor_id: int
    status_byte: int
    faults: List[str]
    mode_status: int


class FrameReader:
    def __init__(self) -> None:
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._frames: Deque[bytes] = deque(maxlen=64)

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        with self._lock:
            self._buf.extend(chunk)
            magic_bytes = struct.pack("<I", HOST_FEEDBACK_MAGIC)
            while len(self._buf) >= IMAGE_BYTES:
                if self._buf[:4] != magic_bytes:
                    idx = self._buf.find(magic_bytes)
                    if idx <= 0:
                        self._buf.clear()
                        break
                    del self._buf[:idx]
                    continue
                self._frames.append(bytes(self._buf[:IMAGE_BYTES]))
                del self._buf[:IMAGE_BYTES]

    def pop(self) -> Optional[bytes]:
        with self._lock:
            return self._frames.popleft() if self._frames else None

    def drain(self) -> List[bytes]:
        out: List[bytes] = []
        while True:
            frame = self.pop()
            if frame is None:
                break
            out.append(frame)
        return out


def decode_ext_id(ext_id: int) -> ExtIdInfo:
    id_byte = ext_id & 0xFF
    data16 = (ext_id >> 8) & 0xFFFF
    mode = (ext_id >> 24) & 0x1F
    motor_id = data16 & 0xFF
    status_byte = (data16 >> 8) & 0xFF
    fault_map = (
        (0x01, "under_voltage"),
        (0x02, "drive_fault"),
        (0x04, "over_temp"),
        (0x08, "mag_encoder"),
        (0x10, "stall_overload"),
        (0x20, "uncalibrated"),
    )
    faults = [name for mask, name in fault_map if status_byte & mask]
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


def build_scan_command(motor_id: int, probe_kind: int, seq: int, bus: int = 1) -> bytes:
    """RS2 PDU only — actuator_commands stay zero unless send_diag patches them."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    buf[PDU_OFF + 0] = ord("R")
    buf[PDU_OFF + 1] = ord("S")
    buf[PDU_OFF + 2] = ord("2")
    buf[PDU_OFF + 3] = motor_id & 0xFF
    buf[PDU_OFF + 4] = probe_kind & 0xFF
    patch_pdu_bus(buf, bus)
    return bytes(buf)


def actuator_slot_for_motor(motor_id: int, bus: int = 1) -> int:
    """Map --target + --bus to plant_config actuator_table slot."""
    mid = motor_id & 0xFF
    b = normalize_can_bus(bus)
    for slot, (configured_bus, configured_id) in enumerate(PLANT_ACTUATOR_TABLE):
        if configured_bus == b and configured_id == mid:
            return slot
    for slot, (_, configured_id) in enumerate(PLANT_ACTUATOR_TABLE):
        if configured_id == mid:
            return slot
    return 0


def actuator_slot_offset(slot: int) -> int:
    return ACTUATOR0_OFF + slot * ACTUATOR_SLOT_BYTES


def patch_actuator_desire(
    buf: bytearray,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    torque: float = 0.0,
    slot: int = 0,
) -> None:
    struct.pack_into(
        "<fffff",
        buf,
        actuator_slot_offset(slot),
        position,
        velocity,
        kp,
        kd,
        torque,
    )


def build_rs2_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    param_index: int = 0,
    param_raw_value: int = 0,
    position: float = 0.0,
    kp: float = 0.0,
    kd: float = 1.0,
    bus: int = 1,
) -> bytes:
    buf = bytearray(build_scan_command(motor_id, probe_kind, seq, bus=bus))
    if position != 0.0 or kp != 0.0 or kd != 0.0:
        patch_actuator_desire(buf, position=position, kp=kp, kd=kd)
    buf[PDU_OFF + 5] = param_index & 0xFF
    buf[PDU_OFF + 6] = (param_index >> 8) & 0xFF
    buf[PDU_OFF + 7] = param_raw_value & 0xFF
    buf[PDU_OFF + 8] = (param_raw_value >> 8) & 0xFF
    buf[PDU_OFF + 9] = (param_raw_value >> 16) & 0xFF
    buf[PDU_OFF + 10] = (param_raw_value >> 24) & 0xFF
    return bytes(buf)


def build_actuator_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    seq: int,
    slot: int = 0,
) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    patch_actuator_desire(buf, position, velocity, kp, kd, torque, slot=slot)
    return bytes(buf)


def servo_slot_cmd_offset(slot: int) -> int:
    return SERVO0_CMD_OFF + slot * SERVO_SLOT_BYTES


def servo_slot_fb_offset(slot: int) -> int:
    return SERVO0_FB_OFF + slot * SERVO_SLOT_BYTES


def pack_servo_cmd_flags(
    servo_id: int,
    torque_enable: int = 1,
    operating_mode: int = 3,
) -> int:
    """host_servo_command_t bitfield (torque, mode, id)."""
    return (
        (torque_enable & 1)
        | ((operating_mode & 7) << 2)
        | ((servo_id & 0xFF) << 5)
    )


def patch_servo_command(
    buf: bytearray,
    slot: int,
    position: int,
    servo_id: int,
    torque_enable: int = 1,
    speed: int = 0,
    operating_mode: int = 3,
) -> None:
    off = servo_slot_cmd_offset(slot)
    flags = pack_servo_cmd_flags(servo_id, torque_enable, operating_mode)
    struct.pack_into("<hhH", buf, off, position, speed, flags)


def build_plant_servo_command(
    seq: int,
    slot_positions: Optional[Dict[int, Tuple[int, int]]] = None,
) -> bytes:
    """Plant teleop frame with servos[] only (pdu zero — MCU 500 Hz servo path)."""
    buf = bytearray(build_plant_command(seq, {}))
    if slot_positions:
        for slot, (position, servo_id) in slot_positions.items():
            patch_servo_command(buf, slot, position, servo_id)
    return bytes(buf)


def parse_servo_feedback(frame: bytes, slot: int = 0) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    off = servo_slot_fb_offset(slot)
    if off + SERVO_SLOT_BYTES > IMAGE_BYTES:
        return None
    present, speed, flags = struct.unpack_from("<hhH", frame, off)
    return {
        "present_position": present,
        "present_speed": speed,
        "motor_source_id": (flags >> 5) & 0xFF,
        "moving": flags & 1,
    }


def patch_system_mcu_state(buf: bytearray, mcu_state: int) -> None:
    """host_system_command_t.mcu_state lives in bits 1..3 of the u32 at offset 12."""
    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~0x0E) | ((int(mcu_state) & 7) << 1)
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def build_plant_command(
    seq: int,
    slot_commands: Optional[Dict[int, Tuple[float, float, float, float, float]]] = None,
    mcu_state: int = PLANT_MCU_STATE_NORMAL,
) -> bytes:
    """562 B plant teleop: actuator_commands[] only (no RS2 PDU). 500 Hz path on MCU."""
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    patch_system_mcu_state(buf, mcu_state)
    if slot_commands:
        for slot, (pos, vel, kp, kd, tau) in slot_commands.items():
            patch_actuator_desire(buf, pos, vel, kp, kd, tau, slot=slot)
    return bytes(buf)


def describe_actuator_can_sources() -> str:
    lines = [
        "Configured actuator slots (500 Hz loop when NOT in RS2 session):",
    ]
    for slot, (bus, mid) in enumerate(PLANT_ACTUATOR_TABLE):
        lines.append(f"  slot {slot}: 0x{mid:02X} on {can_bus_label(bus)}")
    lines.append(
        "RS2 --bus only selects plant_diag probe target; it does NOT disable other slots."
    )
    return "\n".join(lines)


def build_rs2_teleop_command(
    motor_id: int,
    probe_kind: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    seq: int,
    bus: int = 1,
) -> bytes:
    """Actuator desire + RS2 PDU in one 562 B image (firmware probe uses both)."""
    buf = bytearray(
        build_actuator_command(position, velocity, kp, kd, torque, seq)
    )
    buf[PDU_OFF + 0] = ord("R")
    buf[PDU_OFF + 1] = ord("S")
    buf[PDU_OFF + 2] = ord("2")
    buf[PDU_OFF + 3] = motor_id & 0xFF
    buf[PDU_OFF + 4] = probe_kind & 0xFF
    patch_pdu_bus(buf, bus)
    return bytes(buf)


PDU_TAG_NAMES = {
    "r": "RS2 bench reply",
    "S": "servo diag (SVD)",
    "d": "Dynamixel probe",
    "R": "RS2 cmd (should not appear in FB)",
}


def parse_feedback_header(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, layout_version, byte_size, fb_seq = struct.unpack_from("<IHHI", frame, 0)
    sys_word, = struct.unpack_from("<I", frame, 12)
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    tag = chr(pdu[0]) if 32 <= pdu[0] < 127 else f"0x{pdu[0]:02X}"
    probe = parse_probe_pdu(frame)
    return {
        "magic_ok": magic == HOST_FEEDBACK_MAGIC,
        "magic_hex": f"0x{magic:08X}",
        "layout_version": layout_version,
        "byte_size": byte_size,
        "fb_seq": fb_seq,
        "tick": sys_word & 0xFFF,
        "last_cmd_seq": (sys_word >> 17) & 0xFF,
        "pdu_tag": tag,
        "pdu_tag_name": PDU_TAG_NAMES.get(tag, "unknown"),
        "pdu_head_hex": pdu[:12].hex(),
        "probe": probe,
    }


def format_feedback_line(index: int, hdr: dict, pdu_hex: Optional[str] = None) -> str:
    probe = hdr.get("probe")
    probe_s = ""
    if probe is not None:
        probe_s = (
            f"  probe_id=0x{probe['probe_id']:02X} kind={probe['probe_kind']} "
            f"found={int(probe['found'])} comm={probe['comm_mode']}"
        )
    elif hdr["pdu_tag"] == "S":
        probe_s = "  (servo SVD — MCU idle, no RS2 session)"
    magic_s = "OK" if hdr["magic_ok"] else f"BAD {hdr['magic_hex']}"
    line = (
        f"  [{index:3d}] magic={magic_s}  lv={hdr['layout_version']}  "
        f"size={hdr['byte_size']}  tick={hdr['tick']:4d}  "
        f"ack_seq={hdr['last_cmd_seq']:3d}  pdu={hdr['pdu_tag']} ({hdr['pdu_tag_name']})"
        f"{probe_s}"
    )
    if pdu_hex is not None:
        line += f"\n         pdu[0:32]={pdu_hex}"
    return line


def frame_pdu_hex(frame: bytes) -> str:
    return frame[PDU_OFF : PDU_OFF + 32].hex()


def collect_frames(reader: FrameReader, duration_s: float) -> List[bytes]:
    deadline = time.monotonic() + max(duration_s, 0.0)
    out: List[bytes] = []
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            out.append(frame)
            frame = reader.pop()
        time.sleep(0.005)
    return out


def summarize_frame_batch(frames: List[bytes], dump_pdu: bool = False, max_lines: int = 32) -> None:
    if not frames:
        print("  (no 562 B feedback frames in window)")
        return
    print(f"  {len(frames)} feedback frame(s):")
    tags: Dict[str, int] = {}
    for fr in frames:
        hdr = parse_feedback_header(fr)
        if hdr is None:
            continue
        tags[hdr["pdu_tag"]] = tags.get(hdr["pdu_tag"], 0) + 1
    tag_summary = ", ".join(f"{k}×{v}" for k, v in sorted(tags.items()))
    print(f"  PDU tags: {tag_summary}")
    for i, fr in enumerate(frames[:max_lines]):
        hdr = parse_feedback_header(fr)
        if hdr is None:
            print(f"  [{i:3d}] (unparseable {len(fr)} B)")
            continue
        line = format_feedback_line(
            i,
            hdr,
            pdu_hex=frame_pdu_hex(fr) if dump_pdu else None,
        )
        print(line)
    if len(frames) > max_lines:
        print(f"  ... {len(frames) - max_lines} more (use --dump-frames to raise limit)")


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in sorted(ports, key=lambda x: x.device):
        hint = ""
        if p.vid == STM32_VID:
            hint = "  <- likely STM32 USB CDC (use this, not ST-Link VCP if both appear)"
        vid_s = f"0x{p.vid:04X}" if p.vid is not None else "n/a"
        print(f"  {p.device:16s}  vid={vid_s:>6}  {p.description}{hint}")


def describe_open_port(port: str) -> None:
    for p in list_ports.comports():
        if p.device == port:
            print(
                f"Port {port}: {p.description}  "
                f"vid=0x{(p.vid or 0):04X} pid=0x{(p.pid or 0):04X}"
            )
            if p.vid != STM32_VID:
                print(
                    "  Warning: VID is not STM32 0x0483 — may be ST-Link VCP, not USB CDC."
                )
            return
    print(f"Port {port}: (not in current port list — may still work)")


def wait_probe_response_collect(
    reader: FrameReader,
    probe_id: int,
    timeout_s: float,
    probe_kind: Optional[int] = None,
) -> Tuple[Optional[dict], List[bytes]]:
    deadline = time.monotonic() + timeout_s
    seen: List[bytes] = []
    matched: Optional[dict] = None
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            seen.append(frame)
            parsed = parse_probe_pdu(frame)
            if parsed is not None:
                if probe_kind in (SESSION_BEGIN, SESSION_END):
                    if parsed["probe_kind"] == probe_kind:
                        matched = parsed
                        return matched, seen
                elif parsed["probe_id"] == (probe_id & 0xFF):
                    if probe_kind is not None and parsed["probe_kind"] != probe_kind:
                        frame = reader.pop()
                        continue
                    matched = parsed
                    return matched, seen
            frame = reader.pop()
        time.sleep(0.005)
    return matched, seen


def run_seq_ack_test(
    ser: serial.Serial,
    reader: FrameReader,
    count: int,
    start_seq: int,
    diag_only: bool = True,
) -> Tuple[int, List[int], List[bytes]]:
    """Send plain plant commands; return (next_seq, acks_seen, frames)."""
    seq = start_seq
    frames: List[bytes] = []
    acks: List[int] = []
    mcu_state = PLANT_MCU_STATE_DIAG_ONLY if diag_only else PLANT_MCU_STATE_NORMAL
    for _ in range(count):
        reader.drain()
        ser.write(build_plant_command(seq, {}, mcu_state=mcu_state))
        ser.flush()
        deadline = time.monotonic() + 0.35
        got_fb = False
        while time.monotonic() < deadline:
            frame = reader.pop()
            while frame is not None:
                frames.append(frame)
                hdr = parse_feedback_header(frame)
                if hdr is not None and hdr["magic_ok"]:
                    acks.append(hdr["last_cmd_seq"])
                    got_fb = True
                frame = reader.pop()
            if got_fb:
                break
            time.sleep(0.005)
        seq += 1
        time.sleep(0.02)
    return seq, acks, frames


def run_link_test(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    listen_s = args.listen_first if args.listen_first > 0.0 else 1.0
    session_timeout = args.session_timeout if args.session_timeout is not None else max(args.timeout, 1.0)
    bus = normalize_can_bus(args.bus)
    dump_limit = max(1, int(args.dump_frames))

    print(f"USB link test on {ser.port} @ {args.baud}  (562 B host image, no CAN probes)")
    describe_open_port(ser.port)
    print(f"Expected feedback magic=0x{HOST_FEEDBACK_MAGIC:08X}  image={IMAGE_BYTES} B")
    print()
    print(describe_actuator_can_sources())
    print()

    try:
        print(f"--- 1) Listen {listen_s:.1f}s (unsolicited MCU feedback) ---")
        idle_frames = collect_frames(reader, listen_s)
        summarize_frame_batch(idle_frames, dump_pdu=args.dump_pdu, max_lines=dump_limit)
        print()

        print("--- 2) Seq ack test (8 plain plant commands, mcu_state=DIAG_ONLY, no CAN) ---")
        print("  (Uses DIAG_ONLY so slots 0–3 do not hit CH1/CH2/CH4 — USB seq path only.)")
        seq, acks, seq_frames = run_seq_ack_test(ser, reader, 8, 1, diag_only=True)
        if not seq_frames:
            print("  FAIL: no feedback frames after plant commands.")
            print("  → Wrong COM port (try --list-ports), MCU not running, or USB not enumerated.")
        else:
            uniq_acks = sorted(set(acks))
            print(f"  Frames: {len(seq_frames)}  last_cmd_seq values: {uniq_acks}")
            if max(acks or [0]) >= 1:
                print("  OK: MCU is receiving host commands (seq ack advancing).")
            else:
                print("  WARN: feedback seen but last_cmd_seq never advanced (seq path broken?).")
        print()

        print(f"--- 3) RS2 session begin (kind={SESSION_BEGIN}, bus={bus}) ---")
        reader.drain()
        cmd = build_scan_command(0, SESSION_BEGIN, seq, bus=bus)
        print(
            f"  TX pdu tag=RS2  motor_id=0x00  kind={SESSION_BEGIN}  "
            f"pdu[11]=bus {bus}  seq={seq}"
        )
        ser.write(cmd)
        ser.flush()
        resp, session_frames = wait_probe_response_collect(
            reader, 0, session_timeout, probe_kind=SESSION_BEGIN
        )
        summarize_frame_batch(session_frames, dump_pdu=args.dump_pdu, max_lines=dump_limit)
        if resp is not None:
            print(
                f"  OK: session begin ack  found={resp['found']}  "
                f"probe_kind={resp['probe_kind']}  probe_id=0x{resp['probe_id']:02X}"
            )
            print(format_mcp_status(resp, bus=bus))
            seq += 1
            print()
            print("--- 4) Session end ---")
            reader.drain()
            ser.write(build_scan_command(0, SESSION_END, seq, bus=bus))
            ser.flush()
            end_resp, end_frames = wait_probe_response_collect(
                reader, 0, session_timeout, probe_kind=SESSION_END
            )
            summarize_frame_batch(end_frames, dump_pdu=args.dump_pdu, max_lines=min(8, dump_limit))
            if end_resp is not None:
                print(f"  OK: session end ack  probe_kind={end_resp['probe_kind']}")
            else:
                print("  WARN: session end not acked (often harmless)")
        else:
            print("  FAIL: no RS2 session begin ack within {:.2f}s.".format(session_timeout))
            if session_frames:
                hdrs = [parse_feedback_header(f) for f in session_frames]
                hdrs = [h for h in hdrs if h is not None]
                if hdrs and all(h["pdu_tag"] == "S" for h in hdrs):
                    print(
                        "  → All PDUs are servo SVD — MCU did not process RS2 command "
                        "(wrong image layout / old firmware / command not reaching plant_diag?)."
                    )
                elif not session_frames:
                    pass
                else:
                    kinds = [
                        h["probe"]["probe_kind"]
                        for h in hdrs
                        if h.get("probe") is not None
                    ]
                    if kinds:
                        print(f"  → Saw RS2 replies but probe_kind in {kinds} (want {SESSION_BEGIN}).")
            else:
                print("  → No feedback at all — check COM port and USB cable.")
            print()
            print("Paste this full output when debugging.")
            return 1

        print()
        print("Link test passed. Retry discover with --debug-link for per-probe detail.")
        return 0
    finally:
        stop.set()


def mcp_rail_bit(bus: int) -> int:
    b = normalize_can_bus(bus)
    if b not in MCP_CAN_BUSES:
        return 0
    return 1 << (b - 4)


def mcp_rail_opmod(resp: dict) -> int:
    return int(resp.get("mcp_rail_opmod", resp.get("mcp_ch4_opmod", 0xFF))) & 0xFF


def format_mcp_status(resp: Optional[dict], bus: int = 4) -> str:
    if resp is None:
        return "  mcp_init_mask=?  mcp_opmod=?"
    bus = normalize_can_bus(bus)
    if bus not in MCP_CAN_BUSES:
        tx_ok = int(resp.get("mcp_tx_ok", 0)) & 0xFF
        rx = int(resp.get("raw_frames", 0)) & 0xFF
        return (
            f"  FDCAN {can_bus_label(bus)} — MCP2518 fields not applicable "
            f"(use --bus 4|5|6).  tx_ok={tx_ok} rx={rx}"
        )
    mask = int(resp.get("mcp_init_mask", 0)) & 0xFF
    opmod = mcp_rail_opmod(resp)
    rail_bit = mcp_rail_bit(bus)
    tx_ok = int(resp.get("mcp_tx_ok", 0)) & 0xFF
    tx_fail = int(resp.get("mcp_tx_fail", 0)) & 0xFF
    tx_nack = int(resp.get("mcp_tx_nack", 0)) & 0xFF
    tec = int(resp.get("mcp_tec", 0)) & 0xFF
    tec_before = int(resp.get("mcp_tec_before", 0)) & 0xFF
    tec_delta = tec - tec_before if tec >= tec_before else tec
    rx = int(resp.get("raw_frames", 0)) & 0xFF
    led = can_activity_led(bus)
    tec_note = f" Δ+{tec_delta}" if tec_delta > 0 else ""
    rec = int(resp.get("mcp_rec", 0)) & 0xFF
    ext_lb = int(resp.get("mcp_ext_loopback_ok", 0)) & 0xFF
    if mask == 0x07:
        mask_hint = "rails CH4–CH6 (0x07)"
    elif mask == 0x00:
        mask_hint = "no rail initialized"
    else:
        rails = [str(4 + i) for i in range(3) if (mask >> i) & 1]
        mask_hint = f"rails {','.join(rails) or '?'} (0x{mask:02X})"
    opmod_label = f"{opmod}" if opmod != 0xFF else "n/a"
    line = (
        f"  mcp_init_mask=0x{mask:02X} ({mask_hint})  "
        f"CH{bus}_opmod={opmod_label} (6=CAN2.0 normal)  "
        f"tx_ok={tx_ok} tx_fail={tx_fail} tx_nack={tx_nack} tec={tec}{tec_note} rx={rx}  "
        f"rec={rec} ext_lb={ext_lb}"
    )
    if (mask & rail_bit) == 0:
        return line + (
            f"\n  → MCP2518 CH{bus} rail not initialized (mask bit clear). "
            f"Check SPI CS/INT for {can_bus_label(bus)}."
        )
    if opmod != 6:
        return line + f"\n  → MCP up but opmod {opmod} != 6"
    probe_kind = int(resp.get("probe_kind", 0)) & 0xFF
    nbt_brp = int(resp.get("mcp_nbt_brp", 0xFF)) & 0xFF
    nbt_tseg1 = int(resp.get("mcp_nbt_tseg1", 0)) & 0xFF
    nbt_unset = nbt_tseg1 == 0 and nbt_brp in (0, 0xFF)
    if probe_kind in (SESSION_BEGIN, SESSION_END) or nbt_unset:
        if tx_ok == 0 and tec > 0:
            return line + (
                f"\n  → MCP rail ready (session TX stats reset). "
                f"TEC={tec} is cumulative from earlier probes/smoke, not this session."
            )
        if rx > 0:
            return line + f"\n  → MCP TX+RX path working — {led} should have blinked"
        if tx_nack > 0 or tec_delta > 0 or tec > 0:
            rec_hint = ""
            if rec == 0:
                rec_hint = (
                    "  REC=0 → MCP2518 RXCAN path may be dead (check MCP2562 RXD→MCP RXCAN). "
                    "Scope can look fine while the controller never receives."
                )
            elif rec > 0 and rx == 0:
                rec_hint = (
                    "  REC>0 but rx=0 → bus traffic reached MCP2518 but SW FIFO/filter path "
                    "may be stuck (reflash with latest RX fixes)."
                )
            ext_hint = ""
            if ext_lb == 0:
                ext_hint = "  ext_lb=0 at init → internal TXCAN↔RXCAN loopback failed."
            return line + (
                f"\n  → Frame reached the bus but got NO ACK (TEC{tec_note}, tx_nack={tx_nack}). "
                f"No node acknowledged on {can_bus_label(bus)}.{rec_hint}{ext_hint}"
            )
        return line
    if nbt_brp != 0 or nbt_tseg1 != 17:
        return line + (
            f"\n  → CiNBTCFG readback wrong (BRP={nbt_brp} expect 0, "
            f"TSEG1={nbt_tseg1} expect 17) — bitrate not 1 Mbps"
        )
    if tx_ok == 0:
        sta = int(resp.get("mcp_txq_sta", 0)) & 0xFF
        con = int(resp.get("mcp_txq_con", 0)) & 0xFF
        c1b2 = int(resp.get("mcp_c1con_b2", 0)) & 0xFF
        oscb1 = int(resp.get("mcp_osc_b1", 0)) & 0xFF
        txqen = "yes" if (c1b2 & 0x10) else "NO"
        oscrdy = "yes" if (oscb1 & 0x04) else "no"
        stuck = ""
        if (sta & 0x50) == 0x50 or ((sta & 0x10) and not (sta & 0x04)):
            stuck = "  TXQ stuck (TXATIF) — reflash MCU if this persists after fix."
        return line + (
            f"\n  → MCP could not load TXQ (txq_sta=0x{sta:02X} txq_con=0x{con:02X} "
            f"TXQEN={txqen} OscReady={oscrdy}).{stuck}"
        )
    if rx > 0:
        return line + f"\n  → MCP TX+RX path working — {led} should have blinked"
    if tx_nack > 0 or tec_delta > 0 or tec > 0:
        osc_b0 = int(resp.get("mcp_osc_b0", 0)) & 0xFF
        bdiag_b0 = int(resp.get("mcp_bdiag1_b0", 0)) & 0xFF
        bdiag_b1 = int(resp.get("mcp_bdiag1_b1", 0)) & 0xFF
        nack_bit = "yes" if (bdiag_b1 & 0x04) else "no"
        pll = "BAD(PLLEN=1)" if (osc_b0 & 0x01) else "ok"
        return line + (
            f"\n  → Frame reached the bus but got NO ACK (TEC{tec_note}, tx_nack={tx_nack}). "
            f"That means MCP2518 transmitted; no node acknowledged. "
            f"osc_pll={pll} nbt=1Mbps bdiag=0x{bdiag_b0:02X}{bdiag_b1:02X} NACKERR={nack_bit}."
        )
    return line + (
        f"\n  → TX completed with TEC unchanged — frame was ACK'd on the bus "
        f"(or driver finished before TEC updated; retry if unsure)."
    )


def parse_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    if pdu[0] != ord("r"):
        return None
    ext_id, = struct.unpack_from("<I", pdu, 4)
    temperature, position = struct.unpack_from("<ff", pdu, 16)
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
        "mcp_init_mask": pdu[27],
        "mcp_rail_opmod": pdu[28],
        "mcp_ch4_opmod": pdu[28],
        "mcp_tx_ok": pdu[29],
        "mcp_tx_fail": pdu[30],
        "mcp_tec": pdu[31],
        "mcp_tec_before": pdu[6],
        "mcp_txq_sta": pdu[8],
        "mcp_txq_con": pdu[9],
        "mcp_c1con_b2": pdu[10],
        "mcp_osc_b1": pdu[11],
        "mcp_osc_b0": pdu[12],
        "mcp_nbt_tseg1": pdu[13],
        "mcp_nbt_brp": pdu[15],
        "mcp_bdiag1_b0": pdu[14],
        "mcp_bdiag1_b1": pdu[7],
        "mcp_tx_nack": pdu[24],
        "mcp_rec": pdu[17],
        "mcp_ext_loopback_ok": pdu[18],
    }


def parse_actuator_feedback(frame: bytes, slot: int = 0) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    sys_word, = struct.unpack_from("<I", frame, 12)
    off = actuator_slot_offset(slot)
    pos, vel, torque, temp, fault = struct.unpack_from("<ffffI", frame, off)
    return {
        "tick": sys_word & 0xFFF,
        "ack": (sys_word >> 17) & 0xFF,
        "position": pos,
        "velocity": vel,
        "torque": torque,
        "temperature": temp,
        "fault": fault,
    }


def serial_rx_thread(ser: serial.Serial, reader: FrameReader, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            chunk = ser.read(max(1, ser.in_waiting))
        except serial.SerialException:
            break
        if chunk:
            reader.feed(chunk)
        else:
            time.sleep(0.001)


def wait_probe_response(
    reader: FrameReader,
    probe_id: int,
    timeout_s: float,
    probe_kind: Optional[int] = None,
) -> Optional[dict]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_probe_pdu(frame)
            if parsed is not None:
                if probe_kind in (SESSION_BEGIN, SESSION_END):
                    if parsed["probe_kind"] == probe_kind:
                        return parsed
                elif parsed["probe_id"] == (probe_id & 0xFF):
                    if probe_kind is not None and parsed["probe_kind"] != probe_kind:
                        frame = reader.pop()
                        continue
                    return parsed
            frame = reader.pop()
        time.sleep(0.005)
    return None


def send_parawrite(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    param_index: int,
    raw_value: int,
    seq: int,
    timeout_s: float,
    bus: int = 1,
) -> Tuple[Optional[dict], int]:
    reader.drain()
    ser.write(
        build_rs2_probe_command(
            motor_id, PROBE_PARAWRITE, seq, param_index, raw_value, bus=bus
        )
    )
    ser.flush()
    resp = wait_probe_response(reader, motor_id, timeout_s, probe_kind=PROBE_PARAWRITE)
    return resp, seq + 1


def send_diag(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    probe_kind: int,
    seq: int,
    timeout_s: float,
    probe_param: int = 0,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    bus: int = 1,
) -> Optional[dict]:
    reader.drain()
    if probe_param != 0:
        ser.write(
            build_rs2_probe_command(
                motor_id,
                probe_kind,
                seq,
                probe_param,
                position=position,
                kp=kp,
                kd=kd,
                bus=bus,
            )
        )
    else:
        buf = bytearray(build_scan_command(motor_id, probe_kind, seq, bus=bus))
        if position != 0.0 or velocity != 0.0 or kp != 0.0 or kd != 0.0:
            patch_actuator_desire(buf, position=position, velocity=velocity, kp=kp, kd=kd)
        ser.write(bytes(buf))
    ser.flush()
    return wait_probe_response(reader, motor_id, timeout_s, probe_kind=probe_kind)


def pararead_index_echo(resp: dict) -> int:
    return resp["can_data"][0] | (resp["can_data"][1] << 8)


def pararead_is_hit(resp: dict, param_index: int) -> bool:
    if resp["comm_mode"] not in (0x11, 0x12):
        return False
    if not resp.get("found") and resp.get("raw_frames", 0) == 0:
        return False
    # RS02 does not echo the param index in data[0:1]; accept any comm=0x11/0x12 response.
    # The float value is always at data[4:7] (LE IEEE 754).
    echo = pararead_index_echo(resp)
    return echo == 0 or echo == (param_index & 0xFFFF)


def send_pararead(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    param_index: int,
    seq: int,
    timeout_s: float,
) -> Tuple[Optional[dict], Optional[dict], bool, int]:
    """Return (comm=0x11/0x12 hit, last_usb_or_sniff, usb_probe_seen, next_seq)."""
    reader.drain()
    ser.write(build_rs2_probe_command(motor_id, PROBE_PARAREAD, seq, param_index))
    ser.flush()
    pararead: Optional[dict] = None
    last_usb: Optional[dict] = None
    usb_seen = False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_probe_pdu(frame)
            if (
                parsed is not None
                and parsed["probe_id"] == (motor_id & 0xFF)
                and parsed["probe_kind"] == PROBE_PARAREAD
            ):
                usb_seen = True
                last_usb = parsed
                if pararead_is_hit(parsed, param_index):
                    return parsed, None, True, seq + 1
            frame = reader.pop()
        time.sleep(0.005)
    sniff = last_usb if last_usb is not None and last_usb.get("raw_frames", 0) > 0 else None
    if sniff is None and last_usb is not None and usb_seen and not pararead_is_hit(last_usb, param_index):
        sniff = last_usb
    return pararead, sniff, usb_seen, seq + 1


def param_hint(index: int) -> str:
    hint = RS02_PARAM_HINTS.get(index)
    return hint[0] if hint is not None else "?"


def format_param_scan_line(
    index: int,
    resp: Optional[dict],
    sniff: Optional[dict],
    usb_ok: bool,
) -> str:
    tag = f"0x{index:04X} {param_hint(index):<16s}"
    if not usb_ok:
        return f"  ----  {tag}  (no USB feedback — MCU busy?)"
    if resp is not None:
        idx_echo = pararead_index_echo(resp)
        echo_note = "(no echo — RS02 normal)" if idx_echo == 0 else f"echo=0x{idx_echo:04X}"
        return (
            f"  HIT   {tag}  {rs2_comm_label(resp['comm_mode'])}  "
            f"{echo_note}  float={resp['position']:+.4f}  "
            f"raw={resp['can_data'].hex()}"
        )
    if sniff is not None:
        raw_n = sniff.get("raw_frames", 0)
        if sniff["comm_mode"] in (0x11, 0x12):
            idx_echo = pararead_index_echo(sniff)
            if idx_echo != 0 and idx_echo != (index & 0xFFFF):
                return (
                    f"  EMPTY {tag}  {rs2_comm_label(sniff['comm_mode'])}  "
                    f"echo=0x{idx_echo:04X} (want 0x{index:04X})  "
                    f"ext=0x{sniff['ext_id']:08X}  raw={sniff['can_data'].hex()}"
                )
        return (
            f"  SNIFF {tag}  saw {rs2_comm_label(sniff['comm_mode'])}  "
            f"rx={raw_n}  ext=0x{sniff['ext_id']:08X}  data={sniff['can_data'].hex()}"
        )
    return f"  ....  {tag}  no comm=0x11 pararead reply"


def parse_param_list(text: str) -> List[int]:
    out: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part, 0))
    return out


def build_param_index_list(args: argparse.Namespace) -> List[int]:
    if args.params:
        return parse_param_list(args.params)
    if args.known_only:
        return sorted(RS02_PARAM_HINTS.keys())
    start = int(args.param_start) & 0xFFFF
    end = int(args.param_end) & 0xFFFF
    if end < start:
        start, end = end, start
    return list(range(start, end + 1))


def run_wake_short(
    ser: serial.Serial,
    reader: FrameReader,
    target: int,
    seq: int,
    timeout: float,
    running: bool = False,
    ctrl_pos: float = 0.0,
    bus: int = 1,
) -> int:
    short_wake: Tuple[Tuple[str, int, float, int, float], ...] = (
        ("reset", PROBE_RESET, 0.45, 1, 0.15),
        ("enable_only", PROBE_ENABLE_ONLY, 0.55, 1, 0.12),
        ("full_init", PROBE_FULL, 0.35, 1, 0.10),
    )
    if running:
        short_wake = short_wake + (("ctrl x12", PROBE_CTRL_ONLY, 0.25, 12, 0.04),)
    label_mode = "running ctrl" if running else "rest/enable (vendor pararead style)"
    if running and ctrl_pos != 0.0:
        label_mode += f" pos={ctrl_pos:+.3f} rad"
    print(f"Wake motor {probe_target_label(target, bus)} before scan ({label_mode})...")
    for name, kind, step_timeout, repeat, pause in short_wake:
        for n in range(repeat):
            label = name if repeat == 1 else f"{name} [{n + 1}/{repeat}]"
            pos = ctrl_pos if kind == PROBE_CTRL_ONLY else 0.0
            ctrl_kp = 50.0 if kind == PROBE_CTRL_ONLY else 0.0
            ctrl_kd = 1.0 if kind == PROBE_CTRL_ONLY else 0.0
            resp = send_diag(
                ser, reader, target, kind, seq, max(timeout, step_timeout),
                position=pos, kp=ctrl_kp, kd=ctrl_kd, bus=bus,
            )
            seq += 1
            if resp is not None and resp.get("found"):
                ext = decode_ext_id(resp["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                fault_s = ",".join(ext.faults) if ext.faults else "none"
                print(
                    f"  {label:<28s}  HIT  ext=0x{resp['ext_id']:08X}  "
                    f"mms={mms}  comm={resp['comm_mode']} ({rs2_comm_label(resp['comm_mode'])})  "
                    f"faults=[{fault_s}]"
                )
            time.sleep(pause)

    # After a non-zero ctrl burst, reset to de-energize before pararead.
    # Without this, robstride_maintain_enable keeps the motor fighting at ctrl_pos.
    if running and ctrl_pos != 0.0:
        send_diag(ser, reader, target, PROBE_RESET, seq, 0.45, bus=bus)
        seq += 1
        time.sleep(0.15)
        print(f"  (reset: de-energize from ctrl_pos={ctrl_pos:+.3f} rad before pararead)")

    print()
    return seq


def run_param_scan(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    bus = normalize_can_bus(args.bus)
    seq = 1
    indices = build_param_index_list(args)
    hits: List[Tuple[int, dict]] = []
    sniffs: List[Tuple[int, dict]] = []
    usb_miss = 0

    print(f"Param scan motor {probe_target_label(target, bus)} on {ser.port}  ({len(indices)} indices)")
    print_can_bus_note(bus)
    print("Success = ext-CAN comm 0x11 pararead with float in data[4:8].")
    print("Sweep like ID scan — HIT / SNIFF / .... per index.")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        if args.reset_first:
            send_diag(ser, reader, target, PROBE_RESET, seq, max(args.timeout, 0.45), bus=bus)
            seq += 1
            time.sleep(0.2)
        seq = run_wake_short(
            ser, reader, target, seq, args.timeout,
            running=args.param_running, ctrl_pos=getattr(args, "ctrl_pos", 0.0), bus=bus,
        )

        for index in indices:
            resp, sniff, usb_seen, seq = send_pararead(
                ser, reader, target, index, seq, args.param_timeout, bus=bus,
            )
            if not usb_seen:
                usb_miss += 1
            print(format_param_scan_line(index, resp, sniff, usb_seen))
            if resp is not None:
                hits.append((index, resp))
            elif sniff is not None:
                sniffs.append((index, sniff))
            time.sleep(max(args.gap, 0.05))
    finally:
        end_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        stop.set()

    print()
    print("Summary")
    print(f"  comm=0x11 pararead HITs: {len(hits)}")
    if hits:
        for index, resp in hits:
            name = RS02_PARAM_HINTS.get(index, ("?", "?"))[0]
            print(f"    0x{index:04X} {name:<16s}  float={resp['position']:+.4f}  raw={resp['can_data'].hex()}")
    print(f"  CAN activity (not valid pararead): {len(sniffs)}")
    if sniffs and args.verbose:
        for index, resp in sniffs:
            print(f"    0x{index:04X}  {rs2_comm_label(resp['comm_mode'])}  raw={resp['can_data'].hex()}")
    if not hits:
        if sniffs and all(r["comm_mode"] == 0x11 for _, r in sniffs):
            print("  Motor sent comm=0x11 responses — RS02 does not echo index, but no hit detected.")
            print("  Check: motor ID correct? MCU firmware flashed with echo-fix?")
        print("  Encoder cal requires 48 V supply. Check --bench-cmds (cali now resets first).")
        print("  Try: --param-running   or   --bench-cmds --cal-timeout 90")
    if usb_miss > len(indices) // 2:
        print(f"  USB probe acks missing on {usb_miss}/{len(indices)} reads — MCU busy; replug / wait 30s")
    return 0 if hits else 1


def run_recovery(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    """Exit stuck mms=cali: power-cycle motor first, then reset → enable (no 0x05 cal)."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    bus = normalize_can_bus(args.bus)
    seq = 1
    print(f"Recovery on {probe_target_label(target, bus)} — reset → enable only (no cal).")
    print_can_bus_note(bus)
    print("Power-cycle motor if still mms=cali.")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        for label, kind, wait_s in (
            ("reset comm 0x04", PROBE_RESET, 0.55),
            ("enable comm 0x03", PROBE_ENABLE_ONLY, 0.55),
            ("full_init", PROBE_FULL, 0.45),
        ):
            resp = send_diag(ser, reader, target, kind, seq, wait_s, bus=bus)
            seq += 1
            print(format_probe_line(label, target, resp))
            if resp and resp.get("found"):
                ext = decode_ext_id(resp["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                print(f"         mms={mms}  faults=[{','.join(ext.faults) or 'none'}]")
            print()
    finally:
        end_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        stop.set()
    return 0


def run_bench_cmds(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    """One-shot cal / zero / save probes (no queuing — MCU blocks per command)."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    bus = normalize_can_bus(args.bus)
    seq = 1
    cal_listen_s = max(10, int(args.cal_timeout))
    cal_usb_wait = float(cal_listen_s + 15)
    bench: Tuple[Tuple[str, int, float, int], ...] = (
        ("motor_cali comm 0x05", PROBE_CALI, cal_usb_wait, cal_listen_s),
        ("motor_zero comm 0x06", PROBE_ZERO, 3.0, 0),
        ("proactive enable 0x18", PROBE_PROACTIVE, 1.0, 0),
        ("data_save comm 0x16", PROBE_DATA_SAVE, 4.0, 0),
    )

    print(f"Bench comm probes on {probe_target_label(target, bus)}  (sequential — one MCU block at a time)")
    print_can_bus_note(bus)
    print(f"Cali listen on MCU: {cal_listen_s}s (one 0x05, no retransmit). Shaft must spin freely.")
    print("Prep: reset only (no enable/ctrl before cal — motor must stay at rest).")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        print("--- prep reset (fault clear) ---")
        resp = send_diag(ser, reader, target, PROBE_RESET, seq, 0.55, bus=bus)
        seq += 1
        print(format_probe_line("reset comm 0x04", target, resp))
        print()
        time.sleep(0.3)

        if not args.skip_iq_test:
            print("--- iq_test parawrite 0x702D=1 (unlock encoder export on teardown units) ---")
            resp, seq = send_parawrite(ser, reader, target, 0x702D, 1, seq, 1.5, bus=bus)
            print(format_probe_line("iq_test 0x702D=1 parawrite", target, resp))
            print()
            time.sleep(0.2)

        for label, kind, probe_timeout, probe_param in bench:
            print(f"--- {label} (wait up to {probe_timeout:.0f}s) ---")
            resp = send_diag(ser, reader, target, kind, seq, probe_timeout, probe_param, bus=bus)
            seq += 1
            print(format_probe_line(label, target, resp))
            if resp is None:
                print("  stopped — MCU did not ack (prior probe still running? wait / replug USB)")
                break
            if resp.get("found"):
                ext = decode_ext_id(resp["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                disc = resp.get("discovered_id") or ext.motor_id
                valid = disc == (target & 0xFF) and resp["comm_mode"] in (0x02, 0x18)
                print(f"         mms={mms}  valid_0x70={'yes' if valid else 'NO'}  data={resp['can_data'].hex()}")
                if kind == PROBE_CALI:
                    if resp.get("comm_mode") != 0x02:
                        print(f"         Comm=0x{resp['comm_mode']:02X} (cali stream?) — no mms field. Motor still calibrating.")
                        print("         Power-cycle motor, wait for shaft to stop, then retry.")
                        break
                    cal_ok = ext.mode_status in (0, 2)
                    if ext.mode_status == 1:
                        print("         MCU listen ended with mms=cali — polling enable (may have missed rest/running)...")
                        poll_deadline = time.monotonic() + max(15.0, cal_listen_s * 0.5)
                        while time.monotonic() < poll_deadline:
                            poll = send_diag(ser, reader, target, PROBE_ENABLE_ONLY, seq, 0.6, bus=bus)
                            seq += 1
                            if poll and poll.get("found"):
                                ext = decode_ext_id(poll["ext_id"])
                                if ext.mode_status in (0, 2):
                                    cal_ok = True
                                    print(format_probe_line("post_cal enable", target, poll))
                                    break
                            time.sleep(0.5)
                        if not cal_ok:
                            print("         CALI TIMED OUT (mms=cali) — sweep still running, 30V too low, or other motor flooding CAN.")
                            print("         Try: --recovery, --skip-iq-test, power-cycle, or host_teleop --calibrate. zero/save skipped.")
                            break
                    elif ext.mode_status == 2:
                        print("         mms=running — cal finished (RS01 often skips rest).")
                    elif ext.mode_status != 0:
                        print("         WARNING: cali did not start (mms not cali/rest/running). Check shaft freedom.")
            print()
            time.sleep(0.2)
    finally:
        end_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        stop.set()
    return 0


def run_proactive_off(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    """Disable proactive mode and save to NVM — fixes 'comm=0x18 on every probe' symptom."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    bus = normalize_can_bus(args.bus)
    seq = 1
    # param_index=1 → firmware sends proactive with enable=0 (disable)
    off_seq: Tuple[Tuple[str, int, float, int], ...] = (
        ("proactive disable 0x18", PROBE_PROACTIVE, 1.0, 1),
        ("data_save comm 0x16", PROBE_DATA_SAVE, 4.0, 0),
    )

    print(f"Disabling proactive mode on {probe_target_label(target, bus)} and saving to NVM...")
    print_can_bus_note(bus)
    print("After this, power-cycle the motor to confirm proactive broadcasts stop.")
    print()
    try:
        seq = begin_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        resp = send_diag(ser, reader, target, PROBE_RESET, seq, 0.55, bus=bus)
        seq += 1
        print(format_probe_line("reset", target, resp))
        time.sleep(0.3)

        for label, kind, probe_timeout, probe_param in off_seq:
            print(f"--- {label} ---")
            resp = send_diag(ser, reader, target, kind, seq, probe_timeout, probe_param, bus=bus)
            seq += 1
            print(format_probe_line(label, target, resp))
            if resp is None:
                print("  stopped — MCU did not ack")
                break
            print()
            time.sleep(0.2)
    finally:
        end_session(ser, reader, seq, args.timeout, bus=bus, args=args)
        stop.set()
    return 0


def format_probe_line(label: str, motor_id: int, resp: Optional[dict]) -> str:
    if resp is None:
        return f"  ----  {label:<32s}  (no USB feedback — MCU busy or not running plant_diag?)"

    raw_n = resp.get("raw_frames", 0)
    if not resp["found"] and raw_n == 0:
        return (
            f"  ....  {label:<32s}  MCU replied: no ext-CAN RX  "
            f"(probe_kind={resp['probe_kind']} raw={raw_n} — PC7 may still blink on TX)"
        )

    ext = decode_ext_id(resp["ext_id"])
    disc = resp["discovered_id"] or ext.motor_id or (motor_id & 0xFF)
    valid = disc == (motor_id & 0xFF) and resp["comm_mode"] in (0x02, 0x18)
    fault_s = ",".join(ext.faults) if ext.faults else "none"
    mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else f"mode{ext.mode_status}"
    tag = "HIT" if resp["found"] else "SNIFF"
    if resp["found"] and not valid:
        tag = "NOISE"
    data_hex = resp["can_data"].hex()

    return (
        f"  {tag}  {label:<32s}  ext=0x{resp['ext_id']:08X}  "
        f"motor=0x{disc:02X}  comm={resp['comm_mode']}  mms={mms}  "
        f"faults=[{fault_s}]  T={resp['temperature']:.1f}C  pos={resp['position']:+.4f}  "
        f"data={data_hex}"
    )


def begin_session(
    ser: serial.Serial,
    reader: FrameReader,
    seq: int,
    timeout: float,
    bus: int = 1,
    args: Optional[argparse.Namespace] = None,
    heartbeat: bool = True,
) -> int:
    session_timeout = timeout
    if args is not None and args.session_timeout is not None:
        session_timeout = args.session_timeout
    debug = args is not None and getattr(args, "debug_link", False)
    dump_limit = max(1, int(getattr(args, "dump_frames", 8))) if args is not None else 8
    dump_pdu = bool(getattr(args, "dump_pdu", False)) if args is not None else False

    if debug and args is not None and args.listen_first > 0.0:
        print(f"--- debug-link: listen {args.listen_first:.1f}s before session ---")
        pre = collect_frames(reader, args.listen_first)
        summarize_frame_batch(pre, dump_pdu=dump_pdu, max_lines=dump_limit)
        print()

    if debug:
        reader.drain()
        print(
            f"--- debug-link: session begin  kind={SESSION_BEGIN}  bus={bus}  "
            f"timeout={session_timeout:.2f}s  seq={seq} ---"
        )
        ser.write(build_scan_command(0, SESSION_BEGIN, seq, bus=bus))
        ser.flush()
        resp, frames = wait_probe_response_collect(
            reader, 0, session_timeout, probe_kind=SESSION_BEGIN
        )
        summarize_frame_batch(frames, dump_pdu=dump_pdu, max_lines=dump_limit)
    else:
        resp = send_diag(ser, reader, 0, SESSION_BEGIN, seq, session_timeout, bus=bus)

    if resp is None:
        print("Warning: scan session begin not acknowledged (flash firmware with plant_diag?)")
        if debug:
            print("  Hint: run --link-test on this port, or --list-ports for STM32 USB CDC.")
    else:
        if heartbeat:
            ser.write(build_actuator_command(0.0, 0.0, 0.0, 0.0, 0.0, seq + 1))
            ser.flush()
            seq += 1
        if debug:
            mcp = resp.get("mcp_init_mask", 0)
            print(
                f"  session begin OK  probe_kind={resp['probe_kind']}  found={resp['found']}"
            )
            print(format_mcp_status(resp, bus=bus))
            if (mcp & mcp_rail_bit(bus)) == 0:
                print(
                    "  Next: check MCP2518 power/SPI (PA5-7, PB11 CS) — not a host/USB issue."
                )
    time.sleep(0.05)
    return seq + 1


def end_session(
    ser: serial.Serial,
    reader: FrameReader,
    seq: int,
    timeout: float,
    bus: int = 1,
    args: Optional[argparse.Namespace] = None,
) -> None:
    session_timeout = timeout
    if args is not None and args.session_timeout is not None:
        session_timeout = args.session_timeout
    send_diag(ser, reader, 0, SESSION_END, seq, session_timeout, bus=bus)


def run_exercise(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    bus = normalize_can_bus(args.bus)
    seq = 1
    any_can = False
    any_parsed = False
    session_ended = False

    print(f"Exercise {probe_target_label(target, bus)} on {ser.port}")
    plant_slot = actuator_slot_for_motor(target, bus)
    print_can_bus_note(bus)
    print(
        f"RS2 PDU probes use --target/--bus. "
        f"Actuator-slot path (--actuator-test) uses plant_config slot {plant_slot}."
    )
    if args.wake:
        print("Mode: --wake (reset → enable-only → full → sustained ctrl)")
    print("RS2 bench channel in host image byte 530 (not a power-distribution board).")
    print("CAN activity LEDs: PC7=CH1  PC6=CH2  PB15=CH3  PB14=CH4  PB2=CH5  PC5=CH6")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout, bus=bus, args=args)

        if args.reset_first:
            print("Pre-resetting target motor...")
            resp = send_diag(ser, reader, target, PROBE_RESET, seq, max(args.timeout, 0.45), bus=bus)
            seq += 1
            print(format_probe_line("pre_reset", target, resp))
            time.sleep(0.25)
            print()

        for name, kind, timeout, repeat, pause in (
            WAKE_SEQUENCE if args.wake else EXERCISE_SEQUENCE
        ):
            for n in range(repeat):
                label = name if repeat == 1 else f"{name} [{n + 1}/{repeat}]"
                resp = send_diag(ser, reader, target, kind, seq, timeout, bus=bus)
                seq += 1
                print(format_probe_line(label, target, resp))
                if resp is not None and (resp["found"] or resp.get("raw_frames", 0) > 0):
                    any_can = True
                if resp is not None and resp["found"]:
                    any_parsed = True
                time.sleep(pause)
    finally:
        end_resp = send_diag(ser, reader, 0, SESSION_END, seq, args.timeout, bus=bus)
        session_ended = end_resp is not None
        seq += 1
        print()

    if args.actuator_test:
        print("--- Actuator-slot path (normal --monitor / teleop; CAN LED may stay dark) ---")
        print(
            f"Session ended — MCU 500 Hz loop TX on plant slot {plant_slot} "
            f"(motor 0x{target:02X}), not always slot 0 / 0x70."
        )
        print()
        reader.drain()
        period = 1.0 / max(args.actuator_hz, 1.0)
        for profile_name, pos, vel, kp, kd in ACTUATOR_PROFILES:
            print(f"  profile: {profile_name}  (8 frames @ {args.actuator_hz:.0f} Hz)")
            last_fb: Optional[dict] = None
            for _ in range(8):
                ser.write(build_actuator_command(pos, vel, kp, kd, 0.0, seq, slot=plant_slot))
                ser.flush()
                seq += 1
                time.sleep(period)
                for frame in reader.drain():
                    fb = parse_actuator_feedback(frame, slot=plant_slot)
                    if fb is not None:
                        last_fb = fb
            if last_fb is None:
                print("    no actuator feedback in host image")
            else:
                print(
                    f"    ack={last_fb['ack']:3d}  tick={last_fb['tick']:4d}  "
                    f"pos={last_fb['position']:+.4f}  vel={last_fb['velocity']:+.4f}  "
                    f"T={last_fb['temperature']:.1f}C  fault=0x{last_fb['fault']:08X}"
                )
            time.sleep(0.15)
        print()

    stop.set()

    print("Summary")
    print(f"  MCU saw ext-CAN RX during a probe: {'yes' if any_can else 'NO (TX only — check motor power/fault)'}")
    print(f"  Parsed motor telemetry: {'yes' if any_parsed else 'no'}")
    print(f"  Session end ack: {'yes' if session_ended else 'no (timeout — usually harmless)'}")
    if not any_can:
        print("  Try: power-cycle motor, reflash MCU, then:")
        print(f"    python scripts/rs02_can_scan.py --port {ser.port} --exercise --target 0x{target:02X} --reset-first")
    return 0 if any_can else 1


def format_hit(motor_id: int, resp: dict) -> str:
    return format_probe_line(f"probe 0x{motor_id:02X}", motor_id, resp)


def replied_motor_id(probed_id: int, resp: dict) -> int:
    ext = decode_ext_id(resp["ext_id"])
    disc = resp.get("discovered_id") or ext.motor_id
    if disc:
        return disc & 0xFF
    return probed_id & 0xFF


def discover_has_activity(resp: Optional[dict]) -> bool:
    if resp is None:
        return False
    return bool(resp.get("found") or resp.get("raw_frames", 0) > 0)


def format_discover_line(probed_id: int, probe_label: str, resp: dict) -> str:
    ext = decode_ext_id(resp["ext_id"])
    disc = replied_motor_id(probed_id, resp)
    fault_s = ",".join(ext.faults) if ext.faults else "none"
    mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else f"mode{ext.mode_status}"
    tag = "HIT" if resp.get("found") else "SNIFF"
    id_note = f"probed=0x{probed_id:02X}  replied=0x{disc:02X}"
    if disc != (probed_id & 0xFF):
        id_note += "  (ID mismatch — use replied=)"
    return (
        f"  {tag}  0x{probed_id:02X} via {probe_label:<7s}  {id_note}  "
        f"{rs2_comm_label(resp['comm_mode'])}  mms={mms}  faults=[{fault_s}]  "
        f"ext=0x{resp['ext_id']:08X}  rx={resp.get('raw_frames', 0)}  "
        f"data={resp['can_data'].hex()}"
    )


def dedupe_id_list(ids: List[int]) -> List[int]:
    seen: Set[int] = set()
    out: List[int] = []
    for i in ids:
        if 1 <= i <= 127 and i not in seen:
            seen.add(i)
            out.append(i)
    return out


def discover_range_bounds(args: argparse.Namespace) -> Tuple[int, int]:
    start = max(1, int(args.start) & 0xFF)
    end = min(127, int(args.end) & 0xFF)
    if end < start:
        start, end = end, start
    return start, end


def build_discover_id_list(args: argparse.Namespace) -> List[int]:
    if args.ids:
        return dedupe_id_list(parse_id_list(args.ids))

    start, end = discover_range_bounds(args)
    full_range = list(range(start, end + 1))
    custom_range = (start, end) != (1, 127)

    if custom_range:
        # Honor --start/--end only (priority order within that window).
        priority_in_range = [i for i in DISCOVER_PRIORITY_IDS if start <= i <= end]
        seen = set(priority_in_range)
        tail = [i for i in full_range if i not in seen]
        return priority_in_range + tail

    # Full 1–127: priority IDs ordered first, then the rest.
    if args.discover_all or not args.discover_quick:
        return dedupe_id_list(list(DISCOVER_PRIORITY_IDS) + full_range)

    return dedupe_id_list(list(DISCOVER_PRIORITY_IDS) + list(range(1, 128)))


def describe_discover_scan(args: argparse.Namespace, id_list: List[int]) -> str:
    start, end = discover_range_bounds(args)
    if args.ids:
        return f"custom list ({len(id_list)} IDs from --ids)"
    if start == 1 and end == 127:
        if args.discover_quick:
            return f"full 1–127 / 0x01–0x7F ({len(id_list)} addresses, light probes)"
        return f"full 1–127 / 0x01–0x7F ({len(id_list)} addresses, deep wake on priority IDs first)"
    return f"0x{start:02X}–0x{end:02X} ({len(id_list)} addresses)"


def discover_probe_timeout(base: float, bus: int, probe_kind: int = -1) -> float:
    """MCP2518 SPI-CAN needs longer per-probe listen than on-chip FDCAN."""
    if normalize_can_bus(bus) not in MCP_CAN_BUSES:
        return base
    t = max(base * 3.0, base + 0.6, 2.0)
    if probe_kind in (PROBE_FULL, PROBE_ENABLE_CTRL, PROBE_CTRL_ONLY, PROBE_CTRL_FAST):
        t = max(t, 3.5)
    elif probe_kind == PROBE_RESET:
        t = max(t, 2.5)
    return t


def discover_reset_ids(bus: int) -> Tuple[int, ...]:
    if normalize_can_bus(bus) in MCP_CAN_BUSES:
        return MCP_RESET_IDS
    return (0x7F, 0x70, 0x7E)


def discover_probe_steps(probed_id: int, args: argparse.Namespace) -> Tuple[Tuple[str, int, float, int], ...]:
    if args.discover_light or args.discover_quick:
        return DISCOVER_PROBES_LIGHT
    if args.discover_deep or probed_id in DISCOVER_PRIORITY_IDS:
        return DISCOVER_PROBES_DEEP
    return DISCOVER_PROBES_LIGHT


def probe_one_id(
    ser: serial.Serial,
    reader: FrameReader,
    probed_id: int,
    seq: int,
    args: argparse.Namespace,
    verbose_steps: bool = False,
) -> Tuple[Optional[Tuple[str, dict]], int]:
    """Full deep wake on one ID; return ((best_label, resp)|None, next_seq)."""
    best: Optional[Tuple[str, dict]] = None
    for probe_label, probe_kind, probe_timeout, repeat in discover_probe_steps(probed_id, args):
        for n in range(repeat):
            step_label = probe_label if repeat == 1 else f"{probe_label}[{n + 1}/{repeat}]"
            resp = send_diag(
                ser, reader, probed_id, probe_kind, seq,
                discover_probe_timeout(
                    max(args.timeout, probe_timeout), args.bus, probe_kind=probe_kind
                ),
                bus=normalize_can_bus(args.bus),
            )
            seq += 1
            if verbose_steps:
                if resp is not None and discover_has_activity(resp):
                    print(format_discover_line(probed_id, step_label, resp))
                elif resp is None:
                    print(f"  ----  0x{probed_id:02X} {step_label:<12s}  (no USB feedback)")
                else:
                    print(f"  ....  0x{probed_id:02X} {step_label:<12s}  (no ext-CAN RX)")
            if discover_has_activity(resp):
                if best is None or resp.get("found") or resp.get("raw_frames", 0) > best[1].get("raw_frames", 0):
                    best = (step_label, resp)
                if resp.get("found") and resp["comm_mode"] in (0x02, 0x03):
                    return best, seq
            time.sleep(0.04)
    return best, seq


def mcp_smoke_init_ok(resp: dict, bus: int) -> bool:
    bus = normalize_can_bus(bus)
    if bus not in MCP_CAN_BUSES:
        return False
    mask = int(resp.get("mcp_init_mask", 0)) & 0xFF
    if (mask & mcp_rail_bit(bus)) == 0:
        return False
    if mcp_rail_opmod(resp) != 6:
        return False
    if int(resp.get("mcp_tx_ok", 0)) == 0:
        return False
    nbt_brp = int(resp.get("mcp_nbt_brp", 0xFF)) & 0xFF
    nbt_tseg1 = int(resp.get("mcp_nbt_tseg1", 0)) & 0xFF
    return nbt_brp == 0 and nbt_tseg1 == 17


def mcp_smoke_bus_proven(resp: dict) -> bool:
    rx = int(resp.get("raw_frames", 0))
    tec = int(resp.get("mcp_tec", 0))
    tec_before = int(resp.get("mcp_tec_before", 0))
    tx_nack = int(resp.get("mcp_tx_nack", 0))
    tec_delta = tec - tec_before if tec >= tec_before else 0
    return rx > 0 or tx_nack > 0 or tec_delta > 0


def run_mcp_smoke_one(ser: serial.Serial, args: argparse.Namespace, bus: int) -> int:
    """Standalone MCP2518 smoke on one CH4/5/6 rail."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    bus = normalize_can_bus(bus)
    motor_id = int(args.target) & 0xFF
    seq = 1
    led = can_activity_led(bus)

    print(f"MCP smoke test on {can_bus_label(bus)}  motor_id=0x{motor_id:02X}  probe_kind={PROBE_MCP_SMOKE}")
    print(f"Watch {led} during the test (blinks on successful MCP TX).")
    print("Interpretation (CAN 2.0 error counters):")
    print("  TEC +8 per frame  → transmitted, NO ACK (nobody acknowledged)")
    print("  TEC unchanged     → transmitted, ACK received (at least one node)")
    print("  tx_ok=0           → MCP never saw TXATIF — driver/SPI/TXQ problem")
    print("  rx>0              → motor or bus traffic replied")
    print()

    if bus not in MCP_CAN_BUSES:
        print(f"FAIL: MCP smoke requires --bus 4, 5, or 6 (got CH{bus}).")
        return 1

    try:
        reader.drain()
        resp = send_diag(
            ser, reader, motor_id, PROBE_MCP_SMOKE, seq,
            max(args.timeout, 3.0), bus=bus,
        )
        seq += 1
        if resp is None:
            print("FAIL: no USB feedback from MCP smoke probe.")
            return 1
        print(format_mcp_status(resp, bus=bus))
        if resp.get("probe_kind") != PROBE_MCP_SMOKE:
            print(f"WARN: expected probe_kind {PROBE_MCP_SMOKE}, got {resp.get('probe_kind')}")
        if not mcp_smoke_init_ok(resp, bus):
            if int(resp.get("mcp_tx_ok", 0)) == 0:
                print("\nFAIL: MCP TX did not succeed — fix driver/hardware before motor tests.")
            elif int(resp.get("mcp_nbt_brp", 0xFF)) != 0:
                print("\nFAIL: CiNBTCFG BRP wrong after flash — expected 0.")
            else:
                print("\nFAIL: MCP init/opmode/bitrate check failed.")
            return 1
        rx = int(resp.get("raw_frames", 0))
        tec = int(resp.get("mcp_tec", 0))
        tec_before = int(resp.get("mcp_tec_before", 0))
        tec_delta = tec - tec_before if tec >= tec_before else 0
        if rx > 0:
            print(f"\nPASS: MCP TX+RX on CH{bus} ({rx} frame(s)). Motor or bus traffic replied.")
            return 0
        if int(resp.get("mcp_tx_ok", 0)) > 0 and tec_delta == 0:
            print(f"\nPASS: CH{bus} MCP TX completed, TEC unchanged — frame was ACK'd on the bus.")
            return 0
        if mcp_smoke_bus_proven(resp):
            if tec >= 128:
                print(f"\nFAIL: CH{bus} MCP TX but bus-off (TEC={tec}).")
                return 1
            print(f"\nPASS: CH{bus} MCP TX reached the bus (TEC +{tec_delta}, no ACK).")
            print(f"  Watch {led} — blinking confirms activity on this rail.")
            return 0
        print(f"\nFAIL: CH{bus} did not complete a verified transmission (tx_ok=0 or no TXATIF).")
        return 1
    finally:
        stop.set()


def run_mcp_smoke_all(ser: serial.Serial, args: argparse.Namespace) -> int:
    """Smoke CH4, CH5, and CH6 in one run — find which MCP rail has bus activity."""
    print("MCP smoke all rails (CH4, CH5, CH6) — use ACT LEDs PB14 / PB2 / PC5")
    print()
    rc = 0
    for bus in sorted(MCP_CAN_BUSES):
        if bus != 4:
            print()
            time.sleep(0.35)
        one = run_mcp_smoke_one(ser, args, bus)
        if one != 0:
            rc = 1
    print()
    if rc == 0:
        print("All three MCP rails passed smoke (TX verified on each).")
    else:
        print("One or more MCP rails failed — compare mask bits and ACT LED blinks per CH.")
    return rc


def run_mcp_smoke(ser: serial.Serial, args: argparse.Namespace) -> int:
    if getattr(args, "mcp_smoke_all", False):
        return run_mcp_smoke_all(ser, args)
    return run_mcp_smoke_one(ser, args, normalize_can_bus(args.bus))


def run_mcp_wake(ser: serial.Serial, args: argparse.Namespace) -> int:
    """Step up from smoke: reset → run_mode → enable (3 TX) + 450 ms listen. No session."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    bus = normalize_can_bus(args.bus)
    motor_id = int(args.target) & 0xFF
    seq = 1
    led = can_activity_led(bus)

    print(f"MCP wake test on {can_bus_label(bus)}  motor_id=0x{motor_id:02X}  probe_kind={PROBE_MCP_WAKE}")
    print("Sequence: comm 0x04 reset → parawrite run_mode → comm 0x03 enable → listen 450 ms")
    print(f"Watch {led} (blinks on each successful MCP TX; expect up to 3 blinks).")
    print("No RS2 session — synchronous 3 TX + 450 ms listen (same driver path as smoke).")
    print()

    if bus not in MCP_CAN_BUSES:
        print(f"FAIL: MCP wake requires --bus 4, 5, or 6 (got CH{bus}).")
        return 1

    try:
        reader.drain()
        resp = send_diag(
            ser, reader, motor_id, PROBE_MCP_WAKE, seq,
            max(args.timeout, 4.0), bus=bus,
        )
        if resp is None:
            print("FAIL: no USB feedback from MCP wake probe.")
            return 1
        print(format_mcp_status(resp, bus=bus))
        if resp.get("probe_kind") != PROBE_MCP_WAKE:
            print(f"WARN: expected probe_kind {PROBE_MCP_WAKE}, got {resp.get('probe_kind')}")
        tx_ok = int(resp.get("mcp_tx_ok", 0))
        tx_fail = int(resp.get("mcp_tx_fail", 0))
        if tx_fail > 0:
            print(f"\nFAIL: MCP wake TX queue broke mid-sequence ({tx_ok}/3 ok, {tx_fail} fail).")
            print("  Reflash with latest firmware — this is a driver TXQ recovery bug.")
            return 1
        if tx_ok < 3:
            print(f"\nFAIL: only {tx_ok}/3 wake frames transmitted.")
            return 1
        if resp.get("found"):
            print(
                f"\nPASS: motor replied (comm={rs2_comm_label(resp['comm_mode'])}) "
                f"raw_frames={resp.get('raw_frames', 0)}"
            )
            return 0
        raw = int(resp.get("raw_frames", 0))
        if raw > 0:
            print(f"\nPASS: {raw} ext-CAN frame(s) (comm={rs2_comm_label(resp.get('comm_mode', 0))}).")
            return 0
        tec = int(resp.get("mcp_tec", 0))
        tec_before = int(resp.get("mcp_tec_before", 0))
        tec_delta = tec - tec_before if tec >= tec_before else 0
        print(f"\nPASS: 3/3 MCP wake TX completed (TEC +{tec_delta} = no ACK on those frames).")
        print("  Driver path OK. Motor did not reply — check ID/bus wiring if unexpected.")
        return 0
    finally:
        stop.set()


def run_probe_id(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    motor_id = int(args.probe_id) & 0xFF
    bus = normalize_can_bus(args.bus)
    seq = 1
    print(f"Deep probe {probe_target_label(motor_id, bus)} on {ser.port}  (same wake as --bench-cmds)")
    print(f"Activity LED for this bus: {can_activity_led(bus)}")
    print("Only ONE motor should be on CAN. Power on, shaft free.")
    print()

    try:
        seq = begin_session(ser, reader, seq, max(args.timeout, 0.5), bus=bus, args=args,
                            heartbeat=False)
        best, seq = probe_one_id(ser, reader, motor_id, seq, args, verbose_steps=True)
    finally:
        end_session(ser, reader, seq, max(args.timeout, 0.5), bus=bus, args=args)
        stop.set()

    print()
    if best is None:
        print(f"Result: no CAN reply at probed ID 0x{motor_id:02X}.")
        print("  - Confirm THIS motor is wired (not the other RS02)")
        print("  - Try factory default: --probe-id 0x7F")
        if bus in MCP_CAN_BUSES:
            print(f"  - Run --mcp-smoke --bus {bus} first (verifies MCP TX without needing a reply)")
        print(f"  - {can_activity_led(bus)} should blink on TX even with no reply")
        return 1

    _, resp = best
    disc = replied_motor_id(motor_id, resp)
    print(f"Result: motor replied as 0x{disc:02X}  comm={rs2_comm_label(resp['comm_mode'])}")
    print(f"  python scripts/host_teleop_laptop_usb.py --port {ser.port} --motor-id 0x{disc:02X}")
    return 0


def run_discover(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    id_list = build_discover_id_list(args)
    bus = normalize_can_bus(args.bus)
    seq = 1
    # replied_id -> list of (probed_id, probe_label, resp)
    findings: Dict[int, List[Tuple[int, str, dict]]] = {}

    scan_desc = describe_discover_scan(args, id_list)
    print(f"Discover RobStride CAN ID on {ser.port}  ({scan_desc})  bus={can_bus_label(bus)}")
    print_can_bus_note(bus)
    if args.discover_light or args.discover_deep:
        probe_mode = "light (enable+promisc)" if args.discover_light else "deep wake on every ID"
    else:
        probe_mode = "deep on priority IDs, light (enable+promisc) on the rest"
    print(f"Probe mode: {probe_mode}")
    print("Only ONE unknown motor on the bus (daisy chain OK if IDs differ).")
    print("Factory default is often 0x7F; RS01 may use any ID in 0x01–0x7F.")
    if args.discover_quick:
        print("Quick mode: enable+promisc on every ID (fast). Omit --discover-quick for deep wake on 0x7F/0x70/… first.")
    print()

    try:
        seq = begin_session(ser, reader, seq, max(args.timeout, 0.5), bus=bus, args=args,
                            heartbeat=False)

        if args.reset_first:
            for rid in discover_reset_ids(bus):
                send_diag(ser, reader, rid, PROBE_RESET, seq,
                          discover_probe_timeout(0.45, bus, probe_kind=PROBE_RESET), bus=bus)
                seq += 1
                time.sleep(0.08)
            ids_s = ", ".join(f"0x{r:02X}" for r in discover_reset_ids(bus))
            print(f"Pre-reset sent to {ids_s}")
            print()

        total = len(id_list)
        for idx, probed_id in enumerate(id_list):
            if total > 40 and idx > 0 and idx % 20 == 0:
                print(f"  ... progress {idx}/{total} (probing 0x{probed_id:02X})")
            best, seq = probe_one_id(
                ser, reader, probed_id, seq, args,
                verbose_steps=args.verbose or args.debug_link or args.debug_probes,
            )

            if best is not None:
                probe_label, resp = best
                print(format_discover_line(probed_id, probe_label, resp))
                disc = replied_motor_id(probed_id, resp)
                findings.setdefault(disc, []).append((probed_id, probe_label, resp))
            elif args.verbose:
                print(f"  ....  0x{probed_id:02X}  no CAN reply")

            time.sleep(max(args.gap, 0.05))
    finally:
        end_session(ser, reader, seq, max(args.timeout, 0.5), bus=bus, args=args)
        stop.set()

    print()
    print("Summary")
    if not findings:
        print("  No motor replies on any probed ID.")
        print("  If another motor works but this RS01 shows nothing:")
        print("    → CAN wiring, JST orientation, supply, or only one motor should be unconfigured.")
        print("  Try full ID sweep or a single deep probe:")
        print(f"    python scripts/rs02_can_scan.py --port {ser.port} --discover --discover-all")
        print(f"    python scripts/rs02_can_scan.py --port {ser.port} --discover --start 0x50 --end 0x7F")
        print(f"    python scripts/rs02_can_scan.py --port {ser.port} --probe-id 0x7F")
        return 1

    print(f"  Unique replied motor ID(s): {len(findings)}")
    for disc_id in sorted(findings):
        entries = findings[disc_id]
        best_resp = entries[0][2]
        ext = decode_ext_id(best_resp["ext_id"])
        comms = sorted({rs2_comm_label(e[2]["comm_mode"]) for e in entries})
        print(
            f"    0x{disc_id:02X}  seen from probe(s) "
            f"{', '.join(f'0x{p:02X}' for p, _, _ in entries)}  "
            f"comms=[{'; '.join(comms)}]  faults=[{','.join(ext.faults) or 'none'}]"
        )

    best_id = sorted(findings)[0]
    print()
    print(f"Use this ID for teleop / param scan:")
    print(f"  python scripts/host_teleop_laptop_usb.py --port {ser.port} --motor-id 0x{best_id:02X}")
    print(f"  python scripts/rs02_can_scan.py --port {ser.port} --exercise --target 0x{best_id:02X}")
    if len(findings) == 1:
        print(f"  plant_config: actuator_table[0].motor_id = 0x{best_id:02X};")
    else:
        print("  Multiple IDs replied — confirm only one motor on the bus.")
    return 0


def run_scan(ser: serial.Serial, id_list: List[int], args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    hits: List[tuple[int, dict]] = []
    bus = normalize_can_bus(args.bus)
    seq = 1
    use_fast = False

    print(f"Scanning {len(id_list)} IDs on {ser.port}  ({can_bus_label(bus)})")
    print_can_bus_note(bus)
    print("Probe: RobStride private ext-CAN. Background actuator TX paused during session.")
    print()

    seq = begin_session(ser, reader, seq, args.timeout, bus=bus, args=args)

    if args.reset_first:
        reset_ids = [args.target & 0xFF, 0x7F, 0x6F]
        if args.ids:
            reset_ids = parse_id_list(args.ids)[:4]
        seen: Set[int] = set()
        reset_ids = [i for i in reset_ids if not (i in seen or seen.add(i))]
        print(f"Sending motor reset to: {', '.join(f'0x{i:02X}' for i in reset_ids)}")
        for rid in reset_ids:
            send_diag(ser, reader, rid, PROBE_RESET, seq, max(args.timeout, 0.35), bus=bus)
            seq += 1
            time.sleep(0.1)
        print()

    for motor_id in id_list:
        if use_fast and args.fast:
            kind = PROBE_CTRL_ONLY
        elif args.promisc:
            kind = PROBE_PROMISC
        else:
            kind = PROBE_FULL

        resp = send_diag(ser, reader, motor_id, kind, seq, args.timeout, bus=bus)
        seq += 1

        if resp is None:
            print(f"  ----  0x{motor_id:02X}  (no PDU response)")
            continue

        interesting = resp["found"] or resp.get("raw_frames", 0) > 0
        if interesting:
            print(format_hit(motor_id, resp))
            hits.append((motor_id, resp))
            use_fast = args.fast and resp["found"]
        elif args.verbose:
            print(f"  ....  0x{motor_id:02X}  no CAN activity")

        time.sleep(args.gap)

    end_session(ser, reader, seq, args.timeout, bus=bus, args=args)
    stop.set()
    print()
    if not hits:
        print("No RobStride CAN activity seen.")
        return 1

    parsed_hits = [h for h in hits if h[1]["found"]]
    if parsed_hits:
        print(f"Parsed feedback on {len(parsed_hits)} ID(s):")
        for motor_id, resp in parsed_hits:
            disc = resp["discovered_id"] or (motor_id & 0xFF)
            print(f"  motor_id 0x{disc:02X}  T={resp['temperature']:.1f}C  pos={resp['position']:+.4f}")
        best = parsed_hits[0][1]["discovered_id"] or (parsed_hits[0][0] & 0xFF)
        print()
        print(f"Set App/Src/plant/plant_config.c: actuator_table[0].motor_id = 0x{best:02X};")
    return 0 if parsed_hits else 2


def score_port(device: str, description: str, vid: Optional[int]) -> int:
    score = 0
    text = f"{device} {description}".upper()
    if vid == STM32_VID:
        score += 100
    for token in ("STM", "CDC", "VIRTUAL COM", "G4"):
        if token in text:
            score += 10
    return score


def auto_pick_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        return "COM3" if sys.platform == "win32" else "/dev/ttyACM0"
    best = max(ports, key=lambda p: score_port(p.device, p.description or "", p.vid))
    return best.device


def parse_id_list(text: str) -> List[int]:
    out: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part, 0) & 0xFF)
    return out


def build_id_sequence(args: argparse.Namespace) -> List[int]:
    if args.ids:
        ids = parse_id_list(args.ids)
    else:
        ids = list(range(args.start & 0xFF, (args.end & 0xFF) + 1))
    if args.include_common:
        seen: Set[int] = set()
        merged: List[int] = []
        for i in list(ids) + list(COMMON_IDS):
            if i not in seen:
                seen.add(i)
                merged.append(i)
        ids = merged
    return ids


def main() -> None:
    ap = argparse.ArgumentParser(description="RS-02 CAN scan / exercise via USB")
    ap.add_argument("--port", default=None, help="Serial port (COM5, /dev/ttyACM0)")
    ap.add_argument("--list-ports", action="store_true",
                    help="List COM/tty ports (STM32 USB CDC hint) and exit")
    ap.add_argument("--link-test", action="store_true",
                    help="USB-only diagnostics: listen, seq ack, session begin/end (no CAN scan)")
    ap.add_argument("--debug-link", action="store_true",
                    help="Verbose USB/session diagnostics (use with --discover or --exercise)")
    ap.add_argument("--debug-probes", action="store_true",
                    help="With --discover: print every probe step (slow)")
    ap.add_argument("--listen-first", type=float, default=0.0,
                    help="Seconds to listen for MCU feedback before session begin")
    ap.add_argument("--session-timeout", type=float, default=None,
                    help="Override timeout for session begin/end (default: --timeout or 0.5)")
    ap.add_argument("--dump-frames", type=int, default=8,
                    help="Max feedback frames to print per batch in --link-test / --debug-link")
    ap.add_argument("--dump-pdu", action="store_true",
                    help="Hex-dump pdu[0:32] in link diagnostics")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--mcp-smoke", action="store_true",
                    help="MCP2518 smoke on --bus 4|5|6 (1 enable TX + listen)")
    ap.add_argument("--mcp-smoke-all", action="store_true",
                    help="MCP smoke on CH4, CH5, and CH6 sequentially (find active rail)")
    ap.add_argument("--mcp-wake", action="store_true",
                    help="MCP step-2: reset+run_mode+enable (3 TX) + listen; no session")
    ap.add_argument("--ch4-discover", action="store_true",
                    help="CH4 wide discover: bus 4, full 1-127, deep wake every ID, MCP pre-reset")
    ap.add_argument("--discover", action="store_true",
                    help="Find motor CAN ID (default: full 1–127; priority IDs deep-wake first)")
    ap.add_argument("--discover-all", action="store_true",
                    help="With --discover: sweep --start..--end (default 1–127); overrides --discover-quick")
    ap.add_argument("--discover-quick", action="store_true",
                    help="With --discover: full 1–127 with fast light probes (~3 min; was ~20 IDs only)")
    ap.add_argument("--discover-light", action="store_true",
                    help="Fast discover: enable+promisc only (weaker than --bench-cmds wake)")
    ap.add_argument("--discover-deep", action="store_true",
                    help="Deep wake on every scanned ID (slow; default deep on priority IDs only)")
    ap.add_argument("--probe-id", type=lambda x: int(x, 0), default=None,
                    help="Deep wake one ID with step-by-step log (e.g. 0x7F for new motor)")
    ap.add_argument("--exercise", action="store_true",
                    help="Run probe matrix on --target (default 0x70)")
    ap.add_argument("--param-scan", action="store_true",
                    help="Sweep param indices via comm 0x11 pararead (default 0x7005..0x702E)")
    ap.add_argument(
        "--bus",
        type=int,
        default=1,
        choices=[1, 2, 3, 4, 5, 6],
        help="Schematic CAN bus 1..6 (CH1-3 FDCAN, CH4-6 MCP2518). Sets pdu.data[11].",
    )
    ap.add_argument("--bench-cmds", action="store_true",
                    help="Sequential iq_test/cal/zero/save probes (one MCU block at a time)")
    ap.add_argument("--recovery", action="store_true",
                    help="Reset+enable only (no cal). Use after power-cycle if stuck mms=cali")
    ap.add_argument("--skip-iq-test", action="store_true",
                    help="Skip iq_test=1 parawrite in --bench-cmds (use if motor already initialized)")
    ap.add_argument("--params", default=None,
                    help="Param scan: comma list e.g. 0x7019,0x701C (overrides --param-start/end)")
    ap.add_argument("--param-start", type=lambda x: int(x, 0), default=DEFAULT_PARAM_START,
                    help="Param scan: first index (default 0x7005)")
    ap.add_argument("--param-end", type=lambda x: int(x, 0), default=DEFAULT_PARAM_END,
                    help="Param scan: last index (default 0x702E)")
    ap.add_argument("--known-only", action="store_true",
                    help="Param scan: only documented 0x7005..0x702E names")
    ap.add_argument("--param-timeout", type=float, default=0.75,
                    help="Seconds to wait per pararead (default 0.75; MCU listen ~220ms)")
    ap.add_argument("--param-running", action="store_true",
                    help="Param scan: ctrl burst to mms=running before reads (default: reset+enable only)")
    ap.add_argument("--ctrl-pos", type=float, default=0.0,
                    help="Position setpoint (rad) sent in ctrl-burst probes (default 0.0; use with --param-running)")
    ap.add_argument("--cal-timeout", type=float, default=90.0,
                    help="Seconds for --bench-cmds cal probe on MCU (default 90)")
    ap.add_argument("--proactive-off", action="store_true",
                    help="Disable proactive mode and data_save — fixes 'comm=0x18 on every probe'")
    ap.add_argument("--wake", action="store_true",
                    help="With --exercise: reset, enable-only, then ctrl burst (needs fw probe kind 12)")
    ap.add_argument(
        "--target", "--motor-id",
        dest="target",
        type=lambda x: int(x, 0),
        default=DEFAULT_TARGET,
        help="Motor CAN ID for --exercise/--param-scan/--bench-cmds (default 0x70; use 0x71 after --probe-id)",
    )
    ap.add_argument("--actuator-test", action="store_true",
                    help="After exercise, test actuator_commands path on plant slot for --target (0x74→slot 1)")
    ap.add_argument("--actuator-hz", type=float, default=30.0,
                    help="Rate for --actuator-test (default 30)")
    ap.add_argument("--start", type=lambda x: int(x, 0), default=1, help="Scan: first ID")
    ap.add_argument("--end", type=lambda x: int(x, 0), default=127, help="Scan: last ID")
    ap.add_argument("--ids", default=None, help="Scan: comma list e.g. 0x70,0x7F")
    ap.add_argument("--include-common", action="store_true")
    ap.add_argument("--timeout", type=float, default=0.25)
    ap.add_argument("--gap", type=float, default=0.02)
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--promisc", action="store_true")
    ap.add_argument("--reset-first", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.ch4_discover:
        args.bus = 4
        args.discover = True
        args.discover_all = True
        args.discover_deep = True
        args.reset_first = True
        args.start = 1
        args.end = 127
        args.timeout = max(float(args.timeout), 0.55)
        if not args.debug_link and not args.debug_probes:
            args.debug_link = True

    if args.list_ports:
        list_serial_ports()
        return

    port = args.port or auto_pick_port()
    print(f"Opening {port} @ {args.baud}")
    with serial.Serial(port=port, baudrate=args.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        while ser.in_waiting:
            ser.read(ser.in_waiting)
        if args.link_test:
            rc = run_link_test(ser, args)
        elif args.mcp_smoke or args.mcp_smoke_all:
            rc = run_mcp_smoke(ser, args)
        elif args.mcp_wake:
            rc = run_mcp_wake(ser, args)
        elif args.probe_id is not None:
            rc = run_probe_id(ser, args)
        elif args.discover:
            rc = run_discover(ser, args)
        elif args.param_scan:
            rc = run_param_scan(ser, args.target, args)
        elif args.recovery:
            rc = run_recovery(ser, args.target, args)
        elif args.proactive_off:
            rc = run_proactive_off(ser, args.target, args)
        elif args.bench_cmds:
            rc = run_bench_cmds(ser, args.target, args)
        elif args.exercise or args.wake:
            rc = run_exercise(ser, args.target, args)
        else:
            rc = run_scan(ser, build_id_sequence(args), args)
    sys.exit(rc)


if __name__ == "__main__":
    main()
