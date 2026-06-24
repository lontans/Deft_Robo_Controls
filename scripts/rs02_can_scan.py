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
  python scripts/rs02_can_scan.py --port COM9 --discover
  python scripts/rs02_can_scan.py --port COM9 --discover --discover-quick
  python scripts/rs02_can_scan.py --port COM9 --probe-id 0x7F
  python scripts/rs02_can_scan.py --port COM5 --exercise --target 0x70
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
PDU_OFF = 530

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


DEFAULT_BAUD = 115200
DEFAULT_TARGET = 0x70
STM32_VID = 0x0483

COMMON_IDS = (
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    0x10, 0x11, 0x20, 0x7F, 0xFD,
)

# Factory default 0x7F first, then common bench IDs (new RS02 discovery).
DISCOVER_PRIORITY_IDS = (
    0x7F, 0x70, 0x7E, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x10, 0x11, 0x20, 0x6F,
)

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


def build_scan_command(motor_id: int, probe_kind: int, seq: int) -> bytes:
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
    return bytes(buf)


def build_rs2_probe_command(
    motor_id: int,
    probe_kind: int,
    seq: int,
    param_index: int = 0,
) -> bytes:
    buf = bytearray(build_scan_command(motor_id, probe_kind, seq))
    buf[PDU_OFF + 5] = param_index & 0xFF
    buf[PDU_OFF + 6] = (param_index >> 8) & 0xFF
    return bytes(buf)


def build_actuator_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    seq: int,
) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    struct.pack_into("<fffff", buf, ACTUATOR0_OFF, position, velocity, kp, kd, torque)
    return bytes(buf)


def build_rs2_teleop_command(
    motor_id: int,
    probe_kind: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    seq: int,
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
    return bytes(buf)


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
    }


def parse_actuator_feedback(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", frame, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    sys_word, = struct.unpack_from("<I", frame, 12)
    pos, vel, torque, temp, fault = struct.unpack_from("<ffffI", frame, ACTUATOR0_OFF)
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


def send_diag(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    probe_kind: int,
    seq: int,
    timeout_s: float,
) -> Optional[dict]:
    reader.drain()
    ser.write(build_scan_command(motor_id, probe_kind, seq))
    ser.flush()
    return wait_probe_response(reader, motor_id, timeout_s, probe_kind=probe_kind)


def pararead_index_echo(resp: dict) -> int:
    return resp["can_data"][0] | (resp["can_data"][1] << 8)


def pararead_is_hit(resp: dict, param_index: int) -> bool:
    if not resp.get("found") or resp["comm_mode"] not in (0x11, 0x12):
        return False
    return pararead_index_echo(resp) == (param_index & 0xFFFF)


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
        return (
            f"  HIT   {tag}  {rs2_comm_label(resp['comm_mode'])}  "
            f"echo=0x{idx_echo:04X}  float={resp['position']:+.4f}  "
            f"raw={resp['can_data'].hex()}"
        )
    if sniff is not None:
        raw_n = sniff.get("raw_frames", 0)
        if sniff["comm_mode"] in (0x11, 0x12):
            idx_echo = pararead_index_echo(sniff)
            if idx_echo != (index & 0xFFFF):
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
) -> int:
    short_wake: Tuple[Tuple[str, int, float, int, float], ...] = (
        ("reset", PROBE_RESET, 0.45, 1, 0.15),
        ("enable_only", PROBE_ENABLE_ONLY, 0.55, 1, 0.12),
        ("full_init", PROBE_FULL, 0.35, 1, 0.10),
    )
    if running:
        short_wake = short_wake + (("ctrl x12", PROBE_CTRL_ONLY, 0.25, 12, 0.04),)
    label_mode = "running ctrl" if running else "rest/enable (vendor pararead style)"
    print(f"Wake motor 0x{target:02X} before scan ({label_mode})...")
    for name, kind, step_timeout, repeat, pause in short_wake:
        for n in range(repeat):
            label = name if repeat == 1 else f"{name} [{n + 1}/{repeat}]"
            resp = send_diag(ser, reader, target, kind, seq, max(timeout, step_timeout))
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
    print()
    return seq


def run_param_scan(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    seq = 1
    indices = build_param_index_list(args)
    hits: List[Tuple[int, dict]] = []
    sniffs: List[Tuple[int, dict]] = []
    usb_miss = 0

    print(f"Param scan motor 0x{target:02X} on {ser.port}  ({len(indices)} indices)")
    print("Success = ext-CAN comm 0x11 pararead with float in data[4:8].")
    print("Sweep like ID scan — HIT / SNIFF / .... per index.")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout)
        if args.reset_first:
            send_diag(ser, reader, target, PROBE_RESET, seq, max(args.timeout, 0.45))
            seq += 1
            time.sleep(0.2)
        seq = run_wake_short(ser, reader, target, seq, args.timeout, running=args.param_running)

        for index in indices:
            resp, sniff, usb_seen, seq = send_pararead(
                ser, reader, target, index, seq, args.param_timeout,
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
        end_session(ser, reader, seq, args.timeout)
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
            print("  Motor sends comm 0x11 with empty/wrong echo — likely faulted or uncalibrated.")
        print("  Wake showed comm 0x15 error_feedback? Check 48 V, then encoder cal (0x05) or Motor Studio.")
        print("  Try: --param-running   or   --bench-cmds --cal-timeout 28")
    if usb_miss > len(indices) // 2:
        print(f"  USB probe acks missing on {usb_miss}/{len(indices)} reads — MCU busy; replug / wait 30s")
    return 0 if hits else 1


def run_bench_cmds(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    """One-shot cal / zero / save probes (no queuing — MCU blocks per command)."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    seq = 1
    bench: Tuple[Tuple[str, int, float], ...] = (
        ("motor_cali comm 0x05", PROBE_CALI, args.cal_timeout + 10.0),
        ("motor_zero comm 0x06", PROBE_ZERO, 3.0),
        ("data_save comm 0x16", PROBE_DATA_SAVE, 4.0),
    )

    print(f"Bench comm probes on 0x{target:02X}  (sequential — one MCU block at a time)")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout)
        seq = run_wake_short(ser, reader, target, seq, args.timeout, running=True)
        for label, kind, probe_timeout in bench:
            print(f"--- {label} (wait up to {probe_timeout:.0f}s) ---")
            resp = send_diag(ser, reader, target, kind, seq, probe_timeout)
            seq += 1
            print(format_probe_line(label, target, resp))
            if resp is None:
                print("  stopped — MCU did not ack (prior probe still running? wait / replug USB)")
                break
            if resp.get("found"):
                ext = decode_ext_id(resp["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                print(f"         mms={mms}  data={resp['can_data'].hex()}")
            print()
            time.sleep(0.2)
    finally:
        end_session(ser, reader, seq, args.timeout)
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
    fault_s = ",".join(ext.faults) if ext.faults else "none"
    mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else f"mode{ext.mode_status}"
    tag = "HIT" if resp["found"] else "SNIFF"
    data_hex = resp["can_data"].hex()

    return (
        f"  {tag}  {label:<32s}  ext=0x{resp['ext_id']:08X}  "
        f"motor=0x{disc:02X}  comm={resp['comm_mode']}  mms={mms}  "
        f"faults=[{fault_s}]  T={resp['temperature']:.1f}C  pos={resp['position']:+.4f}  "
        f"data={data_hex}"
    )


def begin_session(ser: serial.Serial, reader: FrameReader, seq: int, timeout: float) -> int:
    if send_diag(ser, reader, 0, SESSION_BEGIN, seq, timeout) is None:
        print("Warning: scan session begin not acknowledged (flash firmware with plant_diag?)")
    time.sleep(0.05)
    return seq + 1


def end_session(ser: serial.Serial, reader: FrameReader, seq: int, timeout: float) -> None:
    send_diag(ser, reader, 0, SESSION_END, seq, timeout)


def run_exercise(ser: serial.Serial, target: int, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    target &= 0xFF
    seq = 1
    any_can = False
    any_parsed = False
    session_ended = False

    print(f"Exercise target motor 0x{target:02X} on {ser.port}")
    if args.wake:
        print("Mode: --wake (reset → enable-only → full → sustained ctrl)")
    print("RS2 bench channel in host image byte 530 (not a power-distribution board).")
    print("PC7 ON = CAN activity; if it blinks but every line says 'no ext-CAN RX', the motor")
    print("is not replying (fault, supply, or needs power-cycle + --reset-first).")
    print()

    try:
        seq = begin_session(ser, reader, seq, args.timeout)

        if args.reset_first:
            print("Pre-resetting target motor...")
            resp = send_diag(ser, reader, target, PROBE_RESET, seq, max(args.timeout, 0.45))
            seq += 1
            print(format_probe_line("pre_reset", target, resp))
            time.sleep(0.25)
            print()

        for name, kind, timeout, repeat, pause in (
            WAKE_SEQUENCE if args.wake else EXERCISE_SEQUENCE
        ):
            for n in range(repeat):
                label = name if repeat == 1 else f"{name} [{n + 1}/{repeat}]"
                resp = send_diag(ser, reader, target, kind, seq, timeout)
                seq += 1
                print(format_probe_line(label, target, resp))
                if resp is not None and (resp["found"] or resp.get("raw_frames", 0) > 0):
                    any_can = True
                if resp is not None and resp["found"]:
                    any_parsed = True
                time.sleep(pause)
    finally:
        end_resp = send_diag(ser, reader, 0, SESSION_END, seq, args.timeout)
        session_ended = end_resp is not None
        seq += 1
        print()

    if args.actuator_test:
        print("--- Actuator-slot path (normal --monitor / teleop; CAN LED may stay dark) ---")
        print("Session ended — MCU 500 Hz loop may TX via actuator_commands[0] again.")
        print()
        reader.drain()
        period = 1.0 / max(args.actuator_hz, 1.0)
        for profile_name, pos, vel, kp, kd in ACTUATOR_PROFILES:
            print(f"  profile: {profile_name}  (8 frames @ {args.actuator_hz:.0f} Hz)")
            last_fb: Optional[dict] = None
            for _ in range(8):
                ser.write(build_actuator_command(pos, vel, kp, kd, 0.0, seq))
                ser.flush()
                seq += 1
                time.sleep(period)
                for frame in reader.drain():
                    fb = parse_actuator_feedback(frame)
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


def build_discover_id_list(args: argparse.Namespace) -> List[int]:
    if args.ids:
        return parse_id_list(args.ids)
    start = max(1, int(args.start) & 0xFF)
    end = min(127, int(args.end) & 0xFF)
    if end < start:
        start, end = end, start
    full_range = list(range(start, end + 1))
    if args.discover_quick:
        candidates = list(DISCOVER_PRIORITY_IDS)
    else:
        candidates = list(DISCOVER_PRIORITY_IDS) + full_range
    seen: Set[int] = set()
    out: List[int] = []
    for i in candidates:
        if 1 <= i <= 127 and i not in seen:
            seen.add(i)
            out.append(i)
    return out


def discover_probe_steps(probed_id: int, args: argparse.Namespace) -> Tuple[Tuple[str, int, float, int], ...]:
    if args.discover_light:
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
                max(args.timeout, probe_timeout),
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


def run_probe_id(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    motor_id = int(args.probe_id) & 0xFF
    seq = 1
    print(f"Deep probe motor 0x{motor_id:02X} on {ser.port}  (same wake as --bench-cmds)")
    print("Only ONE motor should be on CAN. Power on, shaft free.")
    print()

    try:
        seq = begin_session(ser, reader, seq, max(args.timeout, 0.5))
        best, seq = probe_one_id(ser, reader, motor_id, seq, args, verbose_steps=True)
    finally:
        end_session(ser, reader, seq, max(args.timeout, 0.5))
        stop.set()

    print()
    if best is None:
        print(f"Result: no CAN reply at probed ID 0x{motor_id:02X}.")
        print("  - Confirm THIS motor is wired (not the other RS02)")
        print("  - Try factory default: --probe-id 0x7F")
        print("  - PC7 should blink on TX even with no reply")
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
    seq = 1
    # replied_id -> list of (probed_id, probe_label, resp)
    findings: Dict[int, List[Tuple[int, str, dict]]] = {}

    print(f"Discover RS02 CAN ID on {ser.port}  ({len(id_list)} addresses)")
    probe_mode = "light (enable+promisc)" if args.discover_light else "deep on priority IDs (reset→enable→full→ctrl×8)"
    print(f"Probe mode: {probe_mode} — matches --bench-cmds wake on 0x7F/0x70/…")
    print("Only ONE motor on the bus. Swap units between tests — old 0x70 will not find a new motor.")
    print("Factory default is often 0x7F.")
    print()

    try:
        seq = begin_session(ser, reader, seq, max(args.timeout, 0.5))

        if args.reset_first:
            for rid in (0x7F, 0x70, 0x7E):
                send_diag(ser, reader, rid, PROBE_RESET, seq, 0.45)
                seq += 1
                time.sleep(0.08)
            print("Pre-reset sent to 0x7F, 0x70, 0x7E")
            print()

        for probed_id in id_list:
            best, seq = probe_one_id(ser, reader, probed_id, seq, args, verbose_steps=False)

            if best is not None:
                probe_label, resp = best
                print(format_discover_line(probed_id, probe_label, resp))
                disc = replied_motor_id(probed_id, resp)
                findings.setdefault(disc, []).append((probed_id, probe_label, resp))
            elif args.verbose:
                print(f"  ....  0x{probed_id:02X}  no CAN reply")

            time.sleep(max(args.gap, 0.05))
    finally:
        end_session(ser, reader, seq, max(args.timeout, 0.5))
        stop.set()

    print()
    print("Summary")
    if not findings:
        print("  No motor replies on any probed ID.")
        print("  If another RS02 works at 0x70 but this one shows nothing on 0x7F/0x70:")
        print("    → CAN wiring on THIS motor, JST orientation, or dead drive (no TX from motor).")
        print("  Try deep single-ID probe:")
        print(f"    python scripts/rs02_can_scan.py --port {ser.port} --probe-id 0x7F")
        print(f"    python scripts/rs02_can_scan.py --port {ser.port} --probe-id 0x70")
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
    seq = 1
    use_fast = False

    print(f"Scanning {len(id_list)} IDs on {ser.port}")
    print("Probe: RobStride private ext-CAN. Background actuator TX paused during session.")
    print()

    seq = begin_session(ser, reader, seq, args.timeout)

    if args.reset_first:
        reset_ids = [args.target & 0xFF, 0x7F, 0x6F]
        if args.ids:
            reset_ids = parse_id_list(args.ids)[:4]
        seen: Set[int] = set()
        reset_ids = [i for i in reset_ids if not (i in seen or seen.add(i))]
        print(f"Sending motor reset to: {', '.join(f'0x{i:02X}' for i in reset_ids)}")
        for rid in reset_ids:
            send_diag(ser, reader, rid, PROBE_RESET, seq, max(args.timeout, 0.35))
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

        resp = send_diag(ser, reader, motor_id, kind, seq, args.timeout)
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

    end_session(ser, reader, seq, args.timeout)
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
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--discover", action="store_true",
                    help="Find motor CAN ID (enable + promisc per address; 0x7F first)")
    ap.add_argument("--discover-quick", action="store_true",
                    help="With --discover: only priority IDs (0x7F, 0x70, …) not full 1–127")
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
    ap.add_argument("--bench-cmds", action="store_true",
                    help="Sequential cal/zero/save probes (one MCU block at a time)")
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
    ap.add_argument("--cal-timeout", type=float, default=28.0,
                    help="Seconds for --bench-cmds cal probe on MCU (default 28)")
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
                    help="After exercise, test normal actuator_commands path (like --monitor)")
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

    port = args.port or auto_pick_port()
    print(f"Opening {port} @ {args.baud}")
    with serial.Serial(port=port, baudrate=args.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        while ser.in_waiting:
            ser.read(ser.in_waiting)
        if args.probe_id is not None:
            rc = run_probe_id(ser, args)
        elif args.discover:
            rc = run_discover(ser, args)
        elif args.param_scan:
            rc = run_param_scan(ser, args.target, args)
        elif args.bench_cmds:
            rc = run_bench_cmds(ser, args.target, args)
        elif args.exercise or args.wake:
            rc = run_exercise(ser, args.target, args)
        else:
            rc = run_scan(ser, build_id_sequence(args), args)
    sys.exit(rc)


if __name__ == "__main__":
    main()
