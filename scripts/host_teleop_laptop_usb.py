#!/usr/bin/env python3
"""
Laptop USB CDC test for Deft controls PCB (562 B layout v1).

Use this on a Windows/Linux laptop with the board on USB — no Jetson, no UART mode.
Firmware must use HOST_TRANSPORT_UART 0 in App/Inc/host/host_transport.h.

Windows (Lenovo X1 etc.):
  pip install pyserial
  python scripts/host_teleop_laptop_usb.py --list-ports
  python scripts/host_teleop_laptop_usb.py --port COM9

Arrow-key teleop (default, RS2 CAN path — works without encoder feedback):
  Left / Right   small position steps (hold for continuous motion)
  r              re-sync cmd to shaft + zero velocity (stop coast)
  q              quit

Dual motor (alternating CAN, same arrow keys):
  python scripts/host_teleop_laptop_usb.py --port COM9 --motor-ids 0x70,0x74

Other modes: --monitor  --nudge  --read-params  --calibrate  --plant-teleop  --legacy-actuator

Calibrate on a specific FDCAN branch (host sets pdu.data[11]; MCU Phase 4 must read it):
  python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 3 --motor-id 0x74

Multi-bus runtime teleop (500 Hz plant slots — auto-homes to 0, then all actuators):
  python scripts/host_teleop_laptop_usb.py --port COM9 --plant-teleop

Dynamixel neck servos (XL330, plant servos[] @ 500 Hz):
  python scripts/dynamixel_teleop.py --port COM9
  python scripts/host_teleop_laptop_usb.py --port COM9 --servo-teleop

Launch demo (sequential CW sweep 0x70→0x74→0x73→0x75, then all to zero):
  python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from rs02_can_scan import (  # noqa: E402
    PROBE_CALI,
    PROBE_CTRL_FAST,
    PROBE_DATA_SAVE,
    PROBE_ENABLE_ONLY,
    PROBE_FULL,
    PROBE_PARAREAD,
    PROBE_PROACTIVE,
    PROBE_RESET,
    PROBE_ZERO,
    PLANT_ACTUATOR_TABLE,
    RS2_COMM_NAMES,
    SESSION_BEGIN,
    SESSION_END,
    build_plant_command,
    build_rs2_probe_command,
    build_rs2_teleop_command,
    can_bus_label,
    decode_ext_id,
    format_probe_line,
    normalize_can_bus,
    parse_actuator_feedback,
    parse_probe_pdu,
    print_can_bus_note,
    probe_target_label,
    rs2_comm_label,
    pararead_is_hit,
    pararead_index_echo,
    parse_id_list,
    send_diag,
)

PARAM_MECH_ANGLE = 0x7016
PARAM_MECH_POS = 0x7019
PARAM_RUN_MODE = 0x7005
PARAM_SPEED = 0x700A
PARAM_BUS_VOLT = 0x701C
VBUS_POLL_TIMEOUT_S = 0.45
DEFAULT_CAL_TIMEOUT_S = 28.0
CALI_CHUNK_S = 5.0
PARAM_READ_LIST = (
    ("mech_angle (0x7016)", PARAM_MECH_ANGLE),
    ("run_mode (0x7005)", PARAM_RUN_MODE),
    ("speed_setpoint (0x700A)", PARAM_SPEED),
)

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
ACTUATOR0_CMD_OFF = 16
ACTUATOR0_FB_OFF = 16

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -44.0, 44.0
POS_STEP = 0.02
DEFAULT_ARROW_VEL = 22.0
DEFAULT_RAMP_DOWN_S = 0.55
DEFAULT_RAMP_UP_S = 0.06
DEFAULT_KP = 50.0
DEFAULT_KD = 1.0
DEFAULT_HZ = 40.0
# --plant-teleop defaults (gentle; RS01 slots use lower kp than RS02 slot 0)
PLANT_DEFAULT_KP = 10.0
PLANT_DEFAULT_KD = 0.5
PLANT_DEFAULT_ARROW_VEL = 5.0
PLANT_DEFAULT_RAMP_UP_S = 0.4
PLANT_DEFAULT_RAMP_DOWN_S = 1.2
PLANT_SLOT_KP: Tuple[float, ...] = (12.0, 8.0, 8.0, 8.0)
PLANT_HOME_TARGET = 0.0
PLANT_HOME_SLEW_RAD_S = 0.28
PLANT_HOME_KP = 6.0
PLANT_HOME_POS_TOL = 0.05
PLANT_HOME_VEL_TOL = 0.15
PLANT_HOME_DWELL_S = 0.6
PLANT_HOME_TIMEOUT_S = 120.0
# Launch sequence: slot order = 0x70, 0x74, 0x73, 0x75 (slots 0–3)
LAUNCH_ORDER_SLOTS: Tuple[int, ...] = (0, 1, 2, 3)
LAUNCH_START_CW = -5.0
LAUNCH_END_CW = 10.0
LAUNCH_MOVE_VEL = 6.0
LAUNCH_POS_TOL = 0.06
LAUNCH_RAMP_UP_S = 0.44
LAUNCH_STAGGER_FRAC = 0.15
USB_BAUD = 115200
DEFAULT_MOTOR_ID = 0x70

STM32_VID = 0x0483

RS2_WAKE_STEPS = (
    (PROBE_RESET, 0.45),
    (PROBE_ENABLE_ONLY, 0.55),
    (PROBE_FULL, 0.35),
)

# Optional: proactive 0x18 floods the bus and has tripped under_voltage on bench 53V.
RS2_WAKE_STEPS_PROACTIVE = RS2_WAKE_STEPS + ((PROBE_PROACTIVE, 0.40),)

# Proactive (comm 0x18) confuses pararead diagnostics — omit for --read-params.
RS2_WAKE_STEPS_PARAMS = (
    (PROBE_RESET, 0.45),
    (PROBE_ENABLE_ONLY, 0.55),
    (PROBE_FULL, 0.35),
)

RS2_NUDGE_HZ = 15.0
VBUS_POLL_INTERVAL_S = 2.0


_live_line_len = 0


def write_live_line(text: str) -> None:
    """Overwrite one terminal row (keep short — PowerShell wraps long lines)."""
    global _live_line_len
    line = text.replace("\n", " ")[:118]
    pad = max(0, _live_line_len - len(line))
    sys.stdout.write("\r" + line + " " * pad + "\x1b[K")
    sys.stdout.flush()
    _live_line_len = len(line)


def write_live_notice(text: str) -> None:
    """One-shot message above the live row (faults, warnings)."""
    global _live_line_len
    sys.stdout.write("\n" + text + "\n")
    sys.stdout.flush()
    _live_line_len = 0


def decode_rs02_feedback_bytes(data: bytes) -> dict:
    if len(data) < 8:
        return {"position": 0.0, "raw_hex": data.hex(), "payload_empty": True}
    p_raw = (data[0] << 8) | data[1]
    v_raw = (data[2] << 8) | data[3]
    t_raw = (data[4] << 8) | data[5]
    temp_raw = (data[6] << 8) | data[7]
    payload_empty = data == b"\x00" * 8
    span_p = P_MAX - P_MIN
    pos = P_MIN + (p_raw / 65535.0) * span_p if not payload_empty else 0.0
    return {
        "position": pos,
        "p_raw": p_raw,
        "v_raw": v_raw,
        "t_raw": t_raw,
        "temp_raw": temp_raw,
        "raw_hex": data.hex(),
        "payload_empty": payload_empty,
    }


def format_payload_line(decoded: dict) -> str:
    if decoded["payload_empty"]:
        return f"p_raw=0  pos=n/a (8-byte payload empty)"
    return (
        f"p_raw={decoded['p_raw']:5d}  pos={decoded['position']:+.4f}  "
        f"data={decoded['raw_hex']}"
    )


def build_command_image(position: float, seq: int, kp: float, kd: float) -> bytes:
    position = max(P_MIN, min(P_MAX, position))
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    struct.pack_into("<fffff", buf, ACTUATOR0_CMD_OFF, position, 0.0, kp, kd, 0.0)
    return bytes(buf)


def parse_feedback_image(data: bytes) -> Optional[dict]:
    if len(data) != IMAGE_BYTES:
        return None
    magic, = struct.unpack_from("<I", data, 0)
    if magic != HOST_FEEDBACK_MAGIC:
        return None
    layout, = struct.unpack_from("<H", data, 4)
    byte_size, = struct.unpack_from("<H", data, 6)
    if layout != HOST_LAYOUT_VERSION or byte_size != IMAGE_BYTES:
        return None
    sys_word, = struct.unpack_from("<I", data, 12)
    pos, vel, torque, temp, fault = struct.unpack_from("<ffffI", data, ACTUATOR0_FB_OFF)
    return {
        "tick": sys_word & 0xFFF,
        "last_cmd_seq": (sys_word >> 17) & 0xFF,
        "position": pos,
        "velocity": vel,
        "torque": torque,
        "temperature": temp,
        "fault": fault,
    }


class FrameReader:
    def __init__(self) -> None:
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._frames: Deque[bytes] = deque(maxlen=16)

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        with self._lock:
            self._buf.extend(chunk)
            while len(self._buf) >= IMAGE_BYTES:
                magic, = struct.unpack_from("<I", self._buf, 0)
                if magic != HOST_FEEDBACK_MAGIC:
                    magic_bytes = struct.pack("<I", HOST_FEEDBACK_MAGIC)
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

    def drain(self) -> None:
        with self._lock:
            self._frames.clear()


def serial_rx_thread(ser: serial.Serial, reader: FrameReader, stop: threading.Event) -> None:
    try:
        while not stop.is_set():
            try:
                chunk = ser.read(max(1, ser.in_waiting))
            except serial.SerialException:
                break
            if chunk:
                reader.feed(chunk)
            else:
                time.sleep(0.001)
    except Exception:
        import traceback

        traceback.print_exc()


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Plug in USB and check Device Manager (Windows) or dmesg (Linux).")
        return
    print("Available ports:")
    for p in sorted(ports, key=lambda x: x.device):
        hint = ""
        if p.vid == STM32_VID:
            hint = "  <- likely STM32 USB CDC"
        print(f"  {p.device:16s}  {p.description}{hint}")


def score_port(device: str, description: str, vid: Optional[int]) -> int:
    score = 0
    text = f"{device} {description}".upper()
    if vid == STM32_VID:
        score += 100
    for token in ("STM", "CDC", "VIRTUAL COM", "DEft".upper(), "G4"):
        if token in text:
            score += 10
    if device.upper().startswith("COM"):
        score += 1
    if "ACM" in device:
        score += 1
    return score


def auto_pick_port() -> str:
    candidates = list(list_ports.comports())
    if candidates:
        best = max(
            candidates,
            key=lambda p: score_port(p.device, p.description or "", p.vid),
        )
        return best.device

    if sys.platform == "win32":
        return "COM3"

    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]
    return "/dev/ttyACM0"


def poll_key_nonblocking(extra: Optional[Tuple[str, ...]] = None) -> Optional[str]:
    """Non-blocking key poll. extra: additional single-char keys (e.g. plant bus 0–3)."""
    allowed = frozenset(("q", "r"))
    if extra:
        allowed = allowed | frozenset(extra)
    if sys.platform == "win32":
        import msvcrt

        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):
            ch2 = msvcrt.getch()
            if ch2 == b"K":
                return "left"
            if ch2 == b"M":
                return "right"
            return None
        try:
            c = ch.decode("utf-8", errors="ignore").lower()
        except Exception:
            return None
        if c in allowed:
            return c
        return None

    import select

    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip().lower()
        if line in ("q", "quit"):
            return "q"
        if line in allowed:
            return line
        if line in ("left", "right", "l", "o"):
            return line
    return None


def poll_arrow_direction() -> int:
    """Return -1 (left), +1 (right), or 0. Windows reads physical key hold."""
    if sys.platform == "win32":
        import ctypes

        user32 = ctypes.windll.user32
        left = user32.GetAsyncKeyState(0x25) & 0x8000
        right = user32.GetAsyncKeyState(0x27) & 0x8000
        if left and not right:
            return -1
        if right and not left:
            return 1
        return 0
    return 0


@dataclass
class SessionStats:
    fb_count: int = 0
    last_tick: Optional[int] = None
    tick_stalls: int = 0


@dataclass
class VbusPollState:
    volts: Optional[float] = None
    last_request_mono: float = 0.0
    request_pending_mono: Optional[float] = None


@dataclass
class MotorTeleopState:
    motor_id: int
    cmd_position: float = 0.0
    feedback_synced: bool = False
    active_kp: float = 0.0
    last_probe: Optional[dict] = None

    def sync_kp(self, kp: float) -> None:
        self.active_kp = kp if self.feedback_synced else 0.0


def maybe_send_vbus_pararead(
    ser: serial.Serial,
    motor_id: int,
    seq: int,
    state: VbusPollState,
    allow: bool,
) -> int:
    """Fire comm 0x11 pararead for VBUS without blocking the teleop ctrl stream."""
    if not allow:
        return seq
    now = time.monotonic()
    if state.request_pending_mono is not None:
        if now - state.request_pending_mono > VBUS_POLL_TIMEOUT_S:
            state.request_pending_mono = None
        return seq
    if now - state.last_request_mono < VBUS_POLL_INTERVAL_S:
        return seq
    ser.write(
        build_rs2_probe_command(motor_id & 0xFF, PROBE_PARAREAD, seq, PARAM_BUS_VOLT)
    )
    ser.flush()
    state.last_request_mono = now
    state.request_pending_mono = now
    return (seq + 1) & 0xFFFFFFFF


def poll_rs2_reader(
    reader: FrameReader,
    motor_id: int,
    last_probe: Optional[dict],
    vbus: VbusPollState,
) -> Optional[dict]:
    """Drain USB feedback; update move telemetry and any pending VBUS pararead reply."""
    motor = MotorTeleopState(motor_id & 0xFF, last_probe=last_probe)
    poll_rs2_reader_all(reader, [motor], vbus)
    return motor.last_probe


def poll_rs2_reader_all(
    reader: FrameReader,
    motors: List[MotorTeleopState],
    vbus: VbusPollState,
) -> List[MotorTeleopState]:
    """Drain USB feedback and route probe PDU rows to the matching motor_id."""
    by_id = {m.motor_id & 0xFF: m for m in motors}
    frame = reader.pop()
    while frame is not None:
        parsed = parse_probe_pdu(frame)
        if parsed is not None:
            mid = parsed.get("probe_id", 0) & 0xFF
            target = by_id.get(mid)
            if target is not None:
                if (
                    parsed.get("probe_kind") == PROBE_PARAREAD
                    and pararead_is_hit(parsed, PARAM_BUS_VOLT)
                ):
                    vbus.volts = parsed["position"]
                    vbus.request_pending_mono = None
                else:
                    fb = parse_feedback_image(frame)
                    probe = dict(parsed)
                    if fb is not None:
                        probe["ack"] = fb["last_cmd_seq"]
                    target.last_probe = probe
                    decoded = decode_rs02_feedback_bytes(probe["can_data"])
                    if not target.feedback_synced and not decoded["payload_empty"]:
                        target.feedback_synced = True
                        target.cmd_position = decoded["position"]
        frame = reader.pop()
    return motors


def rs2_read_param(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    param_index: int,
    seq: int,
    timeout_s: float = 0.8,
    bus: int = 1,
) -> tuple[int, Optional[dict], Optional[dict]]:
    """Return (next_seq, pararead_reply_or_None, last_sniff_or_None)."""
    reader.drain()
    buf = build_rs2_probe_command(motor_id, PROBE_PARAREAD, seq, param_index, bus=bus)
    ser.write(buf)
    ser.flush()
    resp: Optional[dict] = None
    sniff: Optional[dict] = None
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
                if pararead_is_hit(parsed, param_index):
                    resp = parsed
                elif parsed.get("found"):
                    sniff = parsed
            frame = reader.pop()
        if resp is not None:
            break
        time.sleep(0.005)
    return seq + 1, resp, sniff


def print_rs2_comm_reference() -> None:
    print("RS2 private 29-bit ext-CAN comm types (mode field in ext_id):")
    for comm in sorted(RS2_COMM_NAMES):
        print(f"  {rs2_comm_label(comm)}")
    print("  Register read/write: host sends 0x11/0x12; expect read reply comm=0x11, float in data[4:8].")
    print()


def run_read_params(ser: serial.Serial, motor_id: int, extra_indexes: list[int]) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    seq = 1
    motor_id &= 0xFF
    reads = list(PARAM_READ_LIST)
    for index in extra_indexes:
        reads.append((f"custom (0x{index:04X})", index))

    print(f"RS2 parameter read on {ser.port}  motor=0x{motor_id:02X}")
    print("Requires firmware PROBE_PARAREAD (kind 14). Success = comm 0x11 pararead reply.")
    print_rs2_comm_reference()
    try:
        seq = rs2_begin_session(ser, reader, motor_id, seq)
        seq = rs2_wake_motor(ser, reader, motor_id, seq, steps=RS2_WAKE_STEPS_PARAMS)
        print()
        for label, index in reads:
            seq, resp, sniff = rs2_read_param(ser, reader, motor_id, index, seq)
            time.sleep(0.15)
            if resp is None:
                line = f"  {label}: no comm=0x11 pararead reply"
                if sniff is not None:
                    line += (
                        f"  (saw {rs2_comm_label(sniff['comm_mode'])} instead, "
                        f"data={sniff['can_data'].hex()})"
                    )
                print(line)
                continue
            data_hex = resp["can_data"].hex()
            idx_echo = pararead_index_echo(resp)
            print(
                f"  {label}: {rs2_comm_label(resp['comm_mode'])}  "
                f"index_echo=0x{idx_echo:04X}  "
                f"float@data[4:8]={resp['position']:+.4f}  raw={data_hex}"
            )
        print()
        print("comm=0x02 motor_feedback (move telemetry) is separate from comm=0x11 pararead.")
        print("If every read fails: encoder cal (0x05), RobStride tool, or motor not on private protocol.")
    finally:
        stop.set()
        time.sleep(0.05)
        rs2_end_session(ser, reader, motor_id, seq)


def run_calibrate(
    ser: serial.Serial,
    motor_id: int,
    cal_timeout_s: float,
    kp: float,
    kd: float,
    bus: int = 1,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    seq = 1
    motor_id &= 0xFF
    bus = normalize_can_bus(bus)
    cal_timeout_s = max(5.0, cal_timeout_s)

    print(f"RS2 encoder cal on {ser.port}  {probe_target_label(motor_id, bus)}")
    print_can_bus_note(bus)
    print("Requires firmware with PROBE_CALI/ZERO/DATA_SAVE (kinds 16-18). Reflash MCU after pull.")
    print("Safety: 48 V supply, shaft free to rotate, nothing on the output.")
    print(f"Steps: comm 0x05 cali (up to {cal_timeout_s:.0f}s) → 0x06 zero → 0x16 save → verify.")
    print()

    verify_reads = (
        ("mechPos (0x7019)", PARAM_MECH_POS),
        ("VBUS (0x701C)", PARAM_BUS_VOLT),
        ("run_mode (0x7005)", PARAM_RUN_MODE),
    )

    try:
        seq = rs2_begin_session(ser, reader, motor_id, seq)
        seq = rs2_wake_motor(ser, reader, motor_id, seq, steps=RS2_WAKE_STEPS_PARAMS, bus=bus)

        print("--- enter running (ctrl burst before cal) ---")
        seq, running = rs2_prime_running(ser, reader, motor_id, seq, kp, kd, bus=bus)
        if running:
            print("  mms=running — motor enabled for cal")
        else:
            print("  warning: never saw mms=running (cal may still work if shaft is free)")
        print()

        print(f"--- motor_cali comm 0x05 (listen {cal_timeout_s:.0f}s on MCU — shaft must spin) ---")
        cal_listen_s = max(10, int(cal_timeout_s))
        resp = send_diag(
            ser, reader, motor_id, PROBE_CALI, seq, cal_listen_s + 15.0, cal_listen_s, bus=bus
        )
        seq += 1
        saw_cali_mode = False
        saw_running_after = False
        if resp is None:
            print("  ----  motor_cali  (no USB feedback)")
            print("  stopped — MCU busy or reflash firmware with cal listen fix")
        elif resp.get("found"):
            ext = decode_ext_id(resp["ext_id"])
            mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
            if ext.mode_status == 1:
                saw_cali_mode = True
            if ext.mode_status == 2:
                saw_running_after = True
            print(format_probe_line("motor_cali", motor_id, resp))
            print(f"         mms={mms}  can_data={resp['can_data'].hex()}  rx={resp.get('raw_frames', 0)}")
        else:
            print(format_probe_line("motor_cali", motor_id, resp))

        if saw_cali_mode and not saw_running_after:
            print("  waiting for mms=running after cal (poll enable, do not zero yet)...")
            deadline = time.monotonic() + max(15.0, cal_timeout_s)
            while time.monotonic() < deadline:
                resp = send_diag(ser, reader, motor_id, PROBE_ENABLE_ONLY, seq, 0.6, bus=bus)
                seq += 1
                if resp and resp.get("found"):
                    ext = decode_ext_id(resp["ext_id"])
                    if ext.mode_status == 2:
                        saw_running_after = True
                        print(format_probe_line("post_cal enable", motor_id, resp))
                        break
                time.sleep(0.5)
            if not saw_running_after:
                print("  still not mms=running — power-cycle motor, run --recovery, retry cal")

        if not resp or not resp.get("found"):
            print("  no ext-CAN RX during cal — check 48 V and firmware kinds 16-18")
        elif not saw_cali_mode and not saw_running_after:
            print("  motor never reported mms=cali — 0x05 may not have started encoder cal")
        print()

        print("--- motor_zero comm 0x06 ---")
        resp = send_diag(ser, reader, motor_id, PROBE_ZERO, seq, 4.0, bus=bus)
        seq += 1
        print(format_probe_line("motor_zero", motor_id, resp))
        print()

        print("--- data_save comm 0x16 ---")
        resp = send_diag(ser, reader, motor_id, PROBE_DATA_SAVE, seq, 5.0, bus=bus)
        seq += 1
        print(format_probe_line("data_save", motor_id, resp))
        print()

        print("--- pararead verify ---")
        got_pararead = False
        for label, index in verify_reads:
            seq, resp, sniff = rs2_read_param(ser, reader, motor_id, index, seq, bus=bus)
            time.sleep(0.12)
            if resp is None:
                line = f"  {label}: no comm=0x11 pararead reply"
                if sniff is not None:
                    line += f" (saw {rs2_comm_label(sniff['comm_mode'])})"
                print(line)
                continue
            got_pararead = True
            idx_echo = resp["can_data"][0] | (resp["can_data"][1] << 8)
            print(
                f"  {label}: {rs2_comm_label(resp['comm_mode'])}  "
                f"index_echo=0x{idx_echo:04X}  float={resp['position']:+.4f}  "
                f"raw={resp['can_data'].hex()}"
            )
        print()

        print("--- comm 0x02 feedback after hold at zero ---")
        best_hex = "0000000000000000"
        for _ in range(15):
            seq = rs2_send_ctrl_frame(ser, motor_id, 0.0, kp, kd, seq)
            probe = latest_probe_feedback(reader)
            if probe is not None:
                decoded = decode_rs02_feedback_bytes(probe["can_data"])
                best_hex = decoded["raw_hex"]
                ext = decode_ext_id(probe["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                print(f"  mms={mms}  {format_payload_line(decoded)}")
            rs2_ctrl_sleep(15.0)

        print()
        if best_hex != "0000000000000000" or got_pararead:
            print("Result: calibration may have succeeded — saw non-zero pararead or move feedback.")
        elif saw_cali_mode:
            print("Result: motor entered cal mode but telemetry still empty — retry zero/save or reflash.")
        else:
            print("Result: cal sequence finished but telemetry still empty / no pararead.")
            print("  - Reflash MCU (cali now 5s chunks; host retries; stale probe_kind filter fixed).")
            print("  - Check 48 V, motor faults, shaft free during cal, and mms=running before 0x05.")
    finally:
        stop.set()
        time.sleep(0.05)
        rs2_end_session(ser, reader, motor_id, seq)


def run_nudge_test(ser: serial.Serial, motor_id: int, kp: float, kd: float) -> None:
    """Command small position steps; watch whether CAN feedback bytes ever go non-zero."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    seq = 1
    motor_id &= 0xFF
    best_p_raw = 0
    any_nonzero_data = False

    print(f"RS2 nudge test on {ser.port}  motor=0x{motor_id:02X}  kp={kp}  kd={kd}")
    print("Commands small setpoints — output must be free to move slightly.")
    print("Watch for non-zero can_data / p_raw (no hand rotation needed).")
    print()

    try:
        seq = rs2_begin_session(ser, reader, motor_id, seq)
        seq = rs2_wake_motor(ser, reader, motor_id, seq)

        shaft_pos, _ = rs2_sync_cmd_from_feedback(reader)
        if shaft_pos == 0.0:
            print("  warning: could not read shaft position — relative steps from 0 rad")
        else:
            print(f"  shaft at {shaft_pos:+.4f} rad — nudge steps are relative to here")
        print()

        targets = (
            ("hold at shaft", shaft_pos, 1.0),
            ("step +0.05 rad", shaft_pos + 0.05, 1.0),
            ("step +0.10 rad", shaft_pos + 0.10, 1.0),
            ("back to shaft", shaft_pos, 1.0),
        )
        for label, position, hold_s in targets:
            print(f"--- {label} (pos={position:+.3f} rad, {hold_s:.0f}s) ---")
            deadline = time.monotonic() + hold_s
            last_hex = "0000000000000000"
            fault_announced = False
            while time.monotonic() < deadline:
                hold_pos = position
                hold_kp = kp
                probe = latest_probe_feedback(reader)
                if probe is not None:
                    ext = decode_ext_id(probe["ext_id"])
                    if ext.faults:
                        decoded = decode_rs02_feedback_bytes(probe["can_data"])
                        if not decoded["payload_empty"]:
                            hold_pos = decoded["position"]
                        hold_kp = min(kp, 3.0)
                        if not fault_announced:
                            print(
                                f"  fault {ext.faults} — holding at fb={hold_pos:+.4f} "
                                f"with reduced kp={hold_kp:.1f}"
                            )
                            fault_announced = True
                seq = rs2_send_ctrl_frame(ser, motor_id, hold_pos, hold_kp, kd, seq)
                probe = latest_probe_feedback(reader)
                if probe is not None:
                    decoded = decode_rs02_feedback_bytes(probe["can_data"])
                    last_hex = decoded["raw_hex"]
                    if last_hex != "0000000000000000":
                        any_nonzero_data = True
                    if decoded["p_raw"] > best_p_raw:
                        best_p_raw = decoded["p_raw"]
                    ext = decode_ext_id(probe["ext_id"])
                    mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                    fault_s = ",".join(ext.faults) if ext.faults else "none"
                    print(f"  mms={mms}  faults=[{fault_s}]  {format_payload_line(decoded)}")
                rs2_ctrl_sleep(RS2_NUDGE_HZ)

        print()
        if any_nonzero_data:
            print(f"Result: saw non-zero feedback (best p_raw={best_p_raw}). Encoder path is alive.")
        else:
            print("Result: motor moved but 8-byte feedback payload stayed all-zero.")
            print("  - Control path OK (motion proves CAN TX/RX envelope works).")
            print("  - Telemetry bytes usually need encoder calibration (RobStride tool / MOTOR_CALI).")
            print("  - Reflash firmware for proactive-on wake step, then retry --nudge.")
    finally:
        rs2_end_session(ser, reader, motor_id, seq)
        stop.set()


def rs2_sync_cmd_from_feedback(reader: FrameReader) -> Tuple[float, Optional[dict]]:
    """Align command position with shaft so wake/full_init does not fight a ~6 rad offset."""
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        probe = latest_probe_feedback(reader)
        if probe is not None and probe.get("found"):
            decoded = decode_rs02_feedback_bytes(probe["can_data"])
            if not decoded["payload_empty"]:
                return decoded["position"], probe
        time.sleep(0.01)
    return 0.0, None


def rs2_sync_cmd_for_motor(reader: FrameReader, motor_id: int, timeout_s: float = 0.6) -> Tuple[float, Optional[dict]]:
    """Sync cmd_pos to one motor's comm=0x02 feedback (required on a multi-motor bus)."""
    motor_id &= 0xFF
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_probe_pdu(frame)
            if parsed is not None and (parsed.get("probe_id", 0) & 0xFF) == motor_id:
                decoded = decode_rs02_feedback_bytes(parsed["can_data"])
                if not decoded["payload_empty"]:
                    return decoded["position"], parsed
            frame = reader.pop()
        time.sleep(0.01)
    return 0.0, None


def rs2_init_motor_teleop_state(
    reader: FrameReader,
    motor_id: int,
    kp: float,
) -> MotorTeleopState:
    motor = MotorTeleopState(motor_id & 0xFF)
    synced_pos, sync_probe = rs2_sync_cmd_for_motor(reader, motor.motor_id)
    if sync_probe is not None:
        decoded = decode_rs02_feedback_bytes(sync_probe["can_data"])
        if not decoded["payload_empty"]:
            motor.feedback_synced = True
            motor.cmd_position = synced_pos
            motor.last_probe = sync_probe
            motor.sync_kp(kp)
            ext = decode_ext_id(sync_probe["ext_id"])
            mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
            fault_s = ",".join(ext.faults) if ext.faults else "none"
            print(
                f"  0x{motor.motor_id:02X} synced cmd_pos to shaft  fb={synced_pos:+.4f} rad  "
                f"mms={mms}  faults=[{fault_s}]"
            )
        else:
            print(f"  0x{motor.motor_id:02X}  feedback envelope OK but payload empty")
    else:
        print(f"  0x{motor.motor_id:02X}  no position feedback — kp=0 until sync")
    motor.sync_kp(kp)
    return motor


def rs2_begin_session(ser: serial.Serial, reader: FrameReader, motor_id: int, seq: int) -> int:
    resp = send_diag(ser, reader, motor_id, SESSION_BEGIN, seq, 2.0)
    if resp is None:
        print("Warning: RS2 session begin not acked (timeout — continuing anyway)")
    time.sleep(0.05)
    return seq + 1


def rs2_prime_running(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    kp: float,
    kd: float,
    bursts: int = 25,
    hz: float = 25.0,
    bus: int = 1,
) -> tuple[int, bool]:
    """Ctrl burst until ext-CAN mode_status=running (needed before encoder cal on some RS02)."""
    for _ in range(bursts):
        seq = rs2_send_ctrl_frame(ser, motor_id, 0.0, kp, kd, seq, bus=bus)
        probe = latest_probe_feedback(reader)
        if probe is not None:
            ext = decode_ext_id(probe["ext_id"])
            if ext.mode_status == 2:
                return seq, True
        rs2_ctrl_sleep(hz)
    return seq, False


def rs2_end_session(ser: serial.Serial, reader: FrameReader, motor_id: int, seq: int) -> None:
    send_diag(ser, reader, motor_id, SESSION_END, seq, 0.35)


def rs2_stop_motor(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    hold_position: float = 0.0,
    frames: int = 12,
    hz: float = 40.0,
) -> int:
    """Zero-gain hold then comm 0x04 reset — de-energize after teleop."""
    for _ in range(frames):
        seq = rs2_send_ctrl_frame(ser, motor_id, hold_position, 0.0, 0.0, seq, 0.0)
        rs2_ctrl_sleep(hz)
    send_diag(ser, reader, motor_id, PROBE_RESET, seq, 0.55)
    return (seq + 1) & 0xFFFFFFFF


def rs2_wake_motor(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    steps: tuple[tuple[int, float], ...] = RS2_WAKE_STEPS,
    bus: int = 1,
) -> int:
    print(f"RS2 wake motor 0x{motor_id:02X} on {can_bus_label(bus)} (reset → enable-only → full_init)...")
    last_resp: Optional[dict] = None
    for kind, timeout in steps:
        resp = send_diag(ser, reader, motor_id, kind, seq, timeout, bus=bus)
        seq += 1
        if resp is not None:
            last_resp = resp
        if resp is not None and resp.get("found"):
            ext = decode_ext_id(resp["ext_id"])
            mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
            fault_s = ",".join(ext.faults) if ext.faults else "none"
            print(
                f"  kind={kind:2d}  HIT  ext=0x{resp['ext_id']:08X}  "
                f"mms={mms}  faults=[{fault_s}]  "
                f"T={resp['temperature']:.1f}C  pos={resp['position']:+.4f}"
            )
        elif resp is not None:
            kind_name = {11: "reset", 12: "enable-only", 0: "full_init", 15: "proactive_on"}.get(kind, str(kind))
            print(f"  kind={kind:2d} ({kind_name})  TX ok, no parsed RX (PC7 should blink)")
        else:
            print(f"  kind={kind:2d}  (no USB feedback)")
        time.sleep(0.12)
    if last_resp is not None and last_resp.get("found"):
        ext = decode_ext_id(last_resp["ext_id"])
        mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
        if mms != "running" or ext.faults:
            print(
                "  WARNING: wake finished with "
                f"mms={mms} faults=[{','.join(ext.faults) or 'none'}] — "
                "motion may not work; run rs02_can_scan.py --recovery first."
            )
    print()
    return seq


def rs2_send_ctrl_frame(
    ser: serial.Serial,
    motor_id: int,
    position: float,
    kp: float,
    kd: float,
    seq: int,
    velocity: float = 0.0,
    bus: int = 1,
) -> int:
    """One RS2 PROBE_CTRL_FAST → comm 0x01 MOTOR_CTRL on 29-bit extended CAN."""
    motor_id &= 0xFF
    velocity = max(V_MIN, min(V_MAX, velocity))
    ser.write(
        build_rs2_teleop_command(
            motor_id,
            PROBE_CTRL_FAST,
            position,
            velocity,
            kp,
            kd,
            0.0,
            seq,
            bus=bus,
        )
    )
    ser.flush()
    return (seq + 1) & 0xFFFFFFFF


def rs2_ctrl_sleep(hz: float) -> None:
    time.sleep(1.0 / max(hz, 0.1))


def latest_probe_feedback(reader: FrameReader) -> Optional[dict]:
    latest: Optional[dict] = None
    frame = reader.pop()
    while frame is not None:
        probe = parse_probe_pdu(frame)
        if probe is not None:
            fb = parse_feedback_image(frame)
            if fb is not None:
                probe = dict(probe)
                probe["ack"] = fb["last_cmd_seq"]
            latest = probe
        frame = reader.pop()
    return latest


def format_rs2_line(
    fb_num: int,
    cmd_seq: int,
    probe: dict,
) -> str:
    ack = probe.get("ack", 0)
    ext = decode_ext_id(probe["ext_id"])
    mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
    fault_s = ",".join(ext.faults) if ext.faults else "none"
    hit = "HIT" if probe["found"] else "----"
    return (
        f"{fb_num:5d}  "
        f"ack={ack:3d}  "
        f"cmd={cmd_seq & 0xFF:3d}  "
        f"{hit}  "
        f"ext=0x{probe['ext_id']:08X}  "
        f"mms={mms}  "
        f"pos={probe['position']:+.4f}  "
        f"T={probe['temperature']:5.1f}  "
        f"faults=[{fault_s}]  "
        f"data={probe['can_data'].hex()}"
    )


@dataclass
class PlantSlotTeleop:
    slot: int
    fdcan_bus: int
    motor_id: int
    max_kp: float
    cmd_position: float = 0.0
    cmd_velocity: float = 0.0
    kp: float = 0.0
    feedback_synced: bool = False
    fb_position: float = 0.0
    fb_velocity: float = 0.0

    def label(self) -> str:
        return f"slot{self.slot} {can_bus_label(self.fdcan_bus)} 0x{self.motor_id:02X} kp<={self.max_kp:.0f}"


def plant_teleop_targets(slot_states: List[PlantSlotTeleop], active_bus: int) -> List[PlantSlotTeleop]:
    """active_bus 0 = all slots; 1–3 = schematic CH1–CH3 only."""
    if active_bus == 0:
        return list(slot_states)
    return [st for st in slot_states if st.fdcan_bus == active_bus]


def plant_teleop_bus_label(active_bus: int) -> str:
    if active_bus == 0:
        return "all buses"
    return f"bus {active_bus} ({can_bus_label(active_bus)})"


def plant_active_bus_readback(active_bus: int) -> str:
    """Compact live status tag for teleop HUD."""
    if active_bus == 0:
        return "ACTIVE_BUS=ALL"
    return f"ACTIVE_BUS={active_bus} ({can_bus_label(active_bus)})"


def make_plant_slot_states(
    slots: List[int],
    slot_kp: Tuple[float, ...] = PLANT_SLOT_KP,
) -> List[PlantSlotTeleop]:
    slot_states: List[PlantSlotTeleop] = []
    for slot in slots:
        if slot < 0 or slot >= len(PLANT_ACTUATOR_TABLE):
            print(f"Invalid plant slot {slot} (need 0..{len(PLANT_ACTUATOR_TABLE) - 1})", file=sys.stderr)
            sys.exit(2)
        fdcan_bus, motor_id = PLANT_ACTUATOR_TABLE[slot]
        max_kp = slot_kp[slot] if slot < len(slot_kp) else slot_kp[-1]
        slot_states.append(
            PlantSlotTeleop(
                slot=slot,
                fdcan_bus=fdcan_bus,
                motor_id=motor_id,
                max_kp=max_kp,
                kp=0.0,
            )
        )
    return slot_states


def plant_step_toward(st: PlantSlotTeleop, target: float, rate: float, dt: float) -> bool:
    """Slew cmd_position toward target at rate rad/s; return True when at target."""
    delta = target - st.cmd_position
    step = rate * dt
    if abs(delta) <= step:
        st.cmd_position = target
    else:
        st.cmd_position += math.copysign(step, delta)
    st.cmd_position = max(P_MIN, min(P_MAX, st.cmd_position))
    return abs(st.cmd_position - target) <= max(1e-4, LAUNCH_POS_TOL * 0.25)


def plant_at_target(st: PlantSlotTeleop, target: float, tol: float = LAUNCH_POS_TOL) -> bool:
    return abs(st.cmd_position - target) <= tol


def plant_startup_sync(
    ser: serial.Serial,
    reader: FrameReader,
    slot_states: List[PlantSlotTeleop],
    cmd_seq: int,
    dt: float,
    send_hz: float,
) -> int:
    print("Syncing feedback...")
    reader.drain()
    for _ in range(max(4, int(send_hz * 0.5))):
        ser.write(build_plant_command(cmd_seq, {}))
        ser.flush()
        cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
        time.sleep(dt)
        plant_poll_feedback(reader, slot_states)
    plant_sync_feedback(reader, slot_states, dwell_s=1.0)
    print()
    return cmd_seq


def plant_slew_all_to(
    ser: serial.Serial,
    reader: FrameReader,
    slot_states: List[PlantSlotTeleop],
    cmd_seq: int,
    dt: float,
    send_hz: float,
    kd: float,
    target: float,
    slew_rate: float,
    move_kp: float,
    label: str,
    timeout_s: float = 90.0,
) -> Tuple[int, bool]:
    """Move every synced slot's cmd_position to target; return (cmd_seq, aborted)."""
    active = [st for st in slot_states if st.feedback_synced]
    if not active:
        print(f"{label}: skipped (no feedback).")
        return cmd_seq, False

    print(f"{label}: → {target:+.2f} rad @ {slew_rate:.1f} rad/s (q aborts)")
    deadline = time.monotonic() + timeout_s
    line_n = 0
    aborted = False

    while time.monotonic() < deadline:
        if poll_key_nonblocking() == "q":
            aborted = True
            break

        all_done = True
        for st in active:
            st.cmd_velocity = 0.0
            if not plant_at_target(st, target):
                plant_step_toward(st, target, slew_rate, dt)
                all_done = False
            st.kp = 0.0 if plant_at_target(st, target) else move_kp

        for st in slot_states:
            if st not in active:
                st.kp = 0.0
                st.cmd_velocity = 0.0

        cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
        plant_poll_feedback(reader, slot_states)

        line_n += 1
        if line_n % max(1, int(send_hz / 3)) == 0:
            parts = [f"{st.slot}:cmd={st.cmd_position:+.2f} fb={st.fb_position:+.2f}" for st in active]
            write_live_line(f"{label[:12]:12s}  " + "  ".join(parts))

        if all_done:
            break
        time.sleep(dt)

    for st in slot_states:
        st.cmd_velocity = 0.0
        if st.feedback_synced:
            st.cmd_position = target
        st.kp = 0.0

    for _ in range(max(4, int(send_hz * 0.15))):
        cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
        plant_poll_feedback(reader, slot_states)
        time.sleep(dt)

    if aborted:
        print(f"{label}: aborted (q).")
    print()
    return cmd_seq, aborted


def launch_sweep_progress(st: PlantSlotTeleop, start_pos: float, end_pos: float) -> float:
    span = end_pos - start_pos
    if abs(span) < 1e-6:
        return 1.0
    return max(0.0, min(1.0, (st.cmd_position - start_pos) / span))


def run_plant_launch_seq(
    ser: serial.Serial,
    send_hz: float,
    kd: float,
    move_vel: float = LAUNCH_MOVE_VEL,
    start_pos: float = LAUNCH_START_CW,
    end_pos: float = LAUNCH_END_CW,
    slot_kp: Tuple[float, ...] = PLANT_SLOT_KP,
    order: Tuple[int, ...] = LAUNCH_ORDER_SLOTS,
) -> None:
    """
    Demo launch: prep to start, staggered sweep to end (15% overlap),
    then staggered return to 0 when the last motor finishes the outbound sweep.
    """
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    slots = list(order)
    slot_states = make_plant_slot_states(slots, slot_kp)
    by_slot = {st.slot: st for st in slot_states}
    cmd_seq = 1
    dt = 1.0 / max(send_hz, 0.1)
    hold_kp = PLANT_HOME_KP

    id_chain = " → ".join(f"0x{by_slot[s].motor_id:02X}" for s in order if s in by_slot)
    print(f"Launch sequence on {ser.port} @ {send_hz:.0f} Hz")
    print(f"  Order: {id_chain}")
    print(f"  Start {start_pos:+.1f} rad → end {end_pos:+.1f} rad @ {move_vel:.1f} rad/s")
    print(f"  Return: when last motor hits end, first returns to 0 (same {LAUNCH_STAGGER_FRAC * 100:.0f}% stagger)")
    print("  q aborts during motion")
    print()

    try:
        cmd_seq = plant_startup_sync(ser, reader, slot_states, cmd_seq, dt, send_hz)

        cmd_seq, aborted = plant_slew_all_to(
            ser, reader, slot_states, cmd_seq, dt, send_hz, kd,
            start_pos, min(move_vel * 0.5, 3.0), hold_kp, "Prep (all to start)",
        )
        if aborted:
            return

        valid_order = [
            s for s in order if s in by_slot and by_slot[s].feedback_synced
        ]
        for slot in order:
            if slot in by_slot and not by_slot[slot].feedback_synced:
                print(f"  skip slot {slot} (no feedback)")

        zero_pos = 0.0
        if not valid_order:
            print("Launch sweep skipped: no synced motors.")
        else:
            phase = "forward"
            started: set[int] = {valid_order[0]}
            done: set[int] = set()
            leg_start_mono: dict[int, float] = {valid_order[0]: time.monotonic()}
            deadline = time.monotonic() + 180.0
            line_n = 0
            last_slot = valid_order[-1]

            print(f"--- outbound stagger ({LAUNCH_STAGGER_FRAC * 100:.0f}% overlap) ---")
            for i, slot in enumerate(valid_order):
                mid = by_slot[slot].motor_id
                if i == 0:
                    print(f"  0x{mid:02X} starts immediately")
                else:
                    prev = by_slot[valid_order[i - 1]].motor_id
                    print(f"  0x{mid:02X} starts when 0x{prev:02X} is {LAUNCH_STAGGER_FRAC * 100:.0f}% outbound")
            print(f"  return begins when 0x{by_slot[last_slot].motor_id:02X} reaches end ({end_pos:+.1f} rad)")
            print()

            while time.monotonic() < deadline:
                if poll_key_nonblocking() == "q":
                    print("Launch aborted (q).")
                    return

                if phase == "forward":
                    for i in range(1, len(valid_order)):
                        cur = valid_order[i]
                        if cur in started:
                            continue
                        prev = by_slot[valid_order[i - 1]]
                        if launch_sweep_progress(prev, start_pos, end_pos) >= LAUNCH_STAGGER_FRAC:
                            started.add(cur)
                            leg_start_mono[cur] = time.monotonic()
                            print(
                                f"  → 0x{by_slot[cur].motor_id:02X} outbound "
                                f"(0x{prev.motor_id:02X} at {LAUNCH_STAGGER_FRAC * 100:.0f}%)"
                            )

                    if last_slot in done:
                        phase = "return"
                        started = {valid_order[0]}
                        done = set()
                        leg_start_mono = {valid_order[0]: time.monotonic()}
                        print()
                        print(f"--- return stagger ({LAUNCH_STAGGER_FRAC * 100:.0f}% overlap) ---")
                        print(f"  0x{by_slot[valid_order[0]].motor_id:02X} returns to 0 "
                              f"(0x{by_slot[last_slot].motor_id:02X} at end)")
                        print()
                else:
                    for i in range(1, len(valid_order)):
                        cur = valid_order[i]
                        if cur in started:
                            continue
                        prev = by_slot[valid_order[i - 1]]
                        if launch_sweep_progress(prev, end_pos, zero_pos) >= LAUNCH_STAGGER_FRAC:
                            started.add(cur)
                            leg_start_mono[cur] = time.monotonic()
                            print(
                                f"  → 0x{by_slot[cur].motor_id:02X} return "
                                f"(0x{prev.motor_id:02X} at {LAUNCH_STAGGER_FRAC * 100:.0f}%)"
                            )

                now = time.monotonic()
                if phase == "forward":
                    move_from, move_to = start_pos, end_pos
                else:
                    move_from, move_to = end_pos, zero_pos

                for st in slot_states:
                    st.cmd_velocity = 0.0
                    slot = st.slot
                    if slot not in by_slot or slot not in valid_order:
                        st.kp = 0.0
                        continue
                    if slot in done:
                        st.cmd_position = move_to if phase == "forward" else zero_pos
                        st.kp = hold_kp
                    elif slot in started:
                        elapsed = now - leg_start_mono.get(slot, now)
                        rate = move_vel * min(1.0, elapsed / max(LAUNCH_RAMP_UP_S, 0.05))
                        plant_step_toward(st, move_to, rate, dt)
                        if plant_at_target(st, move_to):
                            done.add(slot)
                            tag = "outbound" if phase == "forward" else "return"
                            print(f"  {tag} done 0x{st.motor_id:02X}  fb={st.fb_position:+.3f} rad")
                        st.kp = (
                            hold_kp
                            if plant_at_target(st, move_to)
                            else by_slot[slot].max_kp
                        )
                    else:
                        hold = start_pos if phase == "forward" else end_pos
                        st.cmd_position = hold
                        st.kp = hold_kp

                cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
                plant_poll_feedback(reader, slot_states)

                line_n += 1
                if line_n % max(1, int(send_hz / 4)) == 0:
                    parts = [phase[:3]]
                    for slot in valid_order:
                        st = by_slot[slot]
                        if slot in done:
                            phase_s = "done"
                        elif slot in started:
                            if phase == "forward":
                                pct = launch_sweep_progress(st, start_pos, end_pos) * 100.0
                            else:
                                pct = launch_sweep_progress(st, end_pos, zero_pos) * 100.0
                            phase_s = f"{pct:.0f}%"
                        else:
                            phase_s = "wait"
                        parts.append(f"0x{st.motor_id:02X}:{phase_s}")
                    write_live_line(" ".join(parts))

                if phase == "return" and len(done) == len(valid_order):
                    break
                time.sleep(dt)

            print()

            for st in slot_states:
                st.cmd_position = zero_pos
                st.cmd_velocity = 0.0
                st.kp = 0.0
            for _ in range(max(4, int(send_hz * 0.15))):
                cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
                time.sleep(dt)

            print("Launch sequence complete.")
            for st in slot_states:
                if st.feedback_synced:
                    print(f"  0x{st.motor_id:02X}  fb={st.fb_position:+.4f} rad")

    except KeyboardInterrupt:
        print("\nLaunch sequence interrupted.")
    finally:
        for st in slot_states:
            st.cmd_velocity = 0.0
            st.kp = 0.0
        for _ in range(max(8, int(send_hz * 0.3))):
            cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
            time.sleep(dt)
        ser.write(build_plant_command(cmd_seq, {}))
        ser.flush()
        stop.set()


def plant_poll_feedback(reader: FrameReader, slot_states: List[PlantSlotTeleop]) -> None:
    frame = reader.pop()
    while frame is not None:
        for st in slot_states:
            fb = parse_actuator_feedback(frame, slot=st.slot)
            if fb is not None:
                st.fb_position = fb["position"]
                st.fb_velocity = fb["velocity"]
                if not st.feedback_synced:
                    st.cmd_position = fb["position"]
                    st.feedback_synced = True
        frame = reader.pop()


def plant_send_slots(
    ser: serial.Serial,
    cmd_seq: int,
    slot_states: List[PlantSlotTeleop],
    kd: float,
) -> int:
    slot_commands = {
        st.slot: (st.cmd_position, st.cmd_velocity, st.kp, kd, 0.0) for st in slot_states
    }
    ser.write(build_plant_command(cmd_seq, slot_commands))
    ser.flush()
    return (cmd_seq + 1) & 0xFFFFFFFF


def plant_home_to_zero(
    ser: serial.Serial,
    reader: FrameReader,
    slot_states: List[PlantSlotTeleop],
    cmd_seq: int,
    dt: float,
    send_hz: float,
    kd: float,
    home_kp: float,
    home_slew: float,
    pos_tol: float,
    vel_tol: float,
    timeout_s: float,
) -> Tuple[int, bool]:
    """Slew each slot's position setpoint to 0 before velocity teleop."""
    active = [st for st in slot_states if st.feedback_synced]
    if not active:
        print("Homing skipped: no feedback yet.")
        return cmd_seq, False

    for st in active:
        if abs(st.cmd_position - PLANT_HOME_TARGET) <= pos_tol:
            st.cmd_position = PLANT_HOME_TARGET
            st.kp = 0.0

    print(
        f"Homing to {PLANT_HOME_TARGET:+.2f} rad "
        f"(slew {home_slew:.2f} rad/s, kp={home_kp:.1f}) — hold still, q aborts"
    )
    deadline = time.monotonic() + timeout_s
    dwell_s = 0.0
    line_n = 0
    aborted = False
    cmd_eps = max(1e-4, pos_tol * 0.1)

    while time.monotonic() < deadline:
        if poll_key_nonblocking() == "q":
            aborted = True
            break

        slew_done = True
        for st in active:
            delta = PLANT_HOME_TARGET - st.cmd_position
            step = home_slew * dt
            if abs(delta) <= step:
                st.cmd_position = PLANT_HOME_TARGET
            else:
                st.cmd_position += math.copysign(step, delta)
                slew_done = False
            st.cmd_position = max(P_MIN, min(P_MAX, st.cmd_position))
            st.cmd_velocity = 0.0
            at_cmd = abs(st.cmd_position - PLANT_HOME_TARGET) <= cmd_eps
            st.kp = 0.0 if at_cmd else home_kp
            if not at_cmd:
                slew_done = False

        cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
        plant_poll_feedback(reader, slot_states)

        if slew_done:
            dwell_s += dt
        else:
            dwell_s = 0.0

        line_n += 1
        if line_n % max(1, int(send_hz / 3)) == 0:
            parts = []
            for st in active:
                parts.append(
                    f"{st.slot}:cmd={st.cmd_position:+.3f} fb={st.fb_position:+.3f} v={st.fb_velocity:+.2f}"
                )
            write_live_line("home  " + "  ".join(parts))

        if slew_done and dwell_s >= PLANT_HOME_DWELL_S:
            break
        time.sleep(dt)

    for st in slot_states:
        st.cmd_position = PLANT_HOME_TARGET
        st.cmd_velocity = 0.0
        st.kp = 0.0

    for _ in range(max(6, int(send_hz * 0.25))):
        cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
        plant_poll_feedback(reader, slot_states)
        time.sleep(dt)

    print()
    for st in active:
        print(
            f"  homed {st.label()}  fb={st.fb_position:+.4f} rad  "
            f"({'ok' if abs(st.fb_position) <= pos_tol else 'off-zero — check limits'})"
        )
    if aborted:
        print("Homing aborted (q).")
    elif dwell_s < PLANT_HOME_DWELL_S:
        print(f"Homing timed out after {timeout_s:.0f}s — continuing at best effort.")
    else:
        print("Homing complete — arrow keys enabled.")
    print()
    return cmd_seq, aborted


def plant_sync_feedback(
    reader: FrameReader,
    slot_states: List[PlantSlotTeleop],
    dwell_s: float = 2.5,
) -> None:
    """Read actuator_feedback slots until each has a position (kp stays 0)."""
    deadline = time.monotonic() + dwell_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            for st in slot_states:
                fb = parse_actuator_feedback(frame, slot=st.slot)
                if fb is not None:
                    st.cmd_position = fb["position"]
                    st.fb_position = fb["position"]
                    st.fb_velocity = fb["velocity"]
                    st.feedback_synced = True
            frame = reader.pop()
        if all(st.feedback_synced for st in slot_states):
            break
        time.sleep(0.02)
    for st in slot_states:
        if st.feedback_synced:
            print(f"  synced {st.label()}  pos={st.cmd_position:+.4f} rad")
        else:
            print(f"  warning: no feedback for {st.label()} — kp=0 until sync")


def run_plant_teleop(
    ser: serial.Serial,
    send_hz: float,
    kd: float,
    slots: List[int],
    arrow_vel: float,
    ramp_down_s: float,
    ramp_up_s: float,
    slot_kp: Tuple[float, ...] = PLANT_SLOT_KP,
    skip_home: bool = False,
    home_kp: float = PLANT_HOME_KP,
    home_slew: float = PLANT_HOME_SLEW_RAD_S,
    home_pos_tol: float = PLANT_HOME_POS_TOL,
    home_vel_tol: float = PLANT_HOME_VEL_TOL,
    home_timeout_s: float = PLANT_HOME_TIMEOUT_S,
) -> None:
    """500 Hz actuator_commands[] — all slots in one 562 B frame (no RS2 PDU)."""
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    slot_states = make_plant_slot_states(slots, slot_kp)
    active_bus = 0
    cmd_seq = 1
    dt = 1.0 / max(send_hz, 0.1)
    vel_stop = 0.08
    fb_line = 0

    print(f"Plant teleop on {ser.port} @ {send_hz:.0f} Hz  (all slots per frame, MCU 500 Hz CAN)")
    for st in slot_states:
        print(f"  {st.label()}")
    print(
        f"Motion: vel ±{arrow_vel:.1f} rad/s  ramp_up={ramp_up_s:.2f}s  ramp_down={ramp_down_s:.2f}s  kd={kd:.2f}"
    )
    print("Motors must be woken (probe/recovery once per branch). Auto-homes to 0.00 before teleop.")
    print("  Left / Right     velocity on active bus selection")
    print("  0                all buses (slots 0x70+0x74, 0x73, 0x75)")
    print("  1 / 2 / 3        CH1 only / CH2 only / CH3 only (live)")
    print("  r                re-sync cmd_pos from feedback + stop velocity")
    print("  q                quit")
    print(f"  Active: {plant_active_bus_readback(active_bus)}")
    print()
    print("Syncing feedback...")
    reader.drain()
    for _ in range(max(4, int(send_hz * 0.5))):
        ser.write(build_plant_command(cmd_seq, {}))
        ser.flush()
        cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
        time.sleep(dt)
        plant_poll_feedback(reader, slot_states)
    plant_sync_feedback(reader, slot_states, dwell_s=1.0)
    print()

    if not skip_home:
        cmd_seq, home_aborted = plant_home_to_zero(
            ser,
            reader,
            slot_states,
            cmd_seq,
            dt,
            send_hz,
            kd,
            home_kp,
            home_slew,
            home_pos_tol,
            home_vel_tol,
            home_timeout_s,
        )
        if home_aborted:
            stop.set()
            return

    print(f"  {plant_active_bus_readback(active_bus)}  (press 0–3 to change bus)")
    print()

    try:
        while True:
            quit_requested = False
            motion_dir = poll_arrow_direction()
            while True:
                key = poll_key_nonblocking(extra=("0", "1", "2", "3"))
                if key is None:
                    break
                if key == "q":
                    quit_requested = True
                    break
                if key == "0":
                    active_bus = 0
                    write_live_notice(plant_active_bus_readback(active_bus))
                elif key in ("1", "2", "3"):
                    pick_bus = int(key)
                    if any(st.fdcan_bus == pick_bus for st in slot_states):
                        active_bus = pick_bus
                        write_live_notice(plant_active_bus_readback(active_bus))
                    else:
                        write_live_notice(f"no slot on bus {pick_bus}")
                elif key == "r":
                    reader.drain()
                    for st in slot_states:
                        st.cmd_velocity = 0.0
                        st.kp = 0.0
                    plant_sync_feedback(reader, slot_states, dwell_s=0.5)
                    write_live_notice("re-synced all slots; velocity cleared")
                elif key in ("left", "l"):
                    motion_dir = -1
                elif key in ("right", "o"):
                    motion_dir = 1
            if quit_requested:
                break

            motion_targets = plant_teleop_targets(slot_states, active_bus)
            target_ids = {id(st) for st in motion_targets}
            for st in slot_states:
                if id(st) not in target_ids:
                    st.cmd_velocity *= math.exp(-dt / max(ramp_down_s, 0.05))
                    if abs(st.cmd_velocity) < vel_stop:
                        st.cmd_velocity = 0.0
                    st.kp = 0.0
                    continue
                if motion_dir != 0:
                    target_vel = motion_dir * abs(arrow_vel)
                    alpha = 1.0 - math.exp(-dt / max(ramp_up_s, 0.05))
                    st.cmd_velocity += (target_vel - st.cmd_velocity) * alpha
                else:
                    st.cmd_velocity *= math.exp(-dt / max(ramp_down_s, 0.05))
                if abs(st.cmd_velocity) < vel_stop:
                    st.cmd_velocity = 0.0
                if not st.feedback_synced:
                    st.kp = 0.0
                elif abs(st.cmd_velocity) < vel_stop:
                    st.kp = 0.0
                else:
                    st.kp = st.max_kp

            for st in slot_states:
                if abs(st.cmd_velocity) >= vel_stop:
                    st.cmd_position = max(
                        P_MIN, min(P_MAX, st.cmd_position + st.cmd_velocity * dt)
                    )

            cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
            plant_poll_feedback(reader, slot_states)

            fb_line += 1
            if fb_line % max(1, int(send_hz / 4)) == 0:
                parts = [plant_active_bus_readback(active_bus)]
                for st in slot_states:
                    mark = "*" if id(st) in target_ids else " "
                    parts.append(
                        f"{mark}{st.slot}:v={st.cmd_velocity:+.1f} kp={st.kp:.0f} "
                        f"cmd={st.cmd_position:+.3f} fb={st.fb_position:+.3f}"
                    )
                write_live_line("  ".join(parts))

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopping plant teleop...")
    finally:
        for st in slot_states:
            st.cmd_velocity = 0.0
            st.kp = 0.0
        for _ in range(max(8, int(send_hz * ramp_down_s))):
            cmd_seq = plant_send_slots(ser, cmd_seq, slot_states, kd)
            time.sleep(dt)
        ser.write(build_plant_command(cmd_seq, {}))
        ser.flush()
        stop.set()
        print("Done.")


def run_rs2_monitor(
    ser: serial.Serial,
    send_hz: float,
    kp: float,
    kd: float,
    motor_id: int,
    wake_steps: tuple[tuple[int, float], ...] = RS2_WAKE_STEPS,
    skip_wake: bool = False,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    cmd_seq = 1
    cmd_position = 0.0
    stats = SessionStats()
    last_probe: Optional[dict] = None

    cmd_seq = rs2_begin_session(ser, reader, motor_id, cmd_seq)
    if not skip_wake:
        cmd_seq = rs2_wake_motor(ser, reader, motor_id, cmd_seq, steps=wake_steps)
    else:
        print("RS2 wake skipped (--skip-wake); motor should already be mms=running.")
        print()

    synced_pos, sync_probe = rs2_sync_cmd_from_feedback(reader)
    if sync_probe is not None:
        cmd_position = synced_pos
        last_probe = sync_probe
        ext = decode_ext_id(sync_probe["ext_id"])
        mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
        fault_s = ",".join(ext.faults) if ext.faults else "none"
        print(
            f"  synced cmd_pos to shaft  fb={synced_pos:+.4f} rad  "
            f"mms={mms}  faults=[{fault_s}]"
        )
    else:
        print("  warning: no feedback to sync cmd_pos — starting at 0 rad (may fight shaft)")
    print()

    print(f"RS2 monitor on {ser.port} @ {send_hz:.0f} Hz  motor=0x{motor_id:02X}  (Ctrl+C to stop)")
    print("ack = low 8 bits of command seq (wraps 255→0). cmd trails ack by ~1 frame.")
    print("columns: fb#  ack  cmd  hit  ext_id  mms  pos  temp  faults  can_data")
    try:
        while True:
            cmd_seq = rs2_send_ctrl_frame(
                ser, motor_id, cmd_position, kp, kd, cmd_seq
            )

            probe = latest_probe_feedback(reader)
            if probe is not None:
                last_probe = probe
                stats.fb_count += 1
                print(format_rs2_line(stats.fb_count, cmd_seq, probe))

            rs2_ctrl_sleep(send_hz)
    except KeyboardInterrupt:
        print(f"\nStopped. rs2_feedback_lines={stats.fb_count}")
        if last_probe is not None:
            print(f"Last: {format_rs2_line(stats.fb_count, cmd_seq, last_probe)}")
    finally:
        rs2_end_session(ser, reader, motor_id, cmd_seq)
        stop.set()


def run_rs2_teleop(
    ser: serial.Serial,
    send_hz: float,
    kp: float,
    kd: float,
    motor_ids: List[int],
    pos_step: float,
    arrow_vel: float,
    ramp_down_s: float,
    ramp_up_s: float,
    wake_steps: tuple[tuple[int, float], ...] = RS2_WAKE_STEPS,
    skip_wake: bool = False,
    vbus_poll: bool = False,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    motor_ids = [int(m) & 0xFF for m in motor_ids]
    multi = len(motor_ids) > 1
    cmd_velocity = 0.0
    cmd_seq = 1
    round_robin = 0

    session_id = motor_ids[0]
    cmd_seq = rs2_begin_session(ser, reader, session_id, cmd_seq)
    if not skip_wake:
        for mid in motor_ids:
            cmd_seq = rs2_wake_motor(ser, reader, mid, cmd_seq, steps=wake_steps)
    else:
        print("RS2 wake skipped (--skip-wake); motors should already be mms=running.")
        print()

    reader.drain()
    motors: List[MotorTeleopState] = []
    for mid in motor_ids:
        motors.append(rs2_init_motor_teleop_state(reader, mid, kp))
    if not any(m.feedback_synced for m in motors):
        print(
            "  warning: no position feedback on any motor — teleop uses kp=0 until sync. "
            "Run --calibrate first or use --nudge --kp 10 to test."
        )
    print()

    id_label = ",".join(f"0x{m:02X}" for m in motor_ids)
    per_motor_hz = send_hz / len(motor_ids)
    print(f"RS2 arrow teleop  {ser.port}  motor(s) {id_label}  {send_hz:.0f} Hz host loop")
    if multi:
        print(f"  Alternating PROBE_CTRL_FAST — ~{per_motor_hz:.1f} Hz per motor per cycle")
    print("Same CAN path as --nudge (PROBE_CTRL_FAST).")
    print(
        f"  Left / Right     hold to move  vel ±{arrow_vel:.1f} rad/s (MOTOR_CTRL)  "
        f"coast {ramp_down_s:.2f}s on release"
    )
    print(f"  r                re-sync all motors to shaft + stop velocity")
    print(f"  q                quit")
    print()
    print("Focus this terminal window, then use arrow keys.")
    print()

    try:
        dt = 1.0 / max(send_hz, 0.1)
        vel_stop = 0.12
        vbus = VbusPollState()
        while True:
            quit_requested = False
            motion_dir = poll_arrow_direction()
            while True:
                key = poll_key_nonblocking()
                if key is None:
                    break
                if key == "q":
                    quit_requested = True
                    break
                if key == "r":
                    reader.drain()
                    for motor in motors:
                        sync_pos, sync_probe = rs2_sync_cmd_for_motor(reader, motor.motor_id)
                        if sync_probe is not None:
                            decoded = decode_rs02_feedback_bytes(sync_probe["can_data"])
                            if not decoded["payload_empty"]:
                                motor.cmd_position = sync_pos
                                motor.feedback_synced = True
                                motor.last_probe = sync_probe
                                motor.sync_kp(kp)
                    cmd_velocity = 0.0
                    motion_dir = 0
                elif key in ("left", "l"):
                    motion_dir = -1
                elif key in ("right", "o"):
                    motion_dir = 1
            if quit_requested:
                break

            if motion_dir != 0:
                target_vel = motion_dir * abs(arrow_vel)
                alpha = 1.0 - math.exp(-dt / max(ramp_up_s, 0.01))
                cmd_velocity += (target_vel - cmd_velocity) * alpha
            else:
                cmd_velocity *= math.exp(-dt / max(ramp_down_s, 0.01))
                if abs(cmd_velocity) < vel_stop:
                    cmd_velocity = 0.0

            cmd_velocity = max(V_MIN, min(V_MAX, cmd_velocity))
            for motor in motors:
                if abs(cmd_velocity) >= vel_stop:
                    motor.cmd_position = max(
                        P_MIN, min(P_MAX, motor.cmd_position + cmd_velocity * dt)
                    )
                elif motion_dir != 0:
                    motor.cmd_position = max(
                        P_MIN, min(P_MAX, motor.cmd_position + motion_dir * pos_step)
                    )

            active = motors[round_robin % len(motors)]
            round_robin += 1

            cmd_seq = maybe_send_vbus_pararead(
                ser, active.motor_id, cmd_seq, vbus, vbus_poll and motion_dir == 0
            )
            poll_rs2_reader_all(reader, motors, vbus)
            for motor in motors:
                motor.sync_kp(kp)

            cmd_seq = rs2_send_ctrl_frame(
                ser,
                active.motor_id,
                active.cmd_position,
                active.active_kp,
                kd,
                cmd_seq,
                cmd_velocity,
            )

            coast = motion_dir == 0 and abs(cmd_velocity) >= vel_stop
            mode_s = "coast" if coast else ("drive" if motion_dir != 0 else "hold")
            parts = [
                f"[live] {mode_s:5s} tx=0x{active.motor_id:02X} cmd_vel {cmd_velocity:+.2f} seq {cmd_seq & 0xFF:3d} |"
            ]
            for motor in motors:
                tag = ">" if motor.motor_id == active.motor_id else " "
                if motor.last_probe is not None:
                    decoded = decode_rs02_feedback_bytes(motor.last_probe["can_data"])
                    ext = decode_ext_id(motor.last_probe["ext_id"])
                    mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                    if decoded["payload_empty"]:
                        parts.append(f"{tag}0x{motor.motor_id:02X} mms={mms} fb=n/a")
                    else:
                        parts.append(
                            f"{tag}0x{motor.motor_id:02X} fb={decoded['position']:+.3f} kp={motor.active_kp:.0f}"
                        )
                else:
                    parts.append(f"{tag}0x{motor.motor_id:02X} waiting")
            write_live_line("  ".join(parts))

            rs2_ctrl_sleep(send_hz)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        print("Stopping motor(s) (kp=0 burst → reset)...")
        for motor in motors:
            cmd_seq = rs2_stop_motor(ser, reader, motor.motor_id, cmd_seq, motor.cmd_position)
        rs2_end_session(ser, reader, session_id, cmd_seq)
        stop.set()


def run_monitor(
    ser: serial.Serial,
    send_hz: float,
    kp: float,
    kd: float,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    cmd_seq = 1
    cmd_position = 0.0
    send_period = 1.0 / max(send_hz, 0.1)
    last_send = 0.0
    stats = SessionStats()

    print(f"Monitor on {ser.port} @ {send_hz:.0f} Hz cmd  (Ctrl+C to stop)")
    print("columns: fb#  ack_seq  tick  cmd_seq  pos  tempC  fault")
    try:
        while True:
            now = time.monotonic()
            if now - last_send >= send_period:
                ser.write(build_command_image(cmd_position, cmd_seq, kp, kd))
                ser.flush()
                last_send = now
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF

            frame = reader.pop()
            while frame is not None:
                fb = parse_feedback_image(frame)
                if fb is not None:
                    stats.fb_count += 1
                    if stats.last_tick is not None and fb["tick"] == stats.last_tick:
                        stats.tick_stalls += 1
                    stats.last_tick = fb["tick"]
                    print(
                        f"{stats.fb_count:5d}  "
                        f"ack={fb['last_cmd_seq']:3d}  "
                        f"tick={fb['tick']:4d}  "
                        f"cmd={cmd_seq & 0xFF:3d}  "
                        f"pos={fb['position']:+.4f}  "
                        f"T={fb['temperature']:5.1f}  "
                        f"fault=0x{fb['fault']:08X}"
                    )
                frame = reader.pop()

            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\nStopped. feedback_frames={stats.fb_count} tick_unchanged_frames={stats.tick_stalls}")
    finally:
        stop.set()


def run_teleop(
    ser: serial.Serial,
    send_hz: float,
    kp: float,
    kd: float,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    cmd_position = 0.0
    cmd_seq = 1
    telemetry: Optional[dict] = None
    enable_period = 1.0 / max(send_hz, 0.1)
    last_send = 0.0
    last_fb_time: Optional[float] = None
    status = "enabling"
    user_took_control = False

    def send_command() -> None:
        nonlocal last_send
        ser.write(build_command_image(cmd_position, cmd_seq, kp, kd))
        ser.flush()
        last_send = time.monotonic()

    print(f"Teleop on {ser.port}  enable_rate={send_hz:.0f}Hz  kp={kp}  kd={kd}")
    print("Sending pos=0 at {:.0f} Hz to enable motor — press Left/Right to take control".format(send_hz))
    print("Keys: Left/Right (or l / o on Linux line mode), r=zero, q=quit")

    try:
        while True:
            now = time.monotonic()

            # Phase 1: blast pos=0 at send_hz until user presses an arrow key
            if not user_took_control and now - last_send >= enable_period:
                send_command()

            frame = reader.pop()
            while frame is not None:
                fb = parse_feedback_image(frame)
                if fb is not None:
                    telemetry = fb
                    last_fb_time = time.monotonic()
                    if status == "enabling" and user_took_control:
                        status = "ok"
                    elif status == "enabling":
                        status = "enabled"
                frame = reader.pop()

            if last_fb_time is not None and time.monotonic() - last_fb_time > 1.0:
                status = "timeout"

            t = telemetry
            line = (
                f"\r[{status}] cmd seq {cmd_seq & 0xFF:3d} pos {cmd_position:+.4f} | "
            )
            if t:
                line += (
                    f"ack {t['last_cmd_seq']:3d} tick {t['tick']:4d} | "
                    f"pos {t['position']:+.4f} vel {t['velocity']:+.4f} "
                    f"T {t['temperature']:.1f}C fault 0x{t['fault']:08X}   "
                )
            else:
                line += "waiting for feedback...                    "
            sys.stdout.write(line)
            sys.stdout.flush()

            key = poll_key_nonblocking()
            if key == "q":
                break
            if key == "r":
                cmd_position = 0.0
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
                user_took_control = True
                status = "ok"
                send_command()
            elif key in ("left", "l"):
                cmd_position = max(P_MIN, cmd_position - POS_STEP)
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
                user_took_control = True
                status = "ok"
                send_command()
            elif key in ("right", "o"):
                cmd_position = min(P_MAX, cmd_position + POS_STEP)
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
                user_took_control = True
                status = "ok"
                send_command()

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        stop.set()


def main() -> None:
    ap = argparse.ArgumentParser(description="Laptop USB CDC host test (562 B images)")
    ap.add_argument("--list-ports", action="store_true", help="List COM / ttyACM ports and exit")
    ap.add_argument("--port", default=None, help="Serial port (e.g. COM5 or /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=USB_BAUD,
                    help="Baud for pyserial open (USB CDC ignores; default 115200)")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="Command send rate (default 40)")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP,
                    help="RS02 MOTOR_CTRL kp (default 50)")
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--monitor", action="store_true",
                    help="Print one line per feedback frame (good first link check)")
    ap.add_argument("--legacy-actuator", action="store_true",
                    help="Actuator slot only — no RS2 PDU CAN path (bench link usually dead)")
    ap.add_argument("--motor-id", type=lambda x: int(x, 0), default=DEFAULT_MOTOR_ID,
                    help="RobStride motor CAN ID when --motor-ids not set (default 0x70)")
    ap.add_argument("--motor-ids", default=None,
                    help="Comma-separated IDs for alternating dual/multi teleop (e.g. 0x70,0x74)")
    ap.add_argument("--read-params", action="store_true",
                    help="Wake motor then read RS02 registers via comm 0x11 pararead")
    ap.add_argument("--pararead", type=lambda x: int(x, 0), action="append", default=[],
                    help="Extra param index for --read-params (hex ok, e.g. 0x7016)")
    ap.add_argument("--nudge", action="store_true",
                    help="Small position steps to test feedback without hand-rotating the shaft")
    ap.add_argument("--calibrate", action="store_true",
                    help="Encoder cal (comm 0x05), zero (0x06), save (0x16) over Deft RS2 path")
    ap.add_argument("--bus", type=int, default=1, choices=[1, 2, 3],
                    help="Plant CAN branch for RS2 --calibrate (1=CH1, 2=CH2 PA8/PA15, 3=CH3 PB12/13)")
    ap.add_argument("--plant-teleop", action="store_true",
                    help="All 4 plant slots in one frame (CH1 0x70/0x74, CH2 0x73, CH3 0x75); gentle ramp defaults")
    ap.add_argument("--servo-teleop", action="store_true",
                    help="Dynamixel neck servos: L/R bottom, U/D top (see scripts/dynamixel_teleop.py)")
    ap.add_argument("--plant-slots", default="0,1,2,3",
                    help="Slot indices for --plant-teleop (default 0,1,2,3)")
    ap.add_argument("--plant-kp", type=float, default=None,
                    help="Override per-slot max kp scale for --plant-teleop (default RS02=12 RS01=8)")
    ap.add_argument("--plant-arrow-vel", type=float, default=PLANT_DEFAULT_ARROW_VEL,
                    help=f"Plant teleop velocity (rad/s, default {PLANT_DEFAULT_ARROW_VEL})")
    ap.add_argument("--plant-ramp-up", type=float, default=PLANT_DEFAULT_RAMP_UP_S,
                    help=f"Plant teleop ramp-up tau (s, default {PLANT_DEFAULT_RAMP_UP_S})")
    ap.add_argument("--plant-ramp-down", type=float, default=PLANT_DEFAULT_RAMP_DOWN_S,
                    help=f"Plant teleop coast-down tau (s, default {PLANT_DEFAULT_RAMP_DOWN_S})")
    ap.add_argument("--plant-kd", type=float, default=PLANT_DEFAULT_KD,
                    help=f"Plant teleop kd (default {PLANT_DEFAULT_KD})")
    ap.add_argument("--plant-skip-home", action="store_true",
                    help="Skip automatic slow homing to 0.00 rad at plant-teleop start")
    ap.add_argument("--plant-home-slew", type=float, default=PLANT_HOME_SLEW_RAD_S,
                    help=f"Homing setpoint slew rate rad/s (default {PLANT_HOME_SLEW_RAD_S})")
    ap.add_argument("--plant-home-kp", type=float, default=PLANT_HOME_KP,
                    help=f"Homing position kp (default {PLANT_HOME_KP})")
    ap.add_argument("--launch-seq", action="store_true",
                    help="Demo: prep to -5 rad, sweep 0x70→0x74→0x73→0x75 to +10, then all to 0")
    ap.add_argument("--launch-ccw", action="store_true",
                    help="Invert launch sweep (start +5, end -10) if default direction is wrong")
    ap.add_argument("--launch-vel", type=float, default=LAUNCH_MOVE_VEL,
                    help=f"Launch sequence slew rate rad/s (default {LAUNCH_MOVE_VEL})")
    ap.add_argument("--launch-start", type=float, default=None,
                    help="Launch start position rad (default -5 CW, +5 CCW)")
    ap.add_argument("--launch-end", type=float, default=None,
                    help="Launch end position rad (default +10 CW, -10 CCW)")
    ap.add_argument("--cal-timeout", type=float, default=DEFAULT_CAL_TIMEOUT_S,
                    help="Max seconds for comm 0x05 encoder cal on MCU (default 28)")
    ap.add_argument("--step", type=float, default=POS_STEP,
                    help="Fallback position nudge per frame at low vel (rad, default 0.02)")
    ap.add_argument("--arrow-vel", type=float, default=DEFAULT_ARROW_VEL,
                    help="RS02 MOTOR_CTRL velocity target while arrow held (rad/s, default 22, max 44)")
    ap.add_argument("--ramp-down", type=float, default=DEFAULT_RAMP_DOWN_S,
                    help="Velocity coast-down time constant after arrow release (s, default 0.55)")
    ap.add_argument("--ramp-up", type=float, default=DEFAULT_RAMP_UP_S,
                    help="Velocity ramp-up time constant when arrow pressed (s, default 0.06)")
    ap.add_argument("--skip-wake", action="store_true",
                    help="Skip RS2 wake (use after rs02_can_scan.py --recovery left mms=running)")
    ap.add_argument("--proactive-on", action="store_true",
                    help="Enable proactive 0x18 at end of wake (default off; can trip under_voltage)")
    ap.add_argument("--vbus", action="store_true",
                    help="Poll VBUS via pararead while holding (off by default; adds CAN traffic)")
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    port = args.port if args.port is not None else auto_pick_port()
    use_rs2 = not args.legacy_actuator
    if args.motor_ids:
        teleop_motor_ids = parse_id_list(args.motor_ids)
        if not teleop_motor_ids:
            print("--motor-ids: need at least one ID", file=sys.stderr)
            sys.exit(2)
    else:
        teleop_motor_ids = [args.motor_id & 0xFF]
    if args.read_params:
        mode = "read-params"
    elif args.plant_teleop:
        mode = "plant-teleop"
    elif args.servo_teleop:
        mode = "servo-teleop"
    elif args.launch_seq:
        mode = "launch-seq"
    elif args.calibrate:
        mode = "calibrate"
    elif args.nudge:
        mode = "nudge"
    elif args.monitor:
        mode = "monitor"
    elif use_rs2:
        mode = "arrow teleop (default)"
    else:
        mode = "legacy actuator"
    if use_rs2 and args.calibrate:
        print(f"Opening {port} (USB CDC)  mode={mode}  bus=FDCAN{args.bus}  cal_timeout={args.cal_timeout:.0f}s")
    elif args.plant_teleop or args.launch_seq or args.servo_teleop:
        print(f"Opening {port} (USB CDC) @ {args.hz:.0f} Hz  mode={mode}")
    elif use_rs2 and not args.read_params and not args.nudge:
        print(f"Opening {port} (USB CDC) @ {args.hz:.0f} Hz RS2 ctrl  mode={mode}")
    elif use_rs2 and args.nudge:
        print(f"Opening {port} (USB CDC) @ {RS2_NUDGE_HZ:.0f} Hz RS2 ctrl  mode={mode}")
    else:
        print(f"Opening {port} (USB CDC) @ {args.baud}  mode={mode}")
    if use_rs2:
        if len(teleop_motor_ids) > 1:
            print(f"Motor IDs {', '.join(f'0x{m:02X}' for m in teleop_motor_ids)} (alternating teleop)")
        else:
            print(f"Motor ID 0x{teleop_motor_ids[0]:02X}")
    print("Firmware: set HOST_TRANSPORT_UART to 0 in App/Inc/host/host_transport.h")

    if len(teleop_motor_ids) > 1 and (args.read_params or args.calibrate or args.nudge or args.monitor):
        print("--motor-ids is only supported for default arrow teleop (not --calibrate/--monitor/etc.)",
              file=sys.stderr)
        sys.exit(2)
    if args.plant_teleop and (args.calibrate or args.read_params or args.nudge or args.monitor):
        print("--plant-teleop cannot combine with --calibrate / --monitor / --nudge / --read-params",
              file=sys.stderr)
        sys.exit(2)
    if args.servo_teleop and (args.calibrate or args.read_params or args.nudge or args.monitor
                              or args.plant_teleop or args.launch_seq):
        print("--servo-teleop cannot combine with other teleop/monitor modes", file=sys.stderr)
        sys.exit(2)
    if args.launch_seq and (args.calibrate or args.read_params or args.nudge or args.monitor
                            or args.plant_teleop):
        print("--launch-seq cannot combine with --plant-teleop / --calibrate / --monitor / "
              "--nudge / --read-params", file=sys.stderr)
        sys.exit(2)

    plant_slots = parse_id_list(args.plant_slots)
    if args.plant_teleop and not plant_slots:
        print("--plant-slots: need at least one slot index", file=sys.stderr)
        sys.exit(2)

    with serial.Serial(port=port, baudrate=args.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        wake_steps = RS2_WAKE_STEPS_PROACTIVE if args.proactive_on else RS2_WAKE_STEPS
        if args.launch_seq:
            if args.launch_ccw:
                start_pos = args.launch_start if args.launch_start is not None else -LAUNCH_START_CW
                end_pos = args.launch_end if args.launch_end is not None else -LAUNCH_END_CW
            else:
                start_pos = args.launch_start if args.launch_start is not None else LAUNCH_START_CW
                end_pos = args.launch_end if args.launch_end is not None else LAUNCH_END_CW
            slot_kp = PLANT_SLOT_KP
            if args.plant_kp is not None:
                slot_kp = tuple(args.plant_kp for _ in range(4))
            run_plant_launch_seq(
                ser,
                args.hz,
                args.plant_kd,
                move_vel=args.launch_vel,
                start_pos=start_pos,
                end_pos=end_pos,
                slot_kp=slot_kp,
            )
        elif args.plant_teleop:
            slot_kp = PLANT_SLOT_KP
            if args.plant_kp is not None:
                slot_kp = tuple(args.plant_kp for _ in range(4))
            run_plant_teleop(
                ser,
                args.hz,
                args.plant_kd,
                plant_slots,
                args.plant_arrow_vel,
                args.plant_ramp_down,
                args.plant_ramp_up,
                slot_kp=slot_kp,
                skip_home=args.plant_skip_home,
                home_kp=args.plant_home_kp,
                home_slew=args.plant_home_slew,
            )
        elif args.servo_teleop:
            from dynamixel_teleop import run_servo_teleop

            run_servo_teleop(
                ser,
                send_hz=args.hz,
                arrow_vel=args.plant_arrow_vel,
                ramp_up_s=args.plant_ramp_up,
                ramp_down_s=args.plant_ramp_down,
            )
        elif args.read_params:
            run_read_params(ser, args.motor_id, args.pararead)
        elif args.calibrate:
            run_calibrate(
                ser, args.motor_id, args.cal_timeout, args.kp, args.kd,
                bus=normalize_can_bus(args.bus),
            )
        elif args.nudge:
            run_nudge_test(ser, args.motor_id, args.kp, args.kd)
        elif use_rs2:
            if args.monitor:
                run_rs2_monitor(
                    ser, args.hz, args.kp, args.kd, args.motor_id,
                    wake_steps=wake_steps, skip_wake=args.skip_wake,
                )
            else:
                run_rs2_teleop(
                    ser,
                    args.hz,
                    args.kp,
                    args.kd,
                    teleop_motor_ids,
                    args.step,
                    args.arrow_vel,
                    args.ramp_down,
                    args.ramp_up,
                    wake_steps=wake_steps,
                    skip_wake=args.skip_wake,
                    vbus_poll=args.vbus,
                )
        elif args.monitor:
            run_monitor(ser, args.hz, args.kp, args.kd)
        else:
            run_teleop(ser, args.hz, args.kp, args.kd)


if __name__ == "__main__":
    main()
