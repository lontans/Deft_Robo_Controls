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
  r              command zero
  q              quit

Other modes: --monitor  --nudge  --read-params  --calibrate  --legacy-actuator
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
from typing import Deque, Optional

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
    RS2_COMM_NAMES,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_probe_command,
    build_rs2_teleop_command,
    decode_ext_id,
    format_probe_line,
    parse_probe_pdu,
    rs2_comm_label,
    pararead_is_hit,
    pararead_index_echo,
    send_diag,
)

PARAM_MECH_ANGLE = 0x7016
PARAM_MECH_POS = 0x7019
PARAM_RUN_MODE = 0x7005
PARAM_SPEED = 0x700A
PARAM_BUS_VOLT = 0x701C
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
USB_BAUD = 115200
DEFAULT_MOTOR_ID = 0x70

STM32_VID = 0x0483

RS2_WAKE_STEPS = (
    (PROBE_RESET, 0.45),
    (PROBE_ENABLE_ONLY, 0.55),
    (PROBE_FULL, 0.35),
    (PROBE_PROACTIVE, 0.40),
)

# Proactive (comm 0x18) confuses pararead diagnostics — omit for --read-params.
RS2_WAKE_STEPS_PARAMS = (
    (PROBE_RESET, 0.45),
    (PROBE_ENABLE_ONLY, 0.55),
    (PROBE_FULL, 0.35),
)

# Nudge stays at 15 Hz (proven on bench). Teleop uses --hz (default 30) for smoother arrows.
RS2_NUDGE_HZ = 15.0


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
    except (serial.SerialException, OSError):
        pass


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


def poll_key_nonblocking() -> Optional[str]:
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
        if c in ("q", "r"):
            return c
        return None

    import select

    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip().lower()
        if line in ("q", "quit", "r", "left", "right", "l", "o"):
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


def rs2_read_param(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    param_index: int,
    seq: int,
) -> tuple[int, Optional[dict], Optional[dict]]:
    """Return (next_seq, pararead_reply_or_None, last_sniff_or_None)."""
    reader.drain()
    buf = build_rs2_probe_command(motor_id, PROBE_PARAREAD, seq, param_index)
    ser.write(buf)
    ser.flush()
    resp: Optional[dict] = None
    sniff: Optional[dict] = None
    deadline = time.monotonic() + 0.8
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
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    seq = 1
    motor_id &= 0xFF
    cal_timeout_s = max(5.0, cal_timeout_s)

    print(f"RS2 encoder cal on {ser.port}  motor=0x{motor_id:02X}")
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
        seq = rs2_wake_motor(ser, reader, motor_id, seq, steps=RS2_WAKE_STEPS_PARAMS)

        print("--- enter running (ctrl burst before cal) ---")
        seq, running = rs2_prime_running(ser, reader, motor_id, seq, kp, kd)
        if running:
            print("  mms=running — motor enabled for cal")
        else:
            print("  warning: never saw mms=running (cal may still work if shaft is free)")
        print()

        print(f"--- motor_cali comm 0x05 (sequential chunks, {CALI_CHUNK_S:.0f}s MCU / chunk) ---")
        cal_attempts = max(1, int(math.ceil(cal_timeout_s / CALI_CHUNK_S)))
        per_probe_timeout = CALI_CHUNK_S + 10.0
        saw_cali_mode = False
        saw_cal_rx = False
        for attempt in range(cal_attempts):
            label = f"motor_cali[{attempt + 1}/{cal_attempts}]" if cal_attempts > 1 else "motor_cali"
            resp = send_diag(ser, reader, motor_id, PROBE_CALI, seq, per_probe_timeout)
            seq += 1
            if resp is None:
                print(f"  ----  {label:<32s}  (no USB feedback)")
                print("  stopped cal retries — MCU still busy (prior probe?) or reflash kinds 16-18")
                break
            if resp.get("found"):
                saw_cal_rx = True
                ext = decode_ext_id(resp["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                if ext.mode_status == 1:
                    saw_cali_mode = True
                print(format_probe_line(label, motor_id, resp))
                print(f"         mms={mms}  can_data={resp['can_data'].hex()}  rx={resp.get('raw_frames', 0)}")
                if saw_cali_mode:
                    print("         saw mms=cali — encoder cal in progress")
                    break
            else:
                print(format_probe_line(label, motor_id, resp))
            time.sleep(0.15)
        if not saw_cal_rx:
            print("  no ext-CAN RX during cal chunks — check 48 V and firmware kinds 16-18")
        elif not saw_cali_mode:
            print("  motor never reported mms=cali — 0x05 may not have started encoder cal")
        print()

        print("--- motor_zero comm 0x06 ---")
        resp = send_diag(ser, reader, motor_id, PROBE_ZERO, seq, 4.0)
        seq += 1
        print(format_probe_line("motor_zero", motor_id, resp))
        print()

        print("--- data_save comm 0x16 ---")
        resp = send_diag(ser, reader, motor_id, PROBE_DATA_SAVE, seq, 5.0)
        seq += 1
        print(format_probe_line("data_save", motor_id, resp))
        print()

        print("--- pararead verify ---")
        got_pararead = False
        for label, index in verify_reads:
            seq, resp, sniff = rs2_read_param(ser, reader, motor_id, index, seq)
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
    targets = (
        ("hold zero", 0.0, 1.0),
        ("step +0.05 rad", 0.05, 1.0),
        ("step +0.10 rad", 0.10, 1.0),
        ("back to zero", 0.0, 1.0),
    )
    best_p_raw = 0
    any_nonzero_data = False

    print(f"RS2 nudge test on {ser.port}  motor=0x{motor_id:02X}  kp={kp}  kd={kd}")
    print("Commands small setpoints — output must be free to move slightly.")
    print("Watch for non-zero can_data / p_raw (no hand rotation needed).")
    print()

    try:
        seq = rs2_begin_session(ser, reader, motor_id, seq)
        seq = rs2_wake_motor(ser, reader, motor_id, seq)

        for label, position, hold_s in targets:
            print(f"--- {label} (pos={position:+.3f} rad, {hold_s:.0f}s) ---")
            deadline = time.monotonic() + hold_s
            last_hex = "0000000000000000"
            while time.monotonic() < deadline:
                seq = rs2_send_ctrl_frame(ser, motor_id, position, kp, kd, seq)
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
                    print(f"  mms={mms}  {format_payload_line(decoded)}")
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
) -> tuple[int, bool]:
    """Ctrl burst until ext-CAN mode_status=running (needed before encoder cal on some RS02)."""
    for _ in range(bursts):
        seq = rs2_send_ctrl_frame(ser, motor_id, 0.0, kp, kd, seq)
        probe = latest_probe_feedback(reader)
        if probe is not None:
            ext = decode_ext_id(probe["ext_id"])
            if ext.mode_status == 2:
                return seq, True
        rs2_ctrl_sleep(hz)
    return seq, False


def rs2_end_session(ser: serial.Serial, reader: FrameReader, motor_id: int, seq: int) -> None:
    send_diag(ser, reader, motor_id, SESSION_END, seq, 0.35)


def rs2_wake_motor(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    steps: tuple[tuple[int, float], ...] = RS2_WAKE_STEPS,
) -> int:
    print(f"RS2 wake motor 0x{motor_id:02X} (reset → enable-only → full_init)...")
    for kind, timeout in steps:
        resp = send_diag(ser, reader, motor_id, kind, seq, timeout)
        seq += 1
        if resp is not None and resp.get("found"):
            ext = decode_ext_id(resp["ext_id"])
            mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
            print(
                f"  kind={kind:2d}  HIT  ext=0x{resp['ext_id']:08X}  "
                f"mms={mms}  T={resp['temperature']:.1f}C  pos={resp['position']:+.4f}"
            )
        elif resp is not None:
            kind_name = {11: "reset", 12: "enable-only", 0: "full_init", 15: "proactive_on"}.get(kind, str(kind))
            print(f"  kind={kind:2d} ({kind_name})  TX ok, no parsed RX (PC7 should blink)")
        else:
            print(f"  kind={kind:2d}  (no USB feedback)")
        time.sleep(0.12)
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


def run_rs2_monitor(
    ser: serial.Serial,
    send_hz: float,
    kp: float,
    kd: float,
    motor_id: int,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    cmd_seq = 1
    cmd_position = 0.0
    stats = SessionStats()
    last_probe: Optional[dict] = None

    cmd_seq = rs2_begin_session(ser, reader, motor_id, cmd_seq)
    cmd_seq = rs2_wake_motor(ser, reader, motor_id, cmd_seq)

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
    motor_id: int,
    pos_step: float,
    arrow_vel: float,
    ramp_down_s: float,
    ramp_up_s: float,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    cmd_position = 0.0
    cmd_velocity = 0.0
    cmd_seq = 1
    last_probe: Optional[dict] = None

    cmd_seq = rs2_begin_session(ser, reader, motor_id, cmd_seq)
    cmd_seq = rs2_wake_motor(ser, reader, motor_id, cmd_seq)

    print()
    print(f"RS2 arrow teleop  {ser.port}  motor=0x{motor_id:02X}  {send_hz:.0f} Hz  kp={kp}  kd={kd}")
    print("Same CAN path as --nudge (PROBE_CTRL_FAST). Feedback not required.")
    print(
        f"  Left / Right     hold to move  vel ±{arrow_vel:.1f} rad/s (MOTOR_CTRL)  "
        f"coast {ramp_down_s:.2f}s on release"
    )
    print(f"  r                command zero (immediate stop)")
    print(f"  q                quit")
    print()
    print("Focus this terminal window, then use arrow keys.")
    print()

    try:
        dt = 1.0 / max(send_hz, 0.1)
        vel_stop = 0.12
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
                    cmd_position = 0.0
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
            if abs(cmd_velocity) >= vel_stop:
                cmd_position = max(P_MIN, min(P_MAX, cmd_position + cmd_velocity * dt))
            elif motion_dir != 0:
                cmd_position = max(P_MIN, min(P_MAX, cmd_position + motion_dir * pos_step))

            cmd_seq = rs2_send_ctrl_frame(
                ser, motor_id, cmd_position, kp, kd, cmd_seq, cmd_velocity
            )

            probe = latest_probe_feedback(reader)
            if probe is not None:
                last_probe = probe

            coast = motion_dir == 0 and abs(cmd_velocity) >= vel_stop
            mode_s = "coast" if coast else ("drive" if motion_dir != 0 else "hold")
            line = (
                f"\r[live] {mode_s:5s}  cmd_pos {cmd_position:+.4f} rad  "
                f"cmd_vel {cmd_velocity:+.2f}  seq {cmd_seq & 0xFF:3d}  | "
            )
            if last_probe is not None:
                decoded = decode_rs02_feedback_bytes(last_probe["can_data"])
                ext = decode_ext_id(last_probe["ext_id"])
                mms = ("rest", "cali", "running")[ext.mode_status] if ext.mode_status < 3 else "?"
                if decoded["payload_empty"]:
                    line += f"mms={mms}  fb=n/a   "
                else:
                    line += (
                        f"mms={mms}  fb={decoded['position']:+.4f} rad  "
                        f"T={last_probe['temperature']:.1f}C   "
                    )
            else:
                line += "link=waiting...                    "
            sys.stdout.write(line)
            sys.stdout.flush()

            rs2_ctrl_sleep(send_hz)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        rs2_end_session(ser, reader, motor_id, cmd_seq)
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
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--monitor", action="store_true",
                    help="Print one line per feedback frame (good first link check)")
    ap.add_argument("--legacy-actuator", action="store_true",
                    help="Actuator slot only — no RS2 PDU CAN path (bench link usually dead)")
    ap.add_argument("--motor-id", type=lambda x: int(x, 0), default=DEFAULT_MOTOR_ID,
                    help="RobStride motor CAN ID (default 0x70)")
    ap.add_argument("--read-params", action="store_true",
                    help="Wake motor then read RS02 registers via comm 0x11 pararead")
    ap.add_argument("--pararead", type=lambda x: int(x, 0), action="append", default=[],
                    help="Extra param index for --read-params (hex ok, e.g. 0x7016)")
    ap.add_argument("--nudge", action="store_true",
                    help="Small position steps to test feedback without hand-rotating the shaft")
    ap.add_argument("--calibrate", action="store_true",
                    help="Encoder cal (comm 0x05), zero (0x06), save (0x16) over Deft RS2 path")
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
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    port = args.port if args.port is not None else auto_pick_port()
    use_rs2 = not args.legacy_actuator
    if args.read_params:
        mode = "read-params"
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
        print(f"Opening {port} (USB CDC)  mode={mode}  cal_timeout={args.cal_timeout:.0f}s")
    elif use_rs2 and not args.read_params and not args.nudge:
        print(f"Opening {port} (USB CDC) @ {args.hz:.0f} Hz RS2 ctrl  mode={mode}")
    elif use_rs2 and args.nudge:
        print(f"Opening {port} (USB CDC) @ {RS2_NUDGE_HZ:.0f} Hz RS2 ctrl  mode={mode}")
    else:
        print(f"Opening {port} (USB CDC) @ {args.baud}  mode={mode}")
    if use_rs2:
        print(f"Motor ID 0x{args.motor_id:02X}")
    print("Firmware: set HOST_TRANSPORT_UART to 0 in App/Inc/host/host_transport.h")

    with serial.Serial(port=port, baudrate=args.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        if args.read_params:
            run_read_params(ser, args.motor_id, args.pararead)
        elif args.calibrate:
            run_calibrate(ser, args.motor_id, args.cal_timeout, args.kp, args.kd)
        elif args.nudge:
            run_nudge_test(ser, args.motor_id, args.kp, args.kd)
        elif use_rs2:
            if args.monitor:
                run_rs2_monitor(ser, args.hz, args.kp, args.kd, args.motor_id)
            else:
                run_rs2_teleop(
                    ser,
                    args.hz,
                    args.kp,
                    args.kd,
                    args.motor_id,
                    args.step,
                    args.arrow_vel,
                    args.ramp_down,
                    args.ramp_up,
                )
        elif args.monitor:
            run_monitor(ser, args.hz, args.kp, args.kd)
        else:
            run_teleop(ser, args.hz, args.kp, args.kd)


if __name__ == "__main__":
    main()
