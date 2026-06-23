#!/usr/bin/env python3
"""
Laptop USB CDC test for Deft controls PCB (562 B layout v1).

Use this on a Windows/Linux laptop with the board on USB — no Jetson, no UART mode.
Firmware must use HOST_TRANSPORT_UART 0 in App/Inc/host/host_transport.h.

Windows (Lenovo X1 etc.):
  pip install pyserial
  python scripts/host_teleop_laptop_usb.py --list-ports
  python scripts/host_teleop_laptop_usb.py --port COM5

Linux laptop:
  python3 scripts/host_teleop_laptop_usb.py --port /dev/ttyACM0

Keys (teleop): Left/Right move cmd, r zero, q quit.
Monitor only: --monitor (no keys; prints one line per feedback frame).
"""

from __future__ import annotations

import argparse
import glob
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

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
POS_STEP = 0.05
DEFAULT_KP = 50.0
DEFAULT_KD = 1.0
DEFAULT_HZ = 30.0
USB_BAUD = 115200

STM32_VID = 0x0483


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


@dataclass
class SessionStats:
    fb_count: int = 0
    last_tick: Optional[int] = None
    tick_stalls: int = 0


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
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="Command send rate (default 30)")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--monitor", action="store_true",
                    help="Print one line per feedback frame (good first link check)")
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    port = args.port if args.port is not None else auto_pick_port()
    print(f"Opening {port} (USB CDC) @ {args.baud}  cmd_rate={args.hz} Hz")
    print("Firmware: set HOST_TRANSPORT_UART to 0 in App/Inc/host/host_transport.h")

    with serial.Serial(port=port, baudrate=args.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        if args.monitor:
            run_monitor(ser, args.hz, args.kp, args.kd)
        else:
            run_teleop(ser, args.hz, args.kp, args.kd)


if __name__ == "__main__":
    main()
