#!/usr/bin/env python3
"""
RS-02 teleop: 562 B host command/feedback images (layout v1).

Command rate (--hz):
  send_period = 1.0 / hz. The main loop writes a command image whenever
  monotonic time since the last send >= send_period (default 30 Hz USB / 10 Hz UART).
  Arrow / r also trigger an immediate send so key latency is not capped by hz.

Transport (Jetson):
  Run script; press 1 = USB CDC (/dev/ttyACM*), 0 = UART (/dev/ttyUSB* or /dev/ttyTHS*).
  Or pass --transport usb|uart to skip the prompt.

Jetson:
  sudo apt install python3-serial
  python3 scripts/host_teleop.py
"""

from __future__ import annotations

import argparse
import glob
import struct
import sys
import termios
import threading
import time
import tty
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, Optional

try:
    import serial
except ImportError:
    print("pyserial required: pip3 install pyserial", file=sys.stderr)
    sys.exit(1)

try:
    import curses
except ImportError:
    print("curses not available", file=sys.stderr)
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
MOTION_EPS_DEFAULT = 0.002

USB_DEFAULT_HZ = 30.0
UART_DEFAULT_HZ = 10.0
UART_BAUD = 115200
USB_BAUD = 115200


class TransportMode(str, Enum):
    USB = "usb"
    UART = "uart"


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
    fb_seq, = struct.unpack_from("<I", data, 8)
    sys_word, = struct.unpack_from("<I", data, 12)
    pos, vel, torque, temp, fault = struct.unpack_from("<ffffI", data, ACTUATOR0_FB_OFF)
    return {
        "fb_seq": fb_seq,
        "tick": sys_word & 0xFFF,
        "last_cmd_seq": (sys_word >> 17) & 0xFF,
        "position": pos,
        "velocity": vel,
        "torque": torque,
        "temperature": temp,
        "fault": fault,
    }


class FrameReader:
    """Reassemble fixed 562 B feedback images from arbitrary byte chunks."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._frames: Deque[bytes] = deque(maxlen=8)

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        with self._lock:
            self._buf.extend(chunk)
            while len(self._buf) >= IMAGE_BYTES:
                magic, = struct.unpack_from("<I", self._buf, 0)
                if magic != HOST_FEEDBACK_MAGIC:
                    idx = self._buf.find(struct.pack("<I", HOST_FEEDBACK_MAGIC))
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


@dataclass
class KeyEvent:
    t0: float
    seq: int
    pos_at_key: float
    ack_ms: Optional[float] = None
    motion_ms: Optional[float] = None


def update_latency(ev: KeyEvent, fb: dict, motion_eps: float, now: float) -> None:
    if ev.ack_ms is None and fb["last_cmd_seq"] == (ev.seq & 0xFF):
        ev.ack_ms = (now - ev.t0) * 1000.0
    if ev.motion_ms is None and abs(fb["position"] - ev.pos_at_key) >= motion_eps:
        ev.motion_ms = (now - ev.t0) * 1000.0


def first_glob(patterns: tuple[str, ...], fallback: str) -> str:
    for pat in patterns:
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return fallback


def jetson_port_for_mode(mode: TransportMode) -> str:
    if mode == TransportMode.USB:
        return first_glob(("/dev/ttyACM*",), "/dev/ttyACM0")
    return first_glob(("/dev/ttyUSB*", "/dev/ttyTHS*"), "/dev/ttyUSB0")


def jetson_baud_for_mode(mode: TransportMode) -> int:
    return UART_BAUD if mode == TransportMode.UART else USB_BAUD


def default_hz_for_mode(mode: TransportMode) -> float:
    return UART_DEFAULT_HZ if mode == TransportMode.UART else USB_DEFAULT_HZ


def uart_sustainable_hz(baud: int) -> float:
    """Rough full-duplex ceiling: one 562 B cmd + 562 B fb per cycle at 8N1."""
    bytes_per_sec = baud / 10.0
    return max(1.0, bytes_per_sec / (2.0 * IMAGE_BYTES))


def prompt_transport_mode() -> TransportMode:
    print("Select host transport (must match firmware HOST_TRANSPORT_UART in host_transport.h):")
    print("  1 = USB CDC   -> /dev/ttyACM*")
    print("  0 = UART      -> /dev/ttyUSB* or /dev/ttyTHS* @ 115200 8N1")
    print("Press 1 or 0: ", end="", flush=True)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == "1":
                print("1  (USB)")
                return TransportMode.USB
            if ch == "0":
                print("0  (UART)")
                return TransportMode.UART
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    raise RuntimeError("unreachable")


def infer_mode_from_port(port: str) -> Optional[TransportMode]:
    p = port.upper()
    if "ACM" in p:
        return TransportMode.USB
    if "USB" in p or "THS" in p:
        return TransportMode.UART
    return None


def resolve_transport(args: argparse.Namespace) -> tuple[TransportMode, str, int, float]:
    if args.transport is not None:
        mode = TransportMode(args.transport)
    elif args.port is not None:
        mode = infer_mode_from_port(args.port) or prompt_transport_mode()
    else:
        mode = prompt_transport_mode()

    port = args.port if args.port is not None else jetson_port_for_mode(mode)
    baud = args.baud if args.baud is not None else jetson_baud_for_mode(mode)
    hz = args.hz if args.hz is not None else default_hz_for_mode(mode)

    cap = uart_sustainable_hz(baud)
    if mode == TransportMode.UART and hz > cap:
        print(
            f"Note: {hz:.0f} Hz exceeds ~{cap:.1f} Hz UART budget at {baud} baud "
            f"({IMAGE_BYTES} B x2). Lower with --hz or raise baud on both sides.",
            file=sys.stderr,
        )

    return mode, port, baud, hz


def run_teleop(
    ser: serial.Serial,
    mode: TransportMode,
    send_hz: float,
    kp: float,
    kd: float,
    motion_eps: float,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    cmd_position = 0.0
    cmd_seq = 1
    telemetry: Optional[dict] = None
    send_period = 1.0 / max(send_hz, 0.1)
    last_send = 0.0
    status = "starting"
    pending: Optional[KeyEvent] = None
    last_ack_ms: Optional[float] = None
    last_motion_ms: Optional[float] = None
    ack_hist: Deque[float] = deque(maxlen=10)
    motion_hist: Deque[float] = deque(maxlen=10)

    def send_command() -> None:
        nonlocal last_send
        ser.write(build_command_image(cmd_position, cmd_seq, kp, kd))
        ser.flush()
        last_send = time.monotonic()

    def on_input_key() -> None:
        nonlocal pending, cmd_seq
        pending = KeyEvent(
            t0=time.monotonic(),
            seq=cmd_seq,
            pos_at_key=telemetry["position"] if telemetry else 0.0,
        )
        send_command()

    def on_feedback(fb: dict) -> None:
        nonlocal pending, last_ack_ms, last_motion_ms
        now = time.monotonic()
        if pending is not None:
            update_latency(pending, fb, motion_eps, now)
            if pending.ack_ms is not None:
                last_ack_ms = pending.ack_ms
                ack_hist.append(pending.ack_ms)
            if pending.motion_ms is not None:
                last_motion_ms = pending.motion_ms
                motion_hist.append(pending.motion_ms)
            if pending.ack_ms is not None and pending.motion_ms is not None:
                pending = None

    def avg(d: Deque[float]) -> Optional[float]:
        return sum(d) / len(d) if d else None

    def draw(stdscr: curses.window) -> None:
        nonlocal status
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        if h < 16 or w < 68:
            stdscr.addstr(0, 0, "Terminal too small (need ~68x16)")
            stdscr.refresh()
            return
        t = telemetry
        link = "USB CDC" if mode == TransportMode.USB else f"UART {ser.baudrate} 8N1"
        stdscr.addstr(0, 0, "Deft host teleop — RS-02 slot 0")
        stdscr.addstr(1, 0, f"{ser.port} ({link})  cmd_rate={send_hz:.0f}Hz  {status}")
        stdscr.addstr(2, 0, "Left/Right: move cmd   r: zero   q: quit")
        stdscr.addstr(4, 0, f"cmd  pos {cmd_position:+.4f} rad  seq {cmd_seq & 0xFF:3d}  kp {kp:.1f}  kd {kd:.1f}")
        if t:
            stdscr.addstr(6, 0, f"fb   pos {t['position']:+.4f} rad  vel {t['velocity']:+.4f}  tau {t['torque']:+.3f}")
            stdscr.addstr(7, 0, f"     temp {t['temperature']:.1f}C  fault 0x{t['fault']:08X}  ack_seq {t['last_cmd_seq']:3d}  tick {t['tick']}")
        else:
            stdscr.addstr(6, 0, "fb   (waiting...)")
        la, lm = last_ack_ms, last_motion_ms
        aa, am = avg(ack_hist), avg(motion_hist)
        stdscr.addstr(9, 0, "Latency last key (host measured, 8-bit seq):")
        stdscr.addstr(10, 0, f"  ack MCU saw cmd     {la:6.1f} ms" if la is not None else "  ack MCU saw cmd        —")
        stdscr.addstr(11, 0, f"  motion state moved  {lm:6.1f} ms" if lm is not None else "  motion state moved     —")
        if aa is not None or am is not None:
            stdscr.addstr(12, 0, f"  avg ({len(ack_hist)} keys) ack {aa:5.1f} ms  motion {am:5.1f} ms" if aa and am else "")
        if pending is not None:
            age = (time.monotonic() - pending.t0) * 1000.0
            stdscr.addstr(13, 0, f"  pending seq {pending.seq & 0xFF:3d}  waiting {age:.0f} ms...")
        stdscr.refresh()

    def main(stdscr: curses.window) -> None:
        nonlocal cmd_position, cmd_seq, telemetry, status
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)
        send_command()
        while True:
            now = time.monotonic()
            if now - last_send >= send_period:
                send_command()
            frame = reader.pop()
            while frame is not None:
                fb = parse_feedback_image(frame)
                if fb is not None:
                    telemetry = fb
                    status = "ok"
                    on_feedback(fb)
                frame = reader.pop()
            if telemetry is None:
                status = "no feedback"
            key = stdscr.getch()
            if key == ord("q"):
                break
            if key == ord("r"):
                cmd_position = 0.0
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
                on_input_key()
            elif key == curses.KEY_LEFT:
                cmd_position = max(P_MIN, cmd_position - POS_STEP)
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
                on_input_key()
            elif key == curses.KEY_RIGHT:
                cmd_position = min(P_MAX, cmd_position + POS_STEP)
                cmd_seq = (cmd_seq + 1) & 0xFFFFFFFF
                on_input_key()
            draw(stdscr)

    try:
        curses.wrapper(main)
    finally:
        stop.set()


def main() -> None:
    ap = argparse.ArgumentParser(description="RS-02 host teleop")
    ap.add_argument("--transport", choices=("usb", "uart"),
                    help="Skip prompt: usb (/dev/ttyACM*) or uart (/dev/ttyUSB* / ttyTHS*)")
    ap.add_argument("--port", default=None, help="Override serial device path")
    ap.add_argument("--baud", type=int, default=None,
                    help=f"UART baud (default {UART_BAUD}; USB CDC ignores but pyserial still sets it)")
    ap.add_argument("--hz", type=float, default=None,
                    help=f"Periodic command rate (default {USB_DEFAULT_HZ} USB / {UART_DEFAULT_HZ} UART)")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--motion-eps", type=float, default=MOTION_EPS_DEFAULT,
                    help="Rad change in fb position counted as motion (default 0.002)")
    args = ap.parse_args()

    mode, port, baud, hz = resolve_transport(args)
    print(f"Opening {port} ({mode.value}) @ {baud}  cmd_rate={hz} Hz  motion_eps={args.motion_eps} rad")

    ser_kwargs: dict = {"port": port, "baudrate": baud, "timeout": 0.05}
    if mode == TransportMode.UART:
        ser_kwargs.update(
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )

    with serial.Serial(**ser_kwargs) as ser:
        time.sleep(0.3)
        run_teleop(ser, mode, hz, args.kp, args.kd, args.motion_eps)


if __name__ == "__main__":
    main()
