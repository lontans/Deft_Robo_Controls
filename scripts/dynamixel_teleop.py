#!/usr/bin/env python3
"""
Dynamixel neck servo teleop over USB CDC (562 B plant image, no DXL PDU).

Slot 0 = bottom neck (ID 1): Left / Right
Slot 1 = top neck (ID 2):    Up / Down

Position ramps up while a key is held; release stops immediately (no coast-down).
When idle, goal is latched at present on entry (not continuously tracked — that caused hunting).

Examples:
  python scripts/dynamixel_teleop.py --port COM9
  python scripts/dynamixel_teleop.py --port COM9 --hz 40 --arrow-vel 90
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

from rs02_can_scan import (  # noqa: E402
    FrameReader,
    build_plant_servo_command,
    parse_servo_feedback,
    serial_rx_thread,
)

HOST_FEEDBACK_MAGIC = 0x46424848
IMAGE_BYTES = 562
USB_BAUD = 115200
PDU_OFF = 530

BUS_STATE_NAMES = {
    0: "torque_on",
    1: "uni_wr",
    2: "uni_rd",
}

# Matches App/Src/plant/plant_config.c neck limits (XL330 ticks, 4096/rev).
NECK_POS_MIN = 1024
NECK_POS_MAX = 3072
TOP_NECK_POS_MIN = 512  # top neck (ID 2): extra down-travel vs bottom
XL330_POS_MAX = 4095

DEFAULT_HZ = 40.0
DEFAULT_ARROW_VEL = 900.0
DEFAULT_RAMP_UP_S = 0.35
VEL_STOP_TICKS_S = 2.0

_live_line_len = 0


def write_live_line(text: str) -> None:
    global _live_line_len
    line = text.replace("\n", " ")[:118]
    pad = max(0, _live_line_len - len(line))
    sys.stdout.write("\r" + line + " " * pad + "\x1b[K")
    sys.stdout.flush()
    _live_line_len = len(line)


def write_live_notice(text: str) -> None:
    global _live_line_len
    sys.stdout.write("\n" + text + "\n")
    sys.stdout.flush()
    _live_line_len = 0


def poll_neck_axes() -> Tuple[int, int]:
    """
    Return (bottom_dir, top_dir) each in {-1, 0, +1}.
    Windows: physical arrow hold via GetAsyncKeyState.
    """
    if sys.platform == "win32":
        import ctypes

        user32 = ctypes.windll.user32
        left = user32.GetAsyncKeyState(0x25) & 0x8000
        right = user32.GetAsyncKeyState(0x27) & 0x8000
        up = user32.GetAsyncKeyState(0x26) & 0x8000
        down = user32.GetAsyncKeyState(0x28) & 0x8000

        bottom = 0
        if left and not right:
            bottom = -1
        elif right and not left:
            bottom = 1

        top = 0
        if up and not down:
            top = 1
        elif down and not up:
            top = -1

        return bottom, top

    return 0, 0


def poll_key_nonblocking() -> Optional[str]:
    if sys.platform == "win32":
        import msvcrt

        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
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
        if line in ("q", "quit"):
            return "q"
        if line == "r":
            return "r"
    return None


@dataclass
class ServoTeleopState:
    slot: int
    servo_id: int
    label: str
    pos_min: int = NECK_POS_MIN
    pos_max: int = NECK_POS_MAX
    cmd_position: int = 0
    cmd_velocity: float = 0.0
    feedback_synced: bool = False
    fb_position: int = 0
    fb_raw_position: int = 0
    fb_raw_motor_id: int = 0
    active: bool = False  # arrow held or coasting after release

    def clamp_cmd(self) -> None:
        self.cmd_position = max(self.pos_min, min(self.pos_max, self.cmd_position))

    def fb_label(self) -> str:
        return str(self.fb_position) if self.feedback_synced else "?"


def make_neck_states() -> List[ServoTeleopState]:
    return [
        ServoTeleopState(0, 1, "bottom ID 1"),
        ServoTeleopState(1, 2, "top ID 2", pos_min=TOP_NECK_POS_MIN),
    ]


def parse_servo_diag(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 24]
    if pdu[0:3] != b"SVD":
        return None
    p0 = struct.unpack_from("<h", pdu, 12)[0]
    p1 = struct.unpack_from("<h", pdu, 14)[0]
    d0 = struct.unpack_from("<h", pdu, 19)[0]
    d1 = struct.unpack_from("<h", pdu, 21)[0]
    bus = pdu[5]
    return {
        "armed": bool(pdu[3]),
        "boot_slot": pdu[4],
        "boot_step": pdu[5],
        "bus_state": bus,
        "bus_name": BUS_STATE_NAMES.get(bus, f"?{bus}"),
        "read_slot": pdu[6],
        "wr_ok": pdu[8],
        "rd_ok": pdu[9],
        "torque_ok": pdu[10],
        "live_p0": p0,
        "live_p1": p1,
        "desire_p0": d0,
        "desire_p1": d1,
        "live_id0": pdu[16],
        "live_id1": pdu[17],
        "host_stale": bool(pdu[18]),
    }


def format_servo_diag(diag: Optional[dict]) -> str:
    if diag is None:
        return "diag=none(SVD missing — reflash firmware?)"
    return (
        f"torque_done={int(diag['armed'])} bus={diag['bus_name']} "
        f"wr={diag['wr_ok']} rd={diag['rd_ok']} tq={diag['torque_ok']} "
        f"stale={int(diag['host_stale'])} "
        f"mcu_p=[{diag['live_p0']},{diag['live_p1']}] "
        f"mcu_goal=[{diag['desire_p0']},{diag['desire_p1']}]"
    )


def feedback_valid(fb: dict, servo_id: int) -> bool:
    pos = fb["present_position"]
    if pos < 0 or pos > XL330_POS_MAX:
        return False
    mid = fb["motor_source_id"]
    if mid == servo_id:
        return True
    if mid == 0:
        return True
    return False


@dataclass
class FeedbackPoll:
    tick: Optional[int] = None
    diag: Optional[dict] = None


def poll_servo_feedback(
    reader: FrameReader,
    states: List[ServoTeleopState],
) -> FeedbackPoll:
    out = FeedbackPoll()
    frame = reader.pop()
    while frame is not None:
        magic, = struct.unpack_from("<I", frame, 0)
        if magic == HOST_FEEDBACK_MAGIC:
            sys_word, = struct.unpack_from("<I", frame, 12)
            out.tick = sys_word & 0xFFF
            out.diag = parse_servo_diag(frame)
            for st in states:
                fb = parse_servo_feedback(frame, st.slot)
                if fb is None:
                    continue
                st.fb_raw_position = fb["present_position"]
                st.fb_raw_motor_id = fb["motor_source_id"]
                if not feedback_valid(fb, st.servo_id):
                    continue
                st.fb_position = fb["present_position"]
                st.feedback_synced = True
        frame = reader.pop()
    return out


def sync_servo_feedback(
    reader: FrameReader,
    states: List[ServoTeleopState],
    dwell_s: float,
    send_hz: float,
    ser: Optional[serial.Serial] = None,
    cmd_seq: int = 0,
    debug: bool = False,
) -> int:
    """Wait for feedback; keep sending commands so MCU bootstrap does not go stale."""
    dt = 1.0 / max(send_hz, 1.0)
    deadline = time.monotonic() + dwell_s
    last_diag_print = 0.0
    last_diag: Optional[dict] = None
    while time.monotonic() < deadline:
        poll = poll_servo_feedback(reader, states)
        last_diag = poll.diag
        for st in states:
            if st.feedback_synced:
                st.cmd_position = st.fb_position
        if ser is not None:
            cmd_seq = send_servo_states(ser, cmd_seq, states)
        if all(st.feedback_synced for st in states):
            break
        if debug and (time.monotonic() - last_diag_print) >= 1.0:
            raw = ", ".join(
                f"s{st.slot}:raw={st.fb_raw_position} id={st.fb_raw_motor_id}"
                for st in states
            )
            write_live_notice(f"sync wait… {raw} | {format_servo_diag(last_diag)}")
            last_diag_print = time.monotonic()
        time.sleep(dt)
    return cmd_seq


def send_servo_states(
    ser: serial.Serial,
    cmd_seq: int,
    states: List[ServoTeleopState],
) -> int:
    slot_positions = {
        st.slot: (st.cmd_position, st.servo_id)
        for st in states
        if st.feedback_synced
    }
    ser.write(build_plant_servo_command(cmd_seq, slot_positions or None))
    ser.flush()
    return (cmd_seq + 1) & 0xFFFFFFFF


def ramp_servo_axis(
    st: ServoTeleopState,
    motion_dir: int,
    arrow_vel: float,
    ramp_up_s: float,
    dt: float,
) -> None:
    target_vel = motion_dir * abs(arrow_vel)
    alpha = 1.0 - math.exp(-dt / max(ramp_up_s, 0.05))
    st.cmd_velocity += (target_vel - st.cmd_velocity) * alpha

    if abs(st.cmd_velocity) >= VEL_STOP_TICKS_S:
        st.cmd_position = int(round(st.cmd_position + st.cmd_velocity * dt))
        st.clamp_cmd()


def update_servo_axis(
    st: ServoTeleopState,
    motion_dir: int,
    arrow_vel: float,
    ramp_up_s: float,
    dt: float,
) -> None:
    """
    Single goal per axis: snap cmd to present on idle entry (once), then hold that
    goal. Do not chase feedback every frame — that lag loop causes oscillation.
    """
    if motion_dir != 0:
        if not st.active and st.feedback_synced:
            st.cmd_position = st.fb_position
        st.active = True
        ramp_servo_axis(st, motion_dir, arrow_vel, ramp_up_s, dt)
        return

    if st.active:
        st.cmd_velocity = 0.0
        st.active = False
        return

    st.cmd_velocity = 0.0


def run_servo_teleop(
    ser: serial.Serial,
    send_hz: float = DEFAULT_HZ,
    arrow_vel: float = DEFAULT_ARROW_VEL,
    ramp_up_s: float = DEFAULT_RAMP_UP_S,
    debug: bool = False,
) -> None:
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=serial_rx_thread, args=(ser, reader, stop), daemon=True).start()

    states = make_neck_states()
    cmd_seq = 1
    dt = 1.0 / max(send_hz, 0.1)
    fb_line = 0
    mcu_tick: Optional[int] = None
    last_diag: Optional[dict] = None

    print(f"Dynamixel neck teleop on {ser.port} @ {send_hz:.0f} Hz  (MCU unicast wr/rd)")
    for st in states:
        print(f"  slot {st.slot}: {st.label}  ticks {st.pos_min}..{st.pos_max}")
    print(
        f"Motion: ±{arrow_vel:.0f} ticks/s  ramp_up={ramp_up_s:.2f}s  "
        f"release=instant-stop  idle=latch-present"
    )
    print("  Left / Right     bottom neck (slot 0, ID 1)")
    print("  Up / Down        top neck (slot 1, ID 2)")
    print("  r                re-sync cmd from feedback + stop motion")
    print("  q                quit")
    if sys.platform != "win32":
        print("  (Arrow hold requires Windows; use host_teleop_laptop_usb on Win for arrows.)")
    print()
    print("Syncing feedback...")
    reader.drain()
    for _ in range(max(4, int(send_hz * 0.5))):
        poll = poll_servo_feedback(reader, states)
        last_diag = poll.diag
        for st in states:
            if st.feedback_synced:
                st.cmd_position = st.fb_position
        cmd_seq = send_servo_states(ser, cmd_seq, states)
        time.sleep(dt)
    cmd_seq = sync_servo_feedback(
        reader, states, dwell_s=5.0, send_hz=send_hz, ser=ser, cmd_seq=cmd_seq, debug=debug,
    )
    poll = poll_servo_feedback(reader, states)
    last_diag = poll.diag
    for st in states:
        if st.feedback_synced:
            st.cmd_position = st.fb_position
            print(f"  synced {st.label}  pos={st.fb_position} ticks")
        else:
            print(
                f"  warning: no feedback for {st.label} — "
                f"raw pos={st.fb_raw_position} id={st.fb_raw_motor_id} "
                f"cmd starts at {st.cmd_position}"
            )
    print(format_servo_diag(last_diag))
    print()

    try:
        while True:
            bottom_dir, top_dir = poll_neck_axes()
            key = poll_key_nonblocking()
            if key == "q":
                break
            if key == "r":
                reader.drain()
                for st in states:
                    st.cmd_velocity = 0.0
                    st.active = False
                cmd_seq = sync_servo_feedback(
                    reader, states, dwell_s=0.5, send_hz=send_hz, ser=ser, cmd_seq=cmd_seq,
                    debug=debug,
                )
                for st in states:
                    if st.feedback_synced:
                        st.cmd_position = st.fb_position
                write_live_notice("re-synced from feedback; velocity cleared")

            poll = poll_servo_feedback(reader, states)
            mcu_tick = poll.tick
            last_diag = poll.diag
            update_servo_axis(states[0], bottom_dir, arrow_vel, ramp_up_s, dt)
            update_servo_axis(states[1], top_dir, arrow_vel, ramp_up_s, dt)
            cmd_seq = send_servo_states(ser, cmd_seq, states)

            fb_line += 1
            if fb_line % max(1, int(send_hz / 4)) == 0:
                parts = []
                if mcu_tick is not None:
                    parts.append(f"tick={mcu_tick}")
                for st in states:
                    fb_txt = st.fb_label()
                    if debug and not st.feedback_synced:
                        fb_txt = f"?({st.fb_raw_position}|id{st.fb_raw_motor_id})"
                    parts.append(
                        f"{st.label}: {'*' if st.active else '·'} "
                        f"v={st.cmd_velocity:+.0f} cmd={st.cmd_position} fb={fb_txt}"
                    )
                if debug and last_diag is not None:
                    parts.append(
                        f"bus={last_diag['bus_name']} stale={int(last_diag['host_stale'])} "
                        f"mcu_goal=[{last_diag['desire_p0']},{last_diag['desire_p1']}]"
                    )
                write_live_line("  ".join(parts))

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopping servo teleop...")
    finally:
        for st in states:
            st.cmd_velocity = 0.0
            st.active = False
        cmd_seq = send_servo_states(ser, cmd_seq, states)
        time.sleep(dt)
        ser.write(build_plant_servo_command(cmd_seq, {}))
        ser.flush()
        stop.set()
        print("Done.")


def main() -> None:
    ap = argparse.ArgumentParser(description="Dynamixel neck servo teleop via MCU USB")
    ap.add_argument("--port", required=True, help="USB CDC port (e.g. COM9)")
    ap.add_argument("--baud", type=int, default=USB_BAUD, help="pyserial baud (USB ignores)")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="Host command rate (default 40)")
    ap.add_argument("--arrow-vel", type=float, default=DEFAULT_ARROW_VEL,
                    help=f"Max slew ticks/s while arrow held (default {DEFAULT_ARROW_VEL})")
    ap.add_argument("--ramp-up", type=float, default=DEFAULT_RAMP_UP_S,
                    help=f"Velocity ramp-up tau (s, default {DEFAULT_RAMP_UP_S})")
    ap.add_argument("--debug", action="store_true",
                    help="Print MCU SVD diagnostics (armed, bus state, read counts)")
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud}")
    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.3)
        run_servo_teleop(
            ser,
            send_hz=args.hz,
            arrow_vel=args.arrow_vel,
            ramp_up_s=args.ramp_up,
            debug=args.debug,
        )


if __name__ == "__main__":
    main()
