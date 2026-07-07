#!/usr/bin/env python3
"""
Raw UART4 <-> Damiao Assistant bridge over the MCU's existing USB CDC link.

Firmware side (see App/Inc/host/uart4_mode.h — UART4_MODE_DAMIAO_BRIDGE):
UART4 (PC10 TX / PC11 RX) is reconfigured to 921600-8N1 and wired to the
Damiao motor's GH1.25 debug port (GND/RX/TX). Raw bytes are tunnelled
through PDU tag 'U','B' (host->MCU, up to 29 B/frame) and 'u' (MCU->host,
up to 30 B/frame) inside the normal 562-byte host_exchange image — no
second USB descriptor needed.

This script bridges those PDU-tunnelled bytes to a local serial port. Pair
it with com0com (a free virtual null-modem driver for Windows):
  1. Install com0com, create a pair, e.g. COM20 <-> COM21.
  2. Run this script with --bridge-port COM20.
  3. Point the real Damiao Assistant at COM21, 921600 baud.

Wiring: MCU UART4 TX (PC10) -> Damiao debug RX, MCU UART4 RX (PC11) <-
Damiao debug TX, common GND. Confirm 3.3V logic before connecting.

Example:
  python scripts/damiao_uart_bridge.py --mcu-port COM9 --bridge-port COM20
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from collections import deque
from typing import Deque, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

HOST_COMMAND_MAGIC = 0x434D4448
HOST_FEEDBACK_MAGIC = 0x46424848
HOST_LAYOUT_VERSION = 1
IMAGE_BYTES = 562
PDU_OFF = 530
PLANT_MCU_STATE_DIAG_ONLY = 2
SYSTEM_CMD_OFF = 12

UB_CMD_MAX_BYTES = 29  # HOST_PDU_PAYLOAD_BYTES(32) - tag(2) - len(1)
UB_FB_MAX_BYTES = 30   # HOST_PDU_PAYLOAD_BYTES(32) - tag(1) - len(1)


def patch_system_mcu_state(buf: bytearray, mcu_state: int) -> None:
    """host_system_command_t.mcu_state is bits 1..3 of the u32 at offset 12."""
    word, = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)
    word = (word & ~0x0E) | ((int(mcu_state) & 7) << 1)
    struct.pack_into("<I", buf, SYSTEM_CMD_OFF, word)


def build_ub_command(chunk: bytes, seq: int) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    patch_system_mcu_state(buf, PLANT_MCU_STATE_DIAG_ONLY)
    n = min(len(chunk), UB_CMD_MAX_BYTES)
    buf[PDU_OFF + 0] = ord("U")
    buf[PDU_OFF + 1] = ord("B")
    buf[PDU_OFF + 2] = n
    buf[PDU_OFF + 3: PDU_OFF + 3 + n] = chunk[:n]
    return bytes(buf)


def parse_ub_feedback(frame: bytes) -> Optional[bytes]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF: PDU_OFF + 32]
    if pdu[0] != ord("u"):
        return None
    n = min(pdu[1], UB_FB_MAX_BYTES)
    return bytes(pdu[2: 2 + n])


class FrameReader:
    def __init__(self) -> None:
        self._buf = bytearray()
        self._frames: Deque[bytes] = deque(maxlen=256)
        self._lock = threading.Lock()

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        with self._lock:
            self._buf.extend(chunk)
            magic = struct.pack("<I", HOST_FEEDBACK_MAGIC)
            while len(self._buf) >= IMAGE_BYTES:
                if self._buf[:4] != magic:
                    idx = self._buf.find(magic)
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


class SerialRxPump:
    """Background reader; join before exit (avoids daemon stderr crash on Windows)."""

    def __init__(self, ser: serial.Serial, reader: FrameReader) -> None:
        self._ser = ser
        self._reader = reader
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="ub-rx", daemon=False)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(max(1, self._ser.in_waiting))
            except serial.SerialException:
                break
            if chunk:
                self._reader.feed(chunk)
            else:
                time.sleep(0.001)

    def __enter__(self) -> "SerialRxPump":
        self._thread.start()
        return self

    def __exit__(self, *args: object) -> None:
        self._stop.set()
        self._thread.join(timeout=1.5)


def run_bridge(mcu_ser: serial.Serial, bridge_ser: serial.Serial) -> int:
    reader = FrameReader()
    pending_tx = bytearray()
    seq = 1
    frames_out = 0
    bytes_to_assistant = 0
    bytes_from_assistant = 0
    last_status = time.monotonic()

    print(f"Bridging {bridge_ser.port} <-> UART4 (Damiao debug port) via {mcu_ser.port}")
    print("Point the Damiao Assistant at the OTHER end of your com0com pair.")
    print("Ctrl+C to stop.\n")

    with SerialRxPump(mcu_ser, reader):
        try:
            while True:
                n = bridge_ser.in_waiting
                if n:
                    chunk = bridge_ser.read(n)
                    pending_tx.extend(chunk)
                    bytes_from_assistant += len(chunk)

                out_chunk = bytes(pending_tx[:UB_CMD_MAX_BYTES])
                del pending_tx[:len(out_chunk)]

                mcu_ser.write(build_ub_command(out_chunk, seq))
                mcu_ser.flush()
                seq += 1
                frames_out += 1

                frame = reader.pop()
                while frame is not None:
                    payload = parse_ub_feedback(frame)
                    if payload:
                        bridge_ser.write(payload)
                        bytes_to_assistant += len(payload)
                    frame = reader.pop()

                now = time.monotonic()
                if now - last_status >= 5.0:
                    print(f"  frames_tx={frames_out}  bytes_from_assistant={bytes_from_assistant}  "
                          f"bytes_to_assistant={bytes_to_assistant}")
                    last_status = now

                if not n:
                    time.sleep(0.001)
        except KeyboardInterrupt:
            print("\nStopped.")
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(description="Bridge MCU UART4 (Damiao debug UART) to a local COM port")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--mcu-port", default=None, help="MCU USB CDC port (e.g. COM9)")
    ap.add_argument("--mcu-baud", type=int, default=115200)
    ap.add_argument("--bridge-port", default=None, help="Local end of a com0com pair (e.g. COM20)")
    ap.add_argument("--bridge-baud", type=int, default=921600,
                     help="Cosmetic only — com0com is a null modem, no real bit timing")
    args = ap.parse_args()

    if args.list_ports:
        for p in list_ports.comports():
            print(p.device, p.description)
        return

    if not args.mcu_port or not args.bridge_port:
        ap.error("--mcu-port and --bridge-port required (or --list-ports)")

    with serial.Serial(args.mcu_port, args.mcu_baud, timeout=0.05) as mcu_ser, \
         serial.Serial(args.bridge_port, args.bridge_baud, timeout=0.01) as bridge_ser:
        time.sleep(0.3)
        raise SystemExit(run_bridge(mcu_ser, bridge_ser))


if __name__ == "__main__":
    main()
