#!/usr/bin/env python3
"""Scan Dynamixel servos on UART5 via MCU PDU tag DXL over USB CDC."""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from collections import deque
from typing import Deque, List, Optional

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

DXL_PROBE_SCAN = 1
DXL_PROBE_PING = 2
DXL_PROBE_FIND_BAUD = 3
DXL_PROBE_TOGGLE_BAUD = 4
DXL_PROBE_SET_BAUD_1M = DXL_PROBE_TOGGLE_BAUD  # legacy alias

XL330_M077_MODEL = 1190
XL330_M288_MODEL = 1200

XL330_MODELS = {
    XL330_M077_MODEL: "XL330-M077-T",
    XL330_M288_MODEL: "XL330-M288-T",
}


class FrameReader:
    def __init__(self) -> None:
        self._buf = bytearray()
        self._frames: Deque[bytes] = deque()
        self._lock = threading.Lock()

    def feed(self, chunk: bytes) -> None:
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


def build_dxl_command(
    kind: int,
    target_id: int = 0,
    id_start: int = 1,
    id_end: int = 20,
    seq: int = 1,
) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, HOST_COMMAND_MAGIC)
    struct.pack_into("<H", buf, 4, HOST_LAYOUT_VERSION)
    struct.pack_into("<H", buf, 6, IMAGE_BYTES)
    struct.pack_into("<I", buf, 8, seq & 0xFFFFFFFF)
    buf[PDU_OFF + 0] = ord("D")
    buf[PDU_OFF + 1] = ord("X")
    buf[PDU_OFF + 2] = ord("L")
    buf[PDU_OFF + 3] = target_id & 0xFF
    buf[PDU_OFF + 4] = kind & 0xFF
    buf[PDU_OFF + 5] = id_start & 0xFF
    buf[PDU_OFF + 6] = id_end & 0xFF
    return bytes(buf)


_HAL_STATUS = {0: "HAL_OK", 1: "HAL_ERROR", 2: "HAL_BUSY", 3: "HAL_TIMEOUT"}
_UART_GSTATE = {
    0x00: "RESET", 0x20: "READY", 0x21: "BUSY",
    0x22: "BUSY_TX", 0x23: "BUSY_RX", 0x24: "BUSY_TX_RX",
}


def parse_dxl_probe_pdu(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    pdu = frame[PDU_OFF : PDU_OFF + 32]
    if pdu[0] != ord("d"):
        return None

    baud, = struct.unpack_from("<I", pdu, 4)
    count = pdu[3]
    hits: List[dict] = []

    for i in range(count):
        off = 8 + i * 3
        if off + 3 > 32:
            break
        sid = pdu[off]
        model = pdu[off + 1] | (pdu[off + 2] << 8)
        name = XL330_MODELS.get(model, f"model_{model}")
        hits.append({"id": sid, "model": model, "name": name})

    return {
        "status": pdu[1],
        "kind": pdu[2],
        "count": count,
        "baud": baud,
        "hits": hits,
        "dbg_init_st": pdu[8],
        "dbg_gstate":  pdu[9],
        "dbg_tx_st":   pdu[10],
        "dbg_lock":    pdu[11],
    }


def wait_dxl_response(reader: FrameReader, timeout_s: float) -> Optional[dict]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            parsed = parse_dxl_probe_pdu(frame)
            if parsed is not None:
                return parsed
            frame = reader.pop()
        time.sleep(0.005)
    return None


def rx_thread(ser: serial.Serial, reader: FrameReader, stop: threading.Event) -> None:
    while not stop.is_set():
        n = ser.in_waiting
        chunk = ser.read(n if n else 1)
        if chunk:
            reader.feed(chunk)
        else:
            time.sleep(0.001)


def main() -> None:
    ap = argparse.ArgumentParser(description="Dynamixel ID scan via MCU USB")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200, help="USB CDC baud (not DXL bus)")
    ap.add_argument("--start", type=int, default=1, help="ID range start (also used for baud discovery fallback)")
    ap.add_argument("--end", type=int, default=253, help="ID range end (also used for baud discovery fallback)")
    ap.add_argument("--ping-id", type=int, default=0, help="If set, PING one ID instead of scan")
    ap.add_argument("--timeout", type=float, default=20.0, help="Seconds to wait for MCU probe feedback")
    ap.add_argument(
        "--toggle-baud",
        action="store_true",
        help="Toggle servos id_start..id_end between 1M and 2M (EEPROM addr 8)",
    )
    ap.add_argument(
        "--set-baud-1m",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    reader = FrameReader()
    stop = threading.Event()
    threading.Thread(target=rx_thread, args=(ser, reader, stop), daemon=True).start()

    if args.toggle_baud or args.set_baud_1m:
        kind = DXL_PROBE_TOGGLE_BAUD
    elif args.ping_id:
        kind = DXL_PROBE_PING
    else:
        kind = DXL_PROBE_SCAN

    cmd = build_dxl_command(
        kind=kind,
        target_id=args.ping_id,
        id_start=args.start,
        id_end=args.end,
        seq=int(time.time()) & 0xFFFFFFFF,
    )
    ser.write(cmd)
    ser.flush()

    print("Sent DXL probe, waiting for feedback...")
    resp = wait_dxl_response(reader, args.timeout)
    stop.set()

    if resp is None:
        print("No DXL feedback (timeout). Check firmware + servo power + UART5 wiring.")
        sys.exit(1)

    if resp["kind"] == DXL_PROBE_TOGGLE_BAUD:
        if resp["status"] == 0:
            print(f"Toggled baud on IDs {args.start}..{args.end}. "
                  f"MCU bus now {resp['baud']} bps. Re-scan to confirm.")
            sys.exit(0)
        print(f"Toggle-baud failed (status={resp['status']}). "
              "No ID replied at any probe baud?")
        sys.exit(1)

    if resp["status"] == 1:
        init_st = resp["dbg_init_st"]
        gstate  = resp["dbg_gstate"]
        tx_st   = resp["dbg_tx_st"]
        lock    = resp["dbg_lock"]
        print("MCU could not find Dynamixel baud rate.")
        print(f"  HAL_UART_Init  return : 0x{init_st:02X}  {_HAL_STATUS.get(init_st, '?')}")
        print(f"  gState before TX      : 0x{gstate:02X}  {_UART_GSTATE.get(gstate, '?')}  (expect 0x20=READY)")
        print(f"  HAL_UART_Transmit ret : 0x{tx_st:02X}  {_HAL_STATUS.get(tx_st, '?')}")
        print(f"  Lock before TX        : 0x{lock:02X}  {'LOCKED' if lock else 'UNLOCKED'}")
        if init_st != 0:
            print("  => HAL_UART_Init failed — UART5 clock/config issue.")
        elif tx_st == 2:
            print("  => HAL_UART_Transmit returned HAL_BUSY — gState not READY or Lock stuck.")
        elif tx_st == 3:
            print("  => HAL_UART_Transmit returned HAL_TIMEOUT — TXE never set (UART5 clock?)")
        elif tx_st == 1:
            print("  => HAL_UART_Transmit returned HAL_ERROR — UART hardware fault.")
        elif tx_st == 0 and init_st == 0:
            print("  => TX OK but no reply at any baud (broadcast + unicast IDs 1..64).")
            print("     Servo may be ID>64, unusual baud, or no power. Try: --start 1 --end 20")
        sys.exit(1)
    if resp["status"] != 0:
        print(f"Probe failed status={resp['status']}")
        sys.exit(1)

    print(f"Bus baud (MCU): {resp['baud']} bps")
    if resp["count"] == 0:
        print("No servos found in ID range.")
        sys.exit(0)

    print(f"Found {resp['count']} servo(s):")
    for h in resp["hits"]:
        print(f"  ID {h['id']:3d}  model={h['model']}  ({h['name']})")


if __name__ == "__main__":
    main()