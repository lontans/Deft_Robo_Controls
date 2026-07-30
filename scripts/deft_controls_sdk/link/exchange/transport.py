"""USB CDC transport: 672 B frame reader and background RX pump."""
from __future__ import annotations

import struct
import threading
import time
from collections import deque
from typing import Deque, List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise ImportError("pyserial required: pip install pyserial") from exc

from .wire_layout import (
    DEFAULT_BAUD,
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_FEEDBACK_MAGIC,
    IMAGE_BYTES,
    STM32_USB_CDC_PID,
    STM32_VID,
)

_FB_MAGICS = (
    struct.pack("<I", HOST_FEEDBACK_MAGIC),
    struct.pack("<I", HOST_DEBUG_FEEDBACK_MAGIC),
)


class FrameReader:
    def __init__(self, *, maxlen: int = 128) -> None:
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._frames: Deque[bytes] = deque(maxlen=maxlen)
        self.total_bytes = 0
        self.total_frames = 0  # plant HBHF only (USB FB rate)
        self.total_debug_frames = 0

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        with self._lock:
            self.total_bytes += len(chunk)
            self._buf.extend(chunk)
            while len(self._buf) >= IMAGE_BYTES:
                head = bytes(self._buf[:4])
                if head not in _FB_MAGICS:
                    idx = -1
                    for magic in _FB_MAGICS:
                        found = self._buf.find(magic)
                        if found > 0 and (idx < 0 or found < idx):
                            idx = found
                    if idx <= 0:
                        self._buf.clear()
                        break
                    del self._buf[:idx]
                    continue
                frame = bytes(self._buf[:IMAGE_BYTES])
                self._frames.append(frame)
                if head == _FB_MAGICS[0]:
                    self.total_frames += 1
                else:
                    self.total_debug_frames += 1
                del self._buf[:IMAGE_BYTES]

    def pop(self) -> Optional[bytes]:
        with self._lock:
            return self._frames.popleft() if self._frames else None

    def push_front(self, frames: List[bytes]) -> None:
        """Re-queue frames at the head (oldest first) for another waiter.

        Used when the plant stream drain must not drop DBGF probe/CFG replies.
        """
        if not frames:
            return
        with self._lock:
            for frame in reversed(frames):
                self._frames.appendleft(frame)

    def drain(self) -> List[bytes]:
        with self._lock:
            out = list(self._frames)
            self._frames.clear()
            return out

    def clear(self) -> None:
        with self._lock:
            self._frames.clear()


class SerialRxPump:
    """Background reader; join before exit (avoids daemon stderr crash on Windows)."""

    def __init__(self, ser: serial.Serial, reader: FrameReader) -> None:
        self._ser = ser
        self._reader = reader
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="pcb-rx", daemon=False)

    def _loop(self) -> None:
        # Legacy SerialRxPump cadence: read(max(1, in_waiting)) so an empty
        # port blocks up to the serial timeout, then brief sleep. Keep this
        # identical — "sleep when empty" diverged from the known-good path.
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


def list_ports_info() -> List[dict]:
    """JSON-friendly port listing (debug_dashboard's /api/ports) — same STM32
    VID/PID hinting as list_serial_ports()/auto_pick_port(), returned as data
    instead of printed, sorted with the likely STM32 USB CDC port first."""
    out = []
    for p in list_ports.comports():
        is_stm32_cdc = p.vid == STM32_VID and p.pid == STM32_USB_CDC_PID
        out.append(
            {
                "device": p.device,
                "description": p.description,
                "vid": f"0x{p.vid:04X}" if p.vid is not None else None,
                "pid": f"0x{p.pid:04X}" if p.pid is not None else None,
                "is_stm32_cdc": is_stm32_cdc,
            }
        )
    out.sort(key=lambda d: (not d["is_stm32_cdc"], d["device"]))
    return out


def list_serial_ports(*, stm32_hint: bool = True) -> None:
    ports = list_ports_info()
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in ports:
        hint = "  <- likely STM32 USB CDC (use this, not ST-Link VCP if both appear)"
        print(f"  {p['device']:16s}  vid={p['vid'] or 'n/a':>6}  {p['description']}{hint if stm32_hint and p['is_stm32_cdc'] else ''}")


def auto_pick_port() -> str:
    ports = list(list_ports.comports())
    stm32 = [p for p in ports if p.vid == STM32_VID]
    cdc = [p for p in stm32 if p.pid == STM32_USB_CDC_PID]
    if len(cdc) == 1:
        return cdc[0].device
    if len(cdc) > 1:
        return sorted(cdc, key=lambda x: x.device)[0].device
    if len(stm32) == 1:
        return stm32[0].device
    if ports:
        return ports[0].device
    raise SystemExit("No serial ports found — use discover --port COM5 (USB CDC, pid=0x5740)")


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


def open_serial(port: str, baud: int = DEFAULT_BAUD) -> serial.Serial:
    try:
        # Read timeout 0: non-blocking read + pump sleep(0.001) on empty.
        # Legacy used timeout=0.05; on Windows that blocking read contended with
        # plant write/flush and left host_tx_gap_max ~27–35 ms even when idle.
        # Do NOT set write_timeout — a mid-frame timeout on Windows CDC can
        # deliver a partial 672 B image and desync the MCU command stream.
        ser = serial.Serial(port, baud, timeout=0)
    except (serial.SerialException, PermissionError, OSError) as exc:
        raise serial.SerialException(format_serial_open_error(port, exc)) from exc
    ser.dtr = True
    ser.rts = False
    return ser


def format_serial_open_error(port: str, exc: BaseException) -> str:
    """Human-readable open failure (PermissionError / port in use on Windows)."""
    msg = str(exc).strip() or repr(exc)
    lower = msg.lower()
    if isinstance(exc, PermissionError) or "access is denied" in lower or "permission" in lower:
        return (
            f"Cannot open {port}: port is in use or access denied.\n\n"
            "Close any other program using this COM port, then retry:\n"
            "  - another debug_dashboard / controls_hub_dashboard window\n"
            "  - python teleop / probe in a terminal\n"
            "  - PuTTY, Arduino IDE Serial Monitor, STM32CubeIDE serial\n\n"
            "If nothing else is open: Disconnect USB, wait 2 s, replug, or reboot the board."
        )
    if "could not open port" in lower or "file not found" in lower:
        return f"Cannot open {port}: device not found. Refresh the port list and check USB cable."
    return f"Cannot open {port}: {msg}"
