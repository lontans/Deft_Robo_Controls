"""USB CDC transport: 562 B frame reader and background RX pump."""
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

from .protocol import DEFAULT_BAUD, HOST_FEEDBACK_MAGIC, IMAGE_BYTES, STM32_USB_CDC_PID, STM32_VID


class FrameReader:
    def __init__(self, *, maxlen: int = 128) -> None:
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._frames: Deque[bytes] = deque(maxlen=maxlen)
        self.total_bytes = 0
        self.total_frames = 0

    def feed(self, chunk: bytes) -> None:
        if not chunk:
            return
        with self._lock:
            self.total_bytes += len(chunk)
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
                self.total_frames += 1
                del self._buf[:IMAGE_BYTES]

    def pop(self) -> Optional[bytes]:
        with self._lock:
            return self._frames.popleft() if self._frames else None

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


def list_serial_ports(*, stm32_hint: bool = True) -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in sorted(ports, key=lambda x: x.device):
        hint = ""
        if stm32_hint and p.vid == STM32_VID:
            hint = "  <- likely STM32 USB CDC (use this, not ST-Link VCP if both appear)"
        vid_s = f"0x{p.vid:04X}" if p.vid is not None else "n/a"
        print(f"  {p.device:16s}  vid={vid_s:>6}  {p.description}{hint}")


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
        ser = serial.Serial(port, baud, timeout=0.05)
    except (serial.SerialException, PermissionError, OSError) as exc:
        raise serial.SerialException(format_serial_open_error(port, exc)) from exc
    # STM32 USB CDC: allow host to assert DTR (some drivers gate OUT until set).
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
            "  - another controls_hub_dashboard window\n"
            "  - python teleop / probe / host_teleop in a terminal\n"
            "  - PuTTY, Arduino IDE Serial Monitor, STM32CubeIDE serial\n\n"
            "If nothing else is open: Disconnect USB, wait 2 s, replug, or reboot the board."
        )
    if "could not open port" in lower or "file not found" in lower:
        return f"Cannot open {port}: device not found. Refresh the port list and check USB cable."
    return f"Cannot open {port}: {msg}"
