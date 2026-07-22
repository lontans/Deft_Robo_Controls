"""Soft-DFU — enter ROM bootloader over CDC, leave DFU over USB (no ST-Link).

Firmware: App/Src/host/soft_dfu.c / App/Inc/host/soft_dfu.h.

Enter: plant CMD with PDU tag DFU! → MCU resets into system memory BL.
Leave: AN3156 Leave DFU — Set Address Pointer then zero-length DFU_DNLOAD
(requires GETSTATUS after SET_ADDRESS so the command actually executes).

Port discovery is OS-aware (Windows COMx / Linux /dev/ttyACM*) and prefers
STM32 USB CDC 0483:5740 — never hard-code COM5.
"""
from __future__ import annotations

import platform
import struct
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Optional, Sequence

from deft_controls_sdk.link.exchange import PDU_OFF, build_plant_command
from deft_controls_sdk.link.exchange.wire_layout import (
    STM32_USB_CDC_PID,
    STM32_VID,
)

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection

_TAG = b"DFU!"  # must match SOFT_DFU_TAG0..3 in App/Inc/host/soft_dfu.h

# STM32 ROM USB DFU (AN2606 / AN3156)
_DFU_VID = 0x0483
_DFU_PID = 0xDF11

DFU_DETACH = 0
DFU_DNLOAD = 1
DFU_GETSTATUS = 3
DFU_CLRSTATUS = 4
DFU_GETSTATE = 5
DFU_ABORT = 6

# bState values (DFU 1.1)
_STATE_IDLE = 2
_STATE_DNLOAD_SYNC = 3
_STATE_DNBUSY = 4
_STATE_DNLOAD_IDLE = 5
_STATE_MANIFEST_SYNC = 6
_STATE_MANIFEST = 7
_STATE_MANIFEST_WAIT_RESET = 8
_STATE_UPLOAD_IDLE = 9
_STATE_ERROR = 10

_APP_FLASH_BASE = 0x08000000
# Mini vector table in app flash: MSP + reset trampoline → NVIC_SystemReset.
# Direct Leave to 0x08000000 skips a full reset; USB CDC often stays dead.
_LEAVE_VT_ADDR = 0x0803F800

_HOST_OS = platform.system()  # "Windows" | "Linux" | "Darwin" | ...


@dataclass(frozen=True)
class CdcPortInfo:
    """One STM32 (or candidate) CDC serial device."""

    device: str
    serial: Optional[str]
    vid: Optional[int]
    pid: Optional[int]
    description: str
    is_stm32_cdc: bool


def host_os() -> str:
    """Normalized host OS name: windows | linux | darwin | other."""
    name = _HOST_OS.lower()
    if name.startswith("win"):
        return "windows"
    if name.startswith("linux"):
        return "linux"
    if name == "darwin":
        return "darwin"
    return name or "other"


def list_cdc_ports(*, serial: Optional[str] = None) -> List[CdcPortInfo]:
    """List serial ports; STM32 app CDC (0483:5740) first.

    Windows: COMx via pyserial. Linux/macOS: typically /dev/ttyACM* (or
    /dev/tty.usbmodem* on Darwin) with the same VID/PID filter.
    """
    from serial.tools import list_ports

    out: List[CdcPortInfo] = []
    for p in list_ports.comports():
        vid = p.vid
        pid = p.pid
        sn = getattr(p, "serial_number", None) or None
        is_cdc = vid == STM32_VID and pid == STM32_USB_CDC_PID
        if serial is not None and sn != serial:
            continue
        out.append(
            CdcPortInfo(
                device=p.device,
                serial=sn,
                vid=vid,
                pid=pid,
                description=p.description or "",
                is_stm32_cdc=is_cdc,
            )
        )
    out.sort(key=lambda x: (not x.is_stm32_cdc, x.device))
    return out


def find_cdc_port(
    *,
    port: Optional[str] = None,
    serial: Optional[str] = None,
) -> str:
    """Resolve the app CDC device path (COMx or /dev/ttyACM*).

    Preference: explicit ``port`` → matching ``serial`` → sole 0483:5740 →
    error if ambiguous or missing. Never falls back to ST-Link VCP.
    """
    if port:
        return port

    ports = list_cdc_ports(serial=serial)
    cdc = [p for p in ports if p.is_stm32_cdc]
    if serial is not None:
        if len(cdc) == 1:
            return cdc[0].device
        if not cdc:
            raise RuntimeError(
                f"no STM32 USB CDC (0483:5740) with serial={serial!r} on "
                f"{host_os()} — is the app running and the USB device cable "
                f"plugged in?"
            )
        raise RuntimeError(
            f"multiple CDC ports match serial={serial!r}: "
            + ", ".join(p.device for p in cdc)
        )

    if len(cdc) == 1:
        return cdc[0].device
    if len(cdc) > 1:
        raise RuntimeError(
            "multiple STM32 USB CDC ports — pass serial= or port=: "
            + ", ".join(f"{p.device}(sn={p.serial})" for p in cdc)
        )
    raise RuntimeError(
        f"no STM32 USB CDC (vid=0x{STM32_VID:04X} pid=0x{STM32_USB_CDC_PID:04X}) "
        f"on {host_os()} — expected Windows COMx or Linux /dev/ttyACM*"
    )


def wait_for_cdc(
    *,
    serial: Optional[str] = None,
    timeout_s: float = 12.0,
    exclude: Sequence[str] = (),
) -> Optional[str]:
    """Poll until app CDC reappears (after Leave). Returns device path or None."""
    deadline = time.monotonic() + timeout_s
    excl = set(exclude)
    while time.monotonic() < deadline:
        for p in list_cdc_ports(serial=serial):
            if p.is_stm32_cdc and p.device not in excl:
                return p.device
        time.sleep(0.2)
    return None


def wait_for_dfu(
    *,
    serial: Optional[str] = None,
    timeout_s: float = 10.0,
) -> bool:
    """Poll until 0483:DF11 is visible to libusb."""
    try:
        import usb1
    except ImportError as exc:
        raise RuntimeError(
            "wait_for_dfu needs libusb1 (pip install libusb1)"
        ) from exc

    ctx = usb1.USBContext()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _find_dfu(ctx, serial=serial) is not None:
            return True
        time.sleep(0.15)
    return False


def enter_bootloader(
    connection: Optional["Connection"] = None,
    *,
    confirm: bool = False,
    port: Optional[str] = None,
    serial: Optional[str] = None,
) -> str:
    """Send DFU! over app CDC. Auto-picks the CDC port if ``connection`` is None.

    Returns the device path used. Requires confirm=True.
    """
    if not confirm:
        raise ValueError(
            "enter_bootloader() requires confirm=True — this resets the board "
            "into the ROM bootloader (CDC will drop)"
        )

    if connection is not None:
        frame = bytearray(build_plant_command(connection.next_seq()))
        frame[PDU_OFF : PDU_OFF + len(_TAG)] = _TAG
        connection.write_raw(bytes(frame))
        return getattr(connection, "port", port or "")

    # Standalone path: find CDC, open briefly, send, close.
    from deft_controls_sdk.link import Connection

    device = find_cdc_port(port=port, serial=serial)
    conn = Connection.connect(device)
    try:
        frame = bytearray(build_plant_command(conn.next_seq()))
        frame[PDU_OFF : PDU_OFF + len(_TAG)] = _TAG
        conn.write_raw(bytes(frame))
    finally:
        try:
            conn.close()
        except Exception:
            pass
    return device


def leave_bootloader(
    *,
    serial: Optional[str] = None,
    address: int = _LEAVE_VT_ADDR,
    timeout_s: float = 8.0,
) -> bool:
    """Exit ROM DFU via AN3156 Leave (default: reset trampoline @ 0x0803F800).

    Works on Windows (WinUSB/ST DFU driver) and Linux (libusb + udev access to
    0483:df11). Returns True if DF11 disappeared within ``timeout_s``.
    """
    try:
        import usb1
    except ImportError as exc:
        raise RuntimeError(
            "leave_bootloader needs libusb1 (pip install libusb1)"
        ) from exc

    ctx = usb1.USBContext()
    dev = _find_dfu(ctx, serial=serial)
    if dev is None:
        raise RuntimeError(
            "no STM32 DFU device (0483:DF11) — enter bootloader first or "
            "pass serial= to disambiguate"
            + (
                " (Linux: check `lsusb` and udev rules for 0483:df11)"
                if host_os() == "linux"
                else ""
            )
        )

    handle = dev.open()
    try:
        _claim_dfu_interface(handle)
        _dfu_leave(handle, address=address)
    finally:
        try:
            handle.releaseInterface(0)
        except Exception:
            pass
        try:
            handle.close()
        except Exception:
            pass

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _find_dfu(ctx, serial=serial) is None:
            return True
        time.sleep(0.1)
    return _find_dfu(ctx, serial=serial) is None


def _claim_dfu_interface(handle) -> None:
    """Claim DFU interface 0; on Linux detach kernel driver if bound."""
    os_name = host_os()
    try:
        if handle.kernelDriverActive(0):
            handle.detachKernelDriver(0)
    except Exception:
        # Windows WinUSB often has no kernel driver to detach; Linux may raise
        # if already detached.
        if os_name == "linux":
            try:
                handle.detachKernelDriver(0)
            except Exception:
                pass
    handle.claimInterface(0)


def _find_dfu(ctx, *, serial: Optional[str]):
    for d in ctx.getDeviceList(skip_on_error=True):
        if d.getVendorID() != _DFU_VID or d.getProductID() != _DFU_PID:
            continue
        try:
            sn = d.getSerialNumber()
        except Exception:
            sn = None
        if serial is None or sn == serial:
            return d
    return None


def _dfu_leave(handle, *, address: int) -> None:
    """AN3156 §5.5 Leave DFU — must GETSTATUS after SET_ADDRESS.

    Coerces the device into dfuIDLE / dfuDNLOAD_IDLE without assuming a single
    recovery path: already-idle is fine; UPLOAD_IDLE tries ABORT then falls
    back to CLRSTATUS / wait if ABORT is ignored or errors.
    """

    def dnload(wvalue: int, data: bytes = b'') -> None:
        handle.controlWrite(0x21, DFU_DNLOAD, wvalue, 0, data, timeout=5000)

    def get_status():
        st = handle.controlRead(0xA1, DFU_GETSTATUS, 0, 0, 6, timeout=5000)
        state = st[4]
        poll_ms = st[1] | (st[2] << 8) | (st[3] << 16)
        return state, poll_ms, st

    def clr_status() -> bool:
        try:
            handle.controlWrite(0x21, DFU_CLRSTATUS, 0, 0, b'', timeout=5000)
            return True
        except Exception:
            return False

    def abort() -> bool:
        try:
            handle.controlWrite(0x21, DFU_ABORT, 0, 0, b'', timeout=5000)
            return True
        except Exception:
            return False

    def is_ready(state: int, *, allow_dnload_idle: bool) -> bool:
        if state == _STATE_IDLE:
            return True
        return allow_dnload_idle and state == _STATE_DNLOAD_IDLE

    def wait_idle(*, allow_dnload_idle: bool = True) -> int:
        """Poll until IDLE (or DNLOAD_IDLE). Escalate recovery, don't assume ABORT."""
        last_state = -1
        stuck = 0
        state = -1
        st = [0] * 6
        for i in range(100):
            try:
                state, poll_ms, st = get_status()
            except Exception:
                if i > 0:
                    return _STATE_IDLE
                raise

            if is_ready(state, allow_dnload_idle=allow_dnload_idle):
                return state

            if state == last_state:
                stuck += 1
            else:
                stuck = 0
                last_state = state

            if state in (_STATE_DNBUSY, _STATE_DNLOAD_SYNC, _STATE_MANIFEST,
                         _STATE_MANIFEST_SYNC, _STATE_MANIFEST_WAIT_RESET):
                time.sleep(max(poll_ms, 1) / 1000.0)
                continue

            if state == _STATE_ERROR:
                clr_status()
                time.sleep(0.01)
                continue

            if state == _STATE_UPLOAD_IDLE or stuck >= 2:
                abort_ok = abort()
                time.sleep(0.01)
                try:
                    state2, _, _ = get_status()
                except Exception:
                    return _STATE_IDLE
                if is_ready(state2, allow_dnload_idle=allow_dnload_idle):
                    return state2
                if not abort_ok or state2 == state:
                    clr_status()
                    time.sleep(0.01)
                continue

            time.sleep(max(poll_ms, 1) / 1000.0)

        raise TimeoutError(f"DFU not idle, last state={state} status={list(st)}")

    wait_idle()

    addr_cmd = bytes([0x21]) + struct.pack("<I", address & 0xFFFFFFFF)
    for attempt in range(2):
        try:
            dnload(0, addr_cmd)
        except Exception:
            wait_idle()
            if attempt == 0:
                continue
            raise
        try:
            state, poll_ms, _ = get_status()
        except Exception:
            return
        if state == _STATE_ERROR:
            clr_status()
            wait_idle()
            continue
        if state in (_STATE_DNBUSY, _STATE_DNLOAD_SYNC):
            time.sleep(max(poll_ms, 1) / 1000.0)
            try:
                get_status()
            except Exception:
                return
        break

    wait_idle()

    try:
        dnload(0, b'')
    except Exception:
        return
    try:
        state, poll_ms, _ = get_status()
        if state in (_STATE_MANIFEST, _STATE_MANIFEST_SYNC,
                     _STATE_MANIFEST_WAIT_RESET):
            time.sleep(max(poll_ms, 1) / 1000.0)
            try:
                get_status()
            except Exception:
                return
        elif is_ready(state, allow_dnload_idle=True):
            try:
                dnload(0, b'')
            except Exception:
                return
    except Exception:
        return
