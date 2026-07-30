"""Soft-DFU flash helpers — USB ROM DFU with ST-Link SWD fallback.

Firmware: App/Src/host/soft_dfu.c / App/Inc/host/soft_dfu.h.

Enter: plant CMD with ``stm32_mode=SOFT_DFU`` (ADR-004); legacy DEBUG tag
``DFU!`` still accepted by firmware. Leave: AN3156 Leave DFU.

One-shot: ``python scripts/soft_dfu_flash.py``. If soft-enter drops CDC but
``0483:DF11`` never appears, CubeProg ST-Link SWD is used automatically when
present so the board is not left bricked mid-flash.

Port discovery is OS-aware (Windows COMx / Linux /dev/ttyACM*) and prefers
STM32 USB CDC 0483:5740 — never hard-code COM5.
"""
from __future__ import annotations

import argparse
import os
import platform
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

from deft_controls_sdk.link.exchange.wire_layout import (
    STM32_USB_CDC_PID,
    STM32_VID,
)

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection

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


def _port_key(device: str) -> str:
    """Normalize COM / tty path for equality (Windows COM is case-insensitive)."""
    d = (device or "").strip()
    if host_os() == "windows":
        return d.upper()
    return d


def cdc_info_for_port(port: str) -> Optional[CdcPortInfo]:
    """Look up ``list_cdc_ports()`` row for ``port`` (e.g. ``COM5``), or None."""
    key = _port_key(port)
    for row in list_cdc_ports():
        if _port_key(row.device) == key:
            return row
    return None


def resolve_flash_identity(
    *,
    port: Optional[str] = None,
    serial: Optional[str] = None,
) -> Tuple[Optional[str], Optional[str]]:
    """Normalize ``(port, serial)`` for Soft-DFU enter + DFU re-enum.

    If only ``port`` is given (e.g. ``COM5``), resolve the USB serial number
    from that CDC device so DF11 programming still targets the same board
    after CDC drops. Explicit ``serial`` wins when both are set; mismatch
    vs the port's USB SN raises.
    """
    if port is None:
        return None, serial
    info = cdc_info_for_port(port)
    if info is None:
        return port, serial
    if serial is not None and info.serial and serial != info.serial:
        raise RuntimeError(
            f"port {port!r} has USB serial {info.serial!r}, not {serial!r}"
        )
    return port, serial or info.serial


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


def wait_for_dfu_access(
    *,
    serial: Optional[str] = None,
    timeout_s: float = 5.0,
) -> bool:
    """Poll until 0483:DF11 can be opened (udev/sudo perms applied).

    Listing can succeed while open still returns ACCESS. Only open+close —
    do not claim the interface (that races with dfu-util).
    """
    try:
        import usb1
    except ImportError as exc:
        raise RuntimeError(
            "wait_for_dfu_access needs libusb1 (pip install libusb1)"
        ) from exc

    ctx = usb1.USBContext()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        dev = _find_dfu(ctx, serial=serial)
        if dev is not None:
            handle = None
            try:
                handle = dev.open()
                return True
            except Exception:
                pass
            finally:
                if handle is not None:
                    try:
                        handle.close()
                    except Exception:
                        pass
        time.sleep(0.15)
    return False


def enter_bootloader(
    connection: Optional["Connection"] = None,
    *,
    confirm: bool = False,
    port: Optional[str] = None,
    serial: Optional[str] = None,
) -> str:
    """Enter ROM DFU via stm32_mode=SOFT_DFU (preferred) or legacy DFU! tag.

    Returns the device path used. Requires confirm=True.
    """
    if not confirm:
        raise ValueError(
            "enter_bootloader() requires confirm=True — this resets the board "
            "into the ROM bootloader (CDC will drop)"
        )

    from deft_controls_sdk.link.exchange.pack import (
        build_plant_command,
        patch_system_stm32_mode,
    )
    from deft_controls_sdk.link.exchange.wire_layout import STM32_MODE_SOFT_DFU

    def _soft_dfu_frame(seq: int) -> bytes:
        buf = bytearray(build_plant_command(seq))
        patch_system_stm32_mode(buf, STM32_MODE_SOFT_DFU)
        return bytes(buf)

    if connection is not None:
        connection.write_raw(_soft_dfu_frame(connection.next_seq()))
        return getattr(connection, "port", port or "")

    # Standalone path: find CDC, open briefly, send, close.
    from deft_controls_sdk.link import Connection

    device = find_cdc_port(port=port, serial=serial)
    conn = Connection.connect(device)
    try:
        conn.write_raw(_soft_dfu_frame(conn.next_seq()))
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


def _pick_firmware_elf(root: Path) -> Path:
    """Newest of ``Release/`` / ``Debug/`` ``DeftRoboticsControlsPCB.elf`` under root."""
    release = root / "Release" / "DeftRoboticsControlsPCB.elf"
    debug = root / "Debug" / "DeftRoboticsControlsPCB.elf"
    if release.is_file() and debug.is_file():
        return release if release.stat().st_mtime >= debug.stat().st_mtime else debug
    if release.is_file():
        return release
    return debug


def default_firmware_elf() -> Path:
    """Pick the newest of ``Release/`` / ``Debug/`` ``DeftRoboticsControlsPCB.elf``.

    See docs/legacy/rfc/rfc-release-build.md — production flashes should use Release when
    that build is current; if Debug was rebuilt more recently, use that so
    ``python scripts/soft_dfu_flash.py`` (no flags) flashes what you just built.
    """
    # scripts/deft_controls_sdk/debug/soft_dfu.py → repo root
    return _pick_firmware_elf(Path(__file__).resolve().parents[3])


def _which_objcopy() -> str:
    for name in ("arm-none-eabi-objcopy", "objcopy", "arm-none-eabi-objcopy.exe"):
        path = shutil.which(name)
        if path:
            return path
    raise RuntimeError(
        "need objcopy for dfu-util path (apt: binutils / gcc-arm-none-eabi). "
        "On Windows without dfu-util, install STM32CubeProgrammer instead "
        "(ELF is flashed directly — no objcopy)."
    )


def elf_to_bin(elf: Path, bin_path: Path) -> Path:
    """Convert full ELF → raw binary (pads gaps — slow for dfu-util)."""
    cmd = [_which_objcopy(), "-O", "binary", str(elf), str(bin_path)]
    subprocess.run(cmd, check=True)
    if not bin_path.is_file() or bin_path.stat().st_size == 0:
        raise RuntimeError(f"objcopy produced empty/missing bin: {bin_path}")
    return bin_path


def elf_to_dfu_parts(elf: Path, app_bin: Path, leave_bin: Path) -> Tuple[Path, Path]:
    """Split ELF for fast dfu-util: app image + leave VT (no 254KB gap pad).

    The leave trampoline lives at 0x0803F800; a naive ``objcopy -O binary``
    pads everything from flash base → that address (~260KB). CubeProg skips
    the gap; we match that by flashing two compact blobs.
    """
    obj = _which_objcopy()
    subprocess.run(
        [
            obj,
            "-O",
            "binary",
            "--remove-section=.soft_dfu_leave_vt",
            str(elf),
            str(app_bin),
        ],
        check=True,
    )
    subprocess.run(
        [
            obj,
            "-O",
            "binary",
            "--only-section=.soft_dfu_leave_vt",
            str(elf),
            str(leave_bin),
        ],
        check=True,
    )
    if not app_bin.is_file() or app_bin.stat().st_size == 0:
        raise RuntimeError(f"objcopy produced empty app bin: {app_bin}")
    if not leave_bin.is_file() or leave_bin.stat().st_size == 0:
        raise RuntimeError(
            f"objcopy produced empty leave VT bin: {leave_bin} "
            "(missing .soft_dfu_leave_vt in ELF?)"
        )
    return app_bin, leave_bin


def _which_dfu_util() -> Optional[str]:
    return shutil.which("dfu-util") or shutil.which("dfu-util.exe")


def _which_cubeprog() -> Optional[str]:
    for name in ("STM32_Programmer_CLI", "STM32_Programmer_CLI.exe"):
        path = shutil.which(name)
        if path:
            return path

    exe = (
        "STM32_Programmer_CLI.exe"
        if host_os() == "windows"
        else "STM32_Programmer_CLI"
    )
    fixed = [
        Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
        / "STMicroelectronics"
        / "STM32Cube"
        / "STM32CubeProgrammer"
        / "bin"
        / exe,
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
        / "STMicroelectronics"
        / "STM32Cube"
        / "STM32CubeProgrammer"
        / "bin"
        / exe,
        Path(os.environ.get("LOCALAPPDATA", ""))
        / "Programs"
        / "STM32Cube"
        / "STM32CubeProgrammer"
        / "bin"
        / exe,
        Path("/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin")
        / exe,
        Path("/opt/st/stm32cubeprogrammer/bin") / exe,
    ]
    for c in fixed:
        if c.is_file():
            return str(c)

    # CubeIDE bundles CubeProg under plugins/…cubeprogrammer…/tools/bin/
    ide_roots = [
        Path(r"C:\ST"),
        Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
        / "STMicroelectronics",
        Path("/opt/st"),
        Path.home() / "stm32cubeide",
    ]
    for root in ide_roots:
        if not root.is_dir():
            continue
        try:
            for plug in root.glob("**/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin"):
                candidate = plug / exe
                if candidate.is_file():
                    return str(candidate)
        except OSError:
            continue
    return None


_DFU_UDEV_HINT = """\
Linux cannot open 0483:df11 (permissions). ATTR was too narrow on some
Jetson/udev builds — use ATTRS + uaccess (repo file):

  sudo cp scripts/udev/99-stm32-dfu.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=usb --action=add
  # unplug/replug once, then re-run soft_dfu_flash.py

Or flash once with sudo (board still in DFU):

  sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D /tmp/controls.bin
"""


def _dfu_util_permission_error(text: str, returncode: int) -> bool:
    if returncode == 74:  # LIBUSB_ERROR_ACCESS / cannot open
        return True
    t = text.lower()
    return "cannot open dfu" in t or "no dfu capable usb device" in t


def _dfu_util_flash(bin_path: Path, *, serial: Optional[str], address: int) -> None:
    dfu = _which_dfu_util()
    if not dfu:
        raise RuntimeError(
            "dfu-util not found — install it (apt: dfu-util) or use "
            "STM32_Programmer_CLI on Windows"
        )
    cmd = [
        dfu,
        "-d",
        f"{_DFU_VID:04x}:{_DFU_PID:04x}",
        "-a",
        "0",
        # Do NOT append :leave — that jumps to 0x08000000 and can kill USB CDC.
        "-s",
        f"0x{address:08X}",
        "-D",
        str(bin_path),
    ]
    if serial:
        cmd.extend(["-S", serial])

    def _run(argv: List[str]) -> int:
        # Stream live — capture_output hides progress and looks hung on Ctrl+C.
        print("+", " ".join(argv), flush=True)
        return subprocess.run(argv, check=False).returncode

    rc = _run(cmd)
    if rc == 0:
        return

    # soft_dfu_flash.sh already runs as root; skip nested sudo there.
    already_root = hasattr(os, "geteuid") and os.geteuid() == 0
    if (
        host_os() == "linux"
        and not already_root
        and _dfu_util_permission_error("", rc)
    ):
        sudo = shutil.which("sudo")
        if sudo:
            for sudo_argv in (
                [sudo, "-n", *cmd],
                [sudo, *cmd],
            ):
                print(
                    "dfu-util could not open device — retrying with sudo…",
                    flush=True,
                )
                rc = _run(sudo_argv)
                if rc == 0:
                    return
        raise RuntimeError(_DFU_UDEV_HINT.strip())

    if host_os() == "linux" and _dfu_util_permission_error("", rc):
        raise RuntimeError(_DFU_UDEV_HINT.strip())

    raise subprocess.CalledProcessError(rc, cmd)


def _cubeprog_flash_usb(image: Path, *, serial: Optional[str]) -> None:
    cli = _which_cubeprog()
    if not cli:
        raise RuntimeError(
            "STM32_Programmer_CLI not found — install STM32CubeProgrammer "
            "or add it to PATH"
        )
    connect = f"port=USB1 sn={serial}" if serial else "port=USB1"
    cmd = [cli, "-c", connect, "-w", str(image), "-v"]
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


# Back-compat alias for tests / callers.
_cubeprog_flash = _cubeprog_flash_usb


def _cubeprog_swd_probe(cli: Optional[str] = None) -> bool:
    """True if CubeProg can open an ST-Link SWD target (mode=UR)."""
    prog = cli or _which_cubeprog()
    if not prog:
        return False
    cmd = [prog, "-c", "port=SWD", "mode=UR", "-q"]
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=12,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    text = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode != 0:
        return False
    # CubeProg prints "Error" on connect failure even with rc quirks.
    low = text.lower()
    if "error:" in low and "no stm32" in low:
        return False
    if "unable to get core id" in low or "no st-link" in low:
        return False
    return True


def _cubeprog_flash_swd(image: Path) -> None:
    cli = _which_cubeprog()
    if not cli:
        raise RuntimeError(
            "STM32_Programmer_CLI not found — install STM32CubeProgrammer"
        )
    cmd = [
        cli,
        "-c",
        "port=SWD",
        "mode=UR",
        "-w",
        str(image),
        "-v",
        "-rst",
    ]
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def _flash_via_swd(image: Path, *, serial: Optional[str]) -> str:
    """Program over ST-Link SWD and wait for app CDC."""
    print(f"flashing via ST-Link SWD: {image}", flush=True)
    _cubeprog_flash_swd(image)
    cdc = wait_for_cdc(serial=serial, timeout_s=15.0)
    if not cdc:
        raise RuntimeError(
            "SWD flash finished but app CDC did not reappear — "
            "check USB device cable / power"
        )
    print(f"flash ok (SWD) — CDC at {cdc}", flush=True)
    try:
        post_flash_listen_pdu(cdc)
    except Exception as exc:  # noqa: BLE001
        print(f"post-flash listen_pdu warn: {exc}", flush=True)
    return cdc


def _program_usb_dfu(
    img: Path,
    *,
    serial: Optional[str],
    flash_address: int,
    use_cube: bool,
    use_dfu_util: bool,
) -> None:
    already_root = hasattr(os, "geteuid") and os.geteuid() == 0
    if use_dfu_util and host_os() == "linux" and not already_root:
        if not wait_for_dfu_access(serial=serial, timeout_s=2.0):
            print(
                "warning: DFU visible but not openable yet — "
                "dfu-util may need sudo / better udev rules",
                flush=True,
            )

    tmp_paths: List[Path] = []
    try:
        if use_cube:
            print(f"flashing with STM32_Programmer_CLI USB DFU: {img}", flush=True)
            _cubeprog_flash_usb(img, serial=serial)
        elif img.suffix.lower() == ".bin":
            _dfu_util_flash(img, serial=serial, address=flash_address)
        else:
            fd_a, name_a = tempfile.mkstemp(prefix="soft_dfu_app_", suffix=".bin")
            fd_l, name_l = tempfile.mkstemp(prefix="soft_dfu_leave_", suffix=".bin")
            os.close(fd_a)
            os.close(fd_l)
            app_bin, leave_bin = Path(name_a), Path(name_l)
            tmp_paths.extend([app_bin, leave_bin])
            print(f"objcopy (split, no gap pad) {img}", flush=True)
            elf_to_dfu_parts(img, app_bin, leave_bin)
            print(
                f"  app {app_bin.stat().st_size} B @ 0x{flash_address:08X}; "
                f"leave VT {leave_bin.stat().st_size} B @ 0x{_LEAVE_VT_ADDR:08X}",
                flush=True,
            )
            _dfu_util_flash(app_bin, serial=serial, address=flash_address)
            _dfu_util_flash(leave_bin, serial=serial, address=_LEAVE_VT_ADDR)
    finally:
        for p in tmp_paths:
            try:
                p.unlink()
            except OSError:
                pass


def flash_firmware(
    image: Optional[os.PathLike[str] | str] = None,
    *,
    port: Optional[str] = None,
    serial: Optional[str] = None,
    flash_address: int = _APP_FLASH_BASE,
    confirm: bool = True,
    require_usb_dfu: bool = False,
) -> str:
    """One-shot flash: soft-DFU USB when possible, else ST-Link SWD fallback.

    ``image`` may be an ``.elf`` or ``.bin``. Default: repo Release ELF if
    present, else Debug ELF (see ``default_firmware_elf``).
    Returns the CDC device path after the app re-enumerates.

    Pass ``port`` (e.g. ``COM5``) and/or USB ``serial``. Port alone is enough:
    the USB serial is resolved from that CDC device for DFU re-enumeration.

    Order:
      1. If ``0483:DF11`` already present → USB program + Leave
      2. Else soft-enter from CDC → wait for DF11 → USB program + Leave
      3. If DF11 never appears and CubeProg can open ST-Link SWD → SWD
         program + reset (recovery only). Disabled when
         ``require_usb_dfu=True`` (prove loops / production USB-only).
    """
    if not confirm:
        raise ValueError("flash_firmware() requires confirm=True")

    port, serial = resolve_flash_identity(port=port, serial=serial)
    if port is not None:
        print(f"target CDC {port}" + (f" sn={serial}" if serial else ""), flush=True)

    img = Path(image) if image is not None else default_firmware_elf()
    if not img.is_file():
        raise FileNotFoundError(
            f"firmware not found: {img} — build in CubeIDE or pass --image"
        )

    cube = _which_cubeprog()
    use_cube = cube is not None
    use_dfu_util = (not use_cube) and (_which_dfu_util() is not None)
    if not use_dfu_util and not use_cube:
        raise RuntimeError(
            "no flasher found — install dfu-util (Linux) or STM32CubeProgrammer"
        )

    swd_ok = bool(use_cube and _cubeprog_swd_probe(cube)) and not require_usb_dfu
    # Option-byte soft-enter + DF11 re-enum needs a few seconds; keep a long
    # wait for USB-only proves. When SWD recovery is allowed, fail faster.
    dfu_wait_s = 12.0 if require_usb_dfu or not swd_ok else 5.0

    def _no_swd(msg: str) -> RuntimeError:
        return RuntimeError(
            f"{msg}. USB-only Soft-DFU required"
            + (" (--require-usb-dfu)." if require_usb_dfu else ".")
            + " Windows: app CDC must be usbser (not WinUSB on 0483:5740);"
            + " DF11 needs WinUSB/ST DFU. Linux: udev"
            + " scripts/udev/99-stm32-dfu.rules or soft_dfu_flash.sh."
            + " ST-Link SWD is recovery-only — re-run without"
            + " --require-usb-dfu if you need it."
        )

    in_dfu = wait_for_dfu(serial=serial, timeout_s=0.5)
    if not in_dfu:
        try:
            entered = enter_bootloader(confirm=True, port=port, serial=serial)
            print(f"soft-entered DFU from {entered}", flush=True)
        except RuntimeError as exc:
            if swd_ok:
                print(
                    f"no app CDC ({exc}); recovering via ST-Link SWD…",
                    flush=True,
                )
                return _flash_via_swd(img, serial=serial)
            raise _no_swd(f"no app CDC for soft-enter ({exc})") from exc

        if not wait_for_dfu(serial=serial, timeout_s=dfu_wait_s):
            print(
                "0483:DF11 did not appear after soft enter "
                "(CDC dropped; ROM DFU not visible to host)",
                flush=True,
            )
            if swd_ok:
                print("falling back to ST-Link SWD…", flush=True)
                return _flash_via_swd(img, serial=serial)
            raise _no_swd("DFU device 0483:DF11 did not appear")
    else:
        print("already in DFU (0483:DF11)", flush=True)

    try:
        _program_usb_dfu(
            img,
            serial=serial,
            flash_address=flash_address,
            use_cube=use_cube,
            use_dfu_util=use_dfu_util,
        )
    except Exception as exc:
        if swd_ok:
            print(f"USB DFU program failed ({exc}); falling back to SWD…", flush=True)
            return _flash_via_swd(img, serial=serial)
        raise

    print("leaving DFU via reset trampoline…", flush=True)
    if not leave_bootloader(serial=serial):
        if swd_ok:
            print("Leave DFU timed out — recovering via ST-Link SWD…", flush=True)
            return _flash_via_swd(img, serial=serial)
        raise _no_swd("Leave DFU timed out — DF11 still present")

    cdc = wait_for_cdc(serial=serial, timeout_s=12.0)
    if not cdc:
        if swd_ok:
            print("CDC missing after Leave — recovering via ST-Link SWD…", flush=True)
            return _flash_via_swd(img, serial=serial)
        raise _no_swd(
            "app CDC did not reappear after Leave — power-cycle the board"
        )
    print(f"flash ok — CDC at {cdc}", flush=True)
    try:
        post_flash_listen_pdu(cdc)
    except Exception as exc:  # noqa: BLE001 — flash succeeded; LED settle is best-effort
        print(f"post-flash listen_pdu warn: {exc}", flush=True)
    return cdc


def post_flash_listen_pdu(port: str, *, hold_s: float = 0.35) -> None:
    """After Soft-DFU, stage NVM ``listen_pdu`` + host LedDesire follow.

    Strip then follows PDU traffic-light (no peer / HARD → blink-red).
    Needs the new app image already running. Safe no-op if COM busy.
    """
    # Late import: soft_dfu must stay importable without hub side-effects.
    from deft_controls_sdk.host_proxy import HostProxy
    from deft_controls_sdk.link import LedDesire
    from deft_controls_sdk.vbeta.cfg import pause_plant_stream

    print(f"staging listen_pdu on {port}…", flush=True)
    # mode=debug required for CFG; do not Soft-DFU from this path.
    with HostProxy.connect(
        port,
        stream_hz=50.0,
        telemetry_hz=20.0,
        idle_first=True,
        listen_pdu=True,
        mode="debug",
    ) as proxy:
        proxy.set_led(LedDesire(mode="follow", master_brightness=8), send=True)
        try:
            with pause_plant_stream(proxy.hub):
                periph = proxy.hub.debug.cfg_get_periph()
                periph["listen_pdu"] = True
                proxy.hub.debug.cfg_set_periph(periph, persist=False)
        except Exception as exc:  # noqa: BLE001 — older FW may lack GET_PERIPH
            print(f"  CFG listen_pdu warn: {exc}", flush=True)
        proxy.listen_pdu = True
        proxy.sleep(float(hold_s))
    print("listen_pdu staged (LedDesire follow)", flush=True)


# Back-compat alias (old Soft-DFU callers / tests).
post_flash_idle_led = post_flash_listen_pdu


def pcb_status(
    *,
    port: Optional[str] = None,
    serial: Optional[str] = None,
    listen_pdu: bool = False,
    stream_hz: float = 200.0,
) -> dict:
    """Open the board briefly and return HostProxy.doctor() (read-only).

    Includes stream rates, MCU host command, PDB (honored only if
    ``listen_pdu``), and inferred LED. Same payload as
    ``python -m pcb_lab doctor``. Does not flash or write NVM.
    """
    # Late import: avoid soft_dfu ↔ hub import cycles at module load.
    from deft_controls_sdk.host_proxy import HostProxy

    port, serial = resolve_flash_identity(port=port, serial=serial)
    device = find_cdc_port(port=port, serial=serial)
    with HostProxy.connect(
        device,
        stream_hz=float(stream_hz),
        telemetry_hz=float(stream_hz),
        idle_first=True,
        listen_pdu=bool(listen_pdu),
    ) as proxy:
        report = proxy.doctor()
        report["usb_serial"] = serial
        info = cdc_info_for_port(device)
        if info is not None:
            report["usb_serial"] = info.serial
            report["cdc_description"] = info.description
        return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI: ``python scripts/soft_dfu_flash.py`` (defaults to flash)."""
    p = argparse.ArgumentParser(
        prog="soft_dfu_flash",
        description=(
            "Flash Controls PCB firmware. Default path: soft-DFU over USB, "
            "with automatic ST-Link SWD fallback when DF11 is not visible."
        ),
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    f = sub.add_parser(
        "flash",
        help="program firmware (soft-DFU, SWD fallback) — default for soft_dfu_flash.py",
    )
    f.add_argument(
        "--image",
        default=None,
        help="path to .elf or .bin (default: Release/*.elf if present, else Debug)",
    )
    f.add_argument(
        "--port",
        default=None,
        help="app CDC port (e.g. COM5); USB serial is resolved for DFU re-enum",
    )
    f.add_argument(
        "--serial",
        default=None,
        help="USB serial of the target board (only needed with multiple boards)",
    )
    f.add_argument(
        "--address",
        default=f"0x{_APP_FLASH_BASE:08X}",
        help=argparse.SUPPRESS,  # advanced dfu-util base; keep hidden
    )
    f.add_argument(
        "--require-usb-dfu",
        action="store_true",
        help=(
            "fail instead of falling back to ST-Link SWD "
            "(USB-only Soft-DFU prove loops)"
        ),
    )

    e = sub.add_parser("enter", help="reset board into ROM DFU")
    e.add_argument("--serial", default=None)
    e.add_argument("--port", default=None)

    l = sub.add_parser("leave", help="AN3156 Leave to reset trampoline")
    l.add_argument("--serial", default=None)
    l.add_argument(
        "--port",
        default=None,
        help="optional CDC port used only to resolve USB serial for DFU Leave",
    )

    s = sub.add_parser("scan", help="list STM32 CDC / DFU presence")
    s.add_argument("--serial", default=None)
    s.add_argument(
        "--port",
        default=None,
        help="highlight / filter to this CDC port and print its USB serial",
    )

    st = sub.add_parser(
        "status",
        help="query PCB host state (HostProxy doctor — read-only, needs app CDC)",
    )
    st.add_argument("--port", default=None, help="CDC port (e.g. COM5)")
    st.add_argument("--serial", default=None, help="USB serial if multiple boards")
    st.add_argument(
        "--listen-pdu",
        action="store_true",
        help="honor PDB kill in status/LED (default: off; wire still under pdb.wire)",
    )
    st.add_argument(
        "--stream-hz",
        type=float,
        default=200.0,
        help="plant stream rate while status is open (default 200)",
    )

    args = p.parse_args(list(argv) if argv is not None else None)

    if args.cmd == "scan":
        port, serial = resolve_flash_identity(
            port=getattr(args, "port", None), serial=args.serial
        )
        print(f"host: {host_os()}  cubeprog: {_which_cubeprog() or '(none)'}  "
              f"dfu-util: {_which_dfu_util() or '(none)'}")
        if port is not None:
            info = cdc_info_for_port(port)
            if info is None:
                print(f"  port {port}: (not currently enumerated)")
            else:
                print(
                    f"  port {info.device} → sn={info.serial}  "
                    f"stm32_cdc={info.is_stm32_cdc}  {info.description}"
                )
        for row in list_cdc_ports(serial=serial):
            if port is not None and _port_key(row.device) != _port_key(port):
                continue
            mark = "CDC" if row.is_stm32_cdc else "   "
            print(f"  [{mark}] {row.device}  sn={row.serial}  "
                  f"vid={row.vid} pid={row.pid}  {row.description}")
        try:
            present = wait_for_dfu(serial=serial, timeout_s=0.3)
        except RuntimeError as exc:
            print(f"  DFU: ({exc})")
        else:
            print(f"  DFU 0483:DF11: {'yes' if present else 'no'}")
        if _which_cubeprog():
            print(
                f"  ST-Link SWD: {'yes' if _cubeprog_swd_probe() else 'no'}"
            )
        return 0

    if args.cmd == "status":
        import json

        report = pcb_status(
            port=args.port,
            serial=args.serial,
            listen_pdu=bool(args.listen_pdu),
            stream_hz=float(args.stream_hz),
        )
        print(json.dumps(report, indent=2))
        return 0

    if args.cmd == "enter":
        port, serial = resolve_flash_identity(port=args.port, serial=args.serial)
        print(enter_bootloader(confirm=True, port=port, serial=serial))
        return 0

    if args.cmd == "leave":
        _port, serial = resolve_flash_identity(
            port=getattr(args, "port", None), serial=args.serial
        )
        ok = leave_bootloader(serial=serial)
        print("left" if ok else "timeout")
        return 0 if ok else 1

    if args.cmd == "flash":
        addr = int(args.address, 0)
        flash_firmware(
            args.image,
            port=args.port,
            serial=args.serial,
            flash_address=addr,
            confirm=True,
            require_usb_dfu=bool(args.require_usb_dfu),
        )
        return 0

    p.error(f"unknown cmd {args.cmd}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
