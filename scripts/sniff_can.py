#!/usr/bin/env python3
"""Sniff CAN with MKS CANable (candleLight / gs_usb). Ctrl+C to stop.

On Windows, gs_usb forces libusb1.get_backend() which fails unless libusb-1.0.dll
is on PATH. pip install libusb fixes that; this script points pyusb at the DLL.
"""
from __future__ import annotations

import argparse
import sys

import can
import usb.core
from usb.backend import libusb1
from libusb._platform import DLL_PATH

_BACKEND = libusb1.get_backend(find_library=lambda _: str(DLL_PATH))
if _BACKEND is None:
    sys.exit(
        "No USB backend. Run:  pip install libusb libusb1\n"
        "Then retry."
    )

# gs_usb hardcodes libusb1.get_backend() (returns None on Windows without the DLL).
import gs_usb.gs_usb as _gs_usb_mod
from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_LISTEN_ONLY
from gs_usb.gs_usb_frame import GS_USB_NONE_ECHO_ID, GsUsbFrame


def _scan_patched(cls):
    return [
        _gs_usb_mod.GsUsb(dev)
        for dev in usb.core.find(
            find_all=True,
            custom_match=_gs_usb_mod.GsUsb.is_gs_usb_device,
            backend=_BACKEND,
        )
        or []
    ]


def _find_patched(cls, bus, address):
    dev = usb.core.find(
        custom_match=_gs_usb_mod.GsUsb.is_gs_usb_device,
        bus=bus,
        address=address,
        backend=_BACKEND,
    )
    return _gs_usb_mod.GsUsb(dev) if dev else None


_gs_usb_mod.GsUsb.scan = classmethod(_scan_patched)
_gs_usb_mod.GsUsb.find = classmethod(_find_patched)


def _usb_open_hint(exc: Exception) -> str:
    err = str(exc).lower()
    if "access denied" in err or "errno 13" in err or getattr(exc, "errno", None) == 13:
        return (
            "\nUSB access denied. Try:\n"
            "  1. Close other programs using the CANable (other Python, SavvyCAN, etc.)\n"
            "  2. Unplug/replug the CANable USB cable\n"
            "  3. In Zadig: reinstall WinUSB on interface 0 (canable2 gs_usb / MI_00)\n"
            "  4. Run PowerShell as Administrator\n"
        )
    return ""


def _prepare_device(raw_dev: usb.core.Device) -> None:
    try:
        if raw_dev.is_kernel_driver_active(0):
            raw_dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass
    try:
        raw_dev.set_configuration()
    except usb.core.USBError as exc:
        raise SystemExit(f"Cannot open CANable: {exc}{_usb_open_hint(exc)}") from exc


def _describe_device(raw_dev: usb.core.Device) -> str:
    return f"VID={raw_dev.idVendor:04X} PID={raw_dev.idProduct:04X} bus={raw_dev.bus} addr={raw_dev.address}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Sniff CAN via gs_usb (candleLight CANable).")
    p.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate (default 1 Mbps)")
    p.add_argument("--channel", type=int, default=0, help="gs_usb device index (default 0)")
    p.add_argument(
        "--listen-only",
        action="store_true",
        help="Passive listen (no ACK); recommended when tapping the bus",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    devs = _gs_usb_mod.GsUsb.scan()
    if args.channel >= len(devs):
        print(f"No gs_usb device at index {args.channel} (found {len(devs)}).", file=sys.stderr)
        return 1

    gs = devs[args.channel]
    raw = gs.gs_usb
    print(f"Device: {_describe_device(raw)}")
    _prepare_device(raw)
    try:
        fclk = gs.device_capability.fclk_can
    except usb.core.USBError as exc:
        raise SystemExit(f"Cannot read CANable capability: {exc}{_usb_open_hint(exc)}") from exc
    bit_timing = can.BitTiming.from_sample_point(
        f_clock=fclk,
        bitrate=args.bitrate,
        sample_point=87.5,
    )
    props_seg = 1
    gs.set_timing(
        prop_seg=props_seg,
        phase_seg1=bit_timing.tseg1 - props_seg,
        phase_seg2=bit_timing.tseg2,
        sjw=bit_timing.sjw,
        brp=bit_timing.brp,
    )

    flags = GS_CAN_MODE_HW_TIMESTAMP
    if args.listen_only:
        flags |= GS_CAN_MODE_LISTEN_ONLY
    gs.start(flags=flags)

    mode = "listen-only" if args.listen_only else "normal"
    print(f"Listening @ {args.bitrate // 1000} kbps ({mode}). Ctrl+C to stop.\n")
    frame = GsUsbFrame()
    try:
        while True:
            if not gs.read(frame, timeout_ms=100):
                continue
            ext = "ext" if frame.is_extended_id else "std"
            data = bytes(frame.data[: frame.can_dlc]).hex(" ")
            ts = frame.timestamp
            rx = frame.echo_id == GS_USB_NONE_ECHO_ID
            dir_s = "RX" if rx else "TX"
            print(
                f"  {ts:12.4f}  {dir_s}  {ext}  "
                f"ID=0x{frame.arbitration_id:08X}  DLC={frame.can_dlc}  {data}"
            )
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        gs.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
