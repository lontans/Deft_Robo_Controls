#!/usr/bin/env python3
"""Minimal CANable sniffer — same core path as sniff_can.py (what worked on CH1)."""
from __future__ import annotations

import argparse
import struct
import sys
import time

import can
import usb.core
from usb.backend import libusb1
from libusb._platform import DLL_PATH

_BACKEND = libusb1.get_backend(find_library=lambda _: str(DLL_PATH))
if _BACKEND is None:
    sys.exit("pip install libusb libusb1")

import gs_usb.gs_usb as _gs_mod
from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_LISTEN_ONLY
from gs_usb.gs_usb_frame import GsUsbFrame


def _scan_patched(cls):
    return [
        _gs_mod.GsUsb(dev)
        for dev in usb.core.find(
            find_all=True,
            custom_match=_gs_mod.GsUsb.is_gs_usb_device,
            backend=_BACKEND,
        )
        or []
    ]


_gs_mod.GsUsb.scan = classmethod(_scan_patched)


def _prepare(dev: usb.core.Device) -> None:
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass
    dev.set_configuration()
    try:
        dev.ctrl_transfer(0x41, 0, 0, 0, struct.pack("<I", 0x0000_BEEF))
    except usb.core.USBError:
        pass


def _solve_timing(fclk: int, bitrate: int) -> can.BitTiming | None:
    for sp in (87.5, 80.0, 75.0, 90.0, 68.0):
        try:
            return can.BitTiming.from_sample_point(
                f_clock=fclk, bitrate=bitrate, sample_point=sp,
            )
        except ValueError:
            continue
    return None


def _apply_timing(gs: _gs_mod.GsUsb, bitrate: int) -> str:
    fclk = gs.device_capability.fclk_can
    bt = _solve_timing(fclk, bitrate)
    if bt is None:
        raise SystemExit(f"No timing for {bitrate} @ fclk={fclk}")
    prop = 1
    gs.set_timing(
        prop_seg=prop,
        phase_seg1=bt.tseg1 - prop,
        phase_seg2=bt.tseg2,
        sjw=bt.sjw,
        brp=bt.brp,
    )
    q = 1 + prop + (bt.tseg1 - prop) + bt.tseg2
    actual = fclk / (bt.brp * q)
    ppm = (actual - bitrate) / bitrate * 1e6
    return f"req={bitrate} actual={actual:.0f} bps ({ppm:+.0f} ppm) brp={bt.brp}"


def _format_frame(frame: GsUsbFrame) -> str:
    ext = "ext" if frame.is_extended_id else "std"
    data = bytes(frame.data[: frame.can_dlc]).hex(" ") if frame.can_dlc else ""
    # listen-only: everything on the bus is unlabeled direction; call it RX like sniff_can.py
    return f"  {frame.timestamp:12.4f}  RX  {ext}  ID=0x{frame.arbitration_id:08X}  DLC={frame.can_dlc}  {data}"


def main() -> int:
    p = argparse.ArgumentParser(
        description="CANable sniffer — defaults to listen-only bus RX.",
    )
    p.add_argument("--bitrate", type=int, default=1_000_000)
    p.add_argument(
        "--normal",
        action="store_true",
        help="Normal mode (not recommended — shows adapter echo garbage)",
    )
    p.add_argument(
        "--hw-timestamp",
        action="store_true",
        help="24-byte USB frames (off by default — matches working sniff_can.py)",
    )
    args = p.parse_args()

    devs = _gs_mod.GsUsb.scan()
    if not devs:
        sys.exit("No CANable — close Python, unplug/replug USB, wait 5s")

    gs = devs[0]
    try:
        _prepare(gs.gs_usb)
        fclk = gs.device_capability.fclk_can
        timing_s = _apply_timing(gs, args.bitrate)
        flags = GS_CAN_MODE_LISTEN_ONLY if not args.normal else 0
        if args.hw_timestamp:
            flags |= GS_CAN_MODE_HW_TIMESTAMP
        gs.start(flags=flags)
    except usb.core.USBError as exc:
        if getattr(exc, "errno", None) == 13:
            sys.exit("USB access denied — kill python, replug CANable")
        raise

    mode = "normal" if args.normal else "listen-only"
    print(f"Device: {gs}")
    print(f"  fclk_can={fclk} Hz")
    print(f"CANable  {timing_s}  mode={mode}")
    print("  If bus_rx=0: use sniff_can.py --listen-only (known-good), check GND, replug USB")
    print("  Terminal 2: python scripts/rs02_can_scan.py --port COM5 --probe-id 0x70 --bus 1\n")

    frame = GsUsbFrame()
    n_rx = 0
    t_hint = time.monotonic()

    try:
        while True:
            if not gs.read(frame, timeout_ms=250):
                if n_rx == 0 and (time.monotonic() - t_hint) > 5.0:
                    print("  … waiting (run probe in other terminal)", flush=True)
                    t_hint = time.monotonic()
                continue

            n_rx += 1
            print(_format_frame(frame), flush=True)

    except KeyboardInterrupt:
        print(f"\nbus_rx={n_rx}")
    finally:
        gs.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
