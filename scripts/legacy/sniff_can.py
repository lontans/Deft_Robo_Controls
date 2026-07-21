#!/usr/bin/env python3
"""CAN sniffer for MKS CANable (candleLight / gs_usb).

Baseline: gs.read() + brief lines (what worked on CH1 originally).
Keeps Windows libusb fix + arbitrary bitrate timing for baud hunts.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

import can
import usb.core
from usb.backend import libusb1
from libusb._platform import DLL_PATH

_BACKEND = libusb1.get_backend(find_library=lambda _: str(DLL_PATH))
if _BACKEND is None:
    sys.exit("No USB backend. Run:  pip install libusb libusb1")

import gs_usb.gs_usb as _gs_usb_mod
from gs_usb.constants import (
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
    GS_CAN_MODE_HW_TIMESTAMP,
    GS_CAN_MODE_LISTEN_ONLY,
)
from gs_usb.gs_usb_frame import GS_USB_NONE_ECHO_ID, GsUsbFrame

_GS_USB_BREQ_HOST_FORMAT = 0


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


_gs_usb_mod.GsUsb.scan = classmethod(_scan_patched)


def _usb_open_hint(exc: Exception) -> str:
    if getattr(exc, "errno", None) == 13 or "access denied" in str(exc).lower():
        return (
            "\nUSB access denied — close other Python/sniffer apps, unplug/replug CANable."
        )
    return ""


def _prepare_device(dev: usb.core.Device) -> None:
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass
    try:
        dev.set_configuration()
    except usb.core.USBError as exc:
        raise SystemExit(f"Cannot open CANable: {exc}{_usb_open_hint(exc)}") from exc
    # candleLight expects little-endian host format (linux gs_usb driver does this too).
    try:
        dev.ctrl_transfer(0x41, _GS_USB_BREQ_HOST_FORMAT, 0, 0, struct.pack("<I", 0x0000_BEEF))
    except usb.core.USBError:
        pass


@dataclass(frozen=True)
class _GsTiming:
    prop_seg: int
    phase_seg1: int
    phase_seg2: int
    sjw: int
    brp: int
    actual_bps: float
    sample_point_pct: float


def _solve_gs_timing(fclk: int, bitrate: int, prop_seg: int = 1) -> _GsTiming | None:
    target = float(bitrate)
    best: _GsTiming | None = None
    best_err = float("inf")
    for brp in range(1, 1025):
        for ph1 in range(1, 64):
            for ph2 in range(1, 17):
                q = 1 + prop_seg + ph1 + ph2
                actual = fclk / (brp * q)
                err = abs(actual - target)
                if err >= best_err:
                    continue
                sp = (1 + prop_seg + ph1) / q * 100.0
                if sp < 65.0 or sp > 92.0:
                    continue
                best_err = err
                best = _GsTiming(prop_seg, ph1, ph2, min(4, max(1, ph2)), brp, actual, sp)
    if best is None or best_err / target > 0.02:
        return None
    return best


def _bitrate_timing(fclk: int, bitrate: int) -> _GsTiming | None:
    for sp in (87.5, 80.0, 75.0, 90.0, 68.0, 92.0):
        try:
            bt = can.BitTiming.from_sample_point(
                f_clock=fclk, bitrate=bitrate, sample_point=sp,
            )
            prop_seg = 1
            ph1 = bt.tseg1 - prop_seg
            ph2 = bt.tseg2
            q = 1 + prop_seg + ph1 + ph2
            return _GsTiming(
                prop_seg, ph1, ph2, bt.sjw, bt.brp, fclk / (bt.brp * q),
                (1 + prop_seg + ph1) / q * 100.0,
            )
        except ValueError:
            continue
    return _solve_gs_timing(fclk, bitrate)


def _apply_bitrate(gs: _gs_usb_mod.GsUsb, bitrate: int) -> _GsTiming | None:
    timing = _bitrate_timing(gs.device_capability.fclk_can, bitrate)
    if timing is None:
        return None
    gs.set_timing(
        prop_seg=timing.prop_seg,
        phase_seg1=timing.phase_seg1,
        phase_seg2=timing.phase_seg2,
        sjw=timing.sjw,
        brp=timing.brp,
    )
    return timing


def _start_gs(gs: _gs_usb_mod.GsUsb, *, listen_only: bool, hw_timestamp: bool) -> None:
    flags = GS_CAN_MODE_LISTEN_ONLY if listen_only else 0
    if hw_timestamp:
        flags |= GS_CAN_MODE_HW_TIMESTAMP
    gs.start(flags=flags)


def _format_brief(frame: GsUsbFrame) -> str:
    kind = "ext" if frame.is_extended_id else "std"
    if frame.is_error_frame:
        kind = "err"
    data = bytes(frame.data[: frame.can_dlc]).hex(" ") if frame.can_dlc else ""
    rx = "RX" if frame.echo_id == GS_USB_NONE_ECHO_ID else "TX"
    return (
        f"  {frame.timestamp:12.4f}  {rx}  {kind}  "
        f"ID=0x{frame.arbitration_id:08X}  DLC={frame.can_dlc}  {data}"
    )


def _cid_flags(cid: int) -> str:
    tags = []
    if cid & CAN_EFF_FLAG:
        tags.append("EFF")
    if cid & CAN_RTR_FLAG:
        tags.append("RTR")
    if cid & CAN_ERR_FLAG:
        tags.append("ERR")
    return "|".join(tags) if tags else "-"


def _format_wide(frame: GsUsbFrame, bitrate: int, seq: int) -> str:
    cid = frame.can_id & 0xFFFFFFFF
    data8 = " ".join(f"{b:02x}" for b in frame.data)
    pay = bytes(frame.data[: frame.can_dlc]).hex(" ") if frame.can_dlc else ""
    echo = "bus" if frame.echo_id == GS_USB_NONE_ECHO_ID else f"echo#{frame.echo_id:08X}"
    kind = "ext" if frame.is_extended_id else "std"
    return (
        f"#{seq:06d} @{bitrate // 1000}k ts_us={frame.timestamp_us} echo={echo} "
        f"cid=0x{cid:08X}[{_cid_flags(cid)}] arb=0x{frame.arbitration_id:08X} {kind} "
        f"dlc={frame.can_dlc} data8=[{data8}] pay=[{pay}]"
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="CANable gs_usb sniffer")
    p.add_argument("--bitrate", type=int, default=1_000_000)
    p.add_argument("--channel", type=int, default=0)
    p.add_argument(
        "--listen-only",
        action="store_true",
        default=True,
        help="Passive listen (default: on)",
    )
    p.add_argument(
        "--normal",
        action="store_true",
        help="Normal mode (adapter may ACK); default is listen-only",
    )
    p.add_argument("--hw-timestamp", action="store_true", help="Enable HW timestamp USB frames (20 vs 24 B)")
    p.add_argument("--wide", action="store_true", help="Verbose gs_usb field dump")
    p.add_argument("--heartbeat", type=float, default=5.0, help="Idle status interval s (0=off)")
    p.add_argument("--timeout-ms", type=int, default=250, help="gs.read timeout ms (default 250)")
    p.add_argument(
        "--rotate-bitrates",
        default="",
        help="Comma list — changes timing only, no USB reset between rates",
    )
    p.add_argument("--rotate-dwell", type=float, default=4.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    listen_only = not args.normal

    bitrates = [int(x.strip()) for x in args.rotate_bitrates.split(",") if x.strip()]
    if not bitrates:
        bitrates = [args.bitrate]

    devs = _gs_usb_mod.GsUsb.scan()
    if args.channel >= len(devs):
        print(f"No CANable at index {args.channel} (found {len(devs)})", file=sys.stderr)
        return 1

    gs = devs[args.channel]
    _prepare_device(gs.gs_usb)

    try:
        cap = gs.device_capability
    except usb.core.USBError as exc:
        raise SystemExit(f"Cannot read CANable: {exc}{_usb_open_hint(exc)}") from exc

    fclk = cap.fclk_can
    resolved = [b for b in bitrates if _bitrate_timing(fclk, b) is not None]
    if not resolved:
        raise SystemExit(f"No timing solution for fclk={fclk} bitrates={bitrates}")

    print(f"Device: {gs}")
    print(f"  fclk_can={fclk} Hz  fw hw from gs_usb")

    bi = 0
    timing = _apply_bitrate(gs, resolved[bi])
    assert timing is not None
    _start_gs(gs, listen_only=listen_only, hw_timestamp=args.hw_timestamp)

    mode = "listen-only" if listen_only else "normal"
    ts_mode = "24B+ts" if args.hw_timestamp else "20B"
    err_ppm = (timing.actual_bps - resolved[bi]) / resolved[bi] * 1e6
    print(
        f"  {mode}  usb_frame={ts_mode}  req={resolved[bi]} actual={timing.actual_bps:.0f} bps "
        f"({err_ppm:+.0f} ppm)  brp={timing.brp} ph1={timing.phase_seg1} ph2={timing.phase_seg2}"
    )
    if len(resolved) > 1:
        print(f"  rotate: {resolved} every {args.rotate_dwell}s")
    print("  Tip: run probe in another terminal while this listens.")
    print("  CH1 check: python scripts/rs02_can_scan.py --port COM5 --probe-id 0x70 --bus 1")
    print("Ctrl+C to stop.\n")

    frame = GsUsbFrame()
    seq = 0
    timeouts = 0
    t_last_frame = time.monotonic()
    t_last_rotate = time.monotonic()
    t_last_hb = time.monotonic()
    active = resolved[bi]

    try:
        while True:
            now = time.monotonic()
            if len(resolved) > 1 and (now - t_last_rotate) >= args.rotate_dwell:
                bi = (bi + 1) % len(resolved)
                active = resolved[bi]
                t = _apply_bitrate(gs, active)
                if t:
                    print(
                        f"\n--- {active} bps actual={t.actual_bps:.0f} ---\n",
                        flush=True,
                    )
                t_last_rotate = now

            if gs.read(frame, timeout_ms=args.timeout_ms):
                seq += 1
                t_last_frame = now
                print(_format_wide(frame, active, seq) if args.wide else _format_brief(frame), flush=True)
            else:
                timeouts += 1
                if args.heartbeat > 0 and (now - t_last_hb) >= args.heartbeat:
                    print(
                        f"~ idle {now - t_last_frame:.1f}s @{active // 1000}k "
                        f"frames={seq} timeouts={timeouts}",
                        flush=True,
                    )
                    t_last_hb = now
    except KeyboardInterrupt:
        print(f"\n--- stopped: frames={seq} timeouts={timeouts} ---")
    finally:
        gs.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
