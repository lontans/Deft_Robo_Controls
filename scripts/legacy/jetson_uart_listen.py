#!/usr/bin/env python3
"""Listen on Jetson serial ports for MCU UART4 probe traffic.

Opens every plausible UART node at 115200 8N1 and reports which ones see
bytes (and whether UART4_PROBE / PDBC / 0x55 show up).

Usage:
  python3 jetson_uart_listen.py
  python3 jetson_uart_listen.py --seconds 5 --ports /dev/ttyTHS1 /dev/ttyTHS2
"""
from __future__ import annotations

import argparse
import glob
import sys
import time


def default_ports() -> list[str]:
    patterns = (
        "/dev/ttyTHS*",
        "/dev/ttyTCU*",
        "/dev/ttyS*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    )
    out: list[str] = []
    for pat in patterns:
        out.extend(sorted(glob.glob(pat)))
    # de-dupe, keep order
    seen = set()
    uniq = []
    for p in out:
        if p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


def classify(buf: bytes) -> str:
    tags = []
    if b"UART4_PROBE" in buf:
        tags.append("UART4_PROBE")
    if b"PDBC" in buf:
        tags.append("PDBC")
    if b"PDBF" in buf:
        tags.append("PDBF")
    if buf and sum(1 for b in buf if b == 0x55) > len(buf) // 4:
        tags.append("0x55_heavy")
    nz = sum(1 for b in buf if b)
    if buf and nz == 0:
        tags.append("all_zero")
    elif buf and nz < max(1, len(buf) // 50):
        tags.append("mostly_zero")
    return ",".join(tags) if tags else ("activity" if buf else "silent")


def listen_one(port: str, baud: int, seconds: float) -> tuple[bytes, str]:
    import serial

    try:
        s = serial.Serial(port, baud, timeout=0.05)
    except Exception as ex:
        return b"", f"OPEN_FAIL:{ex}"
    try:
        s.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = s.read(4096)
            if chunk:
                buf.extend(chunk)
            time.sleep(0.005)
        return bytes(buf), classify(bytes(buf))
    finally:
        s.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--baud-sweep",
        action="store_true",
        help="Also try common bauds on ttyTHS1/THS2",
    )
    ap.add_argument("--ports", nargs="*", default=None)
    args = ap.parse_args()

    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial required", file=sys.stderr)
        return 2

    ports = args.ports or [p for p in default_ports() if "THS" in p]
    bauds = [args.baud]
    if args.baud_sweep:
        for b in (9600, 57600, 115200, 230400, 460800, 921600):
            if b not in bauds:
                bauds.append(b)

    print(f"ports={ports} bauds={bauds} seconds={args.seconds:.1f}")
    any_hit = False
    for baud in bauds:
        print(f"\n=== baud {baud} ===")
        for p in ports:
            buf, tag = listen_one(p, baud, args.seconds)
            if tag.startswith("OPEN_FAIL"):
                print(f".... {p}: {tag}")
                continue
            head = buf[:40].hex() if buf else ""
            ascii_head = "".join(chr(b) if 32 <= b < 127 else "." for b in buf[:48])
            # Real probe hit: ASCII beacon or heavy 0x55, not NUL flood
            real = ("UART4_PROBE" in tag) or ("0x55_heavy" in tag) or ("PDBC" in tag)
            noise = len(buf) > 0 and not real
            if real:
                any_hit = True
                mark = "HIT "
            elif noise:
                mark = "NOIZ"
            else:
                mark = "...."
            print(
                f"{mark} {p}: n={len(buf)} nz={sum(1 for b in buf if b)} {tag}"
            )
            if buf:
                print(f"     hex {head}")
                print(f"     asc {ascii_head!r}")

    return 0 if any_hit else 1


if __name__ == "__main__":
    sys.exit(main())
