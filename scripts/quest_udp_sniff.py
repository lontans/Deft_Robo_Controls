#!/usr/bin/env python3
"""Minimal Quest→PC UDP sniff (no ROS, no PCB).

Matches docs/deft_vbeta_ref/deft_vbeta/src/udp_bridge/src/vr_udp_bridge.cpp
combined binary packet (exactly 116 bytes, little-endian floats).

Layout:
  VrState   20×f32  head_pos[3], head_rot[3], lc_pos[3], lc_rot[4], rc_pos[3], rc_rot[4]
  Axis2d     4×f32  l_tx, l_ty, r_tx, r_ty
  Buttons    4×u8   A, B, X, Y  (nonzero = pressed)
  Triggers   4×f32  left_index, right_index, left_middle, right_middle

Usage:
  python scripts/quest_udp_sniff.py
  python scripts/quest_udp_sniff.py --port 5000 --ndjson
  python scripts/quest_udp_sniff.py --self-test
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time
from dataclasses import asdict, dataclass
from typing import Optional

PACKET_SIZE = 116
# 20 pose floats + 4 axis floats + 4 button bytes + 4 trigger floats
_POSE_AXIS_FMT = "<" + ("f" * 24)
_TRIG_FMT = "<ffff"


@dataclass
class TeleopSample:
    """Logical teleop sample matching deft_vbeta VrState/Axis2d/ButtonState
    and deft_rtc_bridge control_data_t fields (poses + sticks + triggers + buttons).
    """

    t_unix: float
    src: str
    head_pos: list[float]
    head_rot_euler: list[float]
    lc_pos: list[float]
    lc_rot_quat: list[float]  # x,y,z,w
    rc_pos: list[float]
    rc_rot_quat: list[float]  # x,y,z,w
    l_stick: list[float]  # tx, ty
    r_stick: list[float]
    a: bool  # RTC button1
    b: bool  # RTC button2
    x: bool  # RTC button3
    y: bool  # RTC button4
    left_index: float
    right_index: float
    left_middle: float
    right_middle: float
    packet_bytes: int
    left_stick_press: bool = False
    right_stick_press: bool = False


def unpack_udp116(data: bytes, *, t_unix: float, src: str) -> TeleopSample:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE} bytes, got {len(data)}")
    floats = struct.unpack_from(_POSE_AXIS_FMT, data, 0)
    buttons = data[96:100]
    triggers = struct.unpack_from(_TRIG_FMT, data, 100)
    return TeleopSample(
        t_unix=t_unix,
        src=src,
        head_pos=list(floats[0:3]),
        head_rot_euler=list(floats[3:6]),
        lc_pos=list(floats[6:9]),
        lc_rot_quat=list(floats[9:13]),
        rc_pos=list(floats[13:16]),
        rc_rot_quat=list(floats[16:20]),
        l_stick=list(floats[20:22]),
        r_stick=list(floats[22:24]),
        a=buttons[0] != 0,
        b=buttons[1] != 0,
        x=buttons[2] != 0,
        y=buttons[3] != 0,
        left_index=triggers[0],
        right_index=triggers[1],
        left_middle=triggers[2],
        right_middle=triggers[3],
        packet_bytes=len(data),
    )


def pack_udp116(sample: TeleopSample) -> bytes:
    """Legacy UDP bridge layout (116B). Stick presses are not in this blob."""
    floats = (
        sample.head_pos
        + sample.head_rot_euler
        + sample.lc_pos
        + sample.lc_rot_quat
        + sample.rc_pos
        + sample.rc_rot_quat
        + sample.l_stick
        + sample.r_stick
    )
    body = struct.pack(_POSE_AXIS_FMT, *floats)
    buttons = bytes(
        [
            1 if sample.a else 0,
            1 if sample.b else 0,
            1 if sample.x else 0,
            1 if sample.y else 0,
        ]
    )
    trig = struct.pack(
        _TRIG_FMT,
        sample.left_index,
        sample.right_index,
        sample.left_middle,
        sample.right_middle,
    )
    out = body + buttons + trig
    assert len(out) == PACKET_SIZE
    return out


# Matches deft_rtc_bridge control_data_t (packed) + 4-byte XOR checksum = 122B.
RTC_CONTROL_SIZE = 118
RTC_PACKET_SIZE = 122
_RTC_FLOATS_FMT = "<" + ("f" * 28)  # head6 + left7 + right7 + sticks4 + trigs4


def pack_rtc122(sample: TeleopSample) -> bytes:
    """Pack TeleopSample as deft_rtc control blob (118B payload + 4B checksum)."""
    floats = (
        list(sample.head_pos)
        + list(sample.head_rot_euler)
        + list(sample.lc_pos)
        + list(sample.lc_rot_quat)
        + list(sample.rc_pos)
        + list(sample.rc_rot_quat)
        + list(sample.l_stick)
        + list(sample.r_stick)
        + [
            sample.left_index,
            sample.right_index,
            sample.left_middle,
            sample.right_middle,
        ]
    )
    assert len(floats) == 28
    body = struct.pack(_RTC_FLOATS_FMT, *floats)
    buttons = bytes(
        [
            1 if sample.a else 0,
            1 if sample.b else 0,
            1 if sample.x else 0,
            1 if sample.y else 0,
            1 if sample.left_stick_press else 0,
            1 if sample.right_stick_press else 0,
        ]
    )
    payload = body + buttons
    assert len(payload) == RTC_CONTROL_SIZE
    checksum = 0
    for b in payload:
        checksum ^= b
    return payload + struct.pack("<I", checksum)


def _fmt_vec(v: list[float], n: int = 3) -> str:
    return "[" + ", ".join(f"{x: .3f}" for x in v[:n]) + "]"


def print_console(s: TeleopSample, n: int, hz: float) -> None:
    btns = "".join(
        name
        for name, on in (("A", s.a), ("B", s.b), ("X", s.x), ("Y", s.y))
        if on
    ) or "-"
    print(
        f"#{n:6d}  {hz:5.1f} Hz  "
        f"L{_fmt_vec(s.lc_pos)}  R{_fmt_vec(s.rc_pos)}  "
        f"stickL{_fmt_vec(s.l_stick, 2)}  "
        f"idx L={s.left_index:.2f} R={s.right_index:.2f}  "
        f"btn={btns}  from={s.src}",
        flush=True,
    )


def _f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def self_test() -> int:
    # Values chosen / quantized so they survive float32 round-trip.
    gold = TeleopSample(
        t_unix=0.0,
        src="self",
        head_pos=[_f32(0.1), _f32(0.2), _f32(0.3)],
        head_rot_euler=[_f32(0.01), _f32(0.02), _f32(0.03)],
        lc_pos=[1.0, 2.0, 3.0],
        lc_rot_quat=[0.0, 0.0, 0.0, 1.0],
        rc_pos=[-1.0, -2.0, -3.0],
        rc_rot_quat=[0.0, 0.0, 0.0, 1.0],
        l_stick=[0.5, -0.5],
        r_stick=[_f32(-0.25), _f32(0.75)],
        a=True,
        b=False,
        x=True,
        y=False,
        left_index=_f32(0.1),
        right_index=_f32(0.9),
        left_middle=_f32(0.2),
        right_middle=_f32(0.8),
        packet_bytes=PACKET_SIZE,
    )
    raw = pack_udp116(gold)
    got = unpack_udp116(raw, t_unix=0.0, src="self")
    keys = [
        "head_pos",
        "head_rot_euler",
        "lc_pos",
        "lc_rot_quat",
        "rc_pos",
        "rc_rot_quat",
        "l_stick",
        "r_stick",
        "a",
        "b",
        "x",
        "y",
        "left_index",
        "right_index",
        "left_middle",
        "right_middle",
        "packet_bytes",
    ]
    bad = [k for k in keys if getattr(got, k) != getattr(gold, k)]
    if bad:
        print("SELF-TEST FAIL:", bad, file=sys.stderr)
        return 1
    print(f"SELF-TEST PASS  packet={PACKET_SIZE}B  schema=TeleopSample")
    return 0


def serve(host: str, port: int, *, ndjson: bool, idle_s: float) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(0.5)
    print(
        f"Listening UDP {host}:{port} for {PACKET_SIZE}-byte Quest blobs "
        f"(Ctrl+C to stop). Point Quest app at this host.",
        flush=True,
    )
    n = 0
    t0 = time.time()
    last_pkt = t0
    last_sizes: dict[int, int] = {}
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                if n == 0 and (time.time() - t0) > idle_s and int(time.time() - t0) % 5 == 0:
                    # light keepalive hint every ~5s while idle
                    pass
                if n == 0 and time.time() - last_pkt > 5.0:
                    print(
                        f"... still waiting (no packets yet). "
                        f"Quest should send to this PC IP:{port}",
                        flush=True,
                    )
                    last_pkt = time.time()
                continue

            last_pkt = time.time()
            last_sizes[len(data)] = last_sizes.get(len(data), 0) + 1
            if len(data) != PACKET_SIZE:
                print(
                    f"WARN unexpected size={len(data)} from {addr[0]}:{addr[1]} "
                    f"(want {PACKET_SIZE}; seen_sizes={last_sizes})",
                    flush=True,
                )
                continue

            n += 1
            elapsed = max(time.time() - t0, 1e-6)
            sample = unpack_udp116(
                data, t_unix=time.time(), src=f"{addr[0]}:{addr[1]}"
            )
            if ndjson:
                print(json.dumps(asdict(sample), separators=(",", ":")), flush=True)
            else:
                print_console(sample, n, n / elapsed)
    except KeyboardInterrupt:
        print(f"\nStopped. packets_ok={n} size_hist={last_sizes}", flush=True)
        return 0 if n > 0 else 2
    finally:
        sock.close()


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Quest UDP 116-byte control sniff")
    p.add_argument("--host", default="0.0.0.0", help="bind address")
    p.add_argument("--port", type=int, default=5000, help="UDP port (vr_udp_bridge default)")
    p.add_argument("--ndjson", action="store_true", help="emit one TeleopSample JSON per line")
    p.add_argument("--idle-hint-s", type=float, default=5.0, dest="idle_s")
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args(argv)
    if args.self_test:
        return self_test()
    return serve(args.host, args.port, ndjson=args.ndjson, idle_s=args.idle_s)


if __name__ == "__main__":
    raise SystemExit(main())
