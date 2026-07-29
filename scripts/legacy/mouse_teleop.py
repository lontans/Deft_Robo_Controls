#!/usr/bin/env python3
"""In-person mouse INPUT only → TeleopSample / RTC blob (no PCB).

For driving the Damiao arm on laptop CDC, use ``mouse_arm_teleop.py`` instead
(progressive latch + Claude-2 KD + J2 frozen).

Tuned for Logitech M650 (left/right/middle, scroll, 2 thumb keys).
Emits TeleopSample (JSON) and optionally the 122-byte RTC control_data_t+checksum
blob that deft_rtc_bridge expects on the `control` data channel.

Default mapping (right-hand stand-in; left arm zeroed):
  Middle hold     deadman (B) — required to apply mouse motion / scroll to pose
  Move (deadman)  integrate into rc_pos.x/y  (+ r_stick from velocity)
  Scroll          rc_pos.z
  Left hold       right_index = 1  (index trigger / gripper proxy)
  Right hold      right_middle = 1 (hand trigger)
  Thumb back      X
  Thumb forward   A
  Esc             quit

Usage:
  python scripts/mouse_teleop.py
  python scripts/mouse_teleop.py --ndjson
  python scripts/mouse_teleop.py --rtc-bin out.rtc.ndbin   # length-prefixed 122B frames
  python scripts/mouse_teleop.py --self-test
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import threading
import time
from dataclasses import asdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from quest_udp_sniff import (  # noqa: E402
    RTC_PACKET_SIZE,
    TeleopSample,
    pack_rtc122,
    print_console,
)

try:
    from pynput import mouse
except ImportError as e:  # pragma: no cover
    print("Need pynput:  python -m pip install pynput", file=sys.stderr)
    raise SystemExit(2) from e


IDENTITY_QUAT = [0.0, 0.0, 0.0, 1.0]


class MouseTeleopState:
    """M650 state. Joystick mode (default): middle click sets stick origin;
    mouse offset from that point is left/right (sx) and up/down (sy) in [-1, 1].
    """

    def __init__(
        self,
        *,
        xy_scale: float = 0.001,
        z_scale: float = 0.01,
        stick_scale: float = 0.05,
        stick_radius_px: float = 120.0,
        stick_deadzone: float = 0.12,
        mode: str = "joystick",
    ) -> None:
        self.lock = threading.Lock()
        self.xy_scale = xy_scale
        self.z_scale = z_scale
        self.stick_scale = stick_scale
        self.stick_radius_px = max(float(stick_radius_px), 1.0)
        self.stick_deadzone = float(stick_deadzone)
        self.mode = mode  # "joystick" | "integrate"
        self.rc_pos = [0.0, 0.0, 0.0]
        self.last_pos: tuple[int, int] | None = None
        self.origin: tuple[int, int] | None = None
        self.cur_pos: tuple[int, int] | None = None
        self.stick = [0.0, 0.0]  # sx=left/right, sy=up/down
        self.vel = [0.0, 0.0]
        self.left = False
        self.right = False
        self.middle = False
        self.thumb_back = False  # XBUTTON1
        self.thumb_fwd = False  # XBUTTON2
        self.quit = False
        self.scroll_notches = 0.0

    def _recompute_stick(self) -> None:
        if not self.middle or self.origin is None or self.cur_pos is None:
            self.stick = [0.0, 0.0]
            return
        dx = self.cur_pos[0] - self.origin[0]
        dy = self.cur_pos[1] - self.origin[1]
        # Screen y down -> stick up positive
        sx = dx / self.stick_radius_px
        sy = -dy / self.stick_radius_px
        mag = (sx * sx + sy * sy) ** 0.5
        if mag < self.stick_deadzone:
            self.stick = [0.0, 0.0]
            return
        if mag > 1.0:
            sx /= mag
            sy /= mag
            mag = 1.0
        # Rescale so deadzone edge -> 0, rim -> 1
        scale = (mag - self.stick_deadzone) / (1.0 - self.stick_deadzone)
        self.stick = [sx / mag * scale, sy / mag * scale]

    def on_move(self, x: int, y: int) -> None:
        with self.lock:
            self.cur_pos = (x, y)
            if self.mode == "joystick":
                if self.middle and self.origin is None:
                    self.origin = (x, y)
                self._recompute_stick()
                self.vel = list(self.stick)
                return
            # Legacy integrate mode (pixel deltas)
            if self.last_pos is None:
                self.last_pos = (x, y)
                return
            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            self.last_pos = (x, y)
            if self.middle:
                self.rc_pos[0] += dx * self.xy_scale
                self.rc_pos[1] -= dy * self.xy_scale
                self.vel[0] = dx * self.stick_scale
                self.vel[1] = -dy * self.stick_scale
            else:
                self.vel[0] = 0.0
                self.vel[1] = 0.0

    def on_click(self, x: int, y: int, button: mouse.Button, pressed: bool) -> None:
        with self.lock:
            if button == mouse.Button.left:
                self.left = pressed
            elif button == mouse.Button.right:
                self.right = pressed
            elif button == mouse.Button.middle:
                self.middle = pressed
                if pressed:
                    self.origin = (x, y)
                    self.cur_pos = (x, y)
                    self._recompute_stick()
                else:
                    self.origin = None
                    self.stick = [0.0, 0.0]
                    self.vel = [0.0, 0.0]
            elif button == mouse.Button.x1:
                self.thumb_back = pressed
            elif button == mouse.Button.x2:
                self.thumb_fwd = pressed

    def on_scroll(self, _x: int, _y: int, _dx: int, dy: int) -> None:
        with self.lock:
            if self.middle:
                self.rc_pos[2] += dy * self.z_scale
                self.scroll_notches += float(dy)

    def sample(self) -> TeleopSample:
        with self.lock:
            if self.mode == "joystick":
                self._recompute_stick()
                sx, sy = float(self.stick[0]), float(self.stick[1])
            else:
                sx = max(-1.0, min(1.0, self.vel[0]))
                sy = max(-1.0, min(1.0, self.vel[1]))
                self.vel[0] *= 0.7
                self.vel[1] *= 0.7
            return TeleopSample(
                t_unix=time.time(),
                src="mouse_m650",
                head_pos=[0.0, 0.0, 0.0],
                head_rot_euler=[0.0, 0.0, 0.0],
                lc_pos=[0.0, 0.0, 0.0],
                lc_rot_quat=list(IDENTITY_QUAT),
                rc_pos=list(self.rc_pos),
                rc_rot_quat=list(IDENTITY_QUAT),
                l_stick=[0.0, 0.0],
                r_stick=[sx, sy],
                a=self.thumb_fwd,
                b=self.middle,
                x=self.thumb_back,
                y=False,
                left_index=0.0,
                right_index=1.0 if self.left else 0.0,
                left_middle=0.0,
                right_middle=1.0 if self.right else 0.0,
                packet_bytes=RTC_PACKET_SIZE,
                left_stick_press=False,
                right_stick_press=False,
            )


def self_test() -> int:
    s = TeleopSample(
        t_unix=0.0,
        src="self",
        head_pos=[0.0, 0.0, 0.0],
        head_rot_euler=[0.0, 0.0, 0.0],
        lc_pos=[0.0, 0.0, 0.0],
        lc_rot_quat=list(IDENTITY_QUAT),
        rc_pos=[0.1, 0.2, 0.3],
        rc_rot_quat=list(IDENTITY_QUAT),
        l_stick=[0.0, 0.0],
        r_stick=[0.5, -0.25],
        a=True,
        b=True,
        x=False,
        y=False,
        left_index=0.0,
        right_index=1.0,
        left_middle=0.0,
        right_middle=0.5,
        packet_bytes=RTC_PACKET_SIZE,
        left_stick_press=False,
        right_stick_press=True,
    )
    blob = pack_rtc122(s)
    if len(blob) != RTC_PACKET_SIZE:
        print("FAIL size", len(blob), file=sys.stderr)
        return 1
    payload, csum = blob[:118], struct.unpack("<I", blob[118:])[0]
    calc = 0
    for b in payload:
        calc ^= b
    if csum != calc:
        print("FAIL checksum", csum, calc, file=sys.stderr)
        return 1
    # spot-check right pos at float offset: head6 + left7 = 13 floats → byte 52
    rx, ry, rz = struct.unpack_from("<fff", payload, 13 * 4)
    if abs(rx - 0.1) > 1e-5 or abs(ry - 0.2) > 1e-5 or abs(rz - 0.3) > 1e-5:
        print("FAIL rc_pos", rx, ry, rz, file=sys.stderr)
        return 1
    print(f"SELF-TEST PASS  rtc_packet={RTC_PACKET_SIZE}B  schema=TeleopSample")
    return 0


def run(
    *,
    hz: float,
    ndjson: bool,
    rtc_bin: str | None,
    xy_scale: float,
    z_scale: float,
    stick_scale: float,
) -> int:
    state = MouseTeleopState(
        xy_scale=xy_scale, z_scale=z_scale, stick_scale=stick_scale
    )
    listener = mouse.Listener(
        on_move=state.on_move,
        on_click=state.on_click,
        on_scroll=state.on_scroll,
    )
    listener.start()

    bin_f = open(rtc_bin, "ab") if rtc_bin else None
    print(
        "Mouse teleop (M650). Hold MIDDLE=deadman, move/scroll to drive rc_pos; "
        "L=index R=hand; thumb-back=X thumb-fwd=A. Ctrl+C to stop.",
        flush=True,
    )
    n = 0
    t0 = time.time()
    period = 1.0 / max(hz, 1.0)
    try:
        while listener.running:
            sample = state.sample()
            n += 1
            elapsed = max(time.time() - t0, 1e-6)
            if ndjson:
                print(json.dumps(asdict(sample), separators=(",", ":")), flush=True)
            else:
                print_console(sample, n, n / elapsed)
            if bin_f is not None:
                blob = pack_rtc122(sample)
                bin_f.write(struct.pack("<I", len(blob)) + blob)
                if n % int(hz) == 0:
                    bin_f.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\nStopped. samples={n}", flush=True)
        return 0 if n > 0 else 2
    finally:
        listener.stop()
        if bin_f is not None:
            bin_f.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="M650 mouse → TeleopSample / RTC control blob")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--ndjson", action="store_true")
    p.add_argument(
        "--rtc-bin",
        default=None,
        help="append length-prefixed 122B RTC frames to this file",
    )
    p.add_argument(
        "--xy-scale",
        type=float,
        default=0.001,
        help="meters (or unit) per pixel while deadman held",
    )
    p.add_argument("--z-scale", type=float, default=0.01, help="units per scroll notch")
    p.add_argument(
        "--stick-scale",
        type=float,
        default=0.05,
        help="r_stick gain from pixel delta",
    )
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args(argv)
    if args.self_test:
        return self_test()
    return run(
        hz=args.hz,
        ndjson=args.ndjson,
        rtc_bin=args.rtc_bin,
        xy_scale=args.xy_scale,
        z_scale=args.z_scale,
        stick_scale=args.stick_scale,
    )


if __name__ == "__main__":
    raise SystemExit(main())
