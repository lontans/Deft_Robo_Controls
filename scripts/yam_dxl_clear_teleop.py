#!/usr/bin/env python3
"""Neck Dynamixel clear-range teleop (proven host path from ``_tmp_dxl_one.py``).

Uses ``ControlsPcbHub`` + paced ``send_once`` (not PcbRobotSession stream — that
path was not moving DXL on this bench). Native steps, plant table clamps.

Keys (focus the terminal):
  1 / 2   select pitch (id1) / yaw (id2)
  ←/→ or a/d   slew selected
  h       re-seed goals from present FB
  r       reset running min/max
  w       write JSON artifact
  q       quit

    python yam_dxl_clear_teleop.py
"""
from __future__ import annotations

import argparse
import json
import select
import sys
import termios
import time
import tty
from datetime import date
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, McuState, ServoDesire  # noqa: E402
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER  # noqa: E402
from deft_controls_sdk.link.exchange import (  # noqa: E402
    ACTUATOR_COUNT,
    parse_feedback_header,
    parse_servo_feedback,
)

# Mirrors App/Src/plant/plant_config.c servo_table[]
SERVO_CFG = (
    {"slot": 0, "id": 1, "pos_min": 1024, "pos_max": 3072, "name": "pitch"},
    {"slot": 1, "id": 2, "pos_min": 700, "pos_max": 2500, "name": "yaw"},
)
STREAM_HZ = 40.0
# tick/s cruise — cmd-only slew (no FB lead cap; FB is range/minmax only)
ARROW_VEL_TICK_S = 400.0
_CENTER = 2048
_STEPS_PER_DEG = 4096.0 / 360.0

_SESSION_DIR = _SCRIPTS / ".deft_session"


def _steps_to_deg(steps: int) -> float:
    return (int(steps) - _CENTER) / _STEPS_PER_DEG


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _drain(hub: ControlsPcbHub):
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        yield frame


def _clamp(slot: int, steps: float) -> int:
    cfg = SERVO_CFG[slot]
    return int(max(cfg["pos_min"], min(cfg["pos_max"], round(steps))))


if sys.platform == "win32":
    import msvcrt

    def _poll_keys() -> List[str]:
        out: List[str] = []
        while msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\x00", b"\xe0"):
                ch2 = msvcrt.getch()
                if ch2 == b"K":
                    out.append("left")
                elif ch2 == b"M":
                    out.append("right")
            else:
                try:
                    out.append(ch.decode("utf-8", errors="ignore").lower())
                except Exception:
                    pass
        return out

    def _arrow_dir() -> int:
        import ctypes

        u = ctypes.windll.user32
        left = bool(u.GetAsyncKeyState(0x25) & 0x8000)
        right = bool(u.GetAsyncKeyState(0x27) & 0x8000)
        if left and not right:
            return -1
        if right and not left:
            return 1
        return 0

    class _RawStdin:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

else:
    _cruise_dir = 0
    _cruise_until = 0.0

    def _nudge_cruise(direction: int, hold_s: float = 0.25) -> None:
        global _cruise_dir, _cruise_until
        _cruise_dir = int(direction)
        _cruise_until = time.perf_counter() + hold_s

    def _poll_keys() -> List[str]:
        out: List[str] = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                if select.select([sys.stdin], [], [], 0)[0]:
                    n = sys.stdin.read(1)
                    if n == "[" and select.select([sys.stdin], [], [], 0)[0]:
                        a = sys.stdin.read(1)
                        if a == "D":
                            out.append("left")
                        elif a == "C":
                            out.append("right")
                continue
            out.append(ch.lower())
        return out

    def _arrow_dir() -> int:
        if time.perf_counter() < _cruise_until:
            return _cruise_dir
        return 0

    class _RawStdin:
        def __enter__(self):
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            return self

        def __exit__(self, *a):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            return False


def _apply(hub: ControlsPcbHub, cmd: Sequence[float]) -> None:
    for i, cfg in enumerate(SERVO_CFG):
        hub.set_servo(
            cfg["slot"],
            ServoDesire(
                servo_id=int(cfg["id"]),
                native_step_position=_clamp(i, cmd[i]),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )
    _conn(hub).set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )
    _conn(hub).send_once()


def _read_fb(hub: ControlsPcbHub) -> Tuple[Optional[int], Optional[int]]:
    fb: List[Optional[int]] = [None, None]
    for raw in _drain(hub):
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        for slot in (0, 1):
            sv = parse_servo_feedback(raw, slot)
            if sv is None:
                continue
            pos = int(sv["present_position"]) & 0xFFFF
            if pos > 4095:
                pos &= 0x0FFF
            mid = int(sv.get("motor_source_id", 0) or 0)
            if mid in (0, SERVO_CFG[slot]["id"]) or pos != 0:
                fb[slot] = pos
    return fb[0], fb[1]


def _arm_present(hub: ControlsPcbHub, *, hz: float, timeout_s: float = 2.5) -> Optional[List[float]]:
    """Torque-off discover, then hold present (same as _tmp_dxl_one)."""
    dt = 1.0 / hz
    deadline = time.perf_counter() + timeout_s
    last: List[Optional[int]] = [None, None]
    next_t = time.perf_counter()
    frames = 0
    while time.perf_counter() < deadline:
        for i, cfg in enumerate(SERVO_CFG):
            if last[i] is None:
                desire = ServoDesire(
                    servo_id=int(cfg["id"]),
                    native_step_position=0,
                    torque_enable=False,
                    operating_mode=3,
                )
            else:
                desire = ServoDesire(
                    servo_id=int(cfg["id"]),
                    native_step_position=int(last[i]),
                    torque_enable=True,
                    operating_mode=3,
                )
            hub.set_servo(cfg["slot"], desire, send=False)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        _conn(hub).send_once()
        p, y = _read_fb(hub)
        frames += 1
        if p is not None:
            last[0] = p
        if y is not None:
            last[1] = y
        if last[0] is not None and last[1] is not None and frames >= 12:
            return [float(last[0]), float(last[1])]
        next_t += dt
        time.sleep(max(0.0, next_t - time.perf_counter()))
    print(f"arm: frames={frames} last={last}", flush=True)
    if last[0] is not None and last[1] is not None:
        return [float(last[0]), float(last[1])]
    return None


def _print_table(
    sel: int,
    cmd: Sequence[float],
    fb: Sequence[float],
    qmin: Sequence[float],
    qmax: Sequence[float],
) -> None:
    print("\033[H\033[J", end="")
    print(
        "DXL neck clear teleop — 1=pitch 2=yaw | ←/→ a/d | h reseed | r reset | w write | q quit",
        flush=True,
    )
    print(
        f"vel={ARROW_VEL_TICK_S:.0f} tick/s  (cmd-only ramp, FB=range only)  "
        f"table pitch {SERVO_CFG[0]['pos_min']}..{SERVO_CFG[0]['pos_max']}  "
        f"yaw {SERVO_CFG[1]['pos_min']}..{SERVO_CFG[1]['pos_max']}",
        flush=True,
    )
    print(
        f"{'ax':>6} {'sel':>3} {'cmd':>6} {'fb':>6} {'min':>6} {'max':>6} "
        f"{'span':>5} {'cmd°':>7} {'fb°':>7}",
        flush=True,
    )
    for i, cfg in enumerate(SERVO_CFG):
        mark = "*" if i == sel else " "
        span = float(qmax[i] - qmin[i])
        print(
            f"{cfg['name']:>6} {mark:>3} {int(cmd[i]):6d} {int(fb[i]):6d} "
            f"{int(qmin[i]):6d} {int(qmax[i]):6d} {span:5.0f} "
            f"{_steps_to_deg(int(cmd[i])):+7.1f} {_steps_to_deg(int(fb[i])):+7.1f}",
            flush=True,
        )


def _write_outputs(
    home: Sequence[float],
    qmin: Sequence[float],
    qmax: Sequence[float],
    *,
    port: str,
) -> None:
    _SESSION_DIR.mkdir(parents=True, exist_ok=True)
    art = {
        "date": date.today().isoformat(),
        "port": port,
        "frame": "neck DXL native steps (and deg, center 2048=0°)",
        "axes": [c["name"] for c in SERVO_CFG],
        "dxl_ids": [c["id"] for c in SERVO_CFG],
        "table": [
            {"pos_min": c["pos_min"], "pos_max": c["pos_max"]} for c in SERVO_CFG
        ],
        "home_steps": [int(x) for x in home],
        "edge_lo_steps": [int(x) for x in qmin],
        "edge_hi_steps": [int(x) for x in qmax],
        "home_deg": [_steps_to_deg(int(x)) for x in home],
        "edge_lo_deg": [_steps_to_deg(int(x)) for x in qmin],
        "edge_hi_deg": [_steps_to_deg(int(x)) for x in qmax],
        "source": f"yam_dxl_clear_teleop {date.today().isoformat()} port={port}",
    }
    path = _SESSION_DIR / f"yam_dxl_clear_{date.today().isoformat()}.json"
    path.write_text(json.dumps(art, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {path}", flush=True)


def main(argv: Optional[Sequence[str]] = None) -> int:
    global ARROW_VEL_TICK_S
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--hz", type=float, default=STREAM_HZ)
    ap.add_argument("--vel", type=float, default=ARROW_VEL_TICK_S, help="Cruise tick/s")
    args = ap.parse_args(list(argv) if argv is not None else None)
    ARROW_VEL_TICK_S = float(args.vel)
    hz = float(args.hz)
    dt_nom = 1.0 / max(hz, 1.0)

    port = args.port or find_cdc_port()
    print(f"port={port}", flush=True)

    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
            send=True,
        )
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )
        time.sleep(0.15)

        present = _arm_present(hub, hz=hz)
        if present is None:
            print("FAIL: no DXL FB — power/IDs/bus", flush=True)
            return 2

        cmd = list(present)
        home = list(present)
        qmin = list(present)
        qmax = list(present)
        fb = list(present)
        print(
            f"holding present pitch={int(cmd[0])} ({_steps_to_deg(int(cmd[0])):+.1f}°) "
            f"yaw={int(cmd[1])} ({_steps_to_deg(int(cmd[1])):+.1f}°)",
            flush=True,
        )
        # Prove motion briefly so operator sees life before interactive loop.
        print("prove: nudge pitch +80 ticks…", flush=True)
        prove_target = float(_clamp(0, cmd[0] + 80))
        t_prove = time.perf_counter() + 0.8
        while time.perf_counter() < t_prove:
            cmd[0] = min(prove_target, cmd[0] + ARROW_VEL_TICK_S * dt_nom)
            _apply(hub, cmd)
            p, y = _read_fb(hub)
            if p is not None:
                fb[0] = float(p)
            if y is not None:
                fb[1] = float(y)
            time.sleep(dt_nom)
        print(f"  after nudge fb_pitch={int(fb[0])} (was {int(present[0])})", flush=True)
        # return toward present
        t_prove = time.perf_counter() + 0.8
        while time.perf_counter() < t_prove:
            if cmd[0] > present[0]:
                cmd[0] = max(present[0], cmd[0] - ARROW_VEL_TICK_S * dt_nom)
            _apply(hub, cmd)
            time.sleep(dt_nom)
        cmd = list(present)

        sel = 0
        last_ui = 0.0
        try:
            with _RawStdin():
                t_prev = time.perf_counter()
                while True:
                    now = time.perf_counter()
                    dt = min(0.05, max(0.001, now - t_prev))
                    t_prev = now

                    for k in _poll_keys():
                        if k == "1":
                            sel = 0
                        elif k == "2":
                            sel = 1
                        elif k == "q":
                            raise KeyboardInterrupt
                        elif k == "h":
                            p, y = _read_fb(hub)
                            if p is not None and y is not None:
                                cmd = [float(p), float(y)]
                                home = list(cmd)
                                print("re-seeded from FB", flush=True)
                        elif k == "r":
                            qmin = list(fb)
                            qmax = list(fb)
                            print("reset min/max", flush=True)
                        elif k == "w":
                            _write_outputs(home, qmin, qmax, port=str(port))
                        elif k in ("a", "left"):
                            if sys.platform != "win32":
                                _nudge_cruise(-1)
                        elif k in ("d", "right"):
                            if sys.platform != "win32":
                                _nudge_cruise(+1)
                        elif k == " ":
                            if sys.platform != "win32":
                                _nudge_cruise(0, hold_s=0.0)

                    direction = _arrow_dir()
                    if direction != 0:
                        # Cmd-only ramp — ignore FB for slew (dropped frames OK).
                        cmd[sel] = float(
                            _clamp(sel, cmd[sel] + direction * ARROW_VEL_TICK_S * dt)
                        )

                    _apply(hub, cmd)
                    p, y = _read_fb(hub)
                    if p is not None:
                        fb[0] = float(p)
                    if y is not None:
                        fb[1] = float(y)
                    # FB only for running clear-range extrema / display.
                    for i in range(2):
                        qmin[i] = min(qmin[i], fb[i])
                        qmax[i] = max(qmax[i], fb[i])

                    if now - last_ui > 0.15:
                        _print_table(sel, cmd, fb, qmin, qmax)
                        last_ui = now

                    time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))

        except KeyboardInterrupt:
            print("\nquit", flush=True)
        finally:
            _conn(hub).clear_servos(send=False)
            hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
            hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=False,
            )
            _conn(hub).send_once()
            time.sleep(0.1)
            _conn(hub).send_once()

    print("done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
