#!/usr/bin/env python3
"""Manual clear-range capture: select joint 1–7, arrow-teleop, track running min/max.

Uses dual-YAM teleop-proven MIT gains (not the hot loaded-bench set that buzzed J6).
Hold all 7 at brace kp (same as successful plant teleop). Goal slews with MAX_CMD_LEAD
so cmd cannot run far ahead of FB.

Keys (focus the terminal):
  1–7     select joint (J1–J7)
  ←/→     slew selected joint (or a/d)
  h       re-seed ALL setpoints from live FB (safe after big moves)
  r       reset running min/max (all joints)
  w       write JSON + optional bench module
  q       quit → DIAG + cornflower

    python yam_arm_clear_teleop.py --apply-cfg
"""
from __future__ import annotations

import argparse
import json
import math
import select
import sys
import termios
import time
import tty
from datetime import date
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, LedDesire, McuState  # noqa: E402
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT  # noqa: E402
from deft_controls_sdk.vbeta import (  # noqa: E402
    PcbArmDriver,
    PcbRobotSession,
    ensure_yam_left_arm_cfg,
)
from deft_controls_sdk.vbeta.cfg import pause_plant_stream  # noqa: E402
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP  # noqa: E402
from deft_controls_sdk.vbeta.yam_limits import (  # noqa: E402
    DEFAULT_CLEAR_INSET,
    apply_clear_inset,
)

# Match legacy teleop/defaults.py `_ARM_KP` / `DM_KD` (via vbeta DEFAULT_ARM_*).
KP = tuple(float(x) for x in DEFAULT_ARM_KP)
KD = float(DEFAULT_ARM_KD)
STREAM_HZ = 20.0
# Suite / careful teleop default (dual-arm proven). Use --vel to go faster.
ARROW_VEL = 0.12  # rad/s cruise
MAX_CMD_LEAD = 0.18
P_MIN, P_MAX = -12.57, 12.57
ACTIVE = tuple(range(7))

_SESSION_DIR = _SCRIPTS / ".deft_session"
_BENCH_MODULE = (
    _SCRIPTS / "deft_controls_sdk" / "vbeta" / "yam_bench_clear_left.py"
)


def _fmt(q: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(v):+.4f}" for v in q) + "]"


# --- keyboard (Linux Jetson primary; Windows via msvcrt) ---------------------
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

    def _poll_keys() -> List[str]:
        out: List[str] = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                # CSI arrow
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

    _cruise_dir = 0
    _cruise_until = 0.0

    def _nudge_cruise(direction: int, hold_s: float = 0.25) -> None:
        global _cruise_dir, _cruise_until
        _cruise_dir = int(direction)
        _cruise_until = time.perf_counter() + hold_s

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


def _write_mit(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    dq: Optional[np.ndarray] = None,
    kp_scale: float = 1.0,
) -> None:
    q = np.asarray(q, dtype=np.float32).reshape(7)
    vel = (
        np.zeros(7, dtype=np.float32)
        if dq is None
        else np.asarray(dq, dtype=np.float32).reshape(7)
    )
    scale = float(np.clip(kp_scale, 0.0, 1.0))
    desires = {}
    for i, slot in enumerate(arm.slots):
        desires[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(vel[i]),
            kp=float(KP[i]) * scale,
            kd=KD,
        )
    session.set_actuators(desires, send=False)
    arm._setpoint = q.copy()  # noqa: SLF001


def _soft_engage(
    session: PcbRobotSession, arm: PcbArmDriver, q: np.ndarray, engage_s: float = 1.4
) -> None:
    print(f"soft-engage teleop gains over {engage_s:.1f}s…", flush=True)
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / max(engage_s, 1e-3)
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        _write_mit(session, arm, q, kp_scale=s)
        time.sleep(0.02)
    _write_mit(session, arm, q, kp_scale=1.0)


def _acquire_fb(session: PcbRobotSession, arm: PcbArmDriver) -> np.ndarray:
    q = np.zeros(7, dtype=np.float32)
    deadline = time.perf_counter() + 2.0
    while time.perf_counter() < deadline:
        _write_mit(session, arm, q, kp_scale=0.0)
        # kp=0 → _write_mit still sends kd=KD (commanding). Prefer light poke:
        desires = {
            slot: ActuatorDesire(position=float(q[i]), velocity=0.0, kp=0.0, kd=0.5)
            for i, slot in enumerate(arm.slots)
        }
        session.set_actuators(desires, send=False)
        time.sleep(0.04)
        fb = np.asarray(arm.read("Position_Rad"), dtype=np.float32).reshape(7)
        if float(np.max(np.abs(fb))) > 1e-3:
            q = fb.copy()
            break
    else:
        raise RuntimeError("no Damiao FB")
    for _ in range(8):
        desires = {
            slot: ActuatorDesire(position=float(q[i]), velocity=0.0, kp=0.0, kd=0.5)
            for i, slot in enumerate(arm.slots)
        }
        session.set_actuators(desires, send=False)
        arm._setpoint = q.copy()  # noqa: SLF001
        time.sleep(0.05)
    return q


def _print_table(
    sel: int,
    cmd: np.ndarray,
    fb: np.ndarray,
    qmin: np.ndarray,
    qmax: np.ndarray,
) -> None:
    print("\033[H\033[J", end="")
    print(
        "YAM clear teleop — keys: 1-7 select | ←/→ or a/d slew | h reseeds | "
        "r reset min/max | w write | q quit",
        flush=True,
    )
    print(f"gains kp={KP} kd={KD} vel={ARROW_VEL} lead={MAX_CMD_LEAD}", flush=True)
    print(
        f"{'J':>3} {'sel':>3} {'cmd':>9} {'fb':>9} {'min':>9} {'max':>9} {'span':>8}",
        flush=True,
    )
    for i in range(7):
        mark = "*" if i == sel else " "
        span = float(qmax[i] - qmin[i])
        print(
            f"J{i+1:>2} {mark:>3} {cmd[i]:+9.4f} {fb[i]:+9.4f} "
            f"{qmin[i]:+9.4f} {qmax[i]:+9.4f} {span:8.4f}",
            flush=True,
        )


def _write_outputs(
    home: np.ndarray,
    qmin: np.ndarray,
    qmax: np.ndarray,
    *,
    inset: float,
    port: str,
) -> None:
    lo = np.zeros(7)
    hi = np.zeros(7)
    for i in range(7):
        lo[i], hi[i] = apply_clear_inset(
            float(qmin[i]), float(qmax[i]), float(home[i]), inset=inset
        )
    source = (
        f"bench left CH1 teleop-minmax {date.today().isoformat()} "
        f"inset={inset} port={port} gains={KP}"
    )
    _SESSION_DIR.mkdir(parents=True, exist_ok=True)
    art = {
        "date": date.today().isoformat(),
        "port": port,
        "home": [float(x) for x in home],
        "edge_lo": [float(x) for x in qmin],
        "edge_hi": [float(x) for x in qmax],
        "clear_lo": [float(x) for x in lo],
        "clear_hi": [float(x) for x in hi],
        "inset": inset,
        "kp": list(KP),
        "kd": KD,
        "source": source,
    }
    path = _SESSION_DIR / f"yam_clear_left_teleop_{date.today().isoformat()}.json"
    path.write_text(json.dumps(art, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {path}", flush=True)

    # Activate bench module when every joint has a real span.
    if float(np.min(qmax - qmin)) > 0.05:
        body = f'''"""Left-arm motor-frame clear envelope (teleop min/max capture)."""
from __future__ import annotations
from typing import Optional, Tuple

CLEAR_ACTIVE = True
CLEAR_LO: Tuple[float, ...] = {tuple(float(x) for x in lo)!r}
CLEAR_HI: Tuple[float, ...] = {tuple(float(x) for x in hi)!r}
HOME_Q: Tuple[float, ...] = {tuple(float(x) for x in home)!r}
SOURCE = {source!r}
INSET_RAD = {float(inset)!r}


def clear_q7() -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    if not CLEAR_ACTIVE:
        return None
    return tuple(CLEAR_LO), tuple(CLEAR_HI)
'''
        _BENCH_MODULE.write_text(body, encoding="utf-8")
        print(f"wrote {_BENCH_MODULE} CLEAR_ACTIVE=True", flush=True)
    else:
        print("bench module not written — need >0.05 rad span on every joint", flush=True)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--apply-cfg", action="store_true")
    ap.add_argument("--stream-hz", type=float, default=STREAM_HZ)
    ap.add_argument("--inset", type=float, default=DEFAULT_CLEAR_INSET)
    args = ap.parse_args(list(argv) if argv is not None else None)

    port = args.port or find_cdc_port()
    print(f"port={port}", flush=True)

    with PcbRobotSession.connect(
        port,
        apply_yam_cfg=False,
        stream_hz=float(args.stream_hz),
        idle_first=True,
    ) as session:
        with pause_plant_stream(session.hub):
            if args.apply_cfg:
                ensure_yam_left_arm_cfg(session.hub, force=True)

        session.hub.recover()
        time.sleep(0.25)
        session.hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        session.set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )

        arm = PcbArmDriver(
            session,
            side="left",
            skip_home_on_connect=True,
            clamp_goals=False,
            kp=KP,
            kd=KD,
        )
        arm.is_connected = True
        try:
            session.hub.set_mcu_state(McuState.NORMAL, send=True)
            session.hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=True,
            )
            q0 = _acquire_fb(session, arm)
            print(f"home FB {_fmt(q0)}", flush=True)
            _write_mit(session, arm, q0, kp_scale=0.0)
            # explicit kd while kp=0
            for _ in range(5):
                desires = {
                    slot: ActuatorDesire(
                        position=float(q0[i]), velocity=0.0, kp=0.0, kd=KD
                    )
                    for i, slot in enumerate(arm.slots)
                }
                session.set_actuators(desires, send=False)
                time.sleep(0.05)
            _soft_engage(session, arm, q0, engage_s=1.4)
            time.sleep(0.3)

            cmd = q0.copy()
            home = q0.astype(np.float64).copy()
            qmin = q0.astype(np.float64).copy()
            qmax = q0.astype(np.float64).copy()
            sel = 0
            last_ui = 0.0
            dt_nom = 1.0 / max(float(args.stream_hz), 1.0)

            print("engaged — teleop loop (raw terminal)…", flush=True)
            with _RawStdin():
                t_prev = time.perf_counter()
                while True:
                    now = time.perf_counter()
                    dt = min(0.05, max(0.001, now - t_prev))
                    t_prev = now

                    for k in _poll_keys():
                        if k in "1234567":
                            sel = int(k) - 1
                        elif k == "q":
                            raise KeyboardInterrupt
                        elif k == "h":
                            fb = np.asarray(
                                arm.read("Position_Rad"), dtype=np.float32
                            ).reshape(7)
                            cmd = fb.copy()
                            home = fb.astype(np.float64).copy()
                            print("re-seeded cmd/home from FB", flush=True)
                        elif k == "r":
                            fb = np.asarray(
                                arm.read("Position_Rad"), dtype=np.float64
                            ).reshape(7)
                            qmin = fb.copy()
                            qmax = fb.copy()
                            print("reset min/max to current FB", flush=True)
                        elif k == "w":
                            _write_outputs(
                                home, qmin, qmax, inset=float(args.inset), port=port
                            )
                        elif k in ("a", "left"):
                            if sys.platform != "win32":
                                _nudge_cruise(-1)
                        elif k in ("d", "right"):
                            if sys.platform != "win32":
                                _nudge_cruise(+1)
                        elif k in (" ",):
                            if sys.platform != "win32":
                                _nudge_cruise(0, hold_s=0.0)

                    direction = _arrow_dir()

                    if direction != 0:
                        proposed = float(cmd[sel]) + direction * ARROW_VEL * dt
                        proposed = max(P_MIN, min(P_MAX, proposed))
                        fb = np.asarray(
                            arm.read("Position_Rad"), dtype=np.float32
                        ).reshape(7)
                        lead = proposed - float(fb[sel])
                        if abs(lead) > MAX_CMD_LEAD:
                            proposed = float(fb[sel]) + math.copysign(
                                MAX_CMD_LEAD, lead
                            )
                        cmd[sel] = proposed
                    else:
                        fb = np.asarray(
                            arm.read("Position_Rad"), dtype=np.float32
                        ).reshape(7)

                    # Update extrema from FB (operator explores by teleop).
                    for i in range(7):
                        qmin[i] = min(qmin[i], float(fb[i]))
                        qmax[i] = max(qmax[i], float(fb[i]))

                    dq = np.zeros(7, dtype=np.float32)
                    if direction != 0:
                        dq[sel] = float(direction) * ARROW_VEL
                    _write_mit(session, arm, cmd, dq=dq, kp_scale=1.0)

                    if session.service_soft_kill():
                        print("soft-kill — exiting", flush=True)
                        break

                    if now - last_ui > 0.15:
                        _print_table(sel, cmd, fb, qmin, qmax)
                        last_ui = now

                    # Pace roughly to stream
                    time.sleep(max(0.0, dt_nom - (time.perf_counter() - now)))

        except KeyboardInterrupt:
            print("\nquit", flush=True)
        finally:
            try:
                print("cleanup: recover → DIAG…", flush=True)
                session.hub.recover()
                time.sleep(0.2)
                session.hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
                session.set_actuators(
                    {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
                )
                session.hub.set_led(
                    LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                    send=False,
                )
                session.send_once()
            except Exception as exc:
                print(f"cleanup warning: {exc}", flush=True)
            arm.is_connected = False

    print("done — cornflower idle", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
