#!/usr/bin/env python3
"""Operator-supervised left-arm clear-range characterization (bus 1 / 7 Damiao).

    python yam_arm_clear_range.py --apply-cfg
    python yam_arm_clear_range.py --joint 0
    python yam_arm_clear_range.py --dry-run   # inset math only (no COM)

Owns COM exclusively — close the dashboard first.
Conservative: small steps; press Enter *before* contact; recorded edge is the
last safe FB; inset shrinks the published window further.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import date
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.vbeta import (  # noqa: E402
    PcbArmDriver,
    PcbRobotSession,
    ensure_yam_left_arm_cfg,
)
from deft_controls_sdk.vbeta.yam_limits import (  # noqa: E402
    DEFAULT_CLEAR_INSET,
    apply_clear_inset,
)

_BENCH_MODULE = (
    _SCRIPTS / "deft_controls_sdk" / "vbeta" / "yam_bench_clear_left.py"
)
_SESSION_DIR = _SCRIPTS / ".deft_session"
_NOMINAL_ESC = tuple(range(0x01, 0x08))


def _fmt_q(q: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(v):+.4f}" for v in q) + "]"


def apply_clear_inset_q7(
    edge_lo: Sequence[float],
    edge_hi: Sequence[float],
    home: Sequence[float],
    *,
    inset: float,
) -> Tuple[np.ndarray, np.ndarray]:
    lo = np.zeros(7, dtype=np.float64)
    hi = np.zeros(7, dtype=np.float64)
    for i in range(7):
        lo[i], hi[i] = apply_clear_inset(
            float(edge_lo[i]),
            float(edge_hi[i]),
            inset=inset,
            home=float(home[i]),
        )
    return lo, hi


def render_bench_module(
    *,
    lo: Sequence[float],
    hi: Sequence[float],
    home: Sequence[float],
    source: str,
    inset: float,
    step: float,
) -> str:
    def _t(vals: Sequence[float]) -> str:
        return "(" + ", ".join(f"{float(v):.6f}" for v in vals) + ")"

    return f'''"""Left-arm motor-frame clear envelope for the current bench (bus 1 / slots 0–6).

Filled by ``scripts/yam_arm_clear_range.py`` after operator-supervised sweeps.
Until ``CLEAR_ACTIVE`` is True, ``yam_limits`` ignores this module.
"""
from __future__ import annotations

from typing import Optional, Tuple

CLEAR_ACTIVE = True

CLEAR_LO: Tuple[float, ...] = {_t(lo)}
CLEAR_HI: Tuple[float, ...] = {_t(hi)}
HOME_Q: Tuple[float, ...] = {_t(home)}

SOURCE = {source!r}
INSET_RAD = {float(inset):.6f}
STEP_RAD = {float(step):.6f}


def clear_q7() -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    """Return ``(lo, hi)`` when active, else None."""
    if not CLEAR_ACTIVE:
        return None
    if len(CLEAR_LO) != 7 or len(CLEAR_HI) != 7:
        return None
    return tuple(CLEAR_LO), tuple(CLEAR_HI)
'''


def _wait_live_fb(arm: PcbArmDriver, *, timeout_s: float = 8.0) -> np.ndarray:
    """Require non-trivial motion FB on at least one joint (power + CFG)."""
    deadline = time.perf_counter() + timeout_s
    last = np.zeros(7, dtype=np.float32)
    while time.perf_counter() < deadline:
        q = np.asarray(arm.read("Position_Rad"), dtype=np.float32).reshape(7)
        last = q
        if float(np.max(np.abs(q))) > 1e-3:
            return q
        arm._command_joint_pos(q, send=True)  # type: ignore[attr-defined]
        time.sleep(0.05)
    raise RuntimeError(
        "no live Damiao FB on left arm (all ~0) — check 24 V, CFG, ESC IDs. "
        f"last={_fmt_q(last)}"
    )


def _hold_stream(arm: PcbArmDriver, session: PcbRobotSession, q: np.ndarray, hold_s: float) -> None:
    t_end = time.perf_counter() + hold_s
    while time.perf_counter() < t_end:
        if session.service_soft_kill():
            raise RuntimeError("soft-kill park during characterize — aborting")
        arm._command_joint_pos(q, send=True)  # type: ignore[attr-defined]
        time.sleep(0.05)


def _stdin_stop_flag() -> Tuple[threading.Event, threading.Thread]:
    stop = threading.Event()

    def _reader() -> None:
        try:
            sys.stdin.readline()
        except Exception:
            pass
        stop.set()

    th = threading.Thread(target=_reader, daemon=True)
    th.start()
    return stop, th


def _jog_direction(
    arm: PcbArmDriver,
    session: PcbRobotSession,
    *,
    home: np.ndarray,
    joint: int,
    sign: int,
    step: float,
    dwell_s: float,
    max_steps: int,
) -> float:
    """Step joint until Enter; return last safe FB (before the stop step)."""
    q = home.copy()
    last_safe = float(home[joint])
    direction = "plus" if sign > 0 else "minus"
    print(
        f"\n--- J{joint + 1} {direction}: stepping {sign * step:+.3f} rad. "
        f"Press Enter BEFORE contact (last safe kept). q+Enter aborts. ---",
        flush=True,
    )
    stop, _th = _stdin_stop_flag()
    for i in range(max_steps):
        if stop.is_set():
            break
        q[joint] = last_safe + sign * step
        arm._command_joint_pos(q, send=True)  # type: ignore[attr-defined]
        t_end = time.perf_counter() + dwell_s
        while time.perf_counter() < t_end:
            if session.service_soft_kill():
                raise RuntimeError("soft-kill during jog — aborting")
            if stop.is_set():
                break
            time.sleep(0.02)
        fb = np.asarray(arm.read("Position_Rad"), dtype=np.float32).reshape(7)
        if stop.is_set():
            # Do not accept the current step as safe — keep previous.
            print(f"  stop @ step {i}: last_safe={last_safe:+.4f} fb={fb[joint]:+.4f}")
            break
        last_safe = float(fb[joint])
        print(f"  step {i + 1}: cmd={q[joint]:+.4f} fb={last_safe:+.4f}", flush=True)
    else:
        print(f"  hit max_steps={max_steps} without Enter — using last_safe={last_safe:+.4f}")

    # Return home before next direction.
    print(f"  returning J{joint + 1} to home…", flush=True)
    _hold_stream(arm, session, home, 1.2)
    return last_safe


def _probe_known_ids(hub, *, bus: int = 1) -> List[int]:
    """REG_SCAN each nominal ESC; return found motor_ids."""
    found: List[int] = []
    for mid in _NOMINAL_ESC:
        hit = hub.debug.discover_damiao(
            bus=bus, start=mid, end=mid, known_ids=(mid,)
        )
        if hit is not None:
            found.append(int(hit) & 0xFF)
    return found


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None, help="CDC port (default: auto)")
    ap.add_argument("--serial", default=None, help="USB serial disambiguation")
    ap.add_argument(
        "--apply-cfg",
        action="store_true",
        help="RAM-apply left-arm-only YAM CFG (CH1 on, CH2+ off)",
    )
    ap.add_argument(
        "--joint",
        type=int,
        default=None,
        help="Characterize only this arm-local joint 0..6 (default: all)",
    )
    ap.add_argument("--step", type=float, default=0.03, help="Jog step rad (default 0.03)")
    ap.add_argument(
        "--inset",
        type=float,
        default=DEFAULT_CLEAR_INSET,
        help="Conservative inset after stop edges (default 0.08)",
    )
    ap.add_argument("--dwell", type=float, default=0.35, help="Seconds per step")
    ap.add_argument("--max-steps", type=int, default=80)
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="No COM — demo inset math from fake edges",
    )
    ap.add_argument(
        "--write",
        action="store_true",
        default=True,
        help="Write yam_bench_clear_left.py + JSON artifact (default on)",
    )
    ap.add_argument("--no-write", action="store_true", help="Skip writing outputs")
    args = ap.parse_args(list(argv) if argv is not None else None)
    write = bool(args.write) and not bool(args.no_write)

    if args.joint is not None and not (0 <= args.joint <= 6):
        ap.error("--joint must be 0..6")

    joints = [args.joint] if args.joint is not None else list(range(7))

    if args.dry_run:
        home = np.array([0.1, 1.0, 1.0, 0.0, 0.0, 0.0, 1.5], dtype=np.float64)
        edge_lo = home - 0.4
        edge_hi = home + 0.4
        lo, hi = apply_clear_inset_q7(edge_lo, edge_hi, home, inset=args.inset)
        print("dry-run inset:")
        print(f"  home {_fmt_q(home)}")
        print(f"  edge_lo {_fmt_q(edge_lo)}")
        print(f"  edge_hi {_fmt_q(edge_hi)}")
        print(f"  clear_lo {_fmt_q(lo)}")
        print(f"  clear_hi {_fmt_q(hi)}")
        return 0

    port = args.port or find_cdc_port(serial=args.serial)
    print(f"port={port}", flush=True)

    with PcbRobotSession.connect(port, apply_yam_cfg=False) as session:
        if args.apply_cfg:
            ensure_yam_left_arm_cfg(session.hub, force=True)

        print("Damiao discover (nominal 0x01..0x07 on CH1)…", flush=True)
        found = _probe_known_ids(session.hub, bus=1)
        print(f"  found: {[hex(x) for x in found]}")
        missing = [hex(x) for x in _NOMINAL_ESC if x not in found]
        if missing:
            print(
                f"WARNING: missing ESC IDs {missing} — continuing; "
                "FB may be zero on those joints",
                flush=True,
            )
        if len(found) < 1:
            print("FAIL: no Damiao found on bus 1", flush=True)
            return 2

        # No soft clamps while exploring — operator is the stop; inset adds margin.
        arm = PcbArmDriver(
            session,
            side="left",
            skip_home_on_connect=True,
            clamp_goals=False,
        )
        arm.connect()
        home: Optional[np.ndarray] = None
        try:
            home = _wait_live_fb(arm).astype(np.float64)
            print(f"home FB: {_fmt_q(home)}", flush=True)
            _hold_stream(arm, session, home.astype(np.float32), 1.0)

            edge_lo = home.copy()
            edge_hi = home.copy()

            print(
                "\nSupervised clear-range. Keep e-stop ready. "
                "For each direction: watch the arm; Enter = stop (before contact).\n",
                flush=True,
            )
            for j in joints:
                input(f"Ready for J{j + 1}? Press Enter to start PLUS sweep… ")
                plus = _jog_direction(
                    arm,
                    session,
                    home=home.astype(np.float32),
                    joint=j,
                    sign=+1,
                    step=float(args.step),
                    dwell_s=float(args.dwell),
                    max_steps=int(args.max_steps),
                )
                input(f"Ready for J{j + 1} MINUS? Press Enter… ")
                minus = _jog_direction(
                    arm,
                    session,
                    home=home.astype(np.float32),
                    joint=j,
                    sign=-1,
                    step=float(args.step),
                    dwell_s=float(args.dwell),
                    max_steps=int(args.max_steps),
                )
                edge_lo[j] = min(minus, plus, float(home[j]))
                edge_hi[j] = max(minus, plus, float(home[j]))
                print(
                    f"J{j + 1} edges raw: lo={edge_lo[j]:+.4f} hi={edge_hi[j]:+.4f}",
                    flush=True,
                )

            # Joints not swept keep a tiny band around home.
            for j in range(7):
                if j not in joints:
                    edge_lo[j] = float(home[j]) - 0.05
                    edge_hi[j] = float(home[j]) + 0.05

            lo, hi = apply_clear_inset_q7(
                edge_lo, edge_hi, home, inset=float(args.inset)
            )
            source = (
                f"bench left CH1 supervised {date.today().isoformat()} "
                f"step={args.step} inset={args.inset} port={port} esc={found!r}"
            )
            print("\n=== clear envelope (motor frame, after inset) ===")
            print(f"  home     {_fmt_q(home)}")
            print(f"  edge_lo  {_fmt_q(edge_lo)}")
            print(f"  edge_hi  {_fmt_q(edge_hi)}")
            print(f"  clear_lo {_fmt_q(lo)}")
            print(f"  clear_hi {_fmt_q(hi)}")
            print(f"  source   {source}")

            if write:
                _SESSION_DIR.mkdir(parents=True, exist_ok=True)
                artifact = {
                    "date": date.today().isoformat(),
                    "port": port,
                    "found_esc": found,
                    "home": [float(x) for x in home],
                    "edge_lo": [float(x) for x in edge_lo],
                    "edge_hi": [float(x) for x in edge_hi],
                    "clear_lo": [float(x) for x in lo],
                    "clear_hi": [float(x) for x in hi],
                    "inset": float(args.inset),
                    "step": float(args.step),
                    "source": source,
                    "joints": joints,
                }
                json_path = (
                    _SESSION_DIR / f"yam_clear_left_{date.today().isoformat()}.json"
                )
                json_path.write_text(
                    json.dumps(artifact, indent=2) + "\n", encoding="utf-8"
                )
                print(f"wrote {json_path}")

                if set(joints) == set(range(7)):
                    _BENCH_MODULE.write_text(
                        render_bench_module(
                            lo=lo,
                            hi=hi,
                            home=home,
                            source=source,
                            inset=float(args.inset),
                            step=float(args.step),
                        ),
                        encoding="utf-8",
                    )
                    print(f"wrote {_BENCH_MODULE} (CLEAR_ACTIVE=True)")
                else:
                    print(
                        "partial joint set — JSON only; re-run all joints to "
                        "activate yam_bench_clear_left.py",
                        flush=True,
                    )
        finally:
            try:
                if home is not None:
                    _hold_stream(arm, session, home.astype(np.float32), 0.8)
                arm.write("Zero_Torque", True)
                session.send_once()
                time.sleep(0.2)
                # Skip disconnect()'s go_to(sleep_pose=0) — unsafe without zeros.
                arm.is_connected = False
            except Exception as exc:
                print(f"cleanup warning: {exc}", flush=True)

    print("done — next: hold/jog smoke under new clamps; write docs/bench note")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
