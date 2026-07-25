#!/usr/bin/env python3
"""Operator-supervised DXL neck clear-range characterization (servo slots 0/1).

    python dxl_servo_clear_range.py
    python dxl_servo_clear_range.py --slot 0
    python dxl_servo_clear_range.py --dry-run   # inset math only (no COM)

Owns COM exclusively — close the dashboard first. No CFG isolation needed
(unlike the CAN-bus YAM sweep): DXL lives on its own UART5 bus, and firmware
already blocks plant/CAN apply while a servo session is active
(PlantBlockReason.SERVO_SESSION) — no CFG isolation needed here.

Conservative: small steps; press Enter *before* contact; recorded edge is the
last safe FB; inset shrinks the published window further. Units are raw
ticks (int16, 4096/rev XL-class) — not radians.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import date
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link import ServoDesire  # noqa: E402
from deft_controls_sdk.link.exchange import parse_servo_feedback  # noqa: E402
from deft_controls_sdk.vbeta.session import PcbRobotSession  # noqa: E402
from deft_controls_sdk.vbeta.slots import (  # noqa: E402
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
)

_BENCH_MODULE = _SCRIPTS / "deft_controls_sdk" / "vbeta" / "dxl_neck_clear.py"
_SESSION_DIR = _SCRIPTS / ".deft_session"

_SERVO_TABLE: Tuple[dict, ...] = (
    {"slot": NECK_PITCH_SERVO_SLOT, "id": NECK_PITCH_DXL_ID, "label": "neck pitch (slot0 id1)"},
    {"slot": NECK_YAW_SERVO_SLOT, "id": NECK_YAW_DXL_ID, "label": "neck yaw (slot1 id2)"},
)
_ALL_SLOTS: Tuple[int, ...] = tuple(cfg["slot"] for cfg in _SERVO_TABLE)


def _apply_inset(edge_lo: int, edge_hi: int, *, inset: int, home: int) -> Tuple[int, int]:
    """Conservative clear window from operator stop edges (last-safe FB), raw ticks.

    Inset is ``min(inset, 10% of half-span)``. Ensures ``lo <= hi``; if the
    window collapses, keeps a tiny band around ``home``.
    """
    lo_e, hi_e = (edge_lo, edge_hi) if edge_hi >= edge_lo else (edge_hi, edge_lo)
    span = hi_e - lo_e
    half = span / 2.0
    m = float(inset)
    if half > 1e-6:
        m = min(m, 0.10 * half)
    lo = lo_e + m
    hi = hi_e - m
    if lo <= hi:
        return int(round(lo)), int(round(hi))
    mid = float(home)
    band = max(2.0, 0.25 * max(span, 8))
    return int(round(mid - band)), int(round(mid + band))


def render_bench_module(*, results: Dict[int, dict], source: str, inset: int, step: int) -> str:
    lo_by_slot = {slot: r["clear_lo"] for slot, r in results.items()}
    hi_by_slot = {slot: r["clear_hi"] for slot, r in results.items()}
    home_by_slot = {slot: r["home"] for slot, r in results.items()}
    return f'''"""DXL neck clear envelope for the current bench (servo slots 0/1).

Filled by ``scripts/dxl_servo_clear_range.py`` after operator-supervised
sweeps. Until ``CLEAR_ACTIVE`` is True, nothing consumes this module yet.
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

CLEAR_ACTIVE = True

# Raw ticks (int16, 4096/rev XL-class), keyed by servo slot (0=pitch/id1, 1=yaw/id2).
CLEAR_LO: Dict[int, int] = {lo_by_slot!r}
CLEAR_HI: Dict[int, int] = {hi_by_slot!r}
HOME_TICKS: Dict[int, int] = {home_by_slot!r}

SOURCE = {source!r}
INSET_TICKS = {int(inset)}
STEP_TICKS = {int(step)}


def clear_ticks(slot: int) -> Optional[Tuple[int, int]]:
    """Return ``(lo, hi)`` for one servo slot when active, else None."""
    if not CLEAR_ACTIVE or slot not in CLEAR_LO or slot not in CLEAR_HI:
        return None
    return CLEAR_LO[slot], CLEAR_HI[slot]
'''


def _wait_live_fb(
    session: PcbRobotSession,
    slot: int,
    servo_id: int,
    *,
    timeout_s: float = 6.0,
    settle_frames: int = 5,
) -> int:
    """Discover present position with torque OFF — never snap to a guessed pose.

    Requires ``settle_frames`` consecutive accepted reads before trusting the
    value (a single frame could be a stale/spurious zero).
    """
    deadline = time.perf_counter() + timeout_s
    last: Optional[int] = None
    accepted = 0
    while time.perf_counter() < deadline:
        session.set_servo(
            slot,
            ServoDesire(servo_id=servo_id, native_step_position=0, torque_enable=False, operating_mode=3),
            send=False,
        )
        session.send_once()
        fb = session.latest_feedback()
        if fb is not None:
            sv = parse_servo_feedback(fb.raw, slot)
            if sv is not None and sv["motor_source_id"] in (0, servo_id):
                last = int(sv["present_position"])
                accepted += 1
                if accepted >= settle_frames:
                    return last
            else:
                accepted = 0
        time.sleep(0.05)
    if last is not None:
        return last
    raise RuntimeError(
        f"no live DXL FB on slot={slot} id={servo_id} — check 5V/12V DXL power, "
        "ID/baud, UART5 wiring"
    )


def _hold_stream(session: PcbRobotSession, slot: int, servo_id: int, pos: int, hold_s: float) -> None:
    t_end = time.perf_counter() + hold_s
    while time.perf_counter() < t_end:
        if session.service_soft_kill():
            raise RuntimeError("soft-kill park during characterize — aborting")
        session.set_servo(
            slot,
            ServoDesire(servo_id=servo_id, native_step_position=int(pos), torque_enable=True, operating_mode=3),
            send=False,
        )
        session.send_once()
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
    session: PcbRobotSession,
    *,
    slot: int,
    servo_id: int,
    home: int,
    sign: int,
    step: int,
    dwell_s: float,
    max_steps: int,
) -> int:
    """Step servo until Enter; return last safe FB (before the stop step)."""
    last_safe = int(home)
    direction = "plus" if sign > 0 else "minus"
    print(
        f"\n--- slot{slot} id{servo_id} {direction}: stepping {sign * step:+d} ticks. "
        f"Press Enter BEFORE contact (last safe kept). q+Enter aborts. ---",
        flush=True,
    )
    stop, _th = _stdin_stop_flag()
    for i in range(max_steps):
        if stop.is_set():
            break
        cmd = last_safe + sign * step
        session.set_servo(
            slot,
            ServoDesire(servo_id=servo_id, native_step_position=int(cmd), torque_enable=True, operating_mode=3),
            send=False,
        )
        session.send_once()
        t_end = time.perf_counter() + dwell_s
        while time.perf_counter() < t_end:
            if session.service_soft_kill():
                raise RuntimeError("soft-kill during jog — aborting")
            if stop.is_set():
                break
            time.sleep(0.02)
        fb = session.latest_feedback()
        sv = parse_servo_feedback(fb.raw, slot) if fb is not None else None
        fb_pos = int(sv["present_position"]) if sv is not None else last_safe
        if stop.is_set():
            print(f"  stop @ step {i}: last_safe={last_safe:+d} fb={fb_pos:+d}")
            break
        last_safe = fb_pos
        print(f"  step {i + 1}: cmd={cmd:+d} fb={last_safe:+d}", flush=True)
    else:
        print(f"  hit max_steps={max_steps} without Enter — using last_safe={last_safe:+d}")

    print(f"  returning slot{slot} to home…", flush=True)
    _hold_stream(session, slot, servo_id, home, 1.0)
    return last_safe


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="CDC port (default: auto)")
    ap.add_argument("--serial", default=None, help="USB serial disambiguation")
    ap.add_argument(
        "--slot",
        type=int,
        default=None,
        choices=(0, 1),
        help="Characterize only this DXL servo slot (default: both 0 and 1)",
    )
    ap.add_argument(
        "--step", type=int, default=20, help="Jog step, raw ticks (default 20, ~1.8 deg on 4096/rev XL-class)"
    )
    ap.add_argument(
        "--inset",
        type=int,
        default=40,
        help="Conservative inset after stop edges, raw ticks (default 40, ~3.5 deg)",
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
        help="Write dxl_neck_clear.py + JSON artifact (default on)",
    )
    ap.add_argument("--no-write", action="store_true", help="Skip writing outputs")
    args = ap.parse_args(list(argv) if argv is not None else None)
    write = bool(args.write) and not bool(args.no_write)

    slots = [args.slot] if args.slot is not None else list(_ALL_SLOTS)

    if args.dry_run:
        home = 2048
        edge_lo, edge_hi = home - 300, home + 300
        lo, hi = _apply_inset(edge_lo, edge_hi, inset=int(args.inset), home=home)
        print("dry-run inset:")
        print(f"  home     {home:+d}")
        print(f"  edge_lo  {edge_lo:+d}")
        print(f"  edge_hi  {edge_hi:+d}")
        print(f"  clear_lo {lo:+d}")
        print(f"  clear_hi {hi:+d}")
        return 0

    port = args.port or find_cdc_port(serial=args.serial)
    print(f"port={port}", flush=True)

    with PcbRobotSession.connect(port, stream_hz=40.0) as session:
        results: Dict[int, dict] = {}
        try:
            print(
                "\nSupervised DXL clear-range. Keep e-stop ready. "
                "For each direction: watch the servo; Enter = stop (before contact).\n",
                flush=True,
            )
            for cfg in _SERVO_TABLE:
                slot = int(cfg["slot"])
                if slot not in slots:
                    continue
                servo_id = int(cfg["id"])
                label = str(cfg["label"])

                print(f"=== {label} — discovering present position (torque OFF) ===", flush=True)
                home = _wait_live_fb(session, slot, servo_id)
                print(f"  home fb: {home:+d}", flush=True)
                _hold_stream(session, slot, servo_id, home, 1.0)

                input(f"Ready for {label}? Press Enter to start PLUS sweep… ")
                plus = _jog_direction(
                    session,
                    slot=slot,
                    servo_id=servo_id,
                    home=home,
                    sign=+1,
                    step=int(args.step),
                    dwell_s=float(args.dwell),
                    max_steps=int(args.max_steps),
                )
                input(f"Ready for {label} MINUS? Press Enter… ")
                minus = _jog_direction(
                    session,
                    slot=slot,
                    servo_id=servo_id,
                    home=home,
                    sign=-1,
                    step=int(args.step),
                    dwell_s=float(args.dwell),
                    max_steps=int(args.max_steps),
                )

                edge_lo = min(minus, plus, home)
                edge_hi = max(minus, plus, home)
                lo, hi = _apply_inset(edge_lo, edge_hi, inset=int(args.inset), home=home)
                print(
                    f"{label} edges raw: lo={edge_lo:+d} hi={edge_hi:+d}  "
                    f"clear: lo={lo:+d} hi={hi:+d}",
                    flush=True,
                )
                results[slot] = {
                    "id": servo_id,
                    "label": label,
                    "home": home,
                    "edge_lo": edge_lo,
                    "edge_hi": edge_hi,
                    "clear_lo": lo,
                    "clear_hi": hi,
                }

            if not results:
                print("no slots characterized", flush=True)
            else:
                source = (
                    f"bench DXL neck supervised {date.today().isoformat()} "
                    f"step={args.step} inset={args.inset} port={port}"
                )
                print("\n=== clear envelope (raw ticks, after inset) ===")
                for slot in sorted(results):
                    r = results[slot]
                    print(
                        f"  slot{slot} {r['label']}: home={r['home']:+d} "
                        f"clear=[{r['clear_lo']:+d}, {r['clear_hi']:+d}]"
                    )
                print(f"  source   {source}")

                if write:
                    _SESSION_DIR.mkdir(parents=True, exist_ok=True)
                    artifact = {
                        "date": date.today().isoformat(),
                        "port": port,
                        "step": int(args.step),
                        "inset": int(args.inset),
                        "source": source,
                        "slots": results,
                    }
                    json_path = _SESSION_DIR / f"dxl_neck_clear_{date.today().isoformat()}.json"
                    json_path.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
                    print(f"wrote {json_path}")

                    if set(results.keys()) == set(_ALL_SLOTS):
                        _BENCH_MODULE.write_text(
                            render_bench_module(
                                results=results, source=source, inset=int(args.inset), step=int(args.step)
                            ),
                            encoding="utf-8",
                        )
                        print(f"wrote {_BENCH_MODULE} (CLEAR_ACTIVE=True)")
                    else:
                        print(
                            "partial slot set — JSON only; re-run both slots to "
                            "activate dxl_neck_clear.py",
                            flush=True,
                        )
        finally:
            for cfg in _SERVO_TABLE:
                try:
                    session.set_servo(int(cfg["slot"]), ServoDesire(servo_id=0), send=False)
                except Exception:
                    pass
            try:
                session.send_once()
            except Exception as exc:
                print(f"cleanup warning: {exc}", flush=True)

    print("done — next: hold/jog smoke under new clamps; write docs/bench note")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
