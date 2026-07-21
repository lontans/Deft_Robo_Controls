"""Lean COM5 timing probe: blank vs MCP hold — fb_hz, ack_lag, lap_ms.

Run from scripts/ after flashing the plant MCP non-block fix:
  python _tmp_mcp_timing_probe.py --port COM5

Success criteria:
  blank:   ack_lag max ~0, raw fb_hz hundreds+
  mcp x1:  ack_lag max ~0..2, fb_hz not collapsed to ~2
  mcp x3:  ack_lag not climbing to 29+; LEDs keep updating
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import parse_feedback_header


def _pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _drain_latest(hub: ControlsPcbHub):
    raw = None
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        raw = frame
    return raw


def measure(
    hub: ControlsPcbHub,
    label: str,
    desires: Dict[int, ActuatorDesire],
    *,
    seconds: float = 4.0,
    hz: float = 40.0,
) -> dict:
    blank = {slot: ActuatorDesire() for slot in range(ACTUATOR_COUNT)}
    blank.update(desires)
    _conn(hub).set_actuators(blank, send=False)

    dt = 1.0 / hz
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    reader = _conn(hub).reader
    tf0 = reader.total_frames

    fb_times: Deque[float] = deque()
    ack_lags: List[int] = []
    lap_ms: List[int] = []
    lap_max: List[int] = []
    ticks_pend: List[int] = []
    ticks_svc: List[int] = []
    tx_gaps: List[float] = []
    send_ms: List[float] = []
    last_tx: Optional[float] = None
    last_ack: Optional[int] = None
    last_sent: Optional[int] = None
    pdu_tags: Dict[str, int] = {}

    while time.perf_counter() < t_end:
        raw = _drain_latest(hub)
        now = time.perf_counter()
        if raw is not None:
            fb_times.append(now)
            while fb_times and (now - fb_times[0]) > 1.0:
                fb_times.popleft()
            hdr = parse_feedback_header(raw)
            if hdr:
                tag = str(hdr.get("pdu_tag", "?"))
                pdu_tags[tag] = pdu_tags.get(tag, 0) + 1
                ack = int(hdr["last_cmd_seq"]) & 0xFF
                last_ack = ack
                if last_sent is not None:
                    lag = (last_sent - ack) & 0xFF
                    if lag <= 128:
                        ack_lags.append(lag)
                if hdr.get("lap_ms") is not None:
                    lap_ms.append(int(hdr["lap_ms"]))
                if hdr.get("lap_max_ms") is not None:
                    lap_max.append(int(hdr["lap_max_ms"]))
                if hdr.get("ticks_pending") is not None:
                    ticks_pend.append(int(hdr["ticks_pending"]))
                if hdr.get("ticks_svc") is not None:
                    ticks_svc.append(int(hdr["ticks_svc"]))

        t0 = time.perf_counter()
        _conn(hub).send_once()
        send_ms.append((time.perf_counter() - t0) * 1000.0)
        sent = _conn(hub)._last_sent_seq  # noqa: SLF001
        last_sent = (sent & 0xFF) if sent is not None else None
        now_tx = time.perf_counter()
        if last_tx is not None:
            tx_gaps.append((now_tx - last_tx) * 1000.0)
        last_tx = now_tx

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    time.sleep(0.05)
    while _drain_latest(hub) is not None:
        pass

    wall = seconds
    raw_fb = reader.total_frames - tf0
    raw_fb_hz = raw_fb / wall if wall > 0 else None

    def istat(xs: List[int]) -> str:
        if not xs:
            return "n/a"
        return (
            f"n={len(xs)} mean={statistics.mean(xs):.1f} "
            f"p50={_pct([float(x) for x in xs], 50):.0f} "
            f"p95={_pct([float(x) for x in xs], 95):.0f} max={max(xs)}"
        )

    def fstat(xs: List[float]) -> str:
        if not xs:
            return "n/a"
        return (
            f"n={len(xs)} mean={statistics.mean(xs):.1f} "
            f"p95={_pct(xs, 95):.1f} max={max(xs):.1f}"
        )

    print(f"\n=== {label} ===")
    print(f"  desires: { {k: (v.position, v.kp, v.kd) for k, v in desires.items()} }")
    print(f"  raw_fb_hz~{raw_fb_hz:.1f}" if raw_fb_hz is not None else "  raw_fb_hz=n/a",
          f"  frames={raw_fb}")
    print(f"  ack_lag: {istat(ack_lags)}")
    print(f"  lap_ms:  {istat(lap_ms)}")
    print(f"  lap_max: {istat(lap_max)}")
    print(f"  ticks_pending: {istat(ticks_pend)}")
    print(f"  ticks_svc:     {istat(ticks_svc)}")
    print(f"  send_ms: {fstat(send_ms)}")
    print(f"  tx_gap:  {fstat(tx_gaps)}")
    print(f"  pdu_tags: {pdu_tags}")
    print(f"  last_sent={last_sent} last_ack={last_ack}")

    ok_lag = (not ack_lags) or (max(ack_lags) <= 2)
    print(f"  PASS_ack_lag<={2}: {ok_lag}")
    return {
        "label": label,
        "raw_fb_hz": raw_fb_hz,
        "ack_lag_max": max(ack_lags) if ack_lags else None,
        "ok": ok_lag,
    }


def find_mcp_slots(hub: ControlsPcbHub) -> List[Tuple[int, int, int]]:
    out: List[Tuple[int, int, int]] = []
    try:
        table = hub.debug.cfg_get_table()
    except Exception as exc:
        print(f"CFG get failed ({exc}); assuming slots 3,4,5 are MCP")
        return [(3, 4, 0), (4, 5, 0), (5, 6, 0)]

    for slot, row in enumerate(table):
        if isinstance(row, dict):
            enabled = row.get("enabled", True)
            bus = int(row.get("bus", 0))
            mid = int(row.get("motor_id", 0))
        else:
            enabled = getattr(row, "enabled", True)
            bus = int(getattr(row, "bus", 0))
            mid = int(getattr(row, "motor_id", 0))
        if enabled and 4 <= bus <= 6:
            out.append((slot, bus, mid))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--hz", type=float, default=40.0)
    args = ap.parse_args()

    print(f"Opening {args.port} ...")
    results = []
    with ControlsPcbHub.connect(args.port, persist_telemetry=False) as hub:
        mcp = find_mcp_slots(hub)
        print(f"MCP candidates: {mcp}")
        if not mcp:
            print("No MCP slots found — abort")
            return 2

        results.append(measure(hub, "0_blank_all", {}, seconds=2.0, hz=args.hz))
        s0 = mcp[0][0]
        hold = ActuatorDesire(position=1e-6, velocity=0.0, kp=6.0, kd=0.5, torque=0.0)
        results.append(
            measure(hub, f"1_single_mcp_slot{s0}", {s0: hold}, seconds=args.seconds, hz=args.hz)
        )
        if len(mcp) >= 3:
            triple = {mcp[i][0]: hold for i in range(3)}
            results.append(measure(hub, "2_triple_mcp", triple, seconds=args.seconds, hz=args.hz))
        results.append(measure(hub, "3_blank_after", {}, seconds=2.0, hz=args.hz))

        for slot in range(ACTUATOR_COUNT):
            hub.set_actuator(slot, ActuatorDesire(), send=False)
        hub.send_once()

    mcp_ok = all(r["ok"] for r in results if "mcp" in r["label"])
    print("\nOVERALL MCP ack_lag OK:" , mcp_ok)
    return 0 if mcp_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
