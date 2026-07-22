"""Verify last_image_id / last_applied_image_id readbacks + stall map.

Runs focus phases at 200/500 Hz with TX-only and RX-sim so applied_hz /
app_lag show where plant work slows image mount vs USB stage.

  cd scripts
  python _tmp_image_id_verify.py
"""
from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, find_cdc_port

from _tmp_mcp_timing_probe import (  # noqa: E402
    ensure_product_cfg,
    leave_idle,
    measure,
    slots_for,
)


FOCUS = (
    "0_blank",
    "1_CH1_x8",
    "8_CH4-6_mcp",
    "7_CH1-3_fdcan",
    "9_all_CH1-6_x25",
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    port = args.port or find_cdc_port()
    hold = ActuatorDesire(position=0.01, velocity=0.0, kp=2.0, kd=0.5, torque=0.0)

    rows = []
    print(f"Opening {port} ... image_id verify")
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        try:
            by_bus = ensure_product_cfg(hub, force=False)

            def desire_map(*buses: int):
                return {s: hold for s in slots_for(by_bus, buses)}

            phases = {
                "0_blank": {},
                "1_CH1_x8": desire_map(1),
                "7_CH1-3_fdcan": desire_map(1, 2, 3),
                "8_CH4-6_mcp": desire_map(4, 5, 6),
                "9_all_CH1-6_x25": desire_map(1, 2, 3, 4, 5, 6),
            }

            for rx_sim in (False, True):
                for hz in (200.0, 500.0):
                    print(f"\n######## rx_sim={'ON' if rx_sim else 'OFF'} @ {hz:.0f} Hz ########")
                    t0 = time.perf_counter()
                    for label in FOCUS:
                        r = measure(
                            hub,
                            label,
                            phases[label],
                            seconds=1.5 if "blank" in label else args.seconds,
                            hz=hz,
                            rx_sim=rx_sim,
                        )
                        rows.append((hz, rx_sim, r))
                    print(f"  suite wall: {time.perf_counter() - t0:.1f}s")
        finally:
            leave_idle(hub)

    print("\n" + "=" * 96)
    print("STALL MAP — image_id advance vs load (applied_hz drops => plant mount stalls)")
    print("=" * 96)
    print(
        f"{'phase':18s} {'hz':>4} {'rx':>3}  {'fb':>5}  {'lap':>5}  {'pend':>5}  "
        f"{'stg_hz':>6}  {'app_hz':>6}  {'img_lag':>7}  {'app_lag':>7}  "
        f"{'gap':>4}  note"
    )
    for hz, rx_sim, r in rows:
        fb = r.get("raw_fb_hz")
        lap = r.get("lap_mean")
        pend = r.get("pend_max")
        stg = r.get("staged_hz")
        app = r.get("applied_hz")
        img_lag = r.get("img_lag_max")
        app_lag = r.get("app_lag_max")
        gap = r.get("stage_app_gap_max")
        note = ""
        if app is not None and stg is not None and stg > 0 and app < 0.85 * stg:
            note = "APPLIED_STALL"
        elif lap is not None and lap >= 2.0:
            note = "LAP_HEAVY"
        elif pend is not None and pend > 8:
            note = "PEND_BACKLOG"
        elif fb is not None and fb < 0.7 * 500 and hz >= 200:
            note = "FB_SLOW"
        print(
            f"{r['label']:18s} {hz:4.0f} {'on' if rx_sim else 'off':>3}  "
            f"{fb:5.0f}  {lap if lap is not None else 0:5.1f}  "
            f"{pend if pend is not None else 0:5d}  "
            f"{stg if stg is not None else 0:6.0f}  "
            f"{app if app is not None else 0:6.0f}  "
            f"{img_lag if img_lag is not None else 0:7d}  "
            f"{app_lag if app_lag is not None else 0:7d}  "
            f"{gap if gap is not None else 0:4d}  {note}"
        )

    # Sanity: image ids must be present (non-None) on all non-empty runs
    missing = [
        r["label"]
        for _, _, r in rows
        if r.get("staged_hz") is None and r["n_slots"] >= 0
    ]
    if missing and all(r.get("img_lag_max") is None for _, _, r in rows):
        print("\nFAIL: last_image_id not present in FB (firmware not flashed?)")
        return 1

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
