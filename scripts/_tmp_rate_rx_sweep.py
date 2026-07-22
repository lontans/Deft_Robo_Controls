"""Sweep host TX rates with/without RX-sim; print comparison tables.

  cd scripts
  python _tmp_rate_rx_sweep.py --port COM5 --seconds 2.0
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


RATES = (40.0, 100.0, 200.0, 500.0)
# Focus phases for the comparison tables (full matrix still runs).
FOCUS = (
    "1_CH1_x8",
    "4_CH4_x2",
    "7_CH1-3_fdcan",
    "8_CH4-6_mcp",
    "9_all_CH1-6_x25",
)


def _fmt(v, nd=0):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    port = args.port or find_cdc_port()
    hold = ActuatorDesire(position=0.01, velocity=0.0, kp=2.0, kd=0.5, torque=0.0)

    rows = []  # (hz, rx_sim, result_dict)

    print(f"Opening {port} ... seconds={args.seconds} rates={list(RATES)}")
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        try:
            by_bus = ensure_product_cfg(hub, force=False)

            def desire_map(*buses: int):
                return {s: hold for s in slots_for(by_bus, buses)}

            phases = [
                ("0_blank", {}),
                ("1_CH1_x8", desire_map(1)),
                ("2_CH2_x8", desire_map(2)),
                ("3_CH3_x3", desire_map(3)),
                ("4_CH4_x2", desire_map(4)),
                ("5_CH5_x2", desire_map(5)),
                ("6_CH6_x2", desire_map(6)),
                ("7_CH1-3_fdcan", desire_map(1, 2, 3)),
                ("8_CH4-6_mcp", desire_map(4, 5, 6)),
                ("9_all_CH1-6_x25", desire_map(1, 2, 3, 4, 5, 6)),
                ("10_blank_after", {}),
            ]

            for rx_sim in (False, True):
                for hz in RATES:
                    tag = f"rx_sim={'ON' if rx_sim else 'OFF'} @ {hz:.0f} Hz"
                    print(f"\n######## {tag} ########")
                    t0 = time.perf_counter()
                    for label, desires in phases:
                        sec = 1.5 if "blank" in label else args.seconds
                        r = measure(
                            hub,
                            label,
                            desires,
                            seconds=sec,
                            hz=hz,
                            rx_sim=rx_sim,
                        )
                        rows.append((hz, rx_sim, r))
                    print(f"  suite wall: {time.perf_counter() - t0:.1f}s")
        finally:
            leave_idle(hub)

    # --- Comparison tables (focus phases) ---
    print("\n" + "=" * 88)
    print("COMPARISON — focus phases (TX-only vs RX-sim)")
    print("=" * 88)

    for label in FOCUS:
        print(f"\n--- {label} ---")
        print(
            f"{'hz':>5}  {'rx':>3}  {'fb_hz':>6}  {'lag_mn':>6}  {'lag_p95':>7}  "
            f"{'lag_mx':>6}  {'lap_mn':>6}  {'lap_mx':>6}  {'pend_mx':>7}  "
            f"{'svc_mn':>6}  {'rx_fr':>5}  ok"
        )
        for hz in RATES:
            for rx_sim in (False, True):
                matches = [r for h, s, r in rows if h == hz and s == rx_sim and r["label"] == label]
                if not matches:
                    continue
                r = matches[0]
                ok = (
                    "Y"
                    if r["ok_lag"] and r["ok_fb"] and r.get("ok_rx", True)
                    else "N"
                )
                print(
                    f"{hz:5.0f}  {'on' if rx_sim else 'off':>3}  "
                    f"{_fmt(r['raw_fb_hz'], 0):>6}  "
                    f"{_fmt(r.get('ack_lag_mean'), 1):>6}  "
                    f"{_fmt(r.get('ack_lag_p95')):>7}  "
                    f"{_fmt(r.get('ack_lag_max')):>6}  "
                    f"{_fmt(r.get('lap_mean'), 1):>6}  "
                    f"{_fmt(r.get('lap_max_max')):>6}  "
                    f"{_fmt(r.get('pend_max')):>7}  "
                    f"{_fmt(r.get('svc_mean'), 1):>6}  "
                    f"{_fmt(r.get('rx_fresh_max')):>5}  {ok}"
                )

    # Compact all×25 delta table
    print("\n" + "=" * 88)
    print("all×25 delta: RX-sim ON minus TX-only (same hz)")
    print("=" * 88)
    print(
        f"{'hz':>5}  {'dfb':>6}  {'dlag_mx':>7}  {'dlap_mn':>7}  {'dpend_mx':>8}  "
        f"{'tx_ok':>5}  {'rx_ok':>5}"
    )
    for hz in RATES:
        off = next(
            r
            for h, s, r in rows
            if h == hz and not s and r["label"] == "9_all_CH1-6_x25"
        )
        on = next(
            r for h, s, r in rows if h == hz and s and r["label"] == "9_all_CH1-6_x25"
        )
        dfb = (on["raw_fb_hz"] or 0) - (off["raw_fb_hz"] or 0)
        dlag = (on.get("ack_lag_max") or 0) - (off.get("ack_lag_max") or 0)
        dlap = (on.get("lap_mean") or 0) - (off.get("lap_mean") or 0)
        dpend = (on.get("pend_max") or 0) - (off.get("pend_max") or 0)
        print(
            f"{hz:5.0f}  {dfb:+6.0f}  {dlag:+7d}  {dlap:+7.1f}  {dpend:+8d}  "
            f"{'Y' if off['ok_lag'] and off['ok_fb'] else 'N':>5}  "
            f"{'Y' if on['ok_lag'] and on['ok_fb'] and on.get('ok_rx') else 'N':>5}"
        )

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
