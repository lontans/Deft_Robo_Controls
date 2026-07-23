#!/usr/bin/env python3
"""Run load matrices (real CH6 RS02 vs ×25 rx_sim) + bandwidth baseline → markdown.

  cd scripts
  python _tmp_load_matrix_report.py
  python _tmp_load_matrix_report.py --skip-cali --trials 3 --seconds 8
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

from _tmp_bus6_real_hw import (
    DEFAULT_RATE_RAD_S,
    LOAD_HOLD,
    RX_SIM_ACTUATOR,
    SERVO_CFG,
    _conn,
    _rs_phase_seconds,
    _servo_mid,
    leave_idle,
    stream_combo,
)
from _tmp_dxl_one import sample_servo_fb
from _tmp_mcp_timing_probe import ensure_product_cfg, leave_idle as probe_leave_idle
from _tmp_mcp_timing_probe import measure as bw_measure
from _tmp_mcp_timing_probe import slots_for
from rs02_channel_bringup import (
    CANONICAL_SLOT,
    DEFAULT_TELEOP_ANGLE_RAD,
    assign_single_slot,
    rs02_near,
    rs02_plan_angle,
    rs02_resolve_start,
    sample_position,
    seed_idle_at_fb,
)

RATES = (40.0, 100.0, 200.0, 500.0)
BW_FOCUS = (
    "1_CH1_x8",
    "4_CH4_x2",
    "6_CH6_x2",
    "7_CH1-3_fdcan",
    "8_CH4-6_mcp",
    "9_all_CH1-6_x25",
)


def _mean(xs: List[float]) -> Optional[float]:
    return statistics.mean(xs) if xs else None


def _fmt(v: Any, nd: int = 1) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def _seed_servos(hub: ControlsPcbHub, hz: float) -> Dict[int, float]:
    servo_start: Dict[int, float] = {}
    for c in SERVO_CFG:
        fb = sample_servo_fb(
            hub, c["slot"], servo_id=c["id"], timeout_s=1.2, hz=hz
        )
        if fb is None:
            fb = _servo_mid(c)
            print(f"  DXL slot {c['slot']}: no FB — mid={fb}")
        else:
            lo, hi = int(c["pos_min"]), int(c["pos_max"])
            fb = max(lo, min(hi, int(fb)))
            print(f"  DXL slot {c['slot']}: present={fb}")
            sample_servo_fb(
                hub,
                c["slot"],
                servo_id=c["id"],
                hold_pos=fb,
                timeout_s=0.35,
                hz=hz,
            )
        servo_start[c["slot"]] = float(fb)
    _conn(hub).clear_servos(send=True)
    time.sleep(0.12)
    return servo_start


def _calibrate_bus6(hub: ControlsPcbHub, *, bus: int, slot: int, motor_id: int) -> float:
    print("\n=== CALIBRATE RS02 CH6 ===")
    resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
    if resp is None or not resp.get("found"):
        raise RuntimeError(f"probe miss CH{bus} id=0x{motor_id:02X}")
    probe_pos = float(resp["position"])
    print(f"  pre-cali probe pos={probe_pos:+.4f}")
    assign_single_slot(hub, bus=bus, slot=slot, motor_id=motor_id, persist=False)
    seed_idle_at_fb(hub, slot, probe_pos)
    ok = hub.debug.calibrate_robstride(bus=bus, motor_id=motor_id, cal_listen_s=28.0)
    print(f"  cali={'ok' if ok else 'FAIL'}")
    en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
    if en and en.get("found"):
        probe_pos = float(en["position"])
        seed_idle_at_fb(hub, slot, probe_pos)
        print(f"  post-cali probe pos={probe_pos:+.4f}")
    return probe_pos


def _reseed_rs(
    hub: ControlsPcbHub,
    *,
    bus: int,
    slot: int,
    motor_id: int,
    hz: float,
    fallback: float,
) -> Tuple[float, float]:
    """Return (start_pos, planned_angle)."""
    en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
    probe = float(en["position"]) if en and en.get("found") else None
    seed_idle_at_fb(hub, slot, probe if probe is not None else fallback)
    hub.refresh_feedback(slots=[slot], seconds=0.25, hz=hz)
    plant_fb = sample_position(hub, slot)
    start = rs02_resolve_start(probe, plant_fb)
    if start is None:
        start = fallback
    if probe is not None and plant_fb is not None and not rs02_near(probe, plant_fb):
        print(
            f"  note: stale plant FB {plant_fb:+.4f} vs probe {probe:+.4f} — trusting probe"
        )
    angle = rs02_plan_angle(start, DEFAULT_TELEOP_ANGLE_RAD)
    print(f"  RS start={start:+.4f} angle={angle:+.4f}")
    return start, angle


def _trial_row(stats: dict, *, hz: float, trial: int, cfg: str, angle: float) -> dict:
    moved = stats.get("servo_moved") or {}
    return {
        "cfg": cfg,
        "hz": hz,
        "trial": trial,
        "plant_fb_hz": stats.get("plant_fb_hz"),
        "raw_fb_hz": stats.get("raw_fb_hz"),
        "cmd_seq_lag_max": stats.get("cmd_seq_lag_max", stats.get("ack_lag_max")),
        "cmd_seq_lag_p95": stats.get("cmd_seq_lag_p95", stats.get("ack_lag_p95")),
        "cmd_seq_lag_mean": stats.get("cmd_seq_lag_mean", stats.get("ack_lag_mean")),
        "act_lap_mean": stats.get("act_lap_mean"),
        "act_lap_peak": stats.get("act_lap_peak", stats.get("lap_max_sticky")),
        "act_lap_sample_max": stats.get("lap_max_window"),
        "periph_lap_mean": stats.get("periph_lap_mean"),
        "periph_lap_peak": stats.get("periph_lap_peak", stats.get("periph_lap_max_max")),
        "staged_hz": stats.get("staged_hz"),
        "applied_hz": stats.get("applied_hz"),
        "plant_block_hits": stats.get("plant_block_hits"),
        "led_mode_last": stats.get("led_mode_last"),
        "servo0_span": moved.get(0),
        "servo1_span": moved.get(1),
        "servo_err": stats.get("servo_err"),
        "rs_delta": stats.get("rs_delta"),
        "rs_travel_cmd": stats.get("rs_travel_cmd"),
        "rs_end_err": stats.get("rs_end_err"),
        "rs_start": stats.get("rs_start"),
        "rs_target": stats.get("rs_target"),
        "angle_planned": angle,
        "rx_fresh_mean": stats.get("rx_fresh_mean"),
    }


def _weird(row: dict, *, need_rs: bool) -> Optional[str]:
    """cmd_seq_lag_p95 is reported (see the per-trial/aggregate tables) but is
    NOT a pass/fail gate here — at host TX rates above what the plant can
    apply (500 Hz is the known case; see docs/bench-load-matrix-*.md), lag
    climbs by design as commands queue up, and a hard lag<=2 cutoff was
    flagging that healthy backpressure as a fault, masking real wins (e.g.
    a stagger/RX-index change that lowers act_lap peak but doesn't change
    host/plant rate mismatch still got marked NOT ok). The actual health
    signal is whether the plant is still making forward progress: gate on
    applied_hz (cmd_applied_seq advance rate) staying above a floor relative
    to what was asked, which is coalesce-aware — a plant catching up in
    batches still passes; a stalled/wedged plant (applied_hz collapsing
    toward 0) does not.
    """
    reasons = []
    applied_hz = row.get("applied_hz")
    tx_hz = row.get("hz")
    if applied_hz is not None and tx_hz:
        floor = min(float(tx_hz), 200.0) * 0.5
        if float(applied_hz) < floor:
            reasons.append(
                f"applied_hz={applied_hz} below floor={floor:.0f} (tx={tx_hz})"
            )
    if (row.get("plant_fb_hz") or 0) < 20:
        reasons.append(f"plant_fb_hz={row.get('plant_fb_hz')}")
    if row.get("plant_block_hits"):
        reasons.append(f"plant_block={row.get('plant_block_hits')}")
    if need_rs:
        want = float(row.get("rs_travel_cmd") or 0)
        got = float(row.get("rs_delta") or 0)
        end_err = float(row.get("rs_end_err") or 1e9)
        if want >= 0.5 and (got < 0.85 * want or end_err > 0.50):
            reasons.append(
                f"rs_travel got={got:.3f}/{want:.3f} end_err={end_err:.3f}"
            )
    s0, s1 = row.get("servo0_span") or 0, row.get("servo1_span") or 0
    if s0 < 80 or s1 < 80:
        reasons.append(f"servo_span s0={s0} s1={s1}")
    return "; ".join(reasons) if reasons else None


def run_load_cfg(
    hub: ControlsPcbHub,
    *,
    cfg: str,
    rates: Tuple[float, ...],
    trials: int,
    seconds: float,
    soft_hold: float,
    bus: int,
    slot: int,
    motor_id: int,
    use_rs: bool,
    rx_sim: bool,
    notes: List[str],
) -> List[dict]:
    rows: List[dict] = []
    print(f"\n######## LOAD CFG: {cfg} ########")
    if use_rs:
        assign_single_slot(hub, bus=bus, slot=slot, motor_id=motor_id, persist=False)
        hub.set_rx_sim_mask(0)
    else:
        ensure_product_cfg(hub, force=False)
        hub.set_rx_sim_mask(RX_SIM_ACTUATOR if rx_sim else 0)
        # Quiet any lingering desire
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

    print("--- DXL seed ---")
    servo_start = _seed_servos(hub, rates[0])
    servo_hold = dict(servo_start)
    start = 0.01

    for hz in rates:
        for trial in range(1, trials + 1):
            angle = 0.0
            phase_s = float(seconds)
            if use_rs:
                start, angle = _reseed_rs(
                    hub,
                    bus=bus,
                    slot=slot,
                    motor_id=motor_id,
                    hz=hz,
                    fallback=start,
                )
                phase_s = max(
                    phase_s,
                    _rs_phase_seconds(angle, DEFAULT_RATE_RAD_S, soft_hold),
                )
            print(
                f"\n=== {cfg}  tx={hz:.0f} Hz  trial {trial}/{trials}  "
                f"{phase_s:.1f}s ==="
            )
            stats = stream_combo(
                hub,
                hz=hz,
                seconds=phase_s,
                led_brightness=8,
                drive_servos=True,
                drive_led=True,
                servo_rate_rad_s=DEFAULT_RATE_RAD_S,
                servo_start=servo_start,
                soft_hold_s=soft_hold,
                rx_sim=rx_sim,
                rs_slot=slot if use_rs else None,
                start_pos=start,
                angle_rad=angle,
                rate_rad_s=DEFAULT_RATE_RAD_S if use_rs else 0.0,
                kp=8.0,
                kd=0.5,
                accel_s=0.4,
            )
            if stats.get("last_fb") is not None and use_rs:
                start = float(stats["last_fb"])
            if stats.get("servo_end_pos"):
                servo_start = {
                    int(k): float(v) for k, v in stats["servo_end_pos"].items()
                }
                servo_hold = dict(servo_start)

            row = _trial_row(stats, hz=hz, trial=trial, cfg=cfg, angle=angle)
            weird = _weird(row, need_rs=use_rs)
            row["weird"] = weird
            row["ok"] = weird is None
            if weird:
                print(f"  WEIRD: {weird}")
                notes.append(f"{cfg} @{hz:.0f}Hz t{trial}: {weird}")
                # Recalibrate + retry only when the live RS02 path looks wrong.
                if use_rs and (
                    "rs_travel" in weird or "plant_fb" in weird or "applied_hz" in weird
                ):
                    notes.append(f"  → recalibrate CH{bus} and retry once")
                    try:
                        start = _calibrate_bus6(
                            hub, bus=bus, slot=slot, motor_id=motor_id
                        )
                        start, angle = _reseed_rs(
                            hub,
                            bus=bus,
                            slot=slot,
                            motor_id=motor_id,
                            hz=hz,
                            fallback=start,
                        )
                        phase_s = max(
                            float(seconds),
                            _rs_phase_seconds(angle, DEFAULT_RATE_RAD_S, soft_hold),
                        )
                        print(f"  RETRY {cfg} @{hz:.0f}Hz trial {trial}")
                        stats = stream_combo(
                            hub,
                            hz=hz,
                            seconds=phase_s,
                            led_brightness=8,
                            drive_servos=True,
                            drive_led=True,
                            servo_rate_rad_s=DEFAULT_RATE_RAD_S,
                            servo_start=servo_start,
                            soft_hold_s=soft_hold,
                            rx_sim=rx_sim,
                            rs_slot=slot,
                            start_pos=start,
                            angle_rad=angle,
                            rate_rad_s=DEFAULT_RATE_RAD_S,
                            kp=8.0,
                            kd=0.5,
                            accel_s=0.4,
                        )
                        if stats.get("last_fb") is not None:
                            start = float(stats["last_fb"])
                        if stats.get("servo_end_pos"):
                            servo_start = {
                                int(k): float(v)
                                for k, v in stats["servo_end_pos"].items()
                            }
                            servo_hold = dict(servo_start)
                        row = _trial_row(
                            stats, hz=hz, trial=trial, cfg=cfg, angle=angle
                        )
                        weird = _weird(row, need_rs=True)
                        row["weird"] = weird
                        row["ok"] = weird is None
                        row["retried_after_cali"] = True
                        print(
                            f"  retry → {'OK' if row['ok'] else 'STILL WEIRD: ' + str(weird)}"
                        )
                    except Exception as exc:  # noqa: BLE001
                        notes.append(f"cali/retry failed: {exc}")
                        print(f"  cali/retry failed: {exc}")
            else:
                print(
                    f"  OK plant_fb={_fmt(row['plant_fb_hz'],0)}  "
                    f"lag_p95={_fmt(row['cmd_seq_lag_p95'],1)}  "
                    f"act={_fmt(row['act_lap_mean'])}  "
                    f"per={_fmt(row['periph_lap_mean'])}"
                )
            rows.append(row)
            leave_idle(hub, servo_hold=servo_hold)
            time.sleep(0.2)
    return rows


def run_bandwidth(
    hub: ControlsPcbHub, *, seconds: float = 2.0
) -> List[Tuple[float, bool, dict]]:
    print("\n######## BANDWIDTH BASELINE (hold matrix, no DXL/LED teleop) ########")
    by_bus = ensure_product_cfg(hub, force=False)
    hold = LOAD_HOLD
    rows: List[Tuple[float, bool, dict]] = []

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
            print(f"\n--- bw rx_sim={'ON' if rx_sim else 'OFF'} @ {hz:.0f} Hz ---")
            t0 = time.perf_counter()
            for label, desires in phases:
                sec = 1.5 if "blank" in label else seconds
                r = bw_measure(
                    hub, label, desires, seconds=sec, hz=hz, rx_sim=rx_sim
                )
                rows.append((hz, rx_sim, r))
            print(f"  suite wall: {time.perf_counter() - t0:.1f}s")
    probe_leave_idle(hub)
    return rows


def _agg_table(rows: List[dict], cfg: str) -> str:
    subset = [r for r in rows if r["cfg"] == cfg]
    lines = [
        f"| tx Hz | n | plant_fb Hz | raw_fb Hz | applied Hz | "
        f"cmd_seq_lag mean/p95/max | act_lap mean/peak | periph_lap mean/peak | "
        f"s0/s1 span | RS Δ/cmd | ok |",
        f"|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|",
    ]
    for hz in RATES:
        group = [r for r in subset if r["hz"] == hz]
        if not group:
            continue

        def col(key: str) -> List[float]:
            out = []
            for r in group:
                v = r.get(key)
                if v is not None:
                    out.append(float(v))
            return out

        pf = _mean(col("plant_fb_hz"))
        rf = _mean(col("raw_fb_hz"))
        ap = _mean(col("applied_hz"))
        lag_m = _mean(col("cmd_seq_lag_mean"))
        lag_p = _mean(col("cmd_seq_lag_p95"))
        lag_x = max(col("cmd_seq_lag_max")) if col("cmd_seq_lag_max") else None
        act_m = _mean(col("act_lap_mean"))
        act_p = max(col("act_lap_peak")) if col("act_lap_peak") else None
        per_m = _mean(col("periph_lap_mean"))
        per_p = max(col("periph_lap_peak")) if col("periph_lap_peak") else None
        s0 = _mean(col("servo0_span"))
        s1 = _mean(col("servo1_span"))
        rd = _mean(col("rs_delta"))
        rc = _mean(col("rs_travel_cmd"))
        n_ok = sum(1 for r in group if r.get("ok"))
        lines.append(
            f"| {hz:.0f} | {len(group)} | {_fmt(pf, 0)} | {_fmt(rf, 0)} | {_fmt(ap, 0)} | "
            f"{_fmt(lag_m, 2)} / {_fmt(lag_p, 1)} / {_fmt(lag_x, 0)} | "
            f"{_fmt(act_m)} / {_fmt(act_p, 0)} | "
            f"{_fmt(per_m)} / {_fmt(per_p, 0)} | "
            f"{_fmt(s0, 0)}/{_fmt(s1, 0)} | "
            f"{_fmt(rd, 2)}/{_fmt(rc, 2)} | {n_ok}/{len(group)} |"
        )
    return "\n".join(lines)


def _per_trial_table(rows: List[dict], cfg: str) -> str:
    subset = [r for r in rows if r["cfg"] == cfg]
    lines = [
        "| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | "
        "applied | s0 | s1 | rs_Δ | end_err | ok | notes |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for r in subset:
        note = r.get("weird") or ""
        if r.get("retried_after_cali"):
            note = ("retry+cali; " + note).strip("; ")
        lines.append(
            f"| {r['hz']:.0f} | {r['trial']} | {_fmt(r.get('plant_fb_hz'), 0)} | "
            f"{_fmt(r.get('cmd_seq_lag_p95'), 1)} | "
            f"{_fmt(r.get('act_lap_mean'))} | {_fmt(r.get('act_lap_peak'), 0)} | "
            f"{_fmt(r.get('periph_lap_mean'))} | {_fmt(r.get('periph_lap_peak'), 0)} | "
            f"{_fmt(r.get('applied_hz'), 0)} | "
            f"{_fmt(r.get('servo0_span'), 0)} | {_fmt(r.get('servo1_span'), 0)} | "
            f"{_fmt(r.get('rs_delta'), 2)} | {_fmt(r.get('rs_end_err'), 2)} | "
            f"{'Y' if r.get('ok') else 'N'} | {note} |"
        )
    return "\n".join(lines)


def _bw_md(bw_rows: List[Tuple[float, bool, dict]]) -> str:
    if not bw_rows:
        return (
            "## Bandwidth baseline (hold matrix, separate from teleop load)\n\n"
            "_Skipped (`--skip-bw`)._\n"
        )
    chunks = ["## Bandwidth baseline (hold matrix, separate from teleop load)\n"]
    chunks.append(
        "TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.\n"
    )
    for label in BW_FOCUS:
        chunks.append(f"\n### {label}\n")
        chunks.append(
            "| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | "
            "act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |"
        )
        chunks.append(
            "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|"
        )
        for hz in RATES:
            for rx_sim in (False, True):
                matches = [
                    r
                    for h, s, r in bw_rows
                    if h == hz and s == rx_sim and r["label"] == label
                ]
                if not matches:
                    continue
                r = matches[0]
                ok = (
                    "Y"
                    if r.get("ok_lag") and r.get("ok_fb") and r.get("ok_rx", True)
                    else "N"
                )
                chunks.append(
                    f"| {hz:.0f} | {'on' if rx_sim else 'off'} | "
                    f"{_fmt(r.get('raw_fb_hz'), 0)} | "
                    f"{_fmt(r.get('ack_lag_max'), 0)} | "
                    f"{_fmt(r.get('ack_lag_p95'), 0)} | "
                    f"{_fmt(r.get('applied_hz'), 0)} | "
                    f"{_fmt(r.get('lap_mean'))} | "
                    f"{_fmt(r.get('lap_max_max'), 0)} | "
                    f"{_fmt(r.get('periph_lap_mean'))} | "
                    f"{_fmt(r.get('periph_lap_max_max'), 0)} | "
                    f"{_fmt(r.get('pend_max'), 0)} | "
                    f"{_fmt(r.get('rx_fresh_max'), 0)} | {ok} |"
                )

    chunks.append("\n### all×25 delta (RX-sim ON − TX-only)\n")
    chunks.append("| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |")
    chunks.append("|---:|---:|---:|---:|---:|---:|---|---|")
    for hz in RATES:
        try:
            off = next(
                r
                for h, s, r in bw_rows
                if h == hz and not s and r["label"] == "9_all_CH1-6_x25"
            )
            on = next(
                r
                for h, s, r in bw_rows
                if h == hz and s and r["label"] == "9_all_CH1-6_x25"
            )
        except StopIteration:
            chunks.append(f"| {hz:.0f} | — | — | — | — | — | — | — |")
            continue
        chunks.append(
            f"| {hz:.0f} | "
            f"{(on.get('raw_fb_hz') or 0) - (off.get('raw_fb_hz') or 0):+.0f} | "
            f"{(on.get('ack_lag_max') or 0) - (off.get('ack_lag_max') or 0):+d} | "
            f"{(on.get('lap_mean') or 0) - (off.get('lap_mean') or 0):+.1f} | "
            f"{(on.get('periph_lap_mean') or 0) - (off.get('periph_lap_mean') or 0):+.1f} | "
            f"{(on.get('pend_max') or 0) - (off.get('pend_max') or 0):+d} | "
            f"{'Y' if off.get('ok_lag') and off.get('ok_fb') else 'N'} | "
            f"{'Y' if on.get('ok_lag') and on.get('ok_fb') and on.get('ok_rx') else 'N'} |"
        )
    return "\n".join(chunks)


def write_report(
    path: Path,
    *,
    load_rows: List[dict],
    bw_rows: List[Tuple[float, bool, dict]],
    notes: List[str],
    meta: dict,
) -> None:
    ts = datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M %Z")
    parts = [
        f"# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)\n",
        f"Generated: {ts}\n",
        "## Setup\n",
        f"- Port: `{meta.get('port')}`",
        f"- Rates: {', '.join(f'{h:.0f}' for h in meta['rates'])} Hz",
        f"- Trials/rate: {meta['trials']}",
        f"- Soft hold: {meta['soft_hold']} s",
        f"- Base seconds/phase: {meta['seconds']} s (real RS auto-extends for trapezoid)",
        f"- RS02: CH{meta['bus']} id=0x{meta['motor_id']:02X} slot={meta['slot']}",
        f"- DXL: slots 0/1 bounce @ π/4 rad/s; LED FLASH",
        f"- Cali before real matrix: {meta.get('did_cali')}",
        "",
        "## Metric definitions\n",
        "- **tx Hz**: host `send_once` rate for the trial",
        "- **plant_fb Hz**: non-debug feedback frames drained/parsed by the bench",
        "- **raw_fb Hz**: all CDC reader frames (includes debug)",
        "- **applied Hz**: advance rate of `cmd_applied_seq`",
        "- **cmd_seq_lag**: `(host_tx_seq - last_cmd_seq) & 0xFF` (ack lag)",
        "- **act_lap_ms**: PlantTask actuator apply+TX lap",
        "- **periph_lap_ms**: PeripheralTask lap (DXL/LED path)",
        "- **RS Δ/cmd**: measured vs planned MIT travel (real-RS cfg only)",
        "",
        "## A — Real actuator on bus 6 (no ×25 rx_sim) + DXL + LED\n",
        "Single CFG slot on CH6; other actuator slots quieted. Soft-engage + "
        "trapezoid teleop, direction planned to stay inside MIT ±12.57.\n",
        "### Aggregate (mean across trials)\n",
        _agg_table(load_rows, "real_ch6"),
        "",
        "### Per-trial\n",
        _per_trial_table(load_rows, "real_ch6"),
        "",
        "## B — No real actuator on bus 6 (full ×25 ACTUATOR rx_sim) + DXL + LED\n",
        "Product CFG ×25 with ACTUATOR rx_sim mask; no live RS02 MIT. Same DXL+LED load.\n",
        "### Aggregate (mean across trials)\n",
        _agg_table(load_rows, "rx_sim_x25"),
        "",
        "### Per-trial\n",
        _per_trial_table(load_rows, "rx_sim_x25"),
        "",
        _bw_md(bw_rows),
        "",
        "## Notes / anomalies\n",
    ]
    if notes:
        parts.extend(f"- {n}" for n in notes)
    else:
        parts.append("- None recorded.")
    parts.append("")
    parts.append("## Takeaways\n")
    # Auto takeaways from aggregates
    for cfg, title in (
        ("real_ch6", "Real CH6"),
        ("rx_sim_x25", "×25 rx_sim"),
    ):
        parts.append(f"### {title}")
        for hz in RATES:
            g = [r for r in load_rows if r["cfg"] == cfg and r["hz"] == hz]
            if not g:
                continue
            ok_n = sum(1 for r in g if r.get("ok"))
            act = _mean([float(r["act_lap_mean"]) for r in g if r.get("act_lap_mean") is not None])
            per = _mean(
                [float(r["periph_lap_mean"]) for r in g if r.get("periph_lap_mean") is not None]
            )
            fb = _mean([float(r["plant_fb_hz"]) for r in g if r.get("plant_fb_hz") is not None])
            lag = _mean(
                [float(r["cmd_seq_lag_p95"]) for r in g if r.get("cmd_seq_lag_p95") is not None]
            )
            parts.append(
                f"- **{hz:.0f} Hz**: ok {ok_n}/{len(g)}; "
                f"plant_fb≈{_fmt(fb, 0)}; lag_p95≈{_fmt(lag, 1)}; "
                f"act_lap≈{_fmt(act)}; periph_lap≈{_fmt(per)}"
            )
        parts.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")
    print(f"\nWrote {path}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--bus", type=int, default=6)
    ap.add_argument("--motor-id", default="0x70")
    ap.add_argument("--slot", type=int, default=None)
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--soft-hold", type=float, default=0.6)
    ap.add_argument("--bw-seconds", type=float, default=2.0)
    ap.add_argument("--skip-cali", action="store_true")
    ap.add_argument("--skip-real", action="store_true")
    ap.add_argument("--skip-sim", action="store_true")
    ap.add_argument("--skip-bw", action="store_true")
    ap.add_argument(
        "--out",
        default=None,
        help="Markdown output path (default docs/bench-load-matrix-<date>.md)",
    )
    args = ap.parse_args(argv)

    port = args.port or find_cdc_port()
    bus = int(args.bus)
    slot = int(args.slot) if args.slot is not None else CANONICAL_SLOT[bus]
    motor_id = int(str(args.motor_id), 0) & 0xFF
    day = datetime.now().strftime("%Y-%m-%d")
    out = Path(args.out) if args.out else Path("..") / "docs" / f"bench-load-matrix-{day}.md"

    notes: List[str] = []
    load_rows: List[dict] = []
    bw_rows: List[Tuple[float, bool, dict]] = []
    did_cali = False

    print(
        f"Matrix → {out}  port={port}  trials={args.trials}  "
        f"rates={list(RATES)}  CH{bus}/0x{motor_id:02X}"
    )

    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        try:
            leave_idle(hub)
            hub.recover()

            if not args.skip_real and not args.skip_cali:
                _calibrate_bus6(hub, bus=bus, slot=slot, motor_id=motor_id)
                did_cali = True
            elif not args.skip_real:
                notes.append("skipped initial cali (--skip-cali)")

            if not args.skip_real:
                load_rows.extend(
                    run_load_cfg(
                        hub,
                        cfg="real_ch6",
                        rates=RATES,
                        trials=int(args.trials),
                        seconds=float(args.seconds),
                        soft_hold=float(args.soft_hold),
                        bus=bus,
                        slot=slot,
                        motor_id=motor_id,
                        use_rs=True,
                        rx_sim=False,
                        notes=notes,
                    )
                )

            if not args.skip_sim:
                load_rows.extend(
                    run_load_cfg(
                        hub,
                        cfg="rx_sim_x25",
                        rates=RATES,
                        trials=int(args.trials),
                        seconds=float(args.seconds),
                        soft_hold=float(args.soft_hold),
                        bus=bus,
                        slot=slot,
                        motor_id=motor_id,
                        use_rs=False,
                        rx_sim=True,
                        notes=notes,
                    )
                )

            if not args.skip_bw:
                bw_rows = run_bandwidth(hub, seconds=float(args.bw_seconds))
        finally:
            leave_idle(hub)

    write_report(
        out.resolve() if not out.is_absolute() else out,
        load_rows=load_rows,
        bw_rows=bw_rows,
        notes=notes,
        meta={
            "port": port,
            "rates": RATES,
            "trials": int(args.trials),
            "seconds": float(args.seconds),
            "soft_hold": float(args.soft_hold),
            "bus": bus,
            "slot": slot,
            "motor_id": motor_id,
            "did_cali": did_cali,
        },
    )
    n_ok = sum(1 for r in load_rows if r.get("ok"))
    print(f"Load trials OK: {n_ok}/{len(load_rows)}")
    return 0 if n_ok == len(load_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
