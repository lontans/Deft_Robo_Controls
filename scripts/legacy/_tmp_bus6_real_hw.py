#!/usr/bin/env python3
"""Plant load: ×25 ACTUATOR rx_sim + real DXL (π/4) + LED FLASH (no RS02 needed).

  cd scripts
  python _tmp_bus6_real_hw.py --skip-cali              # default: sim-only ×25
  python _tmp_bus6_real_hw.py --with-rs02 --skip-cali  # add real CH6/0x70 MIT
  python _tmp_bus6_real_hw.py --skip-cali --no-rx-sim --rates 40,100,200,500

Real RS teleop defaults to 2π @ π/4 with soft-engage + trapezoid (same shape as
tiny_teleop). Phase length auto-extends if --seconds is too short for the move.
"""
from __future__ import annotations

import argparse
import math
import statistics
import struct
import sys
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, ServoDesire
from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    parse_actuator_feedback,
    parse_feedback_header,
    parse_servo_feedback,
)
from deft_controls_sdk.link.exchange.wire_layout import LED_CMD_OFF
from _tmp_mcp_timing_probe import ensure_product_cfg
from rs02_channel_bringup import (
    CANONICAL_SLOT,
    DEFAULT_TELEOP_ANGLE_RAD,
    DEFAULT_TELEOP_RATE_RAD_S,
    PROTO_ROBSTRIDE,
    RS02_P_MAX,
    RS02_P_MIN,
    assign_single_slot,
    rs02_near,
    rs02_plan_angle,
    rs02_resolve_start,
    sample_position,
    seed_idle_at_fb,
    tiny_teleop,
)

# plant_config.c servo_table limits (neck travel)
SERVO_CFG = (
    {"slot": 0, "id": 1, "pos_min": 1024, "pos_max": 3072},
    {"slot": 1, "id": 2, "pos_min": 700, "pos_max": 2500},
)

DXL_TICKS_PER_REV = 4096.0
LED_MODE_FLASH = 2
DEFAULT_RATE_RAD_S = math.pi / 4.0
RX_SIM_ACTUATOR = 0x1
# Soft hold — kp>0 so rx_sim_actuator_on_apply fires for every staged slot.
LOAD_HOLD = ActuatorDesire(position=0.01, velocity=0.0, kp=2.0, kd=0.5, torque=0.0)


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _servo_mid(cfg: dict) -> int:
    return (int(cfg["pos_min"]) + int(cfg["pos_max"])) // 2


def _pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


def _istat(xs: List[int]) -> str:
    if not xs:
        return "n/a"
    return (
        f"n={len(xs)} mean={statistics.mean(xs):.2f} "
        f"p95={_pct([float(x) for x in xs], 95):.0f} max={max(xs)}"
    )


def _drain_all(hub: ControlsPcbHub):
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        yield frame


def leave_idle(
    hub: ControlsPcbHub,
    *,
    servo_hold: Optional[Dict[int, float]] = None,
) -> None:
    """Always stop DXL torque + LED + rx_sim — safe from finally / Ctrl-C paths."""
    try:
        hub.set_rx_sim_mask(0)
        # Explicit torque-off at last pose before clearing session (servo_id=0).
        for c in SERVO_CFG:
            pos = int((servo_hold or {}).get(c["slot"], _servo_mid(c)))
            hub.set_servo(
                c["slot"],
                ServoDesire(
                    servo_id=c["id"],
                    native_step_position=pos,
                    torque_enable=False,
                    operating_mode=3,
                ),
                send=False,
            )
        hub.set_led(LedDesire(mode=0, master_brightness=0, led_count=0), send=False)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        hub.send_once()
        time.sleep(0.08)
        _conn(hub).clear_servos(send=False)
        hub.set_led(LedDesire(mode=0, master_brightness=0, led_count=0), send=False)
        hub.set_rx_sim_mask(0)
        hub.send_once()
        time.sleep(0.35)
        hub.send_once()
        print("leave_idle: torque-off + LED OFF + rx_sim off + servo clear")
    except Exception as exc:  # noqa: BLE001 — best-effort shutdown
        print(f"leave_idle: WARN {exc}")


def _servo_cmd_at_rate(
    *,
    pos: float,
    direction: float,
    dt: float,
    rate_steps_s: float,
    pos_min: int,
    pos_max: int,
) -> tuple[float, float]:
    nxt = pos + direction * rate_steps_s * dt
    if nxt >= pos_max:
        return float(pos_max), -1.0
    if nxt <= pos_min:
        return float(pos_min), 1.0
    return nxt, direction


def _trap_profile(
    travel: float, v_max: float, accel_s: float = 0.4
) -> Tuple[float, float, float, float, float]:
    """Return (t_acc, t_cruise, t_total, v_peak, d_acc) for a trapezoid move."""
    t_acc = max(0.05, float(accel_s))
    a = v_max / t_acc
    d_acc = 0.5 * v_max * t_acc
    if 2.0 * d_acc >= travel:
        v_peak = math.sqrt(max(a * travel, 1e-9))
        t_acc = v_peak / a
        t_cruise = 0.0
        d_acc = 0.5 * v_peak * t_acc
    else:
        v_peak = v_max
        t_cruise = (travel - 2.0 * d_acc) / v_peak
    t_total = 2.0 * t_acc + t_cruise
    return t_acc, t_cruise, t_total, v_peak, d_acc


def _trap_at(
    t: float,
    *,
    t_acc: float,
    t_cruise: float,
    t_total: float,
    v_peak: float,
    d_acc: float,
    travel: float,
) -> Tuple[float, float]:
    """Path distance and speed along trapezoid (non-negative)."""
    a = v_peak / max(t_acc, 1e-9)
    d_cruise = max(0.0, travel - 2.0 * d_acc)
    if t <= 0.0:
        return 0.0, 0.0
    if t < t_acc:
        return 0.5 * a * t * t, a * t
    if t < t_acc + t_cruise:
        dt_c = t - t_acc
        return d_acc + v_peak * dt_c, v_peak
    if t < t_total:
        td = t - t_acc - t_cruise
        v = max(0.0, v_peak - a * td)
        s = d_acc + d_cruise + v_peak * td - 0.5 * a * td * td
        return min(s, travel), v
    return travel, 0.0


def _rs_phase_seconds(
    angle_rad: float,
    rate_rad_s: float,
    soft_hold_s: float,
    *,
    accel_s: float = 0.4,
    end_hold_s: float = 0.35,
) -> float:
    """Wall time for soft-engage + trapezoid travel + short end hold."""
    travel = abs(float(angle_rad))
    v_max = abs(float(rate_rad_s))
    if travel < 1e-3 or v_max < 1e-3:
        return max(0.0, float(soft_hold_s))
    _, _, t_total, _, _ = _trap_profile(travel, v_max, accel_s)
    return float(soft_hold_s) + t_total + float(end_hold_s)


def stream_combo(
    hub: ControlsPcbHub,
    *,
    hz: float,
    seconds: float,
    led_brightness: int,
    drive_servos: bool,
    drive_led: bool,
    servo_rate_rad_s: float,
    servo_start: Optional[Dict[int, float]] = None,
    soft_hold_s: float = 0.6,
    rx_sim: bool = True,
    rs_slot: Optional[int] = None,
    start_pos: float = 0.0,
    angle_rad: float = 0.0,
    rate_rad_s: float = 0.0,
    kp: float = 8.0,
    kd: float = 0.5,
    accel_s: float = 0.4,
) -> dict:
    """DXL + LED FLASH + either ×25 ACTUATOR rx_sim or a single real RS02 slot.

    soft_hold_s: hold present pose after torque-on before ramping (kills wake jerk).
    Real RS path uses soft engage + trapezoid velocity (same shape as tiny_teleop).
    rx_sim=False: no synthetic actuators — only command rs_slot (real HW).
    """
    dt = 1.0 / max(hz, 1.0)
    t0 = time.perf_counter()
    t_end = t0 + seconds
    next_t = t0
    t_ramp = t0 + max(0.0, soft_hold_s)

    pos_cmd = float(start_pos)
    planned = rs02_plan_angle(pos_cmd, angle_rad) if abs(angle_rad) > 1e-6 else 0.0
    sign = 1.0 if planned >= 0.0 else -1.0
    travel = abs(planned)
    use_rs = rs_slot is not None and travel > 1e-3 and abs(rate_rad_s) > 1e-3
    v_max = abs(rate_rad_s) if use_rs else 0.0
    if use_rs:
        t_acc, t_cruise, t_move, v_peak, d_acc = _trap_profile(
            travel, v_max, accel_s
        )
        if abs(abs(angle_rad) - travel) > 0.05 or (
            angle_rad != 0.0 and (planned > 0) != (angle_rad > 0)
        ):
            print(
                f"  RS plan: start={pos_cmd:+.4f} angle {angle_rad:+.4f} → "
                f"{planned:+.4f} (MIT [{RS02_P_MIN}, {RS02_P_MAX}])"
            )
    else:
        t_acc = t_cruise = t_move = v_peak = d_acc = 0.0

    servo_seen: Dict[int, List[int]] = {0: [], 1: []}
    servo_err = 0
    led_modes: List[int] = []
    last_fb = pos_cmd
    pos0 = pos_cmd
    act_samples = 0
    plant_block_hits = 0

    servo_pos = {
        c["slot"]: float(
            (servo_start or {}).get(c["slot"], _servo_mid(c))
        )
        for c in SERVO_CFG
    }
    # Start toward mid so we don't sit on a hard stop with dir=+1.
    servo_dir = {
        c["slot"]: (
            -1.0
            if servo_pos[c["slot"]] >= 0.5 * (c["pos_min"] + c["pos_max"])
            else 1.0
        )
        for c in SERVO_CFG
    }
    rate_steps_s = abs(servo_rate_rad_s) / (2.0 * math.pi) * DXL_TICKS_PER_REV

    ack_lags: List[int] = []
    lap_ms: List[int] = []
    lap_max_sticky: List[int] = []
    periph_lap_ms: List[int] = []
    periph_lap_max: List[int] = []
    img_lags: List[int] = []
    applied_lags: List[int] = []
    stage_vs_applied: List[int] = []
    img_id_samples: List[tuple] = []
    ticks_pend: List[int] = []
    ticks_svc: List[int] = []
    rx_fresh_samples: List[int] = []
    last_sent: Optional[int] = None
    last_sent_full: Optional[int] = None
    reader = _conn(hub).reader
    tf0 = reader.total_frames
    plant_fb_n = 0
    if rx_sim:
        held_slots = list(range(ACTUATOR_COUNT))
    elif use_rs and rs_slot is not None:
        held_slots = [rs_slot]
    else:
        held_slots = []

    hub.set_rx_sim_mask(RX_SIM_ACTUATOR if rx_sim else 0)
    if drive_led:
        hub.set_led(
            LedDesire(mode=LED_MODE_FLASH, master_brightness=led_brightness, led_count=0),
            send=False,
        )
    else:
        hub.set_led(LedDesire(mode=0), send=False)
    if not drive_servos:
        _conn(hub).clear_servos(send=False)

    while time.perf_counter() < t_end:
        now = time.perf_counter()
        # RS teleop clock starts after soft_hold (DXL + RS soft-engage settle).
        elapsed = max(0.0, now - t_ramp) if use_rs else (now - t0)

        if rx_sim:
            desires = {s: LOAD_HOLD for s in range(ACTUATOR_COUNT)}
        else:
            desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        if use_rs and rs_slot is not None:
            if now < t_ramp:
                # Soft engage: hold trusted start pose at v=0 (no step into cruise).
                cmd = pos_cmd
                vel = 0.0
            else:
                s_path, v_path = _trap_at(
                    elapsed,
                    t_acc=t_acc,
                    t_cruise=t_cruise,
                    t_total=t_move,
                    v_peak=v_peak,
                    d_acc=d_acc,
                    travel=travel,
                )
                cmd = pos_cmd + sign * s_path
                vel = sign * v_path
            desires[rs_slot] = ActuatorDesire(position=cmd, velocity=vel, kp=kp, kd=kd)

        _conn(hub).set_actuators(desires, send=False)

        if drive_servos:
            ramping = now >= t_ramp
            for c in SERVO_CFG:
                sslot = c["slot"]
                if ramping:
                    servo_pos[sslot], servo_dir[sslot] = _servo_cmd_at_rate(
                        pos=servo_pos[sslot],
                        direction=servo_dir[sslot],
                        dt=dt,
                        rate_steps_s=rate_steps_s,
                        pos_min=int(c["pos_min"]),
                        pos_max=int(c["pos_max"]),
                    )
                hub.set_servo(
                    sslot,
                    ServoDesire(
                        servo_id=c["id"],
                        native_step_position=int(round(servo_pos[sslot])),
                        torque_enable=True,
                        operating_mode=3,
                    ),
                    send=False,
                )

        for raw in _drain_all(hub):
            hdr = parse_feedback_header(raw)
            if hdr is None or hdr.get("is_debug"):
                continue
            plant_fb_n += 1
            if len(raw) >= LED_CMD_OFF + 2:
                led_modes.append(struct.unpack_from("<H", raw, LED_CMD_OFF)[0] & 0x1F)
            if int(hdr.get("plant_block", 0) or 0) != 0:
                plant_block_hits += 1
            ack = int(hdr["last_cmd_seq"]) & 0xFF
            if last_sent is not None:
                lag = (last_sent - ack) & 0xFF
                if lag <= 128:
                    ack_lags.append(lag)
            img_id = hdr.get("cmd_rx_seq", hdr.get("last_image_id"))
            app_id = hdr.get("cmd_applied_seq", hdr.get("last_applied_image_id"))
            if img_id is not None and app_id is not None:
                img_id_samples.append(
                    (time.perf_counter(), int(img_id), int(app_id))
                )
                if last_sent_full is not None:
                    img_lags.append(int(last_sent_full) - int(img_id))
                    applied_lags.append(int(last_sent_full) - int(app_id))
                stage_vs_applied.append(int(img_id) - int(app_id))
            act = hdr.get("act_lap_ms", hdr.get("lap_ms"))
            if act is not None:
                lap_ms.append(int(act))
            act_pk = hdr.get("act_lap_peak_ms", hdr.get("lap_max_ms"))
            if act_pk is not None:
                lap_max_sticky.append(int(act_pk))
            if hdr.get("periph_lap_ms") is not None:
                periph_lap_ms.append(int(hdr["periph_lap_ms"]))
            per_pk = hdr.get("periph_lap_peak_ms", hdr.get("periph_lap_max_ms"))
            if per_pk is not None:
                periph_lap_max.append(int(per_pk))
            if hdr.get("ticks_pending") is not None:
                ticks_pend.append(int(hdr["ticks_pending"]))
            if hdr.get("ticks_svc") is not None:
                ticks_svc.append(int(hdr["ticks_svc"]))

            n_fresh = 0
            for s in held_slots:
                act_s = parse_actuator_feedback(raw, s)
                if act_s is None:
                    continue
                want = float(desires[s].position)
                if abs(float(act_s["position"]) - want) < 0.08:
                    n_fresh += 1
            rx_fresh_samples.append(n_fresh)

            if rs_slot is not None:
                act = parse_actuator_feedback(raw, rs_slot)
                if act is not None:
                    last_fb = float(act["position"])
                    act_samples += 1
                    # Soft-engage may adopt plant FB only when it matches the
                    # trusted start — never a stale rail-opposite reading.
                    if use_rs and now < t_ramp and rs02_near(last_fb, pos_cmd, 0.40):
                        pos_cmd = last_fb
                        pos0 = pos_cmd
            for sslot in (0, 1):
                sv = parse_servo_feedback(raw, sslot)
                if sv is None:
                    continue
                servo_seen[sslot].append(int(sv["present_position"]))
                if sv.get("hw_err_any"):
                    servo_err += 1

        hub.send_once()
        sent = _conn(hub)._last_sent_seq  # noqa: SLF001
        last_sent_full = int(sent) if sent is not None else None
        last_sent = (sent & 0xFF) if sent is not None else None

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    elapsed_wall = max(time.perf_counter() - t0, 1e-6)
    raw_fb = reader.total_frames - tf0
    raw_fb_hz = raw_fb / elapsed_wall
    plant_fb_hz = plant_fb_n / elapsed_wall

    staged_hz = None
    applied_hz = None
    if len(img_id_samples) >= 2:
        t_a, s0, a0 = img_id_samples[0]
        t_b, s1, a1 = img_id_samples[-1]
        dt_s = t_b - t_a
        if dt_s > 0:
            staged_hz = (s1 - s0) / dt_s
            applied_hz = (a1 - a0) / dt_s

    servo_moved = {}
    for sslot, samples in servo_seen.items():
        if len(samples) < 2:
            servo_moved[sslot] = 0
        else:
            servo_moved[sslot] = max(samples) - min(samples)

    return {
        "last_fb": last_fb,
        "act_samples": act_samples,
        "servo_moved": servo_moved,
        "servo_err": servo_err,
        "plant_block_hits": plant_block_hits,
        # Signed progress from soft-engage origin (not abs to a rail-stale pose).
        "rs_delta": abs(last_fb - pos0) if use_rs else 0.0,
        "rs_end_err": abs(last_fb - (pos0 + sign * travel)) if use_rs else 0.0,
        "rs_travel_cmd": travel if use_rs else 0.0,
        "rs_t_move": t_move if use_rs else 0.0,
        "rs_start": pos0 if use_rs else None,
        "rs_target": (pos0 + sign * travel) if use_rs else None,
        "raw_fb_hz": raw_fb_hz,
        "plant_fb_hz": plant_fb_hz,
        "raw_fb_frames": raw_fb,
        # cmd_seq_lag = (last_sent_seq - last_cmd_seq) & 0xFF  (aka ack lag)
        "cmd_seq_lag_max": max(ack_lags) if ack_lags else None,
        "cmd_seq_lag_mean": statistics.mean(ack_lags) if ack_lags else None,
        "cmd_seq_lag_p95": _pct([float(x) for x in ack_lags], 95),
        "cmd_seq_lag_n": len(ack_lags),
        "cmd_seq_lag_str": _istat(ack_lags),
        # keep old keys as aliases for any external consumers
        "ack_lag_max": max(ack_lags) if ack_lags else None,
        "ack_lag_mean": statistics.mean(ack_lags) if ack_lags else None,
        "ack_lag_p95": _pct([float(x) for x in ack_lags], 95),
        "ack_lag_n": len(ack_lags),
        "ack_lag_str": _istat(ack_lags),
        "act_lap_mean": statistics.mean(lap_ms) if lap_ms else None,
        "act_lap_peak": max(lap_max_sticky) if lap_max_sticky else None,
        "periph_lap_mean": statistics.mean(periph_lap_ms) if periph_lap_ms else None,
        "periph_lap_peak": max(periph_lap_max) if periph_lap_max else None,
        "img_lag_str": _istat(img_lags),
        "app_lag_str": _istat(applied_lags),
        "stage_app_gap_str": _istat(stage_vs_applied),
        "img_lag_max": max(img_lags) if img_lags else None,
        "app_lag_max": max(applied_lags) if applied_lags else None,
        "staged_hz": staged_hz,
        "applied_hz": applied_hz,
        "lap_ms_str": _istat(lap_ms),
        "lap_max_window": max(lap_ms) if lap_ms else None,
        "lap_max_sticky": max(lap_max_sticky) if lap_max_sticky else None,
        "periph_lap_str": _istat(periph_lap_ms),
        "periph_lap_max_str": _istat(periph_lap_max),
        "periph_lap_max_max": max(periph_lap_max) if periph_lap_max else None,
        "ticks_pend_str": _istat(ticks_pend),
        "ticks_svc_str": _istat(ticks_svc),
        "servo_rate_steps_s": rate_steps_s,
        "staged_slots": len(held_slots),
        "rx_fresh_mean": statistics.mean(rx_fresh_samples) if rx_fresh_samples else None,
        "rx_fresh_max": max(rx_fresh_samples) if rx_fresh_samples else None,
        "rx_fresh_min": min(rx_fresh_samples) if rx_fresh_samples else None,
        "rx_fresh_n": len(rx_fresh_samples),
        "led_mode_last": led_modes[-1] if led_modes else None,
        "led_mode_unique": sorted(set(led_modes)) if led_modes else [],
        "servo_end_pos": {k: float(v) for k, v in servo_pos.items()},
    }


def _parse_rates(s: str) -> List[float]:
    out: List[float] = []
    for part in str(s).split(","):
        part = part.strip()
        if not part:
            continue
        out.append(float(part))
    if not out:
        raise ValueError("empty --rates")
    return out


def _print_phase(stats: dict, hz: float) -> None:
    print(
        f"  staged={stats['staged_slots']}  "
        f"servo_moved={stats['servo_moved']} err_flags={stats['servo_err']}  "
        f"plant_block_hits={stats['plant_block_hits']}  "
        f"led_mode={stats.get('led_mode_last')} unique={stats.get('led_mode_unique')}"
    )
    if stats.get("act_samples"):
        print(
            f"  RS02: {stats.get('rs_start'):+.4f}→{stats.get('rs_target'):+.4f}  "
            f"Δ={stats.get('rs_delta', 0):.4f}/{stats.get('rs_travel_cmd', 0):.4f} rad  "
            f"end_err={stats.get('rs_end_err', 0):.4f}  "
            f"trap={stats.get('rs_t_move', 0):.1f}s  "
            f"samples={stats.get('act_samples')}  last_fb={stats.get('last_fb')}"
        )
    print(
        f"  fb: raw_hz={stats['raw_fb_hz']:.1f} plant_hz={stats['plant_fb_hz']:.1f} "
        f"frames={stats['raw_fb_frames']}"
    )
    if stats.get("rx_fresh_mean") is not None:
        print(
            f"  rx_fresh: mean={stats['rx_fresh_mean']:.1f}/{ACTUATOR_COUNT}  "
            f"min={stats['rx_fresh_min']} max={stats['rx_fresh_max']}"
        )
    print(f"  cmd_seq_lag: {stats['cmd_seq_lag_str']}")
    print(f"  cmd_rx_lag:  {stats['img_lag_str']}")
    print(f"  cmd_applied_lag: {stats['app_lag_str']}")
    if stats.get("staged_hz") is not None and stats.get("applied_hz") is not None:
        print(
            f"  cmd seq advance: rx_hz~{stats['staged_hz']:.1f}  "
            f"applied_hz~{stats['applied_hz']:.1f}  (host target {hz:.0f})"
        )
    print(
        f"  act_lap_ms: {stats['lap_ms_str']}  "
        f"sample_max={stats['lap_max_window']} peak={stats['lap_max_sticky']}"
    )
    print(
        f"  periph_lap_ms: {stats['periph_lap_str']}  "
        f"peak={stats['periph_lap_max_str']}"
    )
    print(f"  ticks_pending: {stats['ticks_pend_str']}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="×25 ACTUATOR rx_sim + DXL + LED flash (RS02 optional)"
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--hz", type=float, default=None, help="Single host TX rate (legacy)")
    ap.add_argument(
        "--rates",
        default="40,100,200",
        help="Comma host TX rates to sweep (default 40,100,200). Ignored if --hz set.",
    )
    ap.add_argument("--seconds", type=float, default=8.0, help="Seconds per rate")
    ap.add_argument(
        "--soft-hold",
        type=float,
        default=0.6,
        help="Seconds to hold present pose after torque-on before DXL ramp",
    )
    ap.add_argument(
        "--servo-rate",
        type=float,
        default=DEFAULT_RATE_RAD_S,
        help="DXL goal rate rad/s (default π/4 → 512 tick/s)",
    )
    ap.add_argument("--led-brightness", type=int, default=8)
    ap.add_argument("--no-servo", action="store_true")
    ap.add_argument("--no-led", action="store_true")
    ap.add_argument(
        "--with-rs02",
        action="store_true",
        help="Probe/drive real RS02 (default bus=6 id=0x70)",
    )
    ap.add_argument(
        "--no-rx-sim",
        action="store_true",
        help="No ×25 ACTUATOR rx_sim — real RS02 slot only (+ LED/DXL). Implies --with-rs02.",
    )
    ap.add_argument("--bus", type=int, default=6, choices=range(1, 7))
    ap.add_argument("--motor-id", default="0x70")
    ap.add_argument("--slot", type=int, default=None)
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--kd", type=float, default=0.5)
    ap.add_argument(
        "--angle",
        type=float,
        default=DEFAULT_TELEOP_ANGLE_RAD,
        help="RS02 travel rad (default 2π = full turn)",
    )
    ap.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_TELEOP_RATE_RAD_S,
        help="RS02 cruise rate rad/s (default π/4)",
    )
    ap.add_argument(
        "--accel",
        type=float,
        default=0.4,
        help="RS02 trapezoid accel/decel seconds (default 0.4)",
    )
    ap.add_argument("--skip-cali", action="store_true")
    ap.add_argument("--persist", action="store_true")
    args = ap.parse_args(argv)

    bus = int(args.bus)
    slot = int(args.slot) if args.slot is not None else CANONICAL_SLOT[bus]
    motor_id = int(str(args.motor_id), 0) & 0xFF
    servo_steps = abs(float(args.servo_rate)) / (2.0 * math.pi) * DXL_TICKS_PER_REV
    no_rx_sim = bool(args.no_rx_sim)
    with_rs = bool(args.with_rs02) or no_rx_sim
    if no_rx_sim and not with_rs:
        print("FAIL: --no-rx-sim needs a real RS02 (--with-rs02)", file=sys.stderr)
        return 2
    rates: List[float] = (
        [float(args.hz)] if args.hz is not None else _parse_rates(args.rates)
    )
    # Real RS phases must cover soft-hold + full trapezoid; don't truncate early.
    phase_seconds = float(args.seconds)
    if with_rs:
        need_s = _rs_phase_seconds(
            float(args.angle),
            float(args.rate),
            float(args.soft_hold),
            accel_s=float(args.accel),
        )
        if phase_seconds + 1e-6 < need_s:
            print(
                f"note: --seconds {phase_seconds:.1f}s too short for "
                f"{math.degrees(abs(float(args.angle))):.0f}° @ "
                f"{abs(float(args.rate)):.4f} rad/s → using {need_s:.1f}s/phase"
            )
            phase_seconds = need_s

    print(
        f"Load: "
        f"{'REAL RS02-only (no rx_sim)' if no_rx_sim else f'staged={ACTUATOR_COUNT} ACTUATOR_rx_sim=ON'}  "
        f"RS02={'CH%d/0x%02X slot=%d' % (bus, motor_id, slot) if with_rs else 'OFF'}  "
        f"LED=FLASH×300  DXL={float(args.servo_rate):.4f} rad/s ({servo_steps:.0f} tick/s)  "
        f"host rates={rates} × {phase_seconds:.1f}s  soft_hold={float(args.soft_hold):.2f}s"
    )
    if with_rs:
        print(
            f"  RS teleop: {math.degrees(abs(float(args.angle))):.0f}° @ "
            f"{float(args.rate):+.4f} rad/s  accel={float(args.accel):.2f}s "
            f"(soft engage + trapezoid, reseed each rate)"
        )

    phase_rows: List[Tuple[float, dict]] = []
    servo_hold: Dict[int, float] = {}
    overall_ok = True

    with ControlsPcbHub.connect(args.port) as hub:
        try:
            leave_idle(hub)  # clean any leftover torque/LED from prior hung run
            hub.recover()
            hub.set_rx_sim_mask(0)
            _conn(hub).set_actuators(
                {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
            )

            start = 0.01
            rs_slot: Optional[int] = None
            if no_rx_sim:
                print("\n--- CFG single real RS02 (quiet all other slots) ---")
                print(f"\n--- probe 0x{motor_id:02X} on CH{bus} ---")
                resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
                if resp is None or not resp.get("found"):
                    print("FAIL: RS02 probe miss — is motor on CH6 id 0x70?")
                    return 1
                probe_pos = float(resp.get("position", 0.0))
                print(f"  found pos={probe_pos:+.4f}")
                assign_single_slot(
                    hub,
                    bus=bus,
                    slot=slot,
                    motor_id=motor_id,
                    persist=bool(args.persist),
                )
                seed_idle_at_fb(hub, slot, probe_pos)
                hub.refresh_feedback(slots=[slot], seconds=0.4, hz=rates[0])
                if not args.skip_cali:
                    print("\n--- encoder cali ---")
                    ok_cali = hub.debug.calibrate_robstride(
                        bus=bus, motor_id=motor_id, cal_listen_s=28.0
                    )
                    print(f"  cali={'ok' if ok_cali else 'FAIL'}")
                    en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
                    if en and en.get("found"):
                        probe_pos = float(en["position"])
                        seed_idle_at_fb(hub, slot, probe_pos)
                plant_fb = sample_position(hub, slot)
                start = rs02_resolve_start(probe_pos, plant_fb) or probe_pos
                if plant_fb is not None and not rs02_near(probe_pos, plant_fb):
                    print(
                        f"  note: stale plant FB {plant_fb:+.4f} vs probe "
                        f"{probe_pos:+.4f} — trusting probe"
                    )
                rs_slot = slot
            elif with_rs:
                print("\n--- CFG product ×25 + real RS02 slot ---")
                ensure_product_cfg(hub, force=False)
                print(f"\n--- probe 0x{motor_id:02X} on CH{bus} ---")
                resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
                if resp is None or not resp.get("found"):
                    print("FAIL: RS02 probe miss (omit --with-rs02 for sim-only)")
                    return 1
                probe_pos = float(resp.get("position", 0.0))
                print(f"  found pos={probe_pos:+.4f}")
                hub.debug.cfg_set_slot(
                    slot=slot,
                    bus=bus,
                    protocol=PROTO_ROBSTRIDE,
                    motor_id=motor_id,
                    enabled=True,
                    persist=bool(args.persist),
                )
                seed_idle_at_fb(hub, slot, probe_pos)
                hub.refresh_feedback(slots=[slot], seconds=0.4, hz=rates[0])
                if not args.skip_cali:
                    print("\n--- encoder cali ---")
                    ok_cali = hub.debug.calibrate_robstride(
                        bus=bus, motor_id=motor_id, cal_listen_s=28.0
                    )
                    print(f"  cali={'ok' if ok_cali else 'FAIL'}")
                    en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
                    if en and en.get("found"):
                        probe_pos = float(en["position"])
                        seed_idle_at_fb(hub, slot, probe_pos)
                plant_fb = sample_position(hub, slot)
                start = rs02_resolve_start(probe_pos, plant_fb) or probe_pos
                if plant_fb is not None and not rs02_near(probe_pos, plant_fb):
                    print(
                        f"  note: stale plant FB {plant_fb:+.4f} vs probe "
                        f"{probe_pos:+.4f} — trusting probe"
                    )
                rs_slot = slot
            else:
                print("\n--- CFG product ×25 ---")
                ensure_product_cfg(hub, force=False)
                print("\n--- RS02 skipped (sim-only; no probe / no bus contention) ---")

            servo_start: Dict[int, float] = {}
            if not args.no_servo:
                print("\n--- DXL present-position seed ---")
                from _tmp_dxl_one import sample_servo_fb

                for c in SERVO_CFG:
                    fb = sample_servo_fb(
                        hub,
                        c["slot"],
                        servo_id=c["id"],
                        timeout_s=1.2,
                        hz=rates[0],
                    )
                    if fb is None:
                        fb = _servo_mid(c)
                        print(f"  slot {c['slot']}: no FB — fallback mid={fb}")
                    else:
                        lo, hi = int(c["pos_min"]), int(c["pos_max"])
                        fb = max(lo, min(hi, int(fb)))
                        print(f"  slot {c['slot']}: present={fb}")
                        sample_servo_fb(
                            hub,
                            c["slot"],
                            servo_id=c["id"],
                            hold_pos=fb,
                            timeout_s=0.4,
                            hz=rates[0],
                        )
                    servo_start[c["slot"]] = float(fb)
                servo_hold = dict(servo_start)
                # End seed session so stream_combo soft-starts with profile+latch.
                _conn(hub).clear_servos(send=True)
                time.sleep(0.15)

            for hz in rates:
                phase_angle = float(args.angle) if with_rs else 0.0
                if rs_slot is not None:
                    # Probe is authoritative. Plant FB can sit on the opposite MIT
                    # rail (±12.57) after a clamp — using that as start_pos makes
                    # kp>0 command a ~25 rad snap ("catch") at phase start.
                    en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
                    probe = (
                        float(en["position"])
                        if en and en.get("found")
                        else None
                    )
                    seed_idle_at_fb(
                        hub, rs_slot, probe if probe is not None else start
                    )
                    hub.refresh_feedback(slots=[rs_slot], seconds=0.25, hz=float(hz))
                    plant_fb = sample_position(hub, rs_slot)
                    resolved = rs02_resolve_start(probe, plant_fb)
                    if resolved is None:
                        print("FAIL: no RS02 start pose from probe/plant FB")
                        return 1
                    if (
                        probe is not None
                        and plant_fb is not None
                        and not rs02_near(probe, plant_fb)
                    ):
                        print(
                            f"  note: stale plant FB {plant_fb:+.4f} vs probe "
                            f"{probe:+.4f} — trusting probe"
                        )
                    start = resolved
                    phase_angle = rs02_plan_angle(start, phase_angle)
                    print(
                        f"  RS start_pos={start:+.4f}  "
                        f"angle={phase_angle:+.4f} rad "
                        f"(reseeded, MIT [{RS02_P_MIN}, {RS02_P_MAX}])"
                    )

                # Per-phase duration from the *planned* angle (may flip/shrink).
                this_seconds = phase_seconds
                if with_rs and abs(phase_angle) > 1e-6:
                    this_seconds = max(
                        phase_seconds,
                        _rs_phase_seconds(
                            phase_angle,
                            float(args.rate),
                            float(args.soft_hold),
                            accel_s=float(args.accel),
                        ),
                    )

                print(
                    f"\n=== combo @ {hz:.0f} Hz × {this_seconds:.1f}s "
                    f"(soft_hold {float(args.soft_hold):.2f}s) ==="
                )
                stats = stream_combo(
                    hub,
                    hz=float(hz),
                    seconds=this_seconds,
                    led_brightness=int(args.led_brightness),
                    drive_servos=not args.no_servo,
                    drive_led=not args.no_led,
                    servo_rate_rad_s=float(args.servo_rate),
                    servo_start=servo_start or None,
                    soft_hold_s=float(args.soft_hold),
                    rx_sim=not no_rx_sim,
                    rs_slot=rs_slot,
                    start_pos=start,
                    angle_rad=phase_angle if with_rs else 0.0,
                    rate_rad_s=float(args.rate),
                    kp=float(args.kp),
                    kd=float(args.kd),
                    accel_s=float(args.accel),
                )
                _print_phase(stats, hz)
                phase_rows.append((hz, stats))
                if stats.get("last_fb") is not None and rs_slot is not None:
                    start = float(stats["last_fb"])
                if stats.get("servo_end_pos"):
                    servo_start = {
                        int(k): float(v) for k, v in stats["servo_end_pos"].items()
                    }
                    servo_hold = dict(servo_start)

                servo_ok = True
                if not args.no_servo:
                    moved = stats.get("servo_moved") or {}
                    servo_ok = (
                        all(v >= 80 for v in moved.values())
                        and stats.get("servo_err", 0) == 0
                    )
                # Gate on p95 — single USB hiccups spike max without hurting teleop.
                lag_lim = 2
                lag_p95 = stats.get("cmd_seq_lag_p95", stats.get("ack_lag_p95"))
                lag_ok = lag_p95 is not None and float(lag_p95) <= lag_lim
                travel_ok = True
                if with_rs:
                    want = float(stats.get("rs_travel_cmd") or 0.0)
                    got = float(stats.get("rs_delta") or 0.0)
                    end_err = float(stats.get("rs_end_err") or 1e9)
                    # Require real progress toward planned target (not rail-span abs).
                    travel_ok = want < 0.5 or (
                        got >= 0.85 * want and end_err <= 0.50
                    )
                fb_ok = (stats.get("plant_fb_hz") or 0) >= 20.0
                led_ok = args.no_led or (stats.get("led_mode_last") == LED_MODE_FLASH)
                phase_ok = (
                    stats.get("plant_block_hits", 0) == 0
                    and servo_ok
                    and lag_ok
                    and fb_ok
                    and led_ok
                    and travel_ok
                )
                print(
                    f"  PHASE {'PASS' if phase_ok else 'FAIL'}: "
                    f"servo={servo_ok} cmd_seq_lag_p95={lag_ok}(≤{lag_lim}) "
                    f"travel={travel_ok} fb={fb_ok} led={led_ok}"
                )
                overall_ok = overall_ok and phase_ok

            if with_rs and (not no_rx_sim) and abs(float(args.angle)) > 1e-3:
                print("\n--- dedicated RS02 teleop (×25 rx_sim held) ---")
                hub.set_rx_sim_mask(RX_SIM_ACTUATOR)
                en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
                hint = (
                    float(en["position"])
                    if en and en.get("found")
                    else (phase_rows[-1][1]["last_fb"] if phase_rows else start)
                )
                desires = {s: LOAD_HOLD for s in range(ACTUATOR_COUNT)}
                desires[slot] = ActuatorDesire(position=hint, kp=0.0, kd=0.0)
                _conn(hub).set_actuators(desires, send=True)
                ok, msg = tiny_teleop(
                    hub,
                    slot=slot,
                    angle_rad=float(args.angle),
                    rate_rad_s=float(args.rate),
                    kp=float(args.kp),
                    kd=float(args.kd),
                    hz=rates[0],
                    start_pos=hint,
                )
                print(f"  {msg}")
                overall_ok = overall_ok and bool(ok)
        finally:
            leave_idle(hub, servo_hold=servo_hold or None)

    print("\n=== RATE SUMMARY ===")
    print(
        f"{'hz':>5}  {'fb':>6}  {'cmd_lag_mx':>10}  {'cmd_lag_p95':>11}  "
        f"{'act_mn':>6}  {'act_pk':>6}  {'per_mn':>6}  {'per_pk':>6}  "
        f"{'s0':>5}  {'s1':>5}  {'led':>4}  ok"
    )
    for hz, st in phase_rows:
        moved = st.get("servo_moved") or {}
        lag_p95 = st.get("cmd_seq_lag_p95", st.get("ack_lag_p95"))
        lag_mx = st.get("cmd_seq_lag_max", st.get("ack_lag_max"))
        lag_ok = lag_p95 is not None and float(lag_p95) <= 2
        fb_ok = (st.get("plant_fb_hz") or 0) >= 20.0
        led = st.get("led_mode_last")
        led_ok = args.no_led or led == LED_MODE_FLASH
        servo_ok = args.no_servo or (
            all(v >= 80 for v in moved.values()) and st.get("servo_err", 0) == 0
        )
        travel_ok = True
        if with_rs:
            want = float(st.get("rs_travel_cmd") or 0.0)
            got = float(st.get("rs_delta") or 0.0)
            end_err = float(st.get("rs_end_err") or 1e9)
            travel_ok = want < 0.5 or (got >= 0.85 * want and end_err <= 0.50)
        ok = (
            lag_ok
            and fb_ok
            and led_ok
            and servo_ok
            and travel_ok
            and st.get("plant_block_hits", 0) == 0
        )
        act_mn = st.get("act_lap_mean")
        act_pk = st.get("act_lap_peak", st.get("lap_max_sticky"))
        per_mn = st.get("periph_lap_mean")
        per_pk = st.get("periph_lap_peak", st.get("periph_lap_max_max"))

        def _f(v: Optional[float], w: int = 6) -> str:
            return f"{float(v):{w}.1f}" if v is not None else f"{'n/a':>{w}}"

        print(
            f"{hz:5.0f}  {st.get('plant_fb_hz') or 0:6.0f}  "
            f"{_f(float(lag_mx) if lag_mx is not None else None, 10)}  "
            f"{_f(float(lag_p95) if lag_p95 is not None else None, 11)}  "
            f"{_f(act_mn)}  {_f(act_pk)}  {_f(per_mn)}  {_f(per_pk)}  "
            f"{moved.get(0, 0):>5}  {moved.get(1, 0):>5}  "
            f"{led if led is not None else '-':>4}  "
            f"{'Y' if ok else 'N'}"
        )

    print(f"\nOVERALL: {'PASS' if overall_ok else 'FAIL'}")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
