#!/usr/bin/env python3
"""Plant load: ×25 ACTUATOR rx_sim + real DXL (π/4) + LED FLASH (no RS02 needed).

  cd scripts
  python _tmp_bus6_real_hw.py --skip-cali              # default: sim-only ×25
  python _tmp_bus6_real_hw.py --with-rs02 --skip-cali  # add real CH6/0x70 MIT

Default skips RS02 probe (motor unplugged OK). Stages product CFG (25 slots)
with ACTUATOR rx_sim, real DXL bounce @ π/4 rad/s, full-strip LED flash.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from typing import Dict, List, Optional

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, ServoDesire
from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    parse_actuator_feedback,
    parse_feedback_header,
    parse_servo_feedback,
)
from _tmp_mcp_timing_probe import ensure_product_cfg
from rs02_channel_bringup import (
    CANONICAL_SLOT,
    PROTO_ROBSTRIDE,
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


def leave_idle(hub: ControlsPcbHub) -> None:
    hub.set_rx_sim_mask(0)
    _conn(hub).clear_servos(send=False)
    hub.set_led(LedDesire(mode=0, master_brightness=0, led_count=0), send=False)
    _conn(hub).set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )
    hub.send_once()
    time.sleep(0.05)
    hub.set_rx_sim_mask(0)
    hub.set_led(LedDesire(mode=0, master_brightness=0, led_count=0), send=False)
    hub.send_once()
    time.sleep(0.35)
    hub.send_once()
    print("leave_idle: blank + rx_sim off + LED OFF + servo clear")


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
    rs_slot: Optional[int] = None,
    start_pos: float = 0.0,
    angle_rad: float = 0.0,
    rate_rad_s: float = 0.0,
    kp: float = 8.0,
    kd: float = 0.5,
) -> dict:
    """×25 LOAD_HOLD + ACTUATOR rx_sim + DXL + LED FLASH; optional real RS02 slot."""
    dt = 1.0 / hz
    t0 = time.perf_counter()
    t_end = t0 + seconds
    next_t = t0

    sign = 1.0 if angle_rad >= 0.0 else -1.0
    travel = abs(angle_rad)
    use_rs = rs_slot is not None and travel > 1e-3 and abs(rate_rad_s) > 1e-3
    t_move = travel / abs(rate_rad_s) if use_rs else 0.0

    servo_seen: Dict[int, List[int]] = {0: [], 1: []}
    servo_err = 0
    last_fb = start_pos
    act_samples = 0
    plant_block_hits = 0

    servo_pos = {
        c["slot"]: float(
            (servo_start or {}).get(c["slot"], _servo_mid(c))
        )
        for c in SERVO_CFG
    }
    servo_dir = {c["slot"]: 1.0 for c in SERVO_CFG}
    rate_steps_s = abs(servo_rate_rad_s) / (2.0 * math.pi) * DXL_TICKS_PER_REV

    ack_lags: List[int] = []
    lap_ms: List[int] = []
    lap_max_sticky: List[int] = []
    ticks_pend: List[int] = []
    ticks_svc: List[int] = []
    rx_fresh_samples: List[int] = []
    last_sent: Optional[int] = None
    reader = _conn(hub).reader
    tf0 = reader.total_frames
    plant_fb_n = 0
    held_slots = list(range(ACTUATOR_COUNT))

    hub.set_rx_sim_mask(RX_SIM_ACTUATOR)
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
        elapsed = now - t0

        desires = {s: LOAD_HOLD for s in range(ACTUATOR_COUNT)}
        if use_rs and rs_slot is not None:
            if elapsed >= t_move:
                cmd = start_pos + sign * travel
                vel = 0.0
            else:
                cmd = start_pos + sign * abs(rate_rad_s) * elapsed
                vel = sign * abs(rate_rad_s)
            desires[rs_slot] = ActuatorDesire(position=cmd, velocity=vel, kp=kp, kd=kd)

        _conn(hub).set_actuators(desires, send=False)

        if drive_servos:
            for c in SERVO_CFG:
                sslot = c["slot"]
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
            if int(hdr.get("plant_block", 0) or 0) != 0:
                plant_block_hits += 1
            ack = int(hdr["last_cmd_seq"]) & 0xFF
            if last_sent is not None:
                lag = (last_sent - ack) & 0xFF
                if lag <= 128:
                    ack_lags.append(lag)
            if hdr.get("lap_ms") is not None:
                lap_ms.append(int(hdr["lap_ms"]))
            if hdr.get("lap_max_ms") is not None:
                lap_max_sticky.append(int(hdr["lap_max_ms"]))
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
            for sslot in (0, 1):
                sv = parse_servo_feedback(raw, sslot)
                if sv is None:
                    continue
                servo_seen[sslot].append(int(sv["present_position"]))
                if sv.get("hw_err_any"):
                    servo_err += 1

        hub.send_once()
        sent = _conn(hub)._last_sent_seq  # noqa: SLF001
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
        "rs_delta": abs(last_fb - start_pos) if use_rs else 0.0,
        "raw_fb_hz": raw_fb_hz,
        "plant_fb_hz": plant_fb_hz,
        "raw_fb_frames": raw_fb,
        "ack_lag_max": max(ack_lags) if ack_lags else None,
        "ack_lag_mean": statistics.mean(ack_lags) if ack_lags else None,
        "ack_lag_p95": _pct([float(x) for x in ack_lags], 95),
        "ack_lag_n": len(ack_lags),
        "ack_lag_str": _istat(ack_lags),
        "lap_ms_str": _istat(lap_ms),
        "lap_max_window": max(lap_ms) if lap_ms else None,
        "lap_max_sticky": max(lap_max_sticky) if lap_max_sticky else None,
        "ticks_pend_str": _istat(ticks_pend),
        "ticks_svc_str": _istat(ticks_svc),
        "servo_rate_steps_s": rate_steps_s,
        "staged_slots": len(held_slots),
        "rx_fresh_mean": statistics.mean(rx_fresh_samples) if rx_fresh_samples else None,
        "rx_fresh_max": max(rx_fresh_samples) if rx_fresh_samples else None,
        "rx_fresh_min": min(rx_fresh_samples) if rx_fresh_samples else None,
        "rx_fresh_n": len(rx_fresh_samples),
    }


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="×25 ACTUATOR rx_sim + DXL + LED flash (RS02 optional)"
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--seconds", type=float, default=10.0)
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
        help="Also probe/drive real RS02 on CH6/0x70 (default: sim-only, no RS02)",
    )
    ap.add_argument("--bus", type=int, default=6, choices=range(1, 7))
    ap.add_argument("--motor-id", default="0x70")
    ap.add_argument("--slot", type=int, default=None)
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--kd", type=float, default=0.5)
    ap.add_argument("--angle", type=float, default=math.pi / 2.0)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_RAD_S)
    ap.add_argument("--skip-cali", action="store_true")
    ap.add_argument("--persist", action="store_true")
    args = ap.parse_args(argv)

    bus = int(args.bus)
    slot = int(args.slot) if args.slot is not None else CANONICAL_SLOT[bus]
    motor_id = int(str(args.motor_id), 0) & 0xFF
    servo_steps = abs(float(args.servo_rate)) / (2.0 * math.pi) * DXL_TICKS_PER_REV
    with_rs = bool(args.with_rs02)

    print(
        f"Load: staged={ACTUATOR_COUNT}  ACTUATOR_rx_sim=ON  "
        f"RS02={'CH%d/0x%02X' % (bus, motor_id) if with_rs else 'OFF'}  "
        f"LED=FLASH×300  DXL={float(args.servo_rate):.4f} rad/s ({servo_steps:.0f} tick/s)  "
        f"host={float(args.hz):.0f} Hz × {float(args.seconds):.1f}s"
    )

    with ControlsPcbHub.connect(args.port) as hub:
        hub.recover()
        hub.set_rx_sim_mask(0)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

        print("\n--- CFG product ×25 ---")
        ensure_product_cfg(hub, force=False)

        start = 0.01
        rs_slot: Optional[int] = None
        if with_rs:
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
            hub.refresh_feedback(slots=[slot], seconds=0.4, hz=float(args.hz))
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
            start = sample_position(hub, slot) or probe_pos
            rs_slot = slot
        else:
            print("\n--- RS02 skipped (sim-only; no probe / no bus contention) ---")

        # Seed DXL sweep from present pose (avoid mid jump / jerk).
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
                    hz=float(args.hz),
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
                        timeout_s=0.35,
                        hz=float(args.hz),
                    )
                servo_start[c["slot"]] = float(fb)

        print(
            f"\n--- combo stream ({args.seconds:.1f}s @ {args.hz:.0f} Hz) "
            f"staged={ACTUATOR_COUNT} rx_sim=ACTUATOR ---"
        )
        stats = stream_combo(
            hub,
            hz=float(args.hz),
            seconds=float(args.seconds),
            led_brightness=int(args.led_brightness),
            drive_servos=not args.no_servo,
            drive_led=not args.no_led,
            servo_rate_rad_s=float(args.servo_rate),
            servo_start=servo_start or None,
            rs_slot=rs_slot,
            start_pos=start,
            angle_rad=float(args.angle) if with_rs else 0.0,
            rate_rad_s=float(args.rate),
            kp=float(args.kp),
            kd=float(args.kd),
        )
        print(
            f"  staged={stats['staged_slots']}  "
            f"servo_moved={stats['servo_moved']} err_flags={stats['servo_err']}  "
            f"plant_block_hits={stats['plant_block_hits']}"
        )
        if with_rs:
            print(f"  RS02 Δ={stats['rs_delta']:.4f} samples={stats['act_samples']}")
        print(
            f"  fb: raw_hz={stats['raw_fb_hz']:.1f} plant_hz={stats['plant_fb_hz']:.1f} "
            f"frames={stats['raw_fb_frames']}"
        )
        if stats.get("rx_fresh_mean") is not None:
            print(
                f"  rx_fresh: mean={stats['rx_fresh_mean']:.1f}/{ACTUATOR_COUNT}  "
                f"min={stats['rx_fresh_min']} max={stats['rx_fresh_max']}  "
                f"n={stats['rx_fresh_n']}"
            )
        print(f"  ack_lag: {stats['ack_lag_str']}")
        print(
            f"  lap_ms:  {stats['lap_ms_str']}  "
            f"window_max={stats['lap_max_window']} sticky={stats['lap_max_sticky']}"
        )
        print(f"  ticks_pending: {stats['ticks_pend_str']}")
        print(f"  ticks_svc:     {stats['ticks_svc_str']}")

        if with_rs and abs(float(args.angle)) > 1e-3:
            print("\n--- dedicated RS02 teleop (×25 load held) ---")
            hub.set_rx_sim_mask(RX_SIM_ACTUATOR)
            en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
            hint = float(en["position"]) if en and en.get("found") else stats["last_fb"]
            desires = {s: LOAD_HOLD for s in range(ACTUATOR_COUNT)}
            desires[slot] = ActuatorDesire(position=hint, kp=0.0, kd=0.0)
            _conn(hub).set_actuators(desires, send=True)
            if not args.no_servo:
                for c in SERVO_CFG:
                    hub.set_servo(
                        c["slot"],
                        ServoDesire(
                            servo_id=c["id"],
                            native_step_position=_servo_mid(c),
                        ),
                        send=False,
                    )
            if not args.no_led:
                hub.set_led(
                    LedDesire(
                        mode=LED_MODE_FLASH,
                        master_brightness=int(args.led_brightness),
                    ),
                    send=False,
                )
            ok, msg = tiny_teleop(
                hub,
                slot=slot,
                angle_rad=float(args.angle),
                rate_rad_s=float(args.rate),
                kp=float(args.kp),
                kd=float(args.kd),
                hz=float(args.hz),
                start_pos=hint,
            )
            print(f"  {msg}")
            stats["teleop_ok"] = ok
        else:
            stats["teleop_ok"] = True

        leave_idle(hub)

    servo_ok = True
    if not args.no_servo:
        moved = stats.get("servo_moved") or {}
        servo_ok = all(v >= 100 for v in moved.values()) and stats.get("servo_err", 0) == 0

    lag_max = stats.get("ack_lag_max")
    fb_hz = stats.get("plant_fb_hz")
    lag_ok = lag_max is not None and lag_max <= 2
    fb_ok = fb_hz is not None and fb_hz >= 20.0
    rx_fresh_ok = (
        stats.get("rx_fresh_mean") is not None
        and float(stats["rx_fresh_mean"]) >= (ACTUATOR_COUNT * 0.8)
    )
    plant_ok = stats.get("plant_block_hits", 0) == 0

    print("\n=== SUMMARY ===")
    print(f"  plant:   {'PASS' if plant_ok else 'FAIL'} (block_hits={stats.get('plant_block_hits')})")
    print(f"  servos:  {'PASS' if servo_ok else 'FAIL / soft'}")
    if with_rs:
        print(f"  teleop:  {'PASS' if stats.get('teleop_ok') else 'FAIL'}")
    print(
        f"  rx_fresh: mean={stats['rx_fresh_mean']:.1f}/{ACTUATOR_COUNT}  "
        f"({'PASS' if rx_fresh_ok else 'FAIL'} ≥80%)"
        if stats.get("rx_fresh_mean") is not None
        else "  rx_fresh: n/a"
    )
    print(
        f"  ack_lag_max={lag_max}  ({'PASS' if lag_ok else 'FAIL'} ≤2)  "
        f"plant_fb_hz={fb_hz:.1f}  ({'PASS' if fb_ok else 'FAIL'} ≥20)"
        if fb_hz is not None
        else f"  ack_lag_max={lag_max}  plant_fb_hz=n/a"
    )
    ok = plant_ok and bool(stats.get("teleop_ok", True))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
