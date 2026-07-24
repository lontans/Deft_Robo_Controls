#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RS02 CH6 temperature study — calibrate, then disabled / enabled / slow motion.

  python _tmp_rs02_ch6_temp_study.py --port COM5

Phases (defaults):
  1) encoder recalibrate
  2) resolve start pose (probe + plant FB; no jerk from p=0)
  3) DISABLED idle (kp=0) at start — several minutes
  4) ENABLED hold (kp>0) at start — several minutes
  5) SLOW MOTION |v|≤π/2 rad/s sine around start — several minutes
  6) return to start, disable, write CSV + summary

Heating is allowed; collect temperature from plant FB (RobStride °C/10).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Line-buffered logs when piped (Windows PowerShell).
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
    sys.stderr.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except Exception:
    pass

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT  # noqa: E402
from deft_controls_sdk.link.exchange.parse import (  # noqa: E402
    parse_actuator_feedback,
    parse_feedback_header,
)
from rs02_channel_bringup import (  # noqa: E402
    CANONICAL_SLOT,
    PROTO_ROBSTRIDE,
    RS02_P_MAX,
    RS02_P_MIN,
    assign_single_slot,
    rs02_near,
    rs02_resolve_start,
    sample_position,
    seed_idle_at_fb,
)

_SCRIPTS = Path(__file__).resolve().parent
_REPO = _SCRIPTS.parent


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


@dataclass
class Sample:
    t_s: float
    phase: str
    pos: float
    vel: float
    torque: float
    temp_c: float
    kp: float
    cmd_pos: float
    cmd_vel: float


def _drain_act(hub: ControlsPcbHub, slot: int) -> Optional[dict]:
    raw = None
    while True:
        chunk = _conn(hub).reader.pop()
        if chunk is None:
            break
        hdr = parse_feedback_header(chunk)
        if hdr is None or hdr.get("is_debug"):
            continue
        raw = chunk
    if raw is None:
        raw = _conn(hub)._latest_fb_raw  # noqa: SLF001
    if raw is None:
        return None
    return parse_actuator_feedback(raw, slot)


def sample_full(
    hub: ControlsPcbHub, slot: int, *, timeout_s: float = 1.0
) -> Optional[dict]:
    deadline = time.monotonic() + timeout_s
    _conn(hub).send_once()
    while time.monotonic() < deadline:
        act = _drain_act(hub, slot)
        if act is not None:
            return act
        time.sleep(0.005)
        _conn(hub).send_once()
    return None


def resolve_start(hub: ControlsPcbHub, *, bus: int, motor_id: int, slot: int) -> float:
    """Probe + plant FB → trusted start; seed idle; refresh until they agree."""
    resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
    if resp is None or not resp.get("found"):
        raise RuntimeError(f"probe miss CH{bus} id=0x{motor_id:02X}")
    probe_pos = float(resp["position"])
    seed_idle_at_fb(hub, slot, probe_pos)
    hub.refresh_feedback(slots=[slot], seconds=0.6, hz=40.0)
    plant = sample_position(hub, slot, timeout_s=1.5)
    start = rs02_resolve_start(probe_pos, plant)
    if start is None:
        raise RuntimeError("no start pose")
    if plant is not None and not rs02_near(plant, start, 0.40):
        print(
            f"  note: plant FB {plant:+.4f} vs probe {probe_pos:+.4f} "
            f"— using {start:+.4f}"
        )
    seed_idle_at_fb(hub, slot, start)
    hub.refresh_feedback(slots=[slot], seconds=0.4, hz=40.0)
    # Soft-engage settle at kp=0 then confirm FB near start.
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    desires[slot] = ActuatorDesire(position=start, velocity=0.0, kp=0.0, kd=0.0)
    t_end = time.perf_counter() + 0.8
    while time.perf_counter() < t_end:
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        time.sleep(0.02)
    fb = sample_position(hub, slot)
    if fb is not None and rs02_near(fb, start, 0.40):
        start = float(fb)
        seed_idle_at_fb(hub, slot, start)
    print(f"  start pose = {start:+.4f} rad (probe={probe_pos:+.4f} plant={plant})")
    return float(start)


def run_phase_hold(
    hub: ControlsPcbHub,
    *,
    slot: int,
    phase: str,
    start: float,
    seconds: float,
    hz: float,
    kp: float,
    kd: float,
    log: List[Sample],
    t0: float,
    sample_period_s: float,
) -> None:
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    desires[slot] = ActuatorDesire(
        position=start, velocity=0.0, kp=kp, kd=kd if kp > 0 else 0.0
    )
    dt = 1.0 / hz
    next_t = time.perf_counter()
    end = next_t + seconds
    next_sample = next_t
    print(
        f"\n=== {phase}  {seconds:.0f}s  kp={kp:.1f} @ pos={start:+.4f} ==="
    )
    while time.perf_counter() < end:
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        now = time.perf_counter()
        if now >= next_sample:
            act = _drain_act(hub, slot)
            if act is not None:
                log.append(
                    Sample(
                        t_s=now - t0,
                        phase=phase,
                        pos=float(act["position"]),
                        vel=float(act["velocity"]),
                        torque=float(act["torque"]),
                        temp_c=float(act["temperature"]),
                        kp=kp,
                        cmd_pos=start,
                        cmd_vel=0.0,
                    )
                )
                if len(log) % 20 == 1 or now + sample_period_s >= end:
                    print(
                        f"  t={log[-1].t_s:7.1f}s  temp={log[-1].temp_c:5.1f}°C  "
                        f"pos={log[-1].pos:+.3f}  τ={log[-1].torque:+.2f}"
                    )
            next_sample = now + sample_period_s
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()


def run_phase_slow_motion(
    hub: ControlsPcbHub,
    *,
    slot: int,
    start: float,
    seconds: float,
    hz: float,
    kp: float,
    kd: float,
    v_max: float,
    amplitude: float,
    log: List[Sample],
    t0: float,
    sample_period_s: float,
) -> float:
    """Sine around start: θ=start+A·sin(ωt), |ωA|≤v_max. Returns final cmd pos."""
    # Fit amplitude into MIT rail with margin.
    margin = 0.25
    room = min(start - (RS02_P_MIN + margin), (RS02_P_MAX - margin) - start)
    amp = min(abs(amplitude), max(0.15, room))
    omega = v_max / max(amp, 1e-3)  # peak vel = A*ω
    print(
        f"\n=== SLOW_MOTION  {seconds:.0f}s  |v|≤{v_max:.3f} rad/s  "
        f"A={amp:.3f} ω={omega:.3f} around {start:+.4f} ==="
    )

    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    # Soft engage at start before oscillating.
    desires[slot] = ActuatorDesire(position=start, velocity=0.0, kp=kp, kd=kd)
    settle_end = time.perf_counter() + 1.0
    while time.perf_counter() < settle_end:
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        time.sleep(0.02)

    dt = 1.0 / hz
    motion_t0 = time.perf_counter()
    next_t = motion_t0
    end = motion_t0 + seconds
    next_sample = motion_t0
    cmd_pos = start
    cmd_vel = 0.0

    while time.perf_counter() < end:
        elapsed = time.perf_counter() - motion_t0
        # Ease-in first 1.5 s on amplitude to avoid jerk at ω start.
        ease = min(1.0, elapsed / 1.5)
        a = amp * ease
        cmd_pos = start + a * math.sin(omega * elapsed)
        cmd_vel = a * omega * math.cos(omega * elapsed)
        # Hard cap (numerical / ease edge).
        if abs(cmd_vel) > v_max:
            cmd_vel = math.copysign(v_max, cmd_vel)
        desires[slot] = ActuatorDesire(
            position=cmd_pos, velocity=cmd_vel, kp=kp, kd=kd
        )
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        now = time.perf_counter()
        if now >= next_sample:
            act = _drain_act(hub, slot)
            if act is not None:
                log.append(
                    Sample(
                        t_s=now - t0,
                        phase="slow_motion",
                        pos=float(act["position"]),
                        vel=float(act["velocity"]),
                        torque=float(act["torque"]),
                        temp_c=float(act["temperature"]),
                        kp=kp,
                        cmd_pos=cmd_pos,
                        cmd_vel=cmd_vel,
                    )
                )
                if len(log) % 20 == 1:
                    print(
                        f"  t={log[-1].t_s:7.1f}s  temp={log[-1].temp_c:5.1f}°C  "
                        f"pos={log[-1].pos:+.3f}  v={log[-1].vel:+.3f}  "
                        f"τ={log[-1].torque:+.2f}"
                    )
            next_sample = now + sample_period_s
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    # Decel to start.
    print("  returning to start…")
    return_s = max(1.5, abs(cmd_pos - start) / max(v_max * 0.5, 0.05))
    t_ret0 = time.perf_counter()
    pos0 = cmd_pos
    while True:
        u = min(1.0, (time.perf_counter() - t_ret0) / return_s)
        # Smoothstep
        s = u * u * (3.0 - 2.0 * u)
        cmd_pos = pos0 + (start - pos0) * s
        cmd_vel = 0.0 if u >= 1.0 else (start - pos0) * (6.0 * u * (1.0 - u) / return_s)
        if abs(cmd_vel) > v_max:
            cmd_vel = math.copysign(v_max, cmd_vel)
        desires[slot] = ActuatorDesire(
            position=cmd_pos, velocity=cmd_vel, kp=kp, kd=kd
        )
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        if u >= 1.0:
            break
        time.sleep(dt)
    return start


def phase_stats(log: List[Sample], phase: str) -> Dict[str, float]:
    rows = [s for s in log if s.phase == phase]
    if not rows:
        return {}
    temps = [s.temp_c for s in rows]
    return {
        "n": float(len(rows)),
        "t0": rows[0].t_s,
        "t1": rows[-1].t_s,
        "temp_min": min(temps),
        "temp_max": max(temps),
        "temp_start": temps[0],
        "temp_end": temps[-1],
        "temp_mean": sum(temps) / len(temps),
    }


def write_outputs(log: List[Sample], meta: dict, out_stem: Path) -> Tuple[Path, Path]:
    csv_path = out_stem.with_suffix(".csv")
    md_path = out_stem.with_suffix(".md")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_s",
                "phase",
                "pos_rad",
                "vel_rad_s",
                "torque_nm",
                "temp_c",
                "kp",
                "cmd_pos",
                "cmd_vel",
            ]
        )
        for s in log:
            w.writerow(
                [
                    f"{s.t_s:.3f}",
                    s.phase,
                    f"{s.pos:.6f}",
                    f"{s.vel:.6f}",
                    f"{s.torque:.6f}",
                    f"{s.temp_c:.3f}",
                    f"{s.kp:.3f}",
                    f"{s.cmd_pos:.6f}",
                    f"{s.cmd_vel:.6f}",
                ]
            )

    lines = [
        "# Bench — RS02 CH6 temperature study",
        "",
        f"Date: {meta.get('date')}",
        "",
        "## Setup",
        "",
        f"| Item | Value |",
        f"|------|--------|",
        f"| Port | {meta.get('port')} |",
        f"| Bus / ID / slot | CH{meta.get('bus')} / 0x{meta.get('motor_id'):02X} / {meta.get('slot')} |",
        f"| Recalibrate | {meta.get('cali')} |",
        f"| Start pose | {meta.get('start'):+.4f} rad |",
        f"| kp / kd | {meta.get('kp')} / {meta.get('kd')} |",
        f"| v_max motion | {meta.get('v_max'):.4f} rad/s |",
        f"| Phase durations (s) | disable={meta.get('disable_s')} enable={meta.get('enable_s')} motion={meta.get('motion_s')} |",
        "",
        "## Temperature by phase",
        "",
        "| Phase | n | t span (s) | start °C | end °C | min | max | mean |",
        "|-------|--:|-----------:|---------:|-------:|----:|----:|-----:|",
    ]
    for phase in ("disabled", "enabled_hold", "slow_motion"):
        st = phase_stats(log, phase)
        if not st:
            continue
        lines.append(
            f"| {phase} | {int(st['n'])} | {st['t0']:.1f}–{st['t1']:.1f} | "
            f"{st['temp_start']:.1f} | {st['temp_end']:.1f} | "
            f"{st['temp_min']:.1f} | {st['temp_max']:.1f} | {st['temp_mean']:.1f} |"
        )
    lines += [
        "",
        f"CSV: `{csv_path.name}`",
        "",
        "## Notes",
        "",
        "- Temperature from plant MIT FB (RobStride raw/10 → °C).",
        "- Disabled = kp=0 idle at measured start; enabled = MIT hold; motion = sine ≤ v_max.",
        "- Heating allowed; motor left durable — no thermal abort.",
        "",
    ]
    md_path.write_text("\n".join(lines), encoding="utf-8")
    return csv_path, md_path


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--bus", type=int, default=6)
    ap.add_argument("--motor-id", type=lambda s: int(s, 0), default=0x70)
    ap.add_argument("--slot", type=int, default=None)
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--disable-s", type=float, default=180.0)
    ap.add_argument("--enable-s", type=float, default=180.0)
    ap.add_argument("--motion-s", type=float, default=300.0)
    ap.add_argument("--v-max", type=float, default=math.pi / 2.0)
    ap.add_argument("--amplitude", type=float, default=1.0, help="Sine amplitude rad")
    ap.add_argument("--sample-period", type=float, default=0.5)
    ap.add_argument("--cal-listen-s", type=float, default=28.0)
    ap.add_argument("--skip-cali", action="store_true")
    args = ap.parse_args(argv)

    bus = int(args.bus)
    slot = int(args.slot) if args.slot is not None else CANONICAL_SLOT[bus]
    motor_id = int(args.motor_id) & 0xFF
    v_max = min(abs(float(args.v_max)), math.pi / 2.0)

    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H%M%SZ")
    out_stem = _REPO / "docs" / f"bench-rs02-ch6-temp-{stamp[:10]}"
    # Avoid overwrite if re-run same day
    if out_stem.with_suffix(".csv").exists():
        out_stem = _REPO / "docs" / f"bench-rs02-ch6-temp-{stamp.replace(':', '')}"

    print(
        f"RS02 temp study  port={args.port} CH{bus} id=0x{motor_id:02X} slot={slot}\n"
        f"  disable={args.disable_s:.0f}s enable={args.enable_s:.0f}s "
        f"motion={args.motion_s:.0f}s v_max={v_max:.3f}"
    )

    log: List[Sample] = []
    meta = {
        "date": stamp,
        "port": args.port,
        "bus": bus,
        "motor_id": motor_id,
        "slot": slot,
        "kp": args.kp,
        "kd": args.kd,
        "v_max": v_max,
        "disable_s": args.disable_s,
        "enable_s": args.enable_s,
        "motion_s": args.motion_s,
        "cali": "skipped" if args.skip_cali else "pending",
        "start": 0.0,
    }

    with ControlsPcbHub.connect(args.port, persist_telemetry=False) as hub:
        hub.recover()
        hub.set_mcu_state(McuState.NORMAL, send=True)
        assign_single_slot(
            hub, bus=bus, slot=slot, motor_id=motor_id, persist=False
        )

        if args.skip_cali:
            print("\n--- cali skipped ---")
            meta["cali"] = "skipped"
        else:
            print("\n--- encoder recalibrate (shaft free, 24–60 V) ---")
            ok = hub.debug.calibrate_robstride(
                bus=bus,
                motor_id=motor_id,
                cal_listen_s=float(args.cal_listen_s),
            )
            meta["cali"] = "ok" if ok else "FAIL"
            print(f"  cali={'PASS' if ok else 'FAIL'}")
            if not ok:
                print("ABORT: recalibrate failed")
                return 1

        print("\n--- resolve start pose ---")
        # Re-enable after cali (cal leaves drive disabled).
        en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
        print(f"  post-cali probe found={en.get('found') if en else None}")
        start = resolve_start(hub, bus=bus, motor_id=motor_id, slot=slot)
        meta["start"] = start

        t0 = time.perf_counter()

        run_phase_hold(
            hub,
            slot=slot,
            phase="disabled",
            start=start,
            seconds=float(args.disable_s),
            hz=float(args.hz),
            kp=0.0,
            kd=0.0,
            log=log,
            t0=t0,
            sample_period_s=float(args.sample_period),
        )

        # Soft engage before enabled hold.
        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        desires[slot] = ActuatorDesire(
            position=start, velocity=0.0, kp=float(args.kp), kd=float(args.kd)
        )
        for _ in range(int(float(args.hz) * 0.8)):
            _conn(hub).set_actuators(desires, send=False)
            _conn(hub).send_once()
            time.sleep(1.0 / float(args.hz))

        run_phase_hold(
            hub,
            slot=slot,
            phase="enabled_hold",
            start=start,
            seconds=float(args.enable_s),
            hz=float(args.hz),
            kp=float(args.kp),
            kd=float(args.kd),
            log=log,
            t0=t0,
            sample_period_s=float(args.sample_period),
        )

        final = run_phase_slow_motion(
            hub,
            slot=slot,
            start=start,
            seconds=float(args.motion_s),
            hz=float(args.hz),
            kp=float(args.kp),
            kd=float(args.kd),
            v_max=v_max,
            amplitude=float(args.amplitude),
            log=log,
            t0=t0,
            sample_period_s=float(args.sample_period),
        )

        print("\n--- disable + leave idle ---")
        seed_idle_at_fb(hub, slot, final)
        hub.refresh_feedback(slots=[slot], seconds=0.3, hz=float(args.hz))
        end_act = sample_full(hub, slot)
        if end_act is not None:
            print(
                f"  final temp={end_act['temperature']:.1f}°C  "
                f"pos={end_act['position']:+.4f}"
            )

    csv_path, md_path = write_outputs(log, meta, out_stem)
    print(f"\nWrote {csv_path}")
    print(f"Wrote {md_path}")
    print(f"samples={len(log)}")
    for phase in ("disabled", "enabled_hold", "slow_motion"):
        st = phase_stats(log, phase)
        if st:
            print(
                f"  {phase}: {st['temp_start']:.1f}→{st['temp_end']:.1f}°C "
                f"(max {st['temp_max']:.1f})"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
