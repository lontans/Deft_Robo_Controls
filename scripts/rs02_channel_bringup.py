#!/usr/bin/env python3
"""Single RobStride (RS02) channel bringup — move the cable, change --bus.

  cd scripts
  python rs02_channel_bringup.py --bus 4
  python rs02_channel_bringup.py --bus 1 --motor-id 0x70 --skip-cali

Steps: discover → CFG canonical slot → metrics hold → cali → tiny teleop.
SDK-only (no scripts/legacy).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from deft_controls_sdk.bench.metrics import drain_latest, measure_hold
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import parse_actuator_feedback, parse_feedback_header

PROTO_ROBSTRIDE = 1

# First product-layout slot per schematic bus (CH1×8, CH2×8, CH3×4, CH4–6×2).
CANONICAL_SLOT: Dict[int, int] = {
    1: 0,
    2: 8,
    3: 16,
    4: 20,
    5: 22,
    6: 24,
}

# Default teleop: one full turn at π/4 rad/s (~8 s).
DEFAULT_TELEOP_ANGLE_RAD = 2.0 * math.pi
DEFAULT_TELEOP_RATE_RAD_S = math.pi / 4.0

# MIT encode range in firmware (robstride.h RS02_P_MIN/MAX). Commands outside
# this clamp; FB near ±12.57 can look like a ~25 rad jump if probe vs plant disagree.
RS02_P_MIN = -12.57
RS02_P_MAX = 12.57
RS02_P_SPAN = RS02_P_MAX - RS02_P_MIN


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def rs02_near(a: float, b: float, eps: float = 0.40) -> bool:
    """True if poses match linearly or as opposite ends of the MIT rail."""
    d = abs(float(a) - float(b))
    return d <= eps or abs(d - RS02_P_SPAN) <= eps


def rs02_resolve_start(
    probe_pos: Optional[float], plant_fb: Optional[float]
) -> Optional[float]:
    """Prefer probe when plant FB is stale across the ±12.57 rail."""
    if probe_pos is None and plant_fb is None:
        return None
    if probe_pos is None:
        return float(plant_fb)  # type: ignore[arg-type]
    if plant_fb is None:
        return float(probe_pos)
    if rs02_near(probe_pos, plant_fb):
        return float(plant_fb)
    return float(probe_pos)


def rs02_plan_angle(
    start: float,
    want_angle: float,
    *,
    margin: float = 0.20,
) -> float:
    """Signed travel that stays inside the MIT range (flip/shrink if needed)."""
    travel = abs(float(want_angle))
    if travel < 1e-6:
        return 0.0
    prefer = 1.0 if want_angle >= 0.0 else -1.0
    lo = RS02_P_MIN + margin
    hi = RS02_P_MAX - margin
    room_pos = max(0.0, hi - float(start))
    room_neg = max(0.0, float(start) - lo)

    def _fit(sign: float) -> float:
        room = room_pos if sign > 0.0 else room_neg
        return sign * min(travel, room)

    primary = _fit(prefer)
    alternate = _fit(-prefer)
    if abs(primary) >= travel - 1e-3:
        return primary
    if abs(alternate) > abs(primary) + 1e-6:
        return alternate
    return primary


def _parse_motor_id(raw: Optional[str]) -> Optional[int]:
    if raw is None:
        return None
    text = raw.strip().lower()
    if text.startswith("0x"):
        return int(text, 16) & 0xFF
    return int(text, 0) & 0xFF


def quiet_all_slots(hub: ControlsPcbHub) -> int:
    """Disable every enabled CFG slot (RAM). Stops ghost MIT at p=0 on other buses."""
    n = 0
    table = hub.debug.cfg_get_table()
    for s, row in enumerate(table):
        if not bool(row.get("enabled", False)):
            continue
        hub.debug.cfg_set_slot(
            slot=s,
            bus=int(row.get("bus", 1)),
            protocol=int(row.get("protocol", PROTO_ROBSTRIDE)),
            motor_id=int(row.get("motor_id", 1)),
            enabled=False,
            persist=False,
        )
        n += 1
    return n


def assign_single_slot(
    hub: ControlsPcbHub,
    *,
    bus: int,
    slot: int,
    motor_id: int,
    persist: bool,
    quiet_others: bool = False,
) -> None:
    """Enable ``slot`` for this motor. Sibling slots on the same bus are left
    alone (daisy-chain / dual RS on MCP). Pass ``quiet_others=True`` only when
    you intentionally want a single-slot bus."""
    print(f"CFG: enable slot {slot} on CH{bus} id=0x{motor_id:02X} (persist={persist})")
    if quiet_others:
        quieted = quiet_all_slots(hub)
        if quieted:
            print(f"  quieted {quieted} other enabled slot(s)")
    else:
        print("  add-only (sibling CFG slots untouched)")
    hub.debug.cfg_set_slot(
        slot=slot,
        bus=bus,
        protocol=PROTO_ROBSTRIDE,
        motor_id=motor_id,
        enabled=True,
        persist=persist,
    )
    table = hub.debug.cfg_get_table()
    row = table[slot]
    print(
        f"  verify slot {slot}: bus={row.get('bus')} proto={row.get('protocol')} "
        f"id=0x{int(row.get('motor_id', 0)):02X} en={row.get('enabled')}"
    )


def seed_idle_at_fb(hub: ControlsPcbHub, slot: int, pos: float) -> None:
    """Hold last FB pose with kp=0 — never leave plant desire at p=0 while shaft elsewhere."""
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    desires[slot] = ActuatorDesire(position=pos, velocity=0.0, kp=0.0, kd=0.0)
    _conn(hub).set_actuators(desires, send=True)


def sample_position(hub: ControlsPcbHub, slot: int, *, timeout_s: float = 1.0) -> Optional[float]:
    deadline = time.monotonic() + timeout_s
    _conn(hub).send_once()
    while time.monotonic() < deadline:
        raw = drain_latest(hub)
        if raw is None:
            time.sleep(0.005)
            continue
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        act = parse_actuator_feedback(raw, slot)
        if act is not None:
            return float(act["position"])
        time.sleep(0.005)
    return None


def _stream_slot(
    hub: ControlsPcbHub,
    *,
    slot: int,
    desires: Dict[int, ActuatorDesire],
    seconds: float,
    hz: float,
    track_cmd: bool = False,
) -> Tuple[float, int, Optional[float]]:
    """Stream ``desires`` for ``seconds``. Returns (err_peak, samples, last_fb)."""
    dt = 1.0 / hz
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    err_peak = 0.0
    samples = 0
    last_fb: Optional[float] = None
    cmd = float(desires[slot].position)

    while time.perf_counter() < t_end:
        _conn(hub).set_actuators(desires, send=False)
        raw = drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr is not None and not hdr.get("is_debug"):
                act = parse_actuator_feedback(raw, slot)
                if act is not None:
                    fb = float(act["position"])
                    last_fb = fb
                    if track_cmd:
                        err = abs(fb - cmd)
                        if err > err_peak:
                            err_peak = err
                        samples += 1
        _conn(hub).send_once()
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()
    return err_peak, samples, last_fb


def tiny_teleop(
    hub: ControlsPcbHub,
    *,
    slot: int,
    angle_rad: float = DEFAULT_TELEOP_ANGLE_RAD,
    rate_rad_s: float = DEFAULT_TELEOP_RATE_RAD_S,
    kp: float,
    kd: float,
    hz: float = 40.0,
    err_ok: float = 0.25,
    settle_s: float = 0.4,
    accel_s: float = 0.4,
    start_pos: Optional[float] = None,
) -> Tuple[bool, str]:
    """Smooth turn (default 360° @ π/4) with soft engage + trapezoid velocity.

    ``start_pos`` should be a fresh probe/enable reading when available. Plant
    slot FB can be stale after cali (e.g. probe≈0 but plant still −1.3) — using
    that for the first kp>0 command causes the startup TX jerk.
    """
    if abs(rate_rad_s) < 1e-3:
        return False, "teleop rate too small"
    if abs(angle_rad) < 1e-3:
        return False, "teleop angle too small"

    plant_fb = sample_position(hub, slot)
    if start_pos is not None:
        pos_cmd = float(start_pos)
        if plant_fb is not None and not rs02_near(plant_fb, pos_cmd, 0.40):
            print(
                f"  note: plant FB {plant_fb:+.4f} disagrees with probe "
                f"{pos_cmd:+.4f} — soft-engage from probe (avoid stale snap)"
            )
    elif plant_fb is not None:
        pos_cmd = plant_fb
    else:
        return False, "no plant feedback for start position"

    planned = rs02_plan_angle(pos_cmd, angle_rad)
    if abs(planned) < 0.5:
        return False, (
            f"no MIT room for teleop at {pos_cmd:+.4f} "
            f"(range [{RS02_P_MIN}, {RS02_P_MAX}])"
        )
    if abs(abs(planned) - abs(angle_rad)) > 0.05:
        print(
            f"  note: angle {angle_rad:+.4f} → {planned:+.4f} to stay in "
            f"MIT [{RS02_P_MIN}, {RS02_P_MAX}]"
        )
    sign = 1.0 if planned >= 0.0 else -1.0
    v_max = abs(rate_rad_s)
    travel = abs(planned)

    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}

    # Soft engage at the trusted start pose (v=0). Only adopt plant FB once it
    # tracks near the command — ignore stale slot state from before cali.
    desires[slot] = ActuatorDesire(position=pos_cmd, velocity=0.0, kp=kp, kd=kd)
    _conn(hub).set_actuators(desires, send=False)
    settle_end = time.perf_counter() + max(0.15, settle_s)
    dt = 1.0 / hz
    next_t = time.perf_counter()
    while time.perf_counter() < settle_end:
        raw = drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr is not None and not hdr.get("is_debug"):
                act = parse_actuator_feedback(raw, slot)
                if act is not None:
                    fb = float(act["position"])
                    if rs02_near(fb, pos_cmd, 0.40):
                        pos_cmd = fb
                        desires[slot] = ActuatorDesire(
                            position=pos_cmd, velocity=0.0, kp=kp, kd=kd
                        )
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    pos0 = pos_cmd
    target_end = pos0 + sign * travel

    # Trapezoid: accel / cruise / decel along path distance.
    t_acc = max(0.05, float(accel_s))
    a = v_max / t_acc
    d_acc = 0.5 * v_max * t_acc
    if 2.0 * d_acc >= travel:
        # Triangular — never reach cruise.
        v_peak = math.sqrt(max(a * travel, 1e-9))
        t_acc = v_peak / a
        t_cruise = 0.0
        d_cruise = 0.0
        d_acc = 0.5 * v_peak * t_acc
    else:
        v_peak = v_max
        d_cruise = travel - 2.0 * d_acc
        t_cruise = d_cruise / v_peak
    t_total = 2.0 * t_acc + t_cruise

    print(
        f"Teleop: slot {slot}  {pos0:+.4f} -> {target_end:+.4f}  "
        f"({math.degrees(travel):.0f} deg @ {sign * v_max:+.4f} rad/s, "
        f"{t_total:.1f}s incl. accel={t_acc:.2f}s)"
    )

    def path_at(t: float) -> Tuple[float, float]:
        """Path distance and speed along the move (non-negative)."""
        if t <= 0.0:
            return 0.0, 0.0
        if t < t_acc:
            v = a * t
            s = 0.5 * a * t * t
            return s, v
        if t < t_acc + t_cruise:
            dt_c = t - t_acc
            return d_acc + v_peak * dt_c, v_peak
        if t < t_total:
            td = t - t_acc - t_cruise
            v = max(0.0, v_peak - a * td)
            s = d_acc + d_cruise + v_peak * td - 0.5 * a * td * td
            return min(s, travel), v
        return travel, 0.0

    t0 = time.perf_counter()
    next_t = t0
    err_peak = 0.0
    samples = 0
    last_fb: Optional[float] = pos0

    while True:
        now = time.perf_counter()
        elapsed = now - t0
        s, v = path_at(elapsed)
        done = elapsed >= t_total
        if done:
            s, v = travel, 0.0
        cmd = pos0 + sign * s
        vel = sign * v

        desires[slot] = ActuatorDesire(position=cmd, velocity=vel, kp=kp, kd=kd)
        _conn(hub).set_actuators(desires, send=False)

        raw = drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr is not None and not hdr.get("is_debug"):
                act = parse_actuator_feedback(raw, slot)
                if act is not None:
                    fb = float(act["position"])
                    last_fb = fb
                    err = abs(fb - cmd)
                    if err > err_peak:
                        err_peak = err
                    samples += 1

        _conn(hub).send_once()
        if done:
            break

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    # Settle at end (v=0), still commanding the real end pose — not p=0.
    desires[slot] = ActuatorDesire(position=target_end, velocity=0.0, kp=kp, kd=kd)
    peak2, n2, fb2 = _stream_slot(
        hub, slot=slot, desires=desires, seconds=0.4, hz=hz, track_cmd=True
    )
    err_peak = max(err_peak, peak2)
    samples += n2
    if fb2 is not None:
        last_fb = fb2

    # Idle at last pose (kp=0). Avoid blank p=0 — next move would snap from 0.
    end_pose = last_fb if last_fb is not None else target_end
    desires[slot] = ActuatorDesire(position=end_pose, velocity=0.0, kp=0.0, kd=0.0)
    _conn(hub).set_actuators(desires, send=True)

    pos1 = sample_position(hub, slot)
    if pos1 is None:
        pos1 = last_fb
    if pos1 is None:
        return False, "no feedback after turn"

    moved = abs(pos1 - pos0)
    end_err = abs(pos1 - target_end)
    print(
        f"  after turn: fb={pos1:+.4f}  moved={moved:.4f}  "
        f"|end_err|={end_err:.4f}  |track|_peak={err_peak:.4f}  n={samples}"
    )

    moved_ok = moved >= travel * 0.75
    tracked = err_peak <= err_ok and end_err <= max(err_ok, 0.15)
    if moved_ok and tracked:
        return (
            True,
            f"smooth turn OK (start={pos0:+.3f} end={pos1:+.3f} "
            f"moved={moved:.3f} track_peak={err_peak:.3f})",
        )
    if moved_ok:
        return (
            False,
            f"turned but tracking loose (peak={err_peak:.3f} end_err={end_err:.3f})",
        )
    return (
        False,
        f"little motion (start={pos0:+.3f} after={pos1:+.3f} want~{travel:.3f}) "
        "— check enable/CFG/bus",
    )


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="RS02 single-actuator channel bringup (SDK-only)"
    )
    ap.add_argument("--bus", type=int, required=True, choices=range(1, 7), help="CH1..CH6")
    ap.add_argument("--port", default=None, help="CDC port (default: auto-find)")
    ap.add_argument("--motor-id", default=None, help="Skip discover (e.g. 0x70 or 112)")
    ap.add_argument("--slot", type=int, default=None, help="CFG slot (default: canonical for bus)")
    ap.add_argument("--seconds", type=float, default=3.0, help="Metrics hold window")
    ap.add_argument("--hz", type=float, default=40.0, help="Host plant TX rate")
    ap.add_argument("--skip-cali", action="store_true", help="Skip encoder calibrate")
    ap.add_argument("--persist", action="store_true", help="CFG flash SAVE for the test slot")
    ap.add_argument(
        "--quiet-all",
        action="store_true",
        help="Disable every other CFG slot before assign (default: add-only)",
    )
    ap.add_argument(
        "--angle",
        type=float,
        default=DEFAULT_TELEOP_ANGLE_RAD,
        help="Teleop travel (rad); default 2π (360 deg)",
    )
    ap.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_TELEOP_RATE_RAD_S,
        help="Teleop rate (rad/s); default π/4",
    )
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--kd", type=float, default=0.5)
    ap.add_argument("--cal-listen-s", type=float, default=28.0)
    args = ap.parse_args(argv)

    bus = int(args.bus)
    slot = int(args.slot) if args.slot is not None else CANONICAL_SLOT[bus]
    if not (0 <= slot < ACTUATOR_COUNT):
        print(f"bad --slot {slot} (need 0..{ACTUATOR_COUNT - 1})", file=sys.stderr)
        return 2

    motor_id = _parse_motor_id(args.motor_id)

    results: Dict[str, bool] = {}
    notes: List[str] = []

    print(f"Opening port={args.port or 'auto'}  bus=CH{bus}  slot={slot}")
    with ControlsPcbHub.connect(args.port) as hub:
        hub.recover()
        # Blank desires stop ghost MIT without ripping sibling CFG off a shared bus.
        if args.quiet_all:
            n_quiet = quiet_all_slots(hub)
            if n_quiet:
                print(f"Quieted {n_quiet} enabled CFG slot(s) before probe")
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

        # --- discover / probe ---
        probe_pos: Optional[float] = None
        if motor_id is None:
            print(f"\n--- discover on CH{bus} ---")
            # Full sweep — daisy-chained RS01/RS02 on one rail must all show up.
            hits = hub.debug.discover_robstride_all(bus=bus, start=0x70, end=0x7F)
            if not hits:
                print("FAIL: no RS02/RS01 found — check cable/power/ID range")
                return 1
            if len(hits) > 1:
                ids_s = ", ".join(f"0x{i:02X}" for i in hits)
                print(
                    f"FAIL: {len(hits)} motors on CH{bus} ({ids_s}). "
                    "Re-run with --motor-id <id> (and --slot) for the one to bring up."
                )
                return 1
            motor_id = hits[0]
        else:
            print(f"\n--- probe id=0x{motor_id:02X} on CH{bus} ---")
            resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
            if resp is None or not resp.get("found"):
                print("FAIL: probe miss — check bus/ID")
                return 1
            probe_pos = float(resp.get("position", 0.0))
        notes.append(f"motor_id=0x{motor_id:02X}")

        # --- CFG ---
        print("\n--- CFG single slot ---")
        assign_single_slot(
            hub,
            bus=bus,
            slot=slot,
            motor_id=motor_id,
            persist=bool(args.persist),
            quiet_others=bool(args.quiet_all),
        )
        # Seed + refresh plant HBHF before any metrics read — DEBUG probe/CFG
        # does not fill actuator_state; blank MCP idle leaves pose at 0.
        if probe_pos is None:
            resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
            if resp is not None and resp.get("found"):
                probe_pos = float(resp.get("position", 0.0))
        if probe_pos is None:
            print("FAIL: no probe pose to seed idle before plant refresh")
            return 1
        seed_idle_at_fb(hub, slot, probe_pos)
        print(f"  seeded idle desire at probe pos={probe_pos:+.4f}")
        hub.refresh_feedback(slots=[slot], seconds=0.5, hz=float(args.hz))
        print("  refreshed plant HBHF after idle seed")

        # --- metrics ---
        print("\n--- metrics hold ---")
        pos = sample_position(hub, slot)
        if pos is None:
            print("FAIL: no fresh plant FB position after refresh — refusing desire p=0")
            return 1
        desire = ActuatorDesire(position=pos, kp=float(args.kp), kd=float(args.kd))
        print(f"  holding current fb pos={pos:+.4f}")
        metrics = measure_hold(
            hub,
            f"CH{bus}_slot{slot}",
            {slot: desire},
            seconds=float(args.seconds),
            hz=float(args.hz),
        )
        results["metrics"] = bool(metrics.get("ok"))
        notes.append(
            f"fb_hz={metrics.get('raw_fb_hz')} ack_max={metrics.get('ack_lag_max')} "
            f"lap_max={metrics.get('lap_max_ms')}"
        )
        seed_idle_at_fb(hub, slot, pos)
        hub.refresh_feedback(slots=[slot], seconds=0.3, hz=float(args.hz))

        # --- cali ---
        if args.skip_cali:
            print("\n--- cali skipped (--skip-cali) ---")
            results["cali"] = True
            notes.append("cali=skipped")
        else:
            print("\n--- encoder cali ---")
            print("  Ensure shaft is free and supply is 24–60 V.")
            ok_cali = hub.debug.calibrate_robstride(
                bus=bus,
                motor_id=motor_id,
                cal_listen_s=float(args.cal_listen_s),
            )
            results["cali"] = bool(ok_cali)
            notes.append(f"cali={'ok' if ok_cali else 'FAIL'}")

        # --- tiny teleop ---
        print("\n--- tiny teleop ---")
        # Cali/reset leaves the drive disabled. Re-arm via enable probe so the
        # first MIT after blank is not racing plant maintain.
        en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
        print(f"  pre-enable  {en.get('found') if en else None}  "
              f"pos={en.get('position') if en else None}")
        start_hint: Optional[float] = None
        if en and en.get("found"):
            start_hint = float(en["position"])
            seed_idle_at_fb(hub, slot, start_hint)
            hub.refresh_feedback(slots=[slot], seconds=0.3, hz=float(args.hz))
        ok_tele, tele_msg = tiny_teleop(
            hub,
            slot=slot,
            angle_rad=float(args.angle),
            rate_rad_s=float(args.rate),
            kp=float(args.kp),
            kd=float(args.kd),
            hz=float(args.hz),
            start_pos=start_hint,
        )
        results["teleop"] = ok_tele
        notes.append(tele_msg)
        print(f"  {tele_msg}")
        end_pos = sample_position(hub, slot)
        if end_pos is not None:
            seed_idle_at_fb(hub, slot, end_pos)

    print("\n=== SUMMARY ===")
    for name, ok in results.items():
        print(f"  {name}: {'PASS' if ok else 'FAIL'}")
    for n in notes:
        print(f"  - {n}")
    overall = all(results.values()) if results else False
    print(f"OVERALL: {'PASS' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
