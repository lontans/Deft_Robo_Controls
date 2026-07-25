#!/usr/bin/env python3
"""Probe + gentle rotate for the four base MCP actuators (no arm / no DXL).

Expected map (free-spinning, no hard stops):
  bus5 slot22 RS02 @ 0x70
  bus5 slot23 RS01 @ 0x74
  bus6 slot24 RS01 @ 0x75
  bus6 slot25 Damiao @ 0x06 / master 0x16

Flow per motor: probe/discover → CFG → seed idle → soft hold → ±angle slew →
return. Sibling base slots are soft-held at present so the shared MCP rail
keeps TX without snapping them to 0.

    python3 yam_base_rotate_prove.py
    python3 yam_base_rotate_prove.py --angle 1.0 --rate 0.4
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState  # noqa: E402
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT  # noqa: E402
from deft_controls_sdk.vbeta.cfg import pause_plant_stream  # noqa: E402
from deft_controls_sdk.vbeta.slots import PROTO_DAMIAO, PROTO_ROBSTRIDE  # noqa: E402
from rs02_channel_bringup import (  # noqa: E402
    quiet_all_slots,
    sample_position,
    seed_idle_at_fb,
)

# (slot, bus, protocol, motor_id, master_id, label)
BASE_ROWS: Tuple[Tuple[int, int, int, int, int, str], ...] = (
    (22, 5, PROTO_ROBSTRIDE, 0x70, 0, "CH5 RS02"),
    (23, 5, PROTO_ROBSTRIDE, 0x74, 0, "CH5 RS01"),
    (24, 6, PROTO_ROBSTRIDE, 0x75, 0, "CH6 RS01"),
    (25, 6, PROTO_DAMIAO, 0x06, 0x16, "CH6 Damiao"),
)

RS_KP = 20.0
RS_KD = 1.0
DM_KP = 8.0
DM_KD = 0.5
HZ = 40.0


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _blank_all(hub: ControlsPcbHub) -> None:
    _conn(hub).set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )
    _conn(hub).send_once()


def apply_base_only_cfg(hub: ControlsPcbHub) -> None:
    """Disable every slot, then enable only the four base MCP rows (RAM)."""
    with pause_plant_stream(hub):
        n = quiet_all_slots(hub)
        print(f"CFG: quieted {n} previously-enabled slot(s)", flush=True)
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            hub.debug.cfg_set_slot(
                slot=slot,
                bus=bus,
                protocol=proto,
                motor_id=mid,
                master_id=master,
                enabled=True,
                persist=False,
            )
            print(
                f"  slot {slot}: bus={bus} proto={proto} id=0x{mid:02X} "
                f"master=0x{master:02X} ({label})",
                flush=True,
            )


def probe_robstride(
    hub: ControlsPcbHub, *, bus: int, motor_id: int
) -> Optional[float]:
    resp = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
    if resp is None or not resp.get("found"):
        return None
    return float(resp["position"])


def discover_rs_on_bus(
    hub: ControlsPcbHub, *, bus: int, prefer: Sequence[int]
) -> List[Tuple[int, float]]:
    """Return [(id, pos), ...] for preferred IDs first, then a short discover."""
    found: List[Tuple[int, float]] = []
    seen = set()
    for mid in prefer:
        pos = probe_robstride(hub, bus=bus, motor_id=mid)
        if pos is None:
            print(f"  probe miss CH{bus} id=0x{mid:02X}", flush=True)
            continue
        print(f"  probe hit  CH{bus} id=0x{mid:02X} pos={pos:+.4f}", flush=True)
        found.append((mid, pos))
        seen.add(mid)
    if len(found) >= len(prefer):
        return found
    # Fall back: discover from 0x01..0x7F, keep anything new.
    print(f"  discover sweep CH{bus} (prefer missed)…", flush=True)
    for start in (0x70, 0x01, 0x40):
        mid = hub.debug.discover_robstride(bus=bus, start=start)
        if mid is None or mid in seen:
            continue
        pos = probe_robstride(hub, bus=bus, motor_id=mid)
        if pos is None:
            continue
        print(f"  discover hit CH{bus} id=0x{mid:02X} pos={pos:+.4f}", flush=True)
        found.append((mid, pos))
        seen.add(mid)
        if len(found) >= 2:
            break
    return found


def hold_siblings(
    hub: ControlsPcbHub,
    *,
    active: int,
    holds: Dict[int, Tuple[float, float, float]],
) -> Dict[int, ActuatorDesire]:
    """Build desire map: blank everywhere, soft-hold siblings, leave active blank."""
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    for slot, (q, kp, kd) in holds.items():
        if slot == active:
            continue
        desires[slot] = ActuatorDesire(
            position=float(q), velocity=0.0, kp=float(kp), kd=float(kd)
        )
    return desires


def soft_hold_slot(
    hub: ControlsPcbHub,
    *,
    slot: int,
    pos: float,
    kp: float,
    kd: float,
    holds: Dict[int, Tuple[float, float, float]],
    seconds: float = 0.6,
) -> Optional[float]:
    desires = hold_siblings(hub, active=-1, holds=holds)
    desires[slot] = ActuatorDesire(
        position=float(pos), velocity=0.0, kp=float(kp), kd=float(kd)
    )
    dt = 1.0 / HZ
    next_t = time.perf_counter()
    t_end = next_t + seconds
    last = None
    while time.perf_counter() < t_end:
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        last = sample_position(hub, slot)
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()
    return last


def rotate_one(
    hub: ControlsPcbHub,
    *,
    slot: int,
    bus: int,
    proto: int,
    motor_id: int,
    master_id: int,
    label: str,
    start: float,
    angle: float,
    rate: float,
    holds: Dict[int, Tuple[float, float, float]],
) -> Tuple[bool, str, Optional[float]]:
    kp = DM_KP if proto == PROTO_DAMIAO else RS_KP
    kd = DM_KD if proto == PROTO_DAMIAO else RS_KD

    # Soft-engage at start while siblings hold.
    seed_idle_at_fb(hub, slot, start)
    holds[slot] = (start, kp, kd)
    fb = soft_hold_slot(
        hub, slot=slot, pos=start, kp=kp, kd=kd, holds=holds, seconds=0.7
    )
    if fb is not None:
        start = float(fb)
        holds[slot] = (start, kp, kd)
        seed_idle_at_fb(hub, slot, start)

    # Re-enable RS after any prior blank/CFG (Damiao needs plant MIT stream).
    if proto == PROTO_ROBSTRIDE:
        en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
        if en and en.get("found"):
            start = float(en["position"])
            holds[slot] = (start, kp, kd)
            seed_idle_at_fb(hub, slot, start)

    print(
        f"  rotate {label} slot{slot} start={start:+.4f} "
        f"angle=±{angle:.2f} rate={rate:.2f} kp={kp}",
        flush=True,
    )

    # tiny_teleop only drives one slot; keep siblings alive around it.
    # Pump sibling holds in a wrapper by monkey-patching set_actuators? Simpler:
    # call tiny_teleop after seeding, then manually do a short slew that holds siblings.
    ok, msg, end = _slew_with_siblings(
        hub,
        slot=slot,
        start=start,
        angle=angle,
        rate=rate,
        kp=kp,
        kd=kd,
        holds=holds,
    )
    if end is not None:
        holds[slot] = (end, kp, kd)
        seed_idle_at_fb(hub, slot, end)
    return ok, msg, end


def _slew_with_siblings(
    hub: ControlsPcbHub,
    *,
    slot: int,
    start: float,
    angle: float,
    rate: float,
    kp: float,
    kd: float,
    holds: Dict[int, Tuple[float, float, float]],
) -> Tuple[bool, str, Optional[float]]:
    """Trapezoid ±angle then return; soft-hold sibling base slots every tick."""
    # Reuse planning from tiny_teleop via a local copy of the ramp.
    from rs02_channel_bringup import rs02_plan_angle, rs02_near

    planned = rs02_plan_angle(start, angle)
    if abs(planned) < 0.15:
        return False, f"no MIT room from {start:+.4f}", start

    targets = [start + planned, start]
    dt = 1.0 / HZ
    last_fb: Optional[float] = start
    pos_cmd = float(start)
    peak_err = 0.0
    samples = 0

    for tgt in targets:
        travel = abs(tgt - pos_cmd)
        if travel < 1e-4:
            continue
        sign = 1.0 if tgt >= pos_cmd else -1.0
        v_max = abs(rate)
        # soft engage
        desires = hold_siblings(hub, active=-1, holds=holds)
        desires[slot] = ActuatorDesire(
            position=pos_cmd, velocity=0.0, kp=kp, kd=kd
        )
        t_eng = time.perf_counter() + 0.35
        next_t = time.perf_counter()
        while time.perf_counter() < t_eng:
            _conn(hub).set_actuators(desires, send=False)
            _conn(hub).send_once()
            fb = sample_position(hub, slot)
            if fb is not None and rs02_near(fb, pos_cmd, 0.50):
                pos_cmd = float(fb)
                desires[slot] = ActuatorDesire(
                    position=pos_cmd, velocity=0.0, kp=kp, kd=kd
                )
                last_fb = fb
            next_t += dt
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()

        # constant-velocity ramp (no fancy accel — short prove)
        while abs(tgt - pos_cmd) > 0.01:
            step = min(v_max * dt, abs(tgt - pos_cmd))
            pos_cmd = pos_cmd + sign * step
            desires = hold_siblings(hub, active=-1, holds=holds)
            desires[slot] = ActuatorDesire(
                position=pos_cmd, velocity=sign * v_max, kp=kp, kd=kd
            )
            _conn(hub).set_actuators(desires, send=False)
            _conn(hub).send_once()
            fb = sample_position(hub, slot)
            if fb is not None:
                last_fb = fb
                err = abs(fb - pos_cmd)
                peak_err = max(peak_err, err)
                samples += 1
            next_t += dt
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()

        # settle
        desires = hold_siblings(hub, active=-1, holds=holds)
        desires[slot] = ActuatorDesire(
            position=tgt, velocity=0.0, kp=kp, kd=kd
        )
        t_set = time.perf_counter() + 0.45
        next_t = time.perf_counter()
        while time.perf_counter() < t_set:
            _conn(hub).set_actuators(desires, send=False)
            _conn(hub).send_once()
            fb = sample_position(hub, slot)
            if fb is not None:
                last_fb = fb
                peak_err = max(peak_err, abs(fb - tgt))
                samples += 1
            next_t += dt
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()
        pos_cmd = float(tgt)

    ok = samples > 5 and peak_err < 0.55
    msg = (
        f"travel={planned:+.3f} samples={samples} peak_err={peak_err:.3f} "
        f"end={last_fb if last_fb is not None else float('nan'):+.4f}"
    )
    return ok, msg, last_fb


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--angle", type=float, default=0.8, help="slew amplitude (rad)")
    ap.add_argument("--rate", type=float, default=0.45, help="slew rate (rad/s)")
    ap.add_argument(
        "--skip-discover",
        action="store_true",
        help="Do not sweep if preferred IDs miss",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    port = args.port or find_cdc_port()
    print(
        f"yam_base_rotate_prove port={port} angle={args.angle} rate={args.rate}",
        flush=True,
    )

    results: List[Tuple[str, bool, str]] = []
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        time.sleep(0.15)
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _blank_all(hub)

        apply_base_only_cfg(hub)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _blank_all(hub)
        time.sleep(0.2)

        # --- discover / probe phase ---
        print("\n== PROBE ==", flush=True)
        starts: Dict[int, float] = {}
        id_overrides: Dict[int, int] = {}

        # Bus 5: two Robstride
        print("CH5 Robstride:", flush=True)
        hits5 = discover_rs_on_bus(hub, bus=5, prefer=(0x70, 0x74))
        if not hits5 and not args.skip_discover:
            results.append(("CH5", False, "no Robstride replies"))
        # Map preferred slots: first hit → 22, second → 23 (or by ID match)
        by_id5 = {mid: pos for mid, pos in hits5}
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            if bus != 5 or proto != PROTO_ROBSTRIDE:
                continue
            if mid in by_id5:
                starts[slot] = by_id5[mid]
                id_overrides[slot] = mid
            elif hits5 and slot not in starts:
                # unexpected ID — adopt next unused hit
                for hm, hp in hits5:
                    if hm not in id_overrides.values():
                        starts[slot] = hp
                        id_overrides[slot] = hm
                        print(
                            f"  remap {label}: expected 0x{mid:02X} → using 0x{hm:02X}",
                            flush=True,
                        )
                        break

        # Bus 6 Robstride
        print("CH6 Robstride:", flush=True)
        hits6 = discover_rs_on_bus(hub, bus=6, prefer=(0x75, 0x70))
        by_id6 = {mid: pos for mid, pos in hits6}
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            if bus != 6 or proto != PROTO_ROBSTRIDE:
                continue
            if mid in by_id6:
                starts[slot] = by_id6[mid]
                id_overrides[slot] = mid
            elif hits6:
                hm, hp = hits6[0]
                starts[slot] = hp
                id_overrides[slot] = hm
                if hm != mid:
                    print(
                        f"  remap {label}: expected 0x{mid:02X} → using 0x{hm:02X}",
                        flush=True,
                    )

        # Bus 6 Damiao
        print("CH6 Damiao:", flush=True)
        dm_id = hub.debug.discover_damiao(bus=6, start=1, end=16)
        if dm_id is None:
            print("  discover miss — trying plant FB after CFG", flush=True)
        else:
            print(f"  discover hit id=0x{dm_id:02X}", flush=True)
            id_overrides[25] = dm_id
            # Remap CFG if discover found a different id
            if dm_id != 0x01:
                master = (dm_id + 0x10) & 0xFF
                print(
                    f"  remap CH6 Damiao: 0x01/0x11 → 0x{dm_id:02X}/0x{master:02X}",
                    flush=True,
                )
                with pause_plant_stream(hub):
                    hub.debug.cfg_set_slot(
                        slot=25,
                        bus=6,
                        protocol=PROTO_DAMIAO,
                        motor_id=dm_id,
                        master_id=master,
                        enabled=True,
                        persist=False,
                    )

        # Apply RS ID remaps to CFG if needed
        for slot, mid in id_overrides.items():
            row = next(r for r in BASE_ROWS if r[0] == slot)
            if row[2] != PROTO_ROBSTRIDE:
                continue
            if mid == row[3]:
                continue
            print(f"  CFG remap slot{slot} → id=0x{mid:02X}", flush=True)
            with pause_plant_stream(hub):
                hub.debug.cfg_set_slot(
                    slot=slot,
                    bus=row[1],
                    protocol=PROTO_ROBSTRIDE,
                    motor_id=mid,
                    master_id=0,
                    enabled=True,
                    persist=False,
                )

        # Seed idle for every known start; refresh plant HBHF
        for slot, q in starts.items():
            seed_idle_at_fb(hub, slot, q)
        # Damiao: seed 0 then refresh — plant FB fills if alive
        seed_idle_at_fb(hub, 25, 0.0)
        for slot in list(starts.keys()) + [25]:
            hub.refresh_feedback(slots=[slot], seconds=0.5, hz=HZ)
            pos = sample_position(hub, slot)
            if pos is not None:
                starts[slot] = float(pos)
                seed_idle_at_fb(hub, slot, float(pos))
                print(f"  plant FB slot{slot} = {pos:+.4f}", flush=True)
            else:
                print(f"  plant FB slot{slot} = NONE", flush=True)

        holds: Dict[int, Tuple[float, float, float]] = {}
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            if slot not in starts:
                continue
            kp = DM_KP if proto == PROTO_DAMIAO else RS_KP
            kd = DM_KD if proto == PROTO_DAMIAO else RS_KD
            holds[slot] = (starts[slot], kp, kd)

        # Soft-hold all present before motion
        print("\n== SOFT HOLD ALL PRESENT ==", flush=True)
        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for slot, (q, kp, kd) in holds.items():
            desires[slot] = ActuatorDesire(
                position=q, velocity=0.0, kp=kp, kd=kd
            )
        t_end = time.perf_counter() + 1.0
        next_t = time.perf_counter()
        dt = 1.0 / HZ
        while time.perf_counter() < t_end:
            _conn(hub).set_actuators(desires, send=False)
            _conn(hub).send_once()
            for slot in list(holds.keys()):
                fb = sample_position(hub, slot)
                if fb is not None:
                    q, kp, kd = holds[slot]
                    holds[slot] = (float(fb), kp, kd)
            next_t += dt
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()
        for slot, (q, _kp, _kd) in holds.items():
            print(f"  hold slot{slot} q={q:+.4f}", flush=True)

        # --- rotate each ---
        print("\n== ROTATE ==", flush=True)
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            mid_use = id_overrides.get(slot, mid)
            if slot not in holds:
                results.append((label, False, "no start pose / no FB"))
                print(f"  SKIP {label}: no FB", flush=True)
                continue
            start = holds[slot][0]
            ok, msg, end = rotate_one(
                hub,
                slot=slot,
                bus=bus,
                proto=proto,
                motor_id=mid_use,
                master_id=master,
                label=label,
                start=start,
                angle=float(args.angle),
                rate=float(args.rate),
                holds=holds,
            )
            results.append((label, ok, msg))
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {label} — {msg}", flush=True)

        # cleanup: blank desires, leave CFG as-is (RAM)
        print("\n== CLEANUP ==", flush=True)
        _blank_all(hub)
        time.sleep(0.1)
        hub.recover()
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _blank_all(hub)

    print("\n=== SUMMARY ===", flush=True)
    all_ok = True
    for name, ok, detail in results:
        mark = "PASS" if ok else "FAIL"
        all_ok = all_ok and ok
        print(f"  {mark}  {name}: {detail}", flush=True)
    print("OVERALL " + ("PASS" if all_ok else "FAIL"), flush=True)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
