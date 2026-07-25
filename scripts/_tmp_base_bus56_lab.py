#!/usr/bin/env python3
"""Interactive bus5/6 base lab — discover, calibrate, teleop with live FB.

No arm / no DXL. Find a working CH5/CH6 arrangement before continuous.

Map:
  slot22  bus5  RS02  0x70
  slot23  bus5  RS01  0x74
  slot24  bus6  RS01  0x75
  slot25  bus6  Damiao 0x06 / master 0x16

Teleop semantics (interactive ``t``)::

  t 22              +360 deg (2π rad) at default rate
  t 22 -            -360 deg
  t 22 3.14 0.4     +π rad @ 0.4 rad/s
  t 25              Damiao +360 deg (no RS rail logic)

CLI::

  python3 _tmp_base_bus56_lab.py --tx-smoke
  python3 _tmp_base_bus56_lab.py --prove-360
  python3 _tmp_base_bus56_lab.py --teleop 22 --angle 6.2832 --rate 0.4

Plant TX rules (MCP CH5/CH6 LEDs stay dark unless all of these hold)::

  1. mcu_state=NORMAL (DIAG/ESTOP gate CAN apply)
  2. background plant stream running (HOST_STALE if silent >500 ms)
  3. non-blank desires on enabled MCP slots (blank desire ⇒ skip SPI TX)
  4. no DEBUG lease / probe in flight (BENCH_SESSION gates apply)

Keys: d discover | c calibrate | cfg | s status | t teleop
      h hold | b blank | r recover | q quit
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, McuState  # noqa: E402
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT  # noqa: E402
from deft_controls_sdk.vbeta.cfg import pause_plant_stream  # noqa: E402
from deft_controls_sdk.vbeta.slots import PROTO_DAMIAO, PROTO_ROBSTRIDE  # noqa: E402
from rs02_channel_bringup import (  # noqa: E402
    quiet_all_slots,
    rs02_plan_angle,
    rs02_resolve_start,
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
BASE_SLOTS = tuple(r[0] for r in BASE_ROWS)

RS_KP = 20.0
RS_KD = 1.0
DM_KP = 8.0
DM_KD = 0.5
HZ = 40.0
RS_P_MIN = -12.57
RS_P_MAX = 12.57
RS_MARGIN = 0.35
# Default teleop = one full turn (tiny_teleop default).
DEFAULT_ANGLE = 2.0 * math.pi
DEFAULT_RATE = math.pi / 4.0
# Track error pass criteria for a "smooth" turn.
TRACK_OK_RAD = 0.55


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _blank(hub: ControlsPcbHub) -> None:
    _conn(hub).set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )


def _gains(proto: int) -> Tuple[float, float]:
    return (DM_KP, DM_KD) if proto == PROTO_DAMIAO else (RS_KP, RS_KD)


def ensure_streaming(hub: ControlsPcbHub, *, hz: float = HZ) -> None:
    """Keep a background plant TX thread alive (clears HOST_STALE)."""
    if not hub.is_streaming:
        hub.start_streaming(hz=hz)
        time.sleep(0.05)


def stop_streaming_quiet(hub: ControlsPcbHub) -> None:
    if hub.is_streaming:
        hub.stop_streaming()


def _plant_sleep(hub: ControlsPcbHub, next_t: float, dt: float) -> float:
    """Pace a control loop. Stream thread owns TX when live — avoid double-send."""
    next_t += dt
    sleep_for = next_t - time.perf_counter()
    if sleep_for > 0:
        time.sleep(sleep_for)
    else:
        next_t = time.perf_counter()
    if not hub.is_streaming:
        hub.send_once()
    return next_t


def plant_tx_health(hub: ControlsPcbHub) -> str:
    """One-line plant gate / stream health for LED debugging."""
    from deft_controls_sdk.bench.metrics import drain_latest
    from deft_controls_sdk.link.api_types import FeedbackImage, PlantBlockReason

    ensure_streaming(hub)
    hub.set_mcu_state(McuState.NORMAL, send=False)
    raw = None
    for _ in range(12):
        time.sleep(0.03)
        got = drain_latest(hub)
        if got is not None:
            from deft_controls_sdk.link.exchange.parse import parse_feedback_header

            hdr = parse_feedback_header(got)
            if hdr is not None and not hdr.get("is_debug"):
                raw = got
                break
    mcu = "?"
    pb = "?"
    if raw is not None:
        try:
            fb = FeedbackImage(raw)
            mcu = str(int(fb.mcu_state))
            pbi = int(fb.plant_block)
            try:
                pb = PlantBlockReason(pbi).name
            except Exception:
                pb = str(pbi)
        except Exception:
            pass
    hot = getattr(_conn(hub), "_hot_stats", {}) or {}
    tx = hot.get("tx_hz")
    tx_s = f"{tx:.1f}" if isinstance(tx, (int, float)) and tx else "?"
    stream = "ON" if hub.is_streaming else "off"
    held = sum(
        1
        for s, d in hub.held_desires().items()
        if s in BASE_SLOTS and abs(float(d.position)) + abs(float(d.kp)) > 1e-9
    )
    return (
        f"stream={stream} tx_hz={tx_s} mcu={mcu} plant_block={pb} "
        f"nonblank_base={held}/4"
    )


def bringup_plant_tx(hub: ControlsPcbHub, holds: Dict[int, float]) -> None:
    """NORMAL + stream + non-blank holds — required for MCP CAN TX LEDs."""
    hub.set_mcu_state(McuState.NORMAL, send=False)
    if holds:
        _write_holds(hub, holds, vel=0.0, gain=1.0)
    ensure_streaming(hub)
    # A few stream ticks so HOST_STALE clears and MCP starts.
    time.sleep(0.20)
    print(f"  plant TX: {plant_tx_health(hub)}", flush=True)


def apply_base_cfg(
    hub: ControlsPcbHub, *, only: Optional[Sequence[int]] = None
) -> None:
    """Enable base rows. ``only`` = teleop one motor (siblings CFG-off — avoids MCP FB alias)."""
    want = set(only) if only is not None else set(BASE_SLOTS)
    with pause_plant_stream(hub):
        n = quiet_all_slots(hub)
        print(f"CFG quieted {n} slot(s); enabling {sorted(want)}", flush=True)
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            en = slot in want
            hub.debug.cfg_set_slot(
                slot=slot,
                bus=bus,
                protocol=proto,
                motor_id=mid,
                master_id=master,
                enabled=en,
                persist=False,
            )
            print(
                f"  slot{slot} {label} bus={bus} id=0x{mid:02X} "
                f"{'ON' if en else 'off'}",
                flush=True,
            )
    # CFG used DIAG-ish mailbox traffic — reassert plant stream immediately.
    ensure_streaming(hub)
    hub.set_mcu_state(McuState.NORMAL, send=False)


def kick_mcp_buses(hub: ControlsPcbHub) -> None:
    """RS2 SESSION_BEGIN/END on bus 5 and 6 — wakes MCP rings after long idle."""
    from deft_controls_sdk.link.exchange import (
        SESSION_BEGIN,
        SESSION_END,
        build_rs2_scan_command,
        parse_probe_pdu,
    )

    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    conn = _conn(hub)
    for bus in (5, 6):
        for kind in (SESSION_BEGIN, SESSION_END):
            try:
                conn.exchange_raw(
                    build_rs2_scan_command(0, kind, conn.next_seq(), bus=bus),
                    parse_probe_pdu,
                    timeout_s=2.5,
                    predicate=lambda p, k=kind: p.get("probe_kind") == k,
                )
            except Exception as exc:
                print(f"  kick bus{bus} kind={kind}: {exc}", flush=True)


def discover_all(hub: ControlsPcbHub) -> Dict[int, float]:
    """Probe/discover each row. Returns slot -> position for hits.

    DEBUG leases gate plant CAN — stream is paused for the whole discover.
    Caller must ``bringup_plant_tx`` / ``seed_holds`` afterward for TX LEDs.
    """
    found: Dict[int, float] = {}
    with pause_plant_stream(hub):
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        kick_mcp_buses(hub)
        for slot, bus, proto, mid, master, label in BASE_ROWS:
            print(f"\n== discover {label} slot{slot} ==", flush=True)
            if proto == PROTO_ROBSTRIDE:
                resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
                if resp is not None and resp.get("found"):
                    q = float(resp["position"])
                    found[slot] = q
                    print(f"  OK probe id=0x{mid:02X} pos={q:+.4f}", flush=True)
                else:
                    hits = hub.debug.discover_robstride_all(
                        bus=bus, start=0x01, end=0x7F
                    )
                    print(f"  MISS preferred; discover={hits}", flush=True)
                    if mid in set(int(x) for x in hits):
                        resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
                        if resp is not None and resp.get("found"):
                            q = float(resp["position"])
                            found[slot] = q
                            print(f"  OK after discover pos={q:+.4f}", flush=True)
            else:
                ids = hub.debug.discover_damiao_all(
                    bus=bus, start=1, end=16, listen_ms=100
                )
                print(f"  Damiao discover={ids}", flush=True)
                if mid in set(int(x) for x in ids):
                    found[slot] = 0.0
                    print(
                        f"  OK id=0x{mid:02X} on bus (use plant FB after CFG)",
                        flush=True,
                    )
    hub.set_mcu_state(McuState.NORMAL, send=False)
    ensure_streaming(hub)
    return found


def calibrate_rs(
    hub: ControlsPcbHub, *, slots: Optional[Sequence[int]] = None
) -> None:
    """Calibrate RobStride rows (shafts must spin freely)."""
    want = set(slots) if slots else {22, 23, 24}
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    _blank(hub)
    _conn(hub).send_once()
    for slot, bus, proto, mid, master, label in BASE_ROWS:
        if slot not in want or proto != PROTO_ROBSTRIDE:
            continue
        print(f"\n== CALI {label} bus={bus} id=0x{mid:02X} ==", flush=True)
        print("  shaft must spin freely (~28s)", flush=True)
        try:
            ok = hub.debug.calibrate_robstride(
                bus=bus, motor_id=mid, cal_listen_s=28.0
            )
        except Exception as exc:
            print(f"  EXCEPTION {exc}", flush=True)
            ok = False
        print(f"  cali={'OK' if ok else 'FAIL'}", flush=True)
        en = hub.debug.probe_robstride(bus=bus, motor_id=mid)
        if en is not None:
            print(
                f"  post pos={en.get('position')} found={en.get('found')}",
                flush=True,
            )


def print_status(hub: ControlsPcbHub, *, holds: Optional[Dict[int, float]] = None) -> None:
    from deft_controls_sdk.bench.metrics import drain_latest
    from deft_controls_sdk.link.api_types import FeedbackImage, PlantBlockReason
    from deft_controls_sdk.link.exchange.parse import (
        parse_actuator_feedback,
        parse_feedback_header,
    )

    raw = None
    for _ in range(10):
        hub.send_once()
        got = drain_latest(hub)
        if got is not None:
            hdr = parse_feedback_header(got)
            if hdr is not None and not hdr.get("is_debug"):
                raw = got
                _conn(hub)._latest_fb_raw = got  # noqa: SLF001
                break
        time.sleep(0.02)

    fb = None
    if raw is not None:
        try:
            fb = FeedbackImage(raw)
        except Exception:
            fb = None
    pdb = hub.pdb_status(raw)
    if fb is not None:
        pb = int(fb.plant_block)
        try:
            pb_name = PlantBlockReason(pb).name
        except Exception:
            pb_name = str(pb)
        print(
            f"mcu={fb.mcu_state} (0=NORMAL 2=DIAG 3=ESTOP) "
            f"plant_block={pb} ({pb_name})",
            flush=True,
        )
    else:
        print("mcu=? (no plant FB yet)", flush=True)
    if pdb is not None:
        print(
            f"pdb={pdb.kill_state_name}/{pdb.kill_reason_name} "
            f"stale={pdb.stale_failsafe} estop_sense={pdb.estop_sense}",
            flush=True,
        )
    for slot, bus, proto, mid, master, label in BASE_ROWS:
        hold = holds.get(slot) if holds else None
        pos = None
        tau = None
        fault = None
        vel = None
        if raw is not None:
            parsed = parse_actuator_feedback(raw, slot)
            if parsed is not None:
                pos = float(parsed["position"])
                tau = float(parsed.get("torque", 0.0))
                fault = int(parsed.get("fault", -1))
                vel = float(parsed.get("velocity", 0.0))
        if pos is None:
            pos = sample_position(hub, slot, timeout_s=0.4)
        if pos is None and proto == PROTO_ROBSTRIDE:
            with pause_plant_stream(hub):
                resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
            ensure_streaming(hub)
            hub.set_mcu_state(McuState.NORMAL, send=False)
            if resp is not None and resp.get("found"):
                pos = float(resp["position"])
        if pos is None:
            print(f"  s{slot} {label}: no FB", flush=True)
            continue
        extra = ""
        if tau is not None:
            extra = f" vel={vel:+.3f} tau={tau:+.2f} fault={fault}"
        print(
            f"  s{slot} {label}: pos={pos:+.4f}{extra}"
            + (f" hold={hold:+.4f}" if hold is not None else ""),
            flush=True,
        )
        if proto == PROTO_ROBSTRIDE and abs(pos) > (RS_P_MAX - RS_MARGIN - 0.15):
            print(
                f"    WARN near MIT rail — teleop toward center "
                f"(e.g. t {slot} {-1.5 if pos > 0 else 1.5})",
                flush=True,
            )


def seed_holds(hub: ControlsPcbHub) -> Dict[int, float]:
    """Soft-hold each base slot at a trusted pose.

    Prefer RS probe over plant FB — after long runs / rail parks, plant FB is
    often stale across the ±12.57 MIT rail and will snap motors if used as Goal.

    Probes pause plant apply (lease). We stream again with non-blank holds so
    MCP TX LEDs come back immediately after seeding.
    """
    holds: Dict[int, float] = {}
    hub.set_mcu_state(McuState.NORMAL, send=False)
    ensure_streaming(hub)
    time.sleep(0.15)
    for slot, bus, proto, mid, master, label in BASE_ROWS:
        plant = sample_position(hub, slot, timeout_s=0.35)
        probe = None
        if proto == PROTO_ROBSTRIDE:
            # Lease gates CAN — keep it short; resume stream after.
            with pause_plant_stream(hub):
                resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
            if resp is not None and resp.get("found"):
                probe = float(resp["position"])
            ensure_streaming(hub)
            hub.set_mcu_state(McuState.NORMAL, send=False)
        q = rs02_resolve_start(probe, plant) if proto == PROTO_ROBSTRIDE else plant
        if q is None:
            q = 1e-6
        if abs(q) < 1e-6:
            q = 1e-6
        holds[slot] = float(q)
        # Keep a non-blank desire on this slot while probing siblings.
        seed_idle_at_fb(hub, slot, float(q))
        print(
            f"  seed s{slot} {label}: probe={probe} plant={plant} -> hold={holds[slot]:+.4f}",
            flush=True,
        )
    # Soft engage: ramp gains while glued to holds (no chase).
    for u in (0.0, 0.25, 0.5, 0.75, 1.0):
        _write_holds(hub, holds, vel=0.0, gain=u)
        time.sleep(0.12)
    bringup_plant_tx(hub, holds)
    print(
        "holds: " + " ".join(f"s{s}={holds[s]:+.3f}" for s in sorted(holds)),
        flush=True,
    )
    return holds


def _write_holds(
    hub: ControlsPcbHub,
    holds: Dict[int, float],
    *,
    vel: float,
    gain: float,
    active: Optional[int] = None,
    active_pos: Optional[float] = None,
    active_vel: Optional[float] = None,
) -> None:
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    g = float(max(0.0, min(1.0, gain)))
    for slot, bus, proto, mid, master, label in BASE_ROWS:
        kp, kd = _gains(proto)
        if active is not None and slot == active and active_pos is not None:
            desires[slot] = ActuatorDesire(
                position=float(active_pos),
                velocity=float(active_vel or 0.0),
                kp=kp * g,
                kd=kd * g,
            )
        elif slot in holds:
            desires[slot] = ActuatorDesire(
                position=float(holds[slot]),
                velocity=float(vel),
                kp=kp * g,
                kd=kd * g,
            )
    _conn(hub).set_actuators(desires, send=False)


def hold_loop(
    hub: ControlsPcbHub,
    holds: Dict[int, float],
    *,
    seconds: float,
    status_hz: float,
) -> None:
    bringup_plant_tx(hub, holds)
    dt = 1.0 / HZ
    t_end = time.perf_counter() + seconds
    next_stat = time.perf_counter()
    next_t = time.perf_counter()
    while time.perf_counter() < t_end:
        # Refresh holds slowly from FB when close (no yank).
        for slot in list(holds):
            fb = sample_position(hub, slot, timeout_s=0.05)
            if fb is not None and abs(fb - holds[slot]) < 0.35:
                holds[slot] = 0.98 * holds[slot] + 0.02 * fb
        _write_holds(hub, holds, vel=0.0, gain=1.0)
        now = time.perf_counter()
        if now >= next_stat:
            print(f"  {plant_tx_health(hub)}", flush=True)
            print_status(hub, holds=holds)
            next_stat = now + 1.0 / max(status_hz, 0.1)
        next_t = _plant_sleep(hub, next_t, dt)


def _rs_ids_on_bus(bus: int) -> List[int]:
    return [
        mid
        for _s, b, proto, mid, _m, _lab in BASE_ROWS
        if b == bus and proto == PROTO_ROBSTRIDE
    ]


def rs_reset_id(hub: ControlsPcbHub, *, bus: int, motor_id: int) -> Optional[dict]:
    """DEBUG reset (0x04) — put drive to rest so a sibling can arm on MCP."""
    from deft_controls_sdk.bench.lease import lease
    from deft_controls_sdk.link.exchange import (
        PROBE_RESET,
        build_rs2_probe_command,
        is_mcp_bus,
        parse_probe_pdu,
    )

    conn = _conn(hub)
    timeout_s = 2.0 if is_mcp_bus(bus) else 0.55
    with lease(conn, hub.telemetry, bus=bus):
        conn.reader.drain()
        frame = build_rs2_probe_command(
            motor_id, PROBE_RESET, conn.next_seq(), bus=bus
        )
        try:
            return conn.exchange_raw(
                frame,
                parse_probe_pdu,
                timeout_s=timeout_s,
                predicate=lambda p: p.get("probe_kind") == PROBE_RESET
                and p.get("probe_id") == (motor_id & 0xFF),
            )
        except Exception as exc:
            print(f"  reset 0x{motor_id:02X}: {exc}", flush=True)
            return None


def rs_arm_for_mit(
    hub: ControlsPcbHub, *, slot: int, bus: int, motor_id: int, label: str
) -> Optional[float]:
    """Quiet same-bus RS siblings, reset+enable target, soft-hold verify.

    Daisy-chained MCP (bus5: 0x70+0x74) — an already-enabled sibling swamps
    enable so the target answers probe FB but never tracks MIT.
    """
    print(f"  arm {label} bus{bus} id=0x{motor_id:02X} for MIT…", flush=True)
    with pause_plant_stream(hub):
        for other in _rs_ids_on_bus(bus):
            if other == (motor_id & 0xFF):
                continue
            print(f"  reset sibling id=0x{other:02X}", flush=True)
            rs_reset_id(hub, bus=bus, motor_id=other)
            time.sleep(0.08)
        # probe_robstride now does reset→enable on MCP too.
        en = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
        if en is None or not en.get("found"):
            print("  FAIL: enable probe miss", flush=True)
            return None
        start = float(en["position"])
        print(f"  enable ok pos={start:+.4f}", flush=True)
        # Second enable after brief settle (post-cali / sticky sibling).
        time.sleep(0.12)
        en2 = hub.debug.probe_robstride(bus=bus, motor_id=motor_id)
        if en2 is not None and en2.get("found"):
            start = float(en2["position"])
            print(f"  re-enable ok pos={start:+.4f}", flush=True)

    if abs(start) < 1e-6:
        start = 1e-6
    hub.set_mcu_state(McuState.NORMAL, send=False)
    ensure_streaming(hub)
    seed_idle_at_fb(hub, slot, float(start))
    time.sleep(0.15)
    # Raise kp at present pose so firmware maintain_enable + MIT actually fire.
    kp, kd = _gains(PROTO_ROBSTRIDE)
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    desires[slot] = ActuatorDesire(
        position=float(start), velocity=0.0, kp=kp, kd=kd
    )
    _conn(hub).set_actuators(desires, send=False)
    t_end = time.perf_counter() + 0.55
    while time.perf_counter() < t_end:
        time.sleep(0.05)
    fb = sample_position(hub, slot, timeout_s=0.3)
    if fb is not None and abs(fb - start) < 0.40:
        start = float(fb)
    print(f"  armed hold fb={fb if fb is not None else float('nan'):+.4f}", flush=True)
    return float(start)


def _parse_teleop_args(
    tokens: Sequence[str], *, default_angle: float, default_rate: float
) -> Tuple[int, float, float]:
    """Parse ``t <slot> [angle|+|-] [rate]`` → (slot, angle_rad, rate)."""
    if len(tokens) < 1:
        raise ValueError("need slot")
    slot = int(tokens[0])
    angle = float(default_angle)
    rate = float(default_rate)
    if len(tokens) >= 2:
        a = tokens[1]
        if a in ("+", "cw", "pos"):
            angle = abs(default_angle)
        elif a in ("-", "ccw", "neg"):
            angle = -abs(default_angle)
        else:
            angle = float(a)
    if len(tokens) >= 3:
        rate = abs(float(tokens[2]))
    if abs(angle) < 1e-6:
        # Bare ``0`` is almost always a mistake — treat as +360°.
        print("  note: angle 0 → default +360 deg", flush=True)
        angle = abs(default_angle)
    return slot, angle, rate


def teleop_slot(
    hub: ControlsPcbHub,
    holds: Dict[int, float],
    *,
    slot: int,
    angle: float,
    rate: float,
    status_hz: float,
) -> Tuple[Dict[int, float], bool, str]:
    """Solo-CFG teleop. Returns (holds, ok, message)."""
    row = next(r for r in BASE_ROWS if r[0] == slot)
    _slot, bus, proto, mid, master, label = row
    if abs(angle) < 1e-6:
        angle = DEFAULT_ANGLE
    deg = math.degrees(angle)
    print(
        f"\n== teleop {label} slot{slot}  {deg:+.1f} deg ({angle:+.3f} rad) "
        f"@ {rate:.3f} rad/s ==",
        flush=True,
    )
    # Solo-CFG: multi-slot MCP plant FB has been stale/aliased and snaps siblings.
    apply_base_cfg(hub, only=[slot])
    hub.set_mcu_state(McuState.NORMAL, send=False)
    ensure_streaming(hub)
    time.sleep(0.15)

    plant = sample_position(hub, slot, timeout_s=0.5)
    if proto == PROTO_ROBSTRIDE:
        start = rs_arm_for_mit(
            hub, slot=slot, bus=bus, motor_id=mid, label=label
        )
        if start is None:
            msg = "FAIL: could not arm RS for MIT (enable)"
            print(f"  {msg}", flush=True)
            apply_base_cfg(hub)
            return seed_holds(hub), False, msg
        if plant is not None and abs(start - plant) > 0.5:
            print(
                f"  note: plant {plant:+.4f} vs armed {start:+.4f} — using armed",
                flush=True,
            )
    else:
        start = plant
        if start is None:
            msg = "FAIL: no start pose"
            print(f"  {msg}", flush=True)
            apply_base_cfg(hub)
            return seed_holds(hub), False, msg
        seed_idle_at_fb(hub, slot, float(start))
        time.sleep(0.15)

    if proto == PROTO_ROBSTRIDE:
        planned = rs02_plan_angle(start, angle)
        if abs(planned) < abs(angle) - 0.05:
            # Auto-take the other direction if that fits better.
            alt = rs02_plan_angle(start, -angle)
            if abs(alt) > abs(planned) + 0.05:
                print(
                    f"  note: requested {angle:+.3f} only fits {planned:+.3f}; "
                    f"using {alt:+.3f} (more MIT room)",
                    flush=True,
                )
                planned = alt
        if abs(planned) < 0.20:
            msg = (
                f"FAIL: almost no MIT room from {start:+.4f} "
                f"(near ±12.57 rail) — calibrate or t {slot} -"
            )
            print(f"  {msg}", flush=True)
            apply_base_cfg(hub)
            return seed_holds(hub), False, msg
    else:
        # Damiao: free travel (not RS MIT rails). -6.5 is a normal pose.
        planned = float(angle)

    print(
        f"  start={start:+.4f} → delta={planned:+.4f} "
        f"({math.degrees(planned):+.1f} deg) solo-CFG",
        flush=True,
    )

    solo_holds = {slot: float(start)}
    bringup_plant_tx(hub, solo_holds)
    peak_err = _slew_with_holds(
        hub,
        solo_holds,
        slot=slot,
        start=float(start),
        delta=planned,
        rate=rate,
        status_hz=status_hz,
    )
    end = sample_position(hub, slot, timeout_s=0.5)
    if proto == PROTO_ROBSTRIDE:
        with pause_plant_stream(hub):
            en = hub.debug.probe_robstride(bus=bus, motor_id=mid)
        ensure_streaming(hub)
        hub.set_mcu_state(McuState.NORMAL, send=False)
        if en is not None and en.get("found"):
            end = float(en["position"])
    moved = (float(end) - float(start)) if end is not None else float("nan")
    ok = (
        end is not None
        and abs(moved - planned) < TRACK_OK_RAD
        and peak_err < TRACK_OK_RAD
    )
    msg = (
        f"{'PASS' if ok else 'FAIL'} moved={moved:+.3f} "
        f"(wanted {planned:+.3f}) peak_err={peak_err:.3f}"
    )
    print(f"  {msg}", flush=True)

    apply_base_cfg(hub)
    return seed_holds(hub), ok, msg


def _slew_with_holds(
    hub: ControlsPcbHub,
    holds: Dict[int, float],
    *,
    slot: int,
    start: float,
    delta: float,
    rate: float,
    status_hz: float,
) -> float:
    """Rate-limited slew. Returns peak |fb-cmd| during cruise.

    Background stream owns CDC TX — this loop only updates desires + paces.
    """
    ensure_streaming(hub)
    hub.set_mcu_state(McuState.NORMAL, send=False)
    target = start + delta
    sign = 1.0 if delta >= 0 else -1.0
    cmd = float(start)
    dt = 1.0 / HZ
    peak_err = 0.0
    # Soft engage at start (v=0).
    t_eng = time.perf_counter() + 0.40
    next_t = time.perf_counter()
    while time.perf_counter() < t_eng:
        fb = sample_position(hub, slot, timeout_s=0.05)
        if fb is not None and abs(fb - cmd) < 0.5:
            cmd = float(fb)
        holds[slot] = cmd
        _write_holds(
            hub, holds, vel=0.0, gain=1.0, active=slot, active_pos=cmd, active_vel=0.0
        )
        next_t = _plant_sleep(hub, next_t, dt)

    next_stat = time.perf_counter()
    next_t = time.perf_counter()
    while abs(target - cmd) > 0.02:
        step = min(abs(rate) * dt, abs(target - cmd))
        cmd += sign * step
        holds[slot] = cmd
        _write_holds(
            hub,
            holds,
            vel=0.0,
            gain=1.0,
            active=slot,
            active_pos=cmd,
            active_vel=sign * abs(rate),
        )
        fb = sample_position(hub, slot, timeout_s=0.05)
        if fb is not None:
            peak_err = max(peak_err, abs(fb - cmd))
        now = time.perf_counter()
        if now >= next_stat:
            print(
                f"  slew cmd={cmd:+.3f} "
                f"fb={fb if fb is not None else float('nan'):+.3f} "
                f"tgt={target:+.3f} err_pk={peak_err:.3f} | {plant_tx_health(hub)}",
                flush=True,
            )
            next_stat = now + 1.0 / max(status_hz, 0.1)
        next_t = _plant_sleep(hub, next_t, dt)
    holds[slot] = target
    _write_holds(hub, holds, vel=0.0, gain=1.0, active=slot, active_pos=target, active_vel=0.0)
    settle_end = time.perf_counter() + 0.20
    next_t = time.perf_counter()
    while time.perf_counter() < settle_end:
        next_t = _plant_sleep(hub, next_t, dt)
    return peak_err


def blank_diag(hub: ControlsPcbHub) -> None:
    """Park: DIAG + blank desires. MCP TX LEDs go dark by design."""
    stop_streaming_quiet(hub)
    hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
    _blank(hub)
    hub.set_led(LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8), send=False)
    for _ in range(6):
        hub.send_once()
        time.sleep(0.04)
    print("blank + DIAG (CAN TX off — expected)", flush=True)


def interactive(hub: ControlsPcbHub, *, status_hz: float, angle: float, rate: float) -> int:
    # Stay in NORMAL+stream with holds at the prompt so CH5/6 TX LEDs stay alive.
    apply_base_cfg(hub)
    holds = seed_holds(hub)
    print(
        "\nbase bus5/6 lab (plant stream live — watch CH5/CH6 TX LEDs)\n"
        "  d=discover  c=calibrate-RS  cfg=apply CFG  s=status\n"
        "  t=teleop  h=hold 10s  b=blank+DIAG  r=recover  q=quit\n"
        f"  {plant_tx_health(hub)}\n",
        flush=True,
    )
    while True:
        try:
            line = input("lab> ").strip()
        except (EOFError, KeyboardInterrupt):
            print(flush=True)
            break
        if not line:
            if holds:
                bringup_plant_tx(hub, holds)
            continue
        cmd = line.split()
        op = cmd[0].lower()
        if op in ("q", "quit", "exit"):
            break
        if op == "d":
            discover_all(hub)
            if holds:
                bringup_plant_tx(hub, holds)
        elif op == "cfg":
            apply_base_cfg(hub)
            holds = seed_holds(hub)
        elif op in ("c", "cal", "calibrate"):
            which = cmd[1] if len(cmd) > 1 else "all"
            stop_streaming_quiet(hub)
            if which == "all":
                calibrate_rs(hub)
            else:
                calibrate_rs(hub, slots=[int(which)])
            apply_base_cfg(hub)
            holds = seed_holds(hub)
        elif op == "s":
            print(f"  {plant_tx_health(hub)}", flush=True)
            print_status(hub, holds=holds or None)
        elif op == "h":
            if not holds:
                apply_base_cfg(hub)
                holds = seed_holds(hub)
            hold_loop(hub, holds, seconds=10.0, status_hz=status_hz)
        elif op == "t":
            if len(cmd) < 2:
                print(
                    "usage: t <slot>           → +360 deg\n"
                    "       t <slot> -         → -360 deg\n"
                    "       t <slot> <rad> [rate]",
                    flush=True,
                )
                continue
            try:
                slot, ang, rt = _parse_teleop_args(
                    cmd[1:], default_angle=angle, default_rate=rate
                )
            except Exception as exc:
                print(f"  bad args: {exc}", flush=True)
                continue
            if slot not in BASE_SLOTS:
                print(f"slot must be one of {BASE_SLOTS}", flush=True)
                continue
            if not holds:
                apply_base_cfg(hub)
                holds = seed_holds(hub)
            holds, _ok, _msg = teleop_slot(
                hub, holds, slot=slot, angle=ang, rate=rt, status_hz=status_hz
            )
        elif op == "b":
            blank_diag(hub)
            holds = {}
        elif op == "r":
            hub.recover()
            time.sleep(0.2)
            ensure_streaming(hub)
            if holds:
                bringup_plant_tx(hub, holds)
            print("recover done", flush=True)
            print(f"  {plant_tx_health(hub)}", flush=True)
            print_status(hub)
        else:
            print("unknown — d/cfg/c/s/t/h/b/r/q", flush=True)
    blank_diag(hub)
    return 0


def fix_74_cal_and_360(
    hub: ControlsPcbHub, *, rate: float, status_hz: float
) -> int:
    """Recalibrate bus5 RS01 0x74, arm cleanly, prove one 360°."""
    print("\n======== FIX 0x74: cali + arm + 360° ========", flush=True)
    apply_base_cfg(hub, only=[23])
    # Quiet daisy-chain peer before cali so 0x70 does not fight the shaft.
    with pause_plant_stream(hub):
        print("  reset sibling 0x70 before cali", flush=True)
        rs_reset_id(hub, bus=5, motor_id=0x70)
    stop_streaming_quiet(hub)
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    print("  calibrating slot23 / 0x74 (shaft must spin freely ~28s)…", flush=True)
    try:
        ok_cal = hub.debug.calibrate_robstride(
            bus=5, motor_id=0x74, cal_listen_s=28.0
        )
    except Exception as exc:
        print(f"  cali EXCEPTION {exc}", flush=True)
        ok_cal = False
    print(f"  cali={'OK' if ok_cal else 'FAIL'}", flush=True)

    apply_base_cfg(hub, only=[23])
    hub.set_mcu_state(McuState.NORMAL, send=False)
    ensure_streaming(hub)
    holds: Dict[int, float] = {}
    holds, ok, msg = teleop_slot(
        hub,
        holds,
        slot=23,
        angle=DEFAULT_ANGLE,
        rate=rate,
        status_hz=status_hz,
    )
    print(
        f"\n======== FIX 0x74 RESULT: {'PASS' if ok else 'FAIL'} — {msg} ========",
        flush=True,
    )
    apply_base_cfg(hub)
    seed_holds(hub)
    blank_diag(hub)
    return 0 if ok else 1


def tx_smoke(hub: ControlsPcbHub, *, seconds: float = 8.0) -> int:
    """CFG + seed + hold — prove MCP CAN TX LEDs without motion."""
    print("\n======== TX SMOKE (hold, watch CH5/CH6 TX LEDs) ========", flush=True)
    apply_base_cfg(hub)
    holds = seed_holds(hub)
    print(f"  {plant_tx_health(hub)}", flush=True)
    hold_loop(hub, holds, seconds=seconds, status_hz=1.0)
    print(f"  final: {plant_tx_health(hub)}", flush=True)
    blank_diag(hub)
    return 0


def prove_360(
    hub: ControlsPcbHub, *, rate: float, status_hz: float
) -> int:
    """Solo 360° on every base actuator. Return 0 if all PASS."""
    print("\n======== PROVE 360° each base actuator ========", flush=True)
    apply_base_cfg(hub)
    discover_all(hub)
    holds = seed_holds(hub)
    print(f"  pre-motion: {plant_tx_health(hub)}", flush=True)
    results: List[Tuple[int, str, bool, str]] = []
    for slot, bus, proto, mid, master, label in BASE_ROWS:
        # Prefer direction with MIT room (RS); Damiao always +2π.
        plant = sample_position(hub, slot, timeout_s=0.3)
        start = plant
        if proto == PROTO_ROBSTRIDE:
            with pause_plant_stream(hub):
                resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
            ensure_streaming(hub)
            hub.set_mcu_state(McuState.NORMAL, send=False)
            if resp is not None and resp.get("found"):
                start = float(resp["position"])
            ang = DEFAULT_ANGLE
            if start is not None:
                p = rs02_plan_angle(start, ang)
                a = rs02_plan_angle(start, -ang)
                ang = ang if abs(p) >= abs(a) else -ang
        else:
            ang = DEFAULT_ANGLE
        holds, ok, msg = teleop_slot(
            hub,
            holds,
            slot=slot,
            angle=ang,
            rate=rate,
            status_hz=status_hz,
        )
        results.append((slot, label, ok, msg))
    print("\n======== PROVE SUMMARY ========", flush=True)
    all_ok = True
    for slot, label, ok, msg in results:
        print(f"  s{slot} {label}: {'PASS' if ok else 'FAIL'} — {msg}", flush=True)
        all_ok = all_ok and ok
    print(f"OVERALL: {'PASS' if all_ok else 'FAIL'}", flush=True)
    blank_diag(hub)
    return 0 if all_ok else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--discover", action="store_true")
    ap.add_argument("--calibrate", action="store_true", help="Calibrate all three RS")
    ap.add_argument("--teleop", type=int, default=None, metavar="SLOT")
    ap.add_argument(
        "--tx-smoke",
        action="store_true",
        help="Hold base slots with live plant stream (verify CAN TX LEDs)",
    )
    ap.add_argument(
        "--fix-74",
        action="store_true",
        help="Recalibrate bus5 0x74 (slot23), arm, prove one 360°",
    )
    ap.add_argument(
        "--prove-360",
        action="store_true",
        help="Non-interactive: 360 deg turn on each of 22..25",
    )
    ap.add_argument(
        "--angle",
        type=float,
        default=DEFAULT_ANGLE,
        help=f"teleop angle rad (default {DEFAULT_ANGLE:.4f} = 360 deg)",
    )
    ap.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE,
        help=f"teleop rate rad/s (default {DEFAULT_RATE:.4f})",
    )
    ap.add_argument("--status-hz", type=float, default=2.0)
    ap.add_argument("--hold-s", type=float, default=0.0, help="after teleop, hold N seconds")
    args = ap.parse_args(list(argv) if argv is not None else None)

    port = args.port or find_cdc_port()
    print(f"base bus5/6 lab port={port}", flush=True)

    with ControlsPcbHub.connect(port, persist_telemetry=True) as hub:
        hub.set_rx_sim_mask(0)
        hub.recover()
        time.sleep(0.15)
        # Do NOT park in blank+DIAG before motion — that kills MCP TX LEDs
        # and left a hung interactive lab holding CDC with zero CAN activity.
        hub.set_mcu_state(McuState.NORMAL, send=True)
        ensure_streaming(hub)

        if args.tx_smoke:
            return tx_smoke(hub, seconds=max(3.0, float(args.hold_s) or 8.0))

        if args.fix_74:
            return fix_74_cal_and_360(
                hub, rate=float(args.rate), status_hz=float(args.status_hz)
            )

        if args.prove_360:
            return prove_360(
                hub, rate=float(args.rate), status_hz=float(args.status_hz)
            )

        # Non-interactive shortcuts.
        if args.discover or args.calibrate or args.teleop is not None:
            apply_base_cfg(hub)
            discover_all(hub)
            if args.calibrate:
                stop_streaming_quiet(hub)
                calibrate_rs(hub)
                apply_base_cfg(hub)
            holds = seed_holds(hub)
            print(f"  {plant_tx_health(hub)}", flush=True)
            print_status(hub, holds=holds)
            if args.teleop is not None:
                holds, _ok, _msg = teleop_slot(
                    hub,
                    holds,
                    slot=int(args.teleop),
                    angle=float(args.angle),
                    rate=float(args.rate),
                    status_hz=float(args.status_hz),
                )
            if args.hold_s > 0:
                hold_loop(
                    hub, holds, seconds=float(args.hold_s), status_hz=float(args.status_hz)
                )
            blank_diag(hub)
            return 0

        return interactive(
            hub,
            status_hz=float(args.status_hz),
            angle=float(args.angle),
            rate=float(args.rate),
        )


if __name__ == "__main__":
    raise SystemExit(main())
