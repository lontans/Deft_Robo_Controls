#!/usr/bin/env python3
"""Continuous YAM left arm + DXL neck + bus5/6 base (teleop / smoke pattern).

Arm (CH1 Damiao slots 0-6): **all-7 CFG enabled**, Goal=FB acquire, soft-engage
with teleop gains. Default cruise: J2 CLEAR bounce while J1/J3-J7 brace at FB.
``--mouse``: M650 joystick teleop instead of J2 bounce (same mapping as
``mouse_arm_teleop.py``); base/DXL keep cruising. Solo-CFG J2 left wrists
faulted (no clear/enable/MIT) — do not use that path.

Skips Damiao REG_SCAN discover by default (trust YAM CFG + plant FB). Optional
``--discover`` / ``--discover-base`` for bus sweeps when bringing up a new rig.

DXL neck (host servo slots 0/1, IDs 1/2): torque-off present → hold →
gentle clear bounce (same as yam_rig_smoke_suite / yam_dxl_clear_teleop).

Base MCP 22-25: continuous spin with vel FF; reverse only when the command
integrator hits an RS MIT rail / Damiao soft window (not FB-lag mid-travel).

Telemetry: ``persist_telemetry=True`` → ``.deft_session/state.json``.
Use ``--record`` for NDJSON under ``.deft_session/recordings/``.
Dashboard: run without Connect COM to follow that file.

PDU / estop: plant stream auto-parks on ``SOFT_KILL_REQ``; status lines log
``pdb_status`` (``estop_sense``, kill reason). Dashboard Soft-kill Park while
following state.json writes ``.deft_session/soft_kill_request`` — this process
parks ESTOP. ``--estop-after SEC`` asserts host ESTOP and checks MCU latch.
Dashboard ``HARD`` + ``COMMS_LOSS`` with host ``mcu_state=0`` means stale PDU
UART, not GPIO.

``killall -9`` does not stop CAN — Ctrl-C / ``stop_can.py``.
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, LedDesire, McuState, ServoDesire  # noqa: E402
from deft_controls_sdk.bench.lease import lease  # noqa: E402
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import (  # noqa: E402
    LED_MODE_IDLE_CORNFLOWER,
    FeedbackImage,
    PlantBlockReason,
)
from deft_controls_sdk.link.exchange import (  # noqa: E402
    ACTUATOR_COUNT,
    PROBE_RESET,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_probe_command,
    build_rs2_scan_command,
    is_mcp_bus,
    parse_probe_pdu,
    parse_servo_feedback,
)
from deft_controls_sdk.vbeta import PcbRobotSession  # noqa: E402
from deft_controls_sdk.vbeta.cfg import ensure_yam_left_arm_cfg, pause_plant_stream  # noqa: E402
from deft_controls_sdk.vbeta.gravity_comp import GravityComp  # noqa: E402
from deft_controls_sdk.vbeta.slots import (  # noqa: E402
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    LEFT_ARM_SLOTS,
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
    PROTO_DAMIAO,
    PROTO_ROBSTRIDE,
    _DAMIAO_MASTER,
)
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI, CLEAR_LO  # noqa: E402
from deft_controls_sdk.bench.rs02_motion import rs02_resolve_start  # noqa: E402

# Host setpoint rate. Was 20Hz; the MCU control loop ticks at 500Hz internally
# but only forwards a Damiao MIT frame per joint every DM_MIT_APPLY_DIV=4th
# tick (App/Src/plant/plugins/damiao.c) — i.e. ~125Hz is the actual per-joint
# CAN ceiling regardless of host rate. 125Hz matches that ceiling (i2rt's
# 250Hz doesn't directly translate: their CAN thread IS the 250Hz clock, ours
# is downstream of a fixed-rate MCU tick). Loop uses a flat time.sleep(dt_nom)
# (not an adaptive sleep), so this is a best-effort target, not a hard
# guarantee — if per-iteration work exceeds dt_nom the achieved rate is
# lower, never higher/unstable. Bench-check CDC link stability before relying
# on the full 125Hz for anything time-critical. See docs/i2rt-vs-ours-arm-compare.md P3.
STREAM_HZ = 125.0
J2 = 1
J2_ESC = 0x02
ARM_KP = tuple(float(x) for x in DEFAULT_ARM_KP)  # (40, 60, 90, 60, 25, 25, 20)
ARM_KD = tuple(float(x) for x in DEFAULT_ARM_KD)  # per-joint, was a flat 1.0
# Normal continuous J2 CLEAR cruise (rad/s). Bring-up stays soft separately.
CRUISE_UP = 0.18
CRUISE_DOWN = 0.12
ENGAGE_S = 2.4
MAX_CMD_LEAD = 0.40  # allow deeper CLEAR chase; never snap past FB at leg end
# Progressive latch uses soft hold gains; full teleop kp only after all-7 green.
LATCH_KP_SCALE = 0.35
LATCH_RAMP_S = 1.6
LATCH_HOLD_S = 1.2

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
DM_BASE_KP = 10.0
DM_BASE_KD = 0.5
BASE_AMP = 0.60  # unused in continuous-spin mode (kept for CLI compat)
# Unidirectional continuous spin (tiny_teleop cruise) — no triangle reverses.
BASE_RATE = math.pi / 4.0  # rad/s
RS_P_MIN = -12.57
RS_P_MAX = 12.57
RS_MARGIN = 0.35
BASE_LEAD = 0.45  # max cmd lead over FB (bus lag); never reverses integrator
# Damiao has no ±12.57 MIT rail — use a soft travel window and bounce so it
# keeps moving (RS stay unidirectional until the real MIT rail).
DM_BASE_TRAVEL = 2.0 * math.pi  # ±one turn from seed center
DM_BASE_MIN_KP = 2.0  # seed floor so MCP enable-latch can fire (kp=0 ⇒ no TX)

# Operator clear (smoke suite), ticks
DXL_IDS = (NECK_PITCH_DXL_ID, NECK_YAW_DXL_ID)
DXL_SLOTS = (NECK_PITCH_SERVO_SLOT, NECK_YAW_SERVO_SLOT)
DXL_LO = (2153 + 40, 871 + 40)
DXL_HI = (3058 - 40, 2622 - 40)
DXL_CRUISE_TICK_S = 280.0


def _blank() -> Dict[int, ActuatorDesire]:
    return {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}


def _gains_base(proto: int) -> Tuple[float, float]:
    if proto == PROTO_DAMIAO:
        return DM_BASE_KP, DM_BASE_KD
    return RS_KP, RS_KD


def _rail_clip(q: float, *, proto: int) -> float:
    if proto != PROTO_ROBSTRIDE:
        return float(q)
    return float(np.clip(q, RS_P_MIN + RS_MARGIN, RS_P_MAX - RS_MARGIN))


def _amp_for_center(center: float, amp: float, *, proto: int) -> float:
    if proto != PROTO_ROBSTRIDE:
        return float(amp)
    lo = RS_P_MIN + RS_MARGIN
    hi = RS_P_MAX - RS_MARGIN
    room = min(center - lo, hi - center)
    return float(max(0.0, min(amp, room)))


def _lead_clamp(cmd: float, fb: float, lead: float = MAX_CMD_LEAD) -> float:
    d = cmd - fb
    if abs(d) > lead:
        return fb + math.copysign(lead, d)
    return cmd


def _write_plant(
    session: PcbRobotSession,
    arm_q: np.ndarray,
    *,
    arm_dq: Optional[np.ndarray] = None,
    arm_kp_scale: float = 1.0,
    j2_kp_scale: Optional[float] = None,
    arm_kd: Sequence[float] = ARM_KD,
    gravity_comp: Optional[GravityComp] = None,
    base_cmd: Optional[Dict[int, float]] = None,
    base_vel: Optional[Dict[int, float]] = None,
    base_proto: Optional[Dict[int, int]] = None,
    base_gain_scale: float = 1.0,
    led: Optional[LedDesire] = None,
) -> None:
    """Full wire image: brace all 7 arm slots + optional base. Never blank siblings."""
    q = np.asarray(arm_q, dtype=np.float32).reshape(7)
    dq = (
        np.zeros(7, dtype=np.float32)
        if arm_dq is None
        else np.asarray(arm_dq, dtype=np.float32).reshape(7)
    )
    scale = float(np.clip(arm_kp_scale, 0.0, 1.0))
    d = _blank()
    # Static gravity feedforward, arm joints only (default None == torque=0.0,
    # unchanged from before this existed). Gripper/J7 (index 6) always 0.
    arm_torque = np.zeros(7, dtype=np.float32)
    if gravity_comp is not None:
        arm_torque[:6] = gravity_comp.compute(q[:6])
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        s = scale
        if i == J2 and j2_kp_scale is not None:
            s = float(j2_kp_scale)  # may be >1 for mouse lift authority
        d[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(dq[i]),
            kp=float(ARM_KP[i]) * s,
            kd=float(arm_kd[i]),
            torque=float(arm_torque[i]),
        )
    bscale = float(base_gain_scale)
    if base_cmd and base_proto:
        for slot, pos in base_cmd.items():
            proto = int(base_proto[slot])
            kp, kd = _gains_base(proto)
            kp_out = float(kp) * bscale
            kd_out = float(kd) * bscale
            # MCP Damiao: kp=0 never latches enable (firmware skips TX). Keep a
            # small floor while seeding so clear/enable + MIT can start.
            if proto == PROTO_DAMIAO and kp_out < DM_BASE_MIN_KP:
                kp_out = DM_BASE_MIN_KP
                kd_out = max(kd_out, DM_BASE_KD * 0.25)
            d[slot] = ActuatorDesire(
                position=float(pos),
                velocity=float((base_vel or {}).get(slot, 0.0)),
                kp=kp_out,
                kd=kd_out,
                torque=0.0,
            )
    session.set_actuators(d, send=False)
    if led is not None:
        session.set_led(led, send=False)


def _write_dxl(session: PcbRobotSession, cmd: Sequence[float], *, torque: bool = True) -> None:
    """Write DXL goals. Clamps to firmware table only (not operator clear)."""
    table = ((1024, 3072), (700, 2500))  # plant_config.c servo_table
    for i, sid in enumerate(DXL_IDS):
        lo, hi = table[i]
        goal = int(max(lo, min(hi, round(cmd[i]))))
        session.set_servo(
            DXL_SLOTS[i],
            ServoDesire(
                servo_id=sid,
                native_step_position=goal if torque else 0,
                torque_enable=bool(torque),
                operating_mode=3,
            ),
            send=False,
        )


def _clear_dxl(session: PcbRobotSession) -> None:
    try:
        session.hub._connection.clear_servos(send=False)  # noqa: SLF001
    except Exception:
        for slot in DXL_SLOTS:
            session.set_servo(slot, ServoDesire(servo_id=0), send=False)


def _kick_fdcan1(hub) -> None:
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    conn = hub._connection  # noqa: SLF001
    for kind in (SESSION_BEGIN, SESSION_END):
        conn.exchange_raw(
            build_rs2_scan_command(0, kind, conn.next_seq(), bus=1),
            parse_probe_pdu,
            timeout_s=3.0,
            predicate=lambda p, k=kind: p.get("probe_kind") == k,
        )


def _cfg_arm_slots(hub, enabled: set) -> None:
    """Enable only the given CH1 Damiao slots (progressive latch helper)."""
    with pause_plant_stream(hub):
        for i in range(7):
            hub.debug.cfg_set_slot(
                slot=i,
                bus=1,
                protocol=PROTO_DAMIAO,
                motor_id=0x01 + i,
                master_id=_DAMIAO_MASTER[i],
                enabled=(i in enabled),
                persist=False,
            )


def _cfg_base_on(hub) -> None:
    with pause_plant_stream(hub):
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
                f"  CFG base slot{slot} {label} bus={bus} id=0x{mid:02X}",
                flush=True,
            )


def _read_arm(session: PcbRobotSession) -> Optional[np.ndarray]:
    fb = session.latest_feedback()
    if fb is None:
        return None
    q = np.zeros(7, dtype=np.float32)
    any_live = False
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        st = fb.actuator(slot)
        if st is None:
            continue
        q[i] = float(st.position)
        if abs(q[i]) > 1e-3:
            any_live = True
    return q if any_live else None


def _arm_faults(session: PcbRobotSession) -> List[int]:
    fb = session.latest_feedback()
    out = [-1] * 7
    if fb is None:
        return out
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        st = fb.actuator(slot)
        if st is not None:
            out[i] = int(st.fault) & 0xFF
    return out


def _read_arm_detail(session: PcbRobotSession, slot: int):
    fb = session.latest_feedback()
    if fb is None or fb.actuator(slot) is None:
        return None
    st = fb.actuator(slot)
    return float(st.position), float(st.velocity), float(st.torque), int(st.fault) & 0xFF


def _read_base(session: PcbRobotSession) -> Dict[int, float]:
    out: Dict[int, float] = {}
    fb = session.latest_feedback()
    if fb is None:
        return out
    for slot in BASE_SLOTS:
        st = fb.actuator(slot)
        if st is None:
            continue
        # After RS encoder cali, mechPos is legitimately ~0 — do not drop it.
        out[slot] = float(st.position)
    return out


def _read_dxl_fb(session: PcbRobotSession) -> List[Optional[int]]:
    fb = session.latest_feedback()
    out: List[Optional[int]] = [None, None]
    if fb is None:
        return out
    for i, slot in enumerate(DXL_SLOTS):
        sv = parse_servo_feedback(fb.raw, slot)
        if sv is None:
            continue
        pos = int(sv["present_position"]) & 0xFFFF
        if pos > 4095:
            pos &= 0x0FFF
        mid = int(sv.get("motor_source_id", 0) or 0)
        if mid in (0, DXL_IDS[i]) or pos != 0:
            out[i] = pos
    return out


def _conn(hub):
    return hub._connection  # noqa: SLF001


def _rs_ids_on_bus(bus: int) -> List[int]:
    return [
        mid
        for _s, b, proto, mid, _m, _lab in BASE_ROWS
        if b == bus and proto == PROTO_ROBSTRIDE
    ]


def _kick_mcp_buses(hub) -> None:
    """RS2 SESSION_BEGIN/END on bus 5/6 — wake MCP rings after idle."""
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


def _rs_reset_id(hub, *, bus: int, motor_id: int) -> None:
    """Put one RS drive to rest so a daisy-chained sibling can arm."""
    conn = _conn(hub)
    timeout_s = 2.0 if is_mcp_bus(bus) else 0.55
    with lease(conn, hub.telemetry, bus=bus):
        conn.reader.drain()
        frame = build_rs2_probe_command(
            motor_id, PROBE_RESET, conn.next_seq(), bus=bus
        )
        try:
            conn.exchange_raw(
                frame,
                parse_probe_pdu,
                timeout_s=timeout_s,
                predicate=lambda p: p.get("probe_kind") == PROBE_RESET
                and p.get("probe_id") == (motor_id & 0xFF),
            )
        except Exception as exc:
            print(f"  reset 0x{motor_id:02X}: {exc}", flush=True)


def _probe_base(hub, *, discover_damiao: bool = False) -> Dict[int, float]:
    """Kick MCP, rest all RS once, then enable each (multi-motor safe).

    Daisy-chained bus5 (0x70+0x74): a leftover MIT-enabled sibling blocks the
    peer. Reset *all* RS first, then probe/enable each — do not reset siblings
    between enables or earlier motors drop out of MIT.

    CH6 Damiao: trust BASE_ROWS CFG (0x06) by default. ``discover_damiao`` only
    when bringing up an unknown ESC — REG_SCAN floods the daisy.
    """
    found: Dict[int, float] = {}
    with pause_plant_stream(hub):
        _kick_mcp_buses(hub)
        seen_reset: set = set()
        for _slot, bus, proto, mid, _master, _label in BASE_ROWS:
            if proto != PROTO_ROBSTRIDE:
                continue
            key = (bus, mid & 0xFF)
            if key in seen_reset:
                continue
            print(f"  rest bus{bus} id=0x{mid:02X}", flush=True)
            _rs_reset_id(hub, bus=bus, motor_id=mid)
            seen_reset.add(key)
            time.sleep(0.05)
        for slot, bus, proto, mid, _master, label in BASE_ROWS:
            if proto != PROTO_ROBSTRIDE:
                continue
            # Bus already reset-swept above. Enable-only — another PROBE_RESET
            # here knocks the daisy sibling (CH5 0x70/0x74) and often leaves
            # the second ID unarmed (fb frozen, cmd integrator still walks).
            resp = None
            attempts = 3 if is_mcp_bus(bus) else 2
            for attempt in range(1, attempts + 1):
                try:
                    resp = hub.debug.probe_robstride(
                        bus=bus,
                        motor_id=mid,
                        timeout_s=2.0 if is_mcp_bus(bus) else 0.55,
                        reset=False,
                    )
                except Exception as exc:
                    print(f"  probe {label} try={attempt}: {exc}", flush=True)
                    resp = None
                if resp and resp.get("found"):
                    break
                time.sleep(0.12)
            if resp and resp.get("found"):
                q = float(resp["position"])
                if abs(q) < 1e-6:
                    q = 1e-6
                found[slot] = q
                print(f"  probe {label} q={q:+.4f}", flush=True)
            else:
                print(f"  probe miss {label}", flush=True)
        if discover_damiao:
            try:
                dm_id = hub.debug.discover_damiao(
                    bus=6, start=1, end=16, listen_ms=80
                )
            except Exception as exc:
                print(f"  CH6 Damiao discover: {exc}", flush=True)
                dm_id = None
            if dm_id is not None:
                print(f"  CH6 Damiao discover id=0x{int(dm_id):02X}", flush=True)
                mid = int(dm_id)
                master = (mid + 0x10) & 0xFF
                hub.debug.cfg_set_slot(
                    slot=25,
                    bus=6,
                    protocol=PROTO_DAMIAO,
                    motor_id=mid,
                    master_id=master,
                    enabled=True,
                    persist=False,
                )
                # Do NOT invent a fake probe pose — Damiao seed must use plant FB
                # after CFG (discover only proves the ESC is on the bus).
            else:
                print("  CH6 Damiao discover miss", flush=True)
        else:
            print(
                "  CH6 Damiao: skip discover; trust CFG id=0x06 (use --discover-base)",
                flush=True,
            )
    hub.set_mcu_state(McuState.NORMAL, send=False)
    return found


def _plant_gate_line(hub) -> str:
    """mcu_state / plant_block from latest plant FB (ESTOP respect checks)."""
    raw = _conn(hub)._latest_fb_raw  # noqa: SLF001
    if raw is None:
        try:
            hub.send_once()
        except Exception:
            pass
        raw = _conn(hub)._latest_fb_raw  # noqa: SLF001
    if raw is None:
        return "mcu=? plant_block=?"
    try:
        fb = FeedbackImage(raw)
        mcu = int(fb.mcu_state)
        pb = int(fb.plant_block)
        try:
            pb_name = PlantBlockReason(pb).name
        except Exception:
            pb_name = str(pb)
        mcu_name = {0: "NORMAL", 1: "RECOVERY", 2: "DIAG_ONLY", 3: "ESTOP"}.get(
            mcu, str(mcu)
        )
        return f"mcu={mcu}({mcu_name}) plant_block={pb}({pb_name})"
    except Exception:
        return "mcu=? plant_block=?"


def _dashboard_soft_kill_requested(hub) -> bool:
    """True once if debug_dashboard wrote ``soft_kill_request`` (follow mode)."""
    flag = Path(hub.state_path).parent / "soft_kill_request"
    if not flag.is_file():
        return False
    try:
        flag.unlink()
    except Exception:
        pass
    return True


def _estop_respect_check(
    session: PcbRobotSession,
    *,
    hold_s: float = 2.5,
) -> Tuple[bool, str]:
    """Assert host ESTOP and verify MCU latches + base motion freezes."""
    hub = session.hub
    before = _read_base(session)
    print(
        f"\n== ESTOP respect: host→ESTOP (pre { _plant_gate_line(hub) }) ==",
        flush=True,
    )
    print(f"  base before: {before}", flush=True)
    hub.set_mcu_state(McuState.ESTOP, send=True)
    # Keep streaming so FB/plant_block update; desires cleared by set_mcu_state.
    t_end = time.perf_counter() + hold_s
    samples: List[Dict[int, float]] = []
    gate_hits = 0
    while time.perf_counter() < t_end:
        try:
            session.send_once()
        except Exception:
            pass
        time.sleep(0.05)
        samples.append(_read_base(session))
        line = _plant_gate_line(hub)
        if "ESTOP" in line or "plant_block=" in line:
            # Count samples that show ESTOP on mcu readback.
            if "mcu=3" in line or "ESTOP" in line:
                gate_hits += 1
        if len(samples) % 10 == 0:
            print(f"  … {line} base={samples[-1]}", flush=True)

    after = samples[-1] if samples else {}
    gate = _plant_gate_line(hub)
    # Freeze: each known slot moved < 0.15 rad while ESTOP held.
    moved = []
    for s, q0 in before.items():
        q1 = after.get(s)
        if q1 is not None:
            moved.append(abs(float(q1) - float(q0)))
    max_move = max(moved) if moved else 0.0
    mcu_ok = gate_hits >= 3 or "mcu=3" in gate or "ESTOP" in gate
    freeze_ok = max_move < 0.20
    ok = mcu_ok and freeze_ok
    msg = (
        f"{'PASS' if ok else 'FAIL'} {gate} max_base_move={max_move:.3f} "
        f"estop_samples={gate_hits}/{len(samples)}"
    )
    print(f"  {msg}", flush=True)
    print(f"  base after: {after}", flush=True)
    return ok, msg


def _cleanup(session: PcbRobotSession) -> None:
    print("cleanup: blank arm/base + clear DXL + DIAG...", flush=True)
    try:
        hub = session.hub
        hub.recover()
        time.sleep(0.1)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        _clear_dxl(session)
        session.set_actuators(_blank(), send=False)
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
            send=False,
        )
        for _ in range(10):
            session.send_once()
            time.sleep(0.05)
    except Exception as exc:
        print(f"cleanup warning: {exc}", flush=True)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--stream-hz", type=float, default=STREAM_HZ)
    ap.add_argument("--cruise-up", type=float, default=CRUISE_UP)
    ap.add_argument("--cruise-down", type=float, default=CRUISE_DOWN)
    ap.add_argument("--engage-s", type=float, default=ENGAGE_S)
    ap.add_argument("--status-s", type=float, default=2.0)
    ap.add_argument("--base-amp", type=float, default=BASE_AMP)
    ap.add_argument(
        "--base-rate",
        type=float,
        default=BASE_RATE,
        help="Base MCP triangle slew rate rad/s (default π/4)",
    )
    ap.add_argument(
        "--base-omega",
        type=float,
        default=None,
        help=argparse.SUPPRESS,  # legacy alias → base-rate
    )
    ap.add_argument("--dxl-cruise", type=float, default=DXL_CRUISE_TICK_S)
    ap.add_argument("--no-base", action="store_true")
    ap.add_argument("--no-dxl", action="store_true")
    ap.add_argument(
        "--mouse",
        action="store_true",
        help=(
            "Cruise with Logitech M650 mouse teleop instead of J2 CLEAR bounce "
            "(same mapping as mouse_arm_teleop.py). Requires pynput. Base/DXL "
            "still cruise unless --no-base / --no-dxl."
        ),
    )
    ap.add_argument(
        "--discover",
        action="store_true",
        help=(
            "Opt-in CH1 Damiao ID_SWEEP before latch (default: skip — REG_SCAN "
            "floods the daisy; trust YAM CFG + plant FB like yam_arm_clear_range)."
        ),
    )
    ap.add_argument(
        "--discover-base",
        action="store_true",
        help="Opt-in CH6 Damiao discover during base probe (default: trust CFG 0x06).",
    )
    ap.add_argument(
        "--gravity-comp",
        action="store_true",
        help=(
            "Enable MuJoCo static-gravity torque feedforward on the 6 arm "
            "joints during cruise (i2rt-style, opt-in — default OFF keeps "
            "torque=0.0, unchanged). Scale starts at 0.0 (no torque sent) "
            "until calibrated; requires `pip install mujoco`. Unvalidated "
            "on real hardware — see docs/i2rt-vs-ours-arm-compare.md P2."
        ),
    )
    ap.add_argument(
        "--record",
        action="store_true",
        help="NDJSON feedback recording under .deft_session/recordings/",
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after N seconds of cruise (0=until Ctrl-C)",
    )
    ap.add_argument(
        "--estop-after",
        type=float,
        default=0.0,
        help="After N seconds of cruise, assert host ESTOP and verify MCU latch",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    gravity_comp: Optional[GravityComp] = None
    if args.gravity_comp:
        gravity_comp = GravityComp()
        print(
            "gravity-comp: ENABLED, scale starts at 0.0 (no torque sent yet) — "
            "call GravityComp.calibrate() or set .scale from a real bench pass",
            flush=True,
        )

    port = args.port or find_cdc_port()
    j2_lo, j2_hi = float(CLEAR_LO[J2]), float(CLEAR_HI[J2])
    with_base = not bool(args.no_base)
    with_dxl = not bool(args.no_dxl)
    use_mouse = bool(args.mouse)
    base_rate = float(args.base_rate if args.base_omega is None else args.base_omega)
    cruise_mode = "mouse" if use_mouse else "J2-CLEAR"
    print(
        f"yam_continuous_all cruise={cruise_mode} "
        f"J2 CLEAR [{j2_lo:+.3f}, {j2_hi:+.3f}] "
        f"kp={ARM_KP} kd={ARM_KD} base={'ON rate='+f'{base_rate:.3f}' if with_base else 'OFF'} "
        f"dxl={'ON' if with_dxl else 'OFF'} "
        f"discover={'ON' if args.discover else 'OFF'} "
        f"discover-base={'ON' if args.discover_base else 'OFF'}",
        flush=True,
    )

    if use_mouse:
        try:
            from mouse_arm_teleop import (  # noqa: WPS433
                ENGAGE_KP,
                J2_HOLD_KP_SCALE,
                J2_KP_SCALE,
                STICK_DEADZONE,
                STICK_RADIUS_PX,
                TELEOP_LED,
                MouseArmRuntime,
                _attach_double_right,
                _hard as _mouse_hard,
            )
            from mouse_teleop import MouseTeleopState  # noqa: WPS433
            from pynput import mouse as pynput_mouse  # noqa: WPS433
        except ImportError as exc:
            print(
                f"FAIL: --mouse needs pynput + mouse_arm_teleop ({exc})",
                file=sys.stderr,
            )
            return 2

    stop = {"flag": False}

    def _sig(_s, _f) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    record_path: Optional[Path] = None
    estop_ok: Optional[bool] = None
    with PcbRobotSession.connect(
        port,
        apply_yam_cfg=False,
        stream_hz=float(args.stream_hz),
        idle_first=True,
        persist_telemetry=True,
    ) as session:
        hub = session.hub
        hub.set_rx_sim_mask(0)
        print(f"telemetry -> {hub.state_path} (dashboard: do not Connect COM)", flush=True)
        if args.record:
            record_path = hub.telemetry.start_recording()
            print(f"recording -> {record_path}", flush=True)

        print("\n== KICK FDCAN1 ==", flush=True)
        try:
            _kick_fdcan1(hub)
            print("  kick ok", flush=True)
        except Exception as exc:
            print(f"  kick warn: {exc}", flush=True)
        if args.discover:
            print("  --discover: CH1 Damiao ID_SWEEP (opt-in)", flush=True)
            j2_ok = False
            for attempt in range(3):
                sweep = hub.debug.discover_damiao_all(
                    bus=1, start=1, end=7, listen_ms=80
                )
                j2 = hub.debug.discover_damiao_all(
                    bus=1, start=J2_ESC, end=J2_ESC, listen_ms=120
                )
                print(
                    f"  attempt {attempt+1} sweep={sweep} j2_probe={j2}",
                    flush=True,
                )
                if J2_ESC in set(int(x) for x in j2) or J2_ESC in set(
                    int(x) for x in sweep
                ):
                    j2_ok = True
                    break
            if not j2_ok:
                print("FAIL: J2 ESC 0x02 not on bus after --discover", flush=True)
                return 3
        else:
            print(
                "  skip Damiao discover (use --discover only if needed); "
                "trusting YAM CFG ESC 0x01..0x07 — verifying via plant FB next",
                flush=True,
            )

        # Progressive all-green: enable CH1 one-by-one with soft Goal=FB hold
        # (not a snap-home). Full teleop kp comes later in soft-engage.
        print("\n== PROGRESSIVE ARM LATCH (soft Goal=FB) ==", flush=True)
        ensure_yam_left_arm_cfg(hub, force=True)
        _cfg_arm_slots(hub, set())  # all off first
        hub.set_mcu_state(McuState.NORMAL, send=True)

        q0 = np.zeros(7, dtype=np.float32)
        armed: set = set()

        def _seed_hold(secs: float) -> None:
            t_end = time.perf_counter() + secs
            while time.perf_counter() < t_end:
                fb = _read_arm(session)
                if fb is not None:
                    for s in armed:
                        if abs(float(fb[s])) > 1e-3:
                            q0[s] = float(fb[s])
                desires = _blank()
                for s in armed:
                    desires[s] = ActuatorDesire(
                        position=float(q0[s]), velocity=0.0, kp=0.0, kd=0.3
                    )
                session.set_actuators(desires, send=False)
                time.sleep(0.05)

        def _latch_armed(*, ramp_s: float, hold_s: float) -> bool:
            ok = False
            t0_latch = time.perf_counter()
            t_latch_end = t0_latch + ramp_s + hold_s
            while time.perf_counter() < t_latch_end:
                fb = _read_arm(session)
                if fb is not None:
                    for s in armed:
                        if abs(float(fb[s])) > 1e-3:
                            q0[s] = (
                                0.85 * float(q0[s]) + 0.15 * float(fb[s])
                                if abs(q0[s]) > 1e-3
                                else float(fb[s])
                            )
                u = (time.perf_counter() - t0_latch) / max(ramp_s, 1e-3)
                s_gain = min(1.0, max(0.0, u))
                s_gain = s_gain * s_gain * (3.0 - 2.0 * s_gain)
                desires = _blank()
                for s in armed:
                    desires[s] = ActuatorDesire(
                        position=float(q0[s]),
                        velocity=0.0,
                        kp=float(ARM_KP[s]) * LATCH_KP_SCALE * s_gain,
                        kd=float(ARM_KD[s]),
                    )
                session.set_actuators(desires, send=False)
                faults = _arm_faults(session)
                if (
                    time.perf_counter() >= t0_latch + ramp_s
                    and all(faults[s] == 1 for s in armed)
                ):
                    ok = True
                    break
                time.sleep(0.05)
            return ok

        def _recover_rearm() -> None:
            """Blank + hub.recover (disable/reset latches) then re-CFG armed set."""
            hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
            session.set_actuators(_blank(), send=False)
            time.sleep(0.15)
            try:
                hub.recover()
            except Exception as exc:
                print(f"  recover warn: {exc}", flush=True)
            time.sleep(0.25)
            _cfg_arm_slots(hub, armed)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            _seed_hold(0.6)

        for i in range(7):
            if stop["flag"]:
                break
            # J4 (index 3) often needs an extra enable latch after siblings are live.
            attempts = 3 if i == 3 else 2
            ok = False
            for attempt in range(attempts):
                armed.add(i)
                _cfg_arm_slots(hub, armed)
                hub.set_mcu_state(McuState.NORMAL, send=True)
                _seed_hold(0.8)
                ramp = LATCH_RAMP_S + (0.6 if i == 3 else 0.0)
                hold = LATCH_HOLD_S + (0.8 if i == 3 else 0.0)
                ok = _latch_armed(ramp_s=ramp, hold_s=hold)
                print(
                    f"  armed={sorted(armed)} faults={_arm_faults(session)} "
                    f"ok={ok} try={attempt + 1}/{attempts}",
                    flush=True,
                )
                if ok:
                    break
                print(f"  J{i + 1} not green — recover + re-arm", flush=True)
                _recover_rearm()
            if not ok:
                print(
                    f"WARN: J{i + 1} still not MIT-green after {attempts} tries",
                    flush=True,
                )

        # Final pass: re-latch any still-dark joints (esp. J4) before continuous.
        for pass_i in range(2):
            faults = _arm_faults(session)
            bad = [s for s in range(7) if faults[s] != 1]
            if not bad or stop["flag"]:
                break
            print(f"  final green pass {pass_i + 1}: retry {bad}", flush=True)
            armed = set(range(7))
            _recover_rearm()
            _latch_armed(
                ramp_s=LATCH_RAMP_S + 0.8,
                hold_s=LATCH_HOLD_S + 1.0,
            )

        # Final freeze at present — this is the continuous home, not zero.
        for _ in range(10):
            fb = _read_arm(session)
            if fb is not None:
                for s in range(7):
                    if abs(float(fb[s])) > 1e-3:
                        q0[s] = float(fb[s])
            desires = _blank()
            for s in range(7):
                desires[s] = ActuatorDesire(
                    position=float(q0[s]),
                    velocity=0.0,
                    kp=float(ARM_KP[s]) * LATCH_KP_SCALE,
                    kd=float(ARM_KD[s]),
                )
            session.set_actuators(desires, send=False)
            time.sleep(0.05)

        print(
            f"arm home(FB)={[float(f'{x:+.3f}') for x in q0]} "
            f"faults={_arm_faults(session)}",
            flush=True,
        )
        if not all(f == 1 for f in _arm_faults(session)):
            print("WARN: not all joints MIT-green; continuing with brace", flush=True)

        # ---- DXL present discover ----
        dxl_cmd: Optional[List[float]] = None
        dxl_dirs = [1.0, -1.0]
        if with_dxl:
            print("\n== DXL PRESENT ==", flush=True)
            last: List[Optional[int]] = [None, None]
            t_dxl = time.perf_counter() + 2.5
            while time.perf_counter() < t_dxl and not stop["flag"]:
                for i, sid in enumerate(DXL_IDS):
                    if last[i] is None:
                        session.set_servo(
                            DXL_SLOTS[i],
                            ServoDesire(
                                servo_id=sid,
                                native_step_position=0,
                                torque_enable=False,
                                operating_mode=3,
                            ),
                            send=False,
                        )
                    else:
                        session.set_servo(
                            DXL_SLOTS[i],
                            ServoDesire(
                                servo_id=sid,
                                native_step_position=int(last[i]),
                                torque_enable=True,
                                operating_mode=3,
                            ),
                            send=False,
                        )
                _write_plant(session, q0, arm_kp_scale=LATCH_KP_SCALE)
                got = _read_dxl_fb(session)
                for i, p in enumerate(got):
                    if p is not None:
                        last[i] = p
                if last[0] is not None and last[1] is not None:
                    break
                time.sleep(0.05)
            if last[0] is not None and last[1] is not None:
                dxl_cmd = [float(last[0]), float(last[1])]
                _write_dxl(session, dxl_cmd, torque=True)
                print(f"DXL present pitch={last[0]} yaw={last[1]}", flush=True)
            else:
                print(f"WARN: DXL present incomplete last={last} — DXL held off", flush=True)
                with_dxl = False

        base_proto = {r[0]: r[2] for r in BASE_ROWS}
        base_center: Dict[int, float] = {}
        base_cmd: Dict[int, float] = {}
        base_dirs: Dict[int, float] = {}
        probe_q: Dict[int, float] = {}
        if with_base:
            print("\n== BASE CFG + SOFT SEED (sibling-reset arm) ==", flush=True)
            # Keep arm at soft latch gains; CFG base without yanking mcu_state.
            _write_plant(session, q0, arm_kp_scale=LATCH_KP_SCALE)
            _cfg_base_on(hub)
            # Brief DIAG for MCP kick + RS enable/probe (arm desires soft-held).
            hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
            time.sleep(0.10)
            probe_q = _probe_base(hub, discover_damiao=bool(args.discover_base))
            hub.set_mcu_state(McuState.NORMAL, send=True)
            for slot, q in probe_q.items():
                # Post-cali RS sit near 0 — still a valid live center.
                base_center[slot] = float(q)
                base_cmd[slot] = float(q)
                base_dirs[slot] = 1.0 if ((slot - 22) % 2 == 0) else -1.0
            # Damiao: no RS probe — warm plant FB after CFG before seeding.
            t_dm = time.perf_counter() + 0.6
            while time.perf_counter() < t_dm:
                seed_dm = {
                    s: (
                        float(base_center[s])
                        if s in base_center and abs(base_center[s]) > 1e-6
                        else 1e-6
                    )
                    for s in BASE_SLOTS
                }
                # Hold RS idle; give Damiao min kp so enable latch can fire.
                _write_plant(
                    session,
                    q0,
                    arm_kp_scale=LATCH_KP_SCALE,
                    base_cmd=seed_dm,
                    base_proto=base_proto,
                    base_gain_scale=0.0,
                )
                live0 = _read_base(session)
                if 25 in live0:
                    qdm = float(live0[25])
                    if abs(qdm) < 1e-6:
                        qdm = 1e-6
                    base_center[25] = qdm
                    base_cmd[25] = qdm
                    base_dirs.setdefault(25, 1.0)
                time.sleep(0.05)
            if 25 in base_center:
                print(f"  Damiao plant seed s25={base_center[25]:+.4f}", flush=True)
            else:
                print("  WARN: no plant FB for CH6 Damiao yet", flush=True)
            t_base = time.perf_counter() + 2.0
            while time.perf_counter() < t_base:
                fb = _read_arm(session)
                if fb is not None:
                    for s in range(7):
                        if abs(float(fb[s])) > 1e-3:
                            q0[s] = float(fb[s])
                seed = {
                    # HOME_POS_EPS: true 0 + idle gains skips MCP SPI.
                    s: (
                        float(base_center[s])
                        if s in base_center and abs(base_center[s]) > 1e-6
                        else (float(base_center[s]) if s in base_center else 0.0) + 1e-6
                    )
                    for s in BASE_SLOTS
                }
                _write_plant(
                    session,
                    q0,
                    arm_kp_scale=LATCH_KP_SCALE,
                    base_cmd=seed,
                    base_proto=base_proto,
                    base_gain_scale=0.0,  # RS idle; Damiao gets DM_BASE_MIN_KP floor
                )
                if with_dxl and dxl_cmd is not None:
                    _write_dxl(session, dxl_cmd, torque=True)
                live = _read_base(session)
                for slot in BASE_SLOTS:
                    plant = live.get(slot)
                    proto = base_proto[slot]
                    if proto == PROTO_DAMIAO:
                        # Plant FB only — never a fake discover placeholder.
                        if plant is None:
                            continue
                        resolved = float(plant)
                    else:
                        resolved = rs02_resolve_start(probe_q.get(slot), plant)
                    if resolved is None:
                        continue
                    if abs(resolved) < 1e-6:
                        resolved = 1e-6
                    if slot not in base_center:
                        base_dirs.setdefault(
                            slot, 1.0 if ((slot - 22) % 2 == 0) else -1.0
                        )
                    base_center[slot] = float(resolved)
                    base_cmd[slot] = float(resolved)
                time.sleep(0.05)
            print(
                "base centers: "
                + (
                    " ".join(f"s{s}={base_center[s]:+.3f}" for s in sorted(base_center))
                    if base_center
                    else "(none)"
                ),
                flush=True,
            )

        print(
            f"soft-engage arm+base {args.engage_s:.1f}s "
            f"(arm {LATCH_KP_SCALE:.2f}->1.0, base 0->1)...",
            flush=True,
        )
        t0 = time.perf_counter()
        while True:
            u = (time.perf_counter() - t0) / max(float(args.engage_s), 1e-3)
            if u >= 1.0:
                break
            s = u * u * (3.0 - 2.0 * u)
            fb = _read_arm(session)
            if fb is not None:
                for i in range(7):
                    if abs(float(fb[i])) > 1e-3:
                        q0[i] = 0.9 * float(q0[i]) + 0.1 * float(fb[i])
            arm_scale = LATCH_KP_SCALE + (1.0 - LATCH_KP_SCALE) * s
            _write_plant(
                session,
                q0,
                arm_kp_scale=arm_scale,
                base_cmd=dict(base_cmd) if base_cmd else None,
                base_proto=base_proto if base_cmd else None,
                base_gain_scale=s,
            )
            if with_dxl and dxl_cmd is not None:
                _write_dxl(session, dxl_cmd, torque=True)
            time.sleep(0.05)
        _write_plant(
            session,
            q0,
            arm_kp_scale=1.0,
            base_cmd=dict(base_cmd) if base_cmd else None,
            base_proto=base_proto if base_cmd else None,
            base_gain_scale=1.0,
        )
        if with_dxl and dxl_cmd is not None:
            _write_dxl(session, dxl_cmd, torque=True)
        print(f"ready faults={_arm_faults(session)} (1=MIT green)", flush=True)

        arm_cmd = q0.copy()
        fb0 = _read_arm(session)
        if fb0 is not None:
            arm_cmd = fb0.copy()
        arm_cmd[J2] = float(np.clip(float(arm_cmd[J2]), j2_lo, j2_hi))
        last_status = time.perf_counter()
        dt_nom = 1.0 / max(float(args.stream_hz), 1.0)

        def _bw_pdb_line() -> str:
            """USB stream health + PDU kill mirror (estop_sense / COMMS_LOSS)."""
            st = hub.telemetry.snapshot()
            fb_hz = st.fb_hz
            tx_hz = st.stream_tx_hz
            ack = st.stream_ack_lag
            gap = st.stream_tx_gap_p95_ms
            parts = [
                f"fb={fb_hz:.0f}" if fb_hz is not None else "fb=?",
                f"tx={tx_hz:.0f}" if tx_hz is not None else "tx=?",
                f"ack={ack}" if ack is not None else "ack=?",
                f"gap95={gap:.1f}ms" if gap is not None else "gap95=?",
            ]
            pdb = hub.pdb_status()
            if pdb is None:
                parts.append("pdb=?")
            else:
                tag = pdb.kill_state_name
                if pdb.stale_failsafe:
                    tag += "/COMMS_LOSS"
                parts.append(
                    f"pdb={tag} estop_gpio={pdb.estop_sense} "
                    f"reason={pdb.kill_reason_name}"
                )
            return " ".join(parts)

        base_vel: Dict[int, float] = {}
        base_integ: Dict[int, float] = {
            s: float(base_cmd.get(s, base_center.get(s, 0.0))) for s in base_center
        }

        def _spin_sign_for(slot: int, q: float, proto: int) -> float:
            """Pick initial sign with more travel room from live FB."""
            if proto == PROTO_ROBSTRIDE:
                lo, hi = RS_P_MIN + RS_MARGIN, RS_P_MAX - RS_MARGIN
            else:
                c0 = float(base_center.get(slot, q))
                lo, hi = c0 - DM_BASE_TRAVEL, c0 + DM_BASE_TRAVEL
            return 1.0 if (hi - q) >= (q - lo) else -1.0

        # Initial signs from room; reverse only when the *integrator* hits a
        # rail (not FB-lag mid-travel — that was the old triangle flip-flop).
        for slot in list(base_center):
            q0b = float(base_integ.get(slot, base_center[slot]))
            base_dirs[slot] = _spin_sign_for(slot, q0b, base_proto[slot])

        def _tick_base_and_dxl() -> Dict[int, float]:
            """Base spin + DXL. Bounce at RS MIT rails / Damiao ±2π window."""
            live = _read_base(session)
            for slot, q in live.items():
                if slot not in base_integ:
                    base_integ[slot] = q
                    base_cmd[slot] = q
                    base_center[slot] = q
                    base_dirs[slot] = _spin_sign_for(slot, q, base_proto[slot])
                    print(f"  base slot{slot} live FB={q:+.3f} (spin dir={base_dirs[slot]:+.0f})", flush=True)
            tick_base: Dict[int, float] = {}
            for slot, integ in list(base_integ.items()):
                proto = base_proto[slot]
                sign = float(base_dirs.get(slot, 1.0))
                if proto == PROTO_ROBSTRIDE:
                    lo, hi = RS_P_MIN + RS_MARGIN, RS_P_MAX - RS_MARGIN
                else:
                    c0 = float(base_center.get(slot, integ))
                    lo, hi = c0 - DM_BASE_TRAVEL, c0 + DM_BASE_TRAVEL

                cmd = float(integ) + sign * base_rate * dt_nom
                fb_q = live.get(slot)
                at_rail = False
                if sign > 0 and cmd >= hi:
                    cmd = hi
                    at_rail = True
                elif sign < 0 and cmd <= lo:
                    cmd = lo
                    at_rail = True

                # Reverse at rail so RS don't soft-hold forever after first leg.
                if at_rail:
                    sign = -sign
                    base_dirs[slot] = sign
                    cmd = float(integ) + sign * base_rate * dt_nom
                    cmd = float(min(hi, max(lo, cmd)))
                    at_rail = False

                # Integrator always tracks planned cmd (not lead-clamped desire).
                base_integ[slot] = cmd
                desire = _rail_clip(cmd, proto=proto) if proto == PROTO_ROBSTRIDE else float(cmd)
                vel = sign * base_rate
                if fb_q is not None:
                    err = desire - fb_q
                    # Cap lead in travel direction only — never pull opposite.
                    if abs(err) > BASE_LEAD:
                        if err * sign > 0:
                            desire = fb_q + sign * BASE_LEAD
                        else:
                            desire = fb_q
                        desire = (
                            _rail_clip(desire, proto=proto)
                            if proto == PROTO_ROBSTRIDE
                            else float(desire)
                        )
                    if abs(desire - fb_q) > BASE_LEAD * 0.85:
                        vel = sign * base_rate * 0.5
                if abs(desire) < 1e-6:
                    desire = 1e-6
                base_cmd[slot] = desire
                base_vel[slot] = vel
                tick_base[slot] = desire
            if with_dxl and dxl_cmd is not None:
                for i in range(2):
                    step = dxl_dirs[i] * float(args.dxl_cruise) * dt_nom
                    if dxl_cmd[i] < DXL_LO[i]:
                        dxl_dirs[i] = 1.0
                        step = abs(step)
                    elif dxl_cmd[i] > DXL_HI[i]:
                        dxl_dirs[i] = -1.0
                        step = -abs(step)
                    nxt = dxl_cmd[i] + step
                    if nxt >= DXL_HI[i]:
                        nxt = float(DXL_HI[i])
                        dxl_dirs[i] = -1.0
                    elif nxt <= DXL_LO[i] and dxl_cmd[i] >= DXL_LO[i]:
                        nxt = float(DXL_LO[i])
                        dxl_dirs[i] = 1.0
                    dxl_cmd[i] = nxt
                _write_dxl(session, dxl_cmd, torque=True)
            return tick_base

        def _cruise_guards(now: float, t_cruise0: float) -> bool:
            """Soft-kill / duration / estop-after. True → break cruise."""
            nonlocal estop_ok
            if session.service_soft_kill():
                print("  SOFT_KILL_REQ → parked ESTOP", flush=True)
                stop["flag"] = True
                return True
            if _dashboard_soft_kill_requested(hub):
                print("  dashboard soft_kill_request → soft_kill_park()", flush=True)
                try:
                    hub.soft_kill_park(send=True)
                except Exception as exc:
                    print(f"  soft_kill_park failed: {exc}", flush=True)
                    hub.set_mcu_state(McuState.ESTOP, send=True)
                stop["flag"] = True
                return True
            cruise_s = now - t_cruise0
            if float(args.estop_after) > 0 and cruise_s >= float(args.estop_after):
                ok, _msg = _estop_respect_check(session, hold_s=2.5)
                estop_ok = ok
                stop["flag"] = True
                return True
            if float(args.duration) > 0 and cruise_s >= float(args.duration):
                print(f"  duration reached ({cruise_s:.1f}s) — stop", flush=True)
                stop["flag"] = True
                return True
            return False

        def _status_base_dxl(tick_base: Dict[int, float]) -> str:
            live = _read_base(session)
            btxt = " ".join(
                f"s{s}={tick_base.get(s, float('nan')):+.2f}/"
                f"{live.get(s, float('nan')):+.2f}"
                f"@{base_vel.get(s, 0):+.2f}"
                for s in BASE_SLOTS
                if s in tick_base or s in live
            )
            dtxt = ""
            if with_dxl and dxl_cmd is not None:
                dfb = _read_dxl_fb(session)
                dtxt = (
                    f" dxl={int(dxl_cmd[0])}/{dfb[0]}|"
                    f"{int(dxl_cmd[1])}/{dfb[1]}"
                )
            return f"{btxt}{dtxt} | {_bw_pdb_line()}"

        mouse_listener = None
        try:
            if use_mouse:
                print(
                    "\n== MOUSE TELEOP + DXL + base spin (Ctrl-C cleans) ==",
                    flush=True,
                )
                print(
                    "  MIDDLE=enable | L/R=J1 | U/D=J2 | +RIGHT=J3 | +LEFT=J4\n"
                    "  double-right=J7 | scroll=J5 | hold frozen on middle release\n"
                    f"  J2 kp x{J2_KP_SCALE} | stick_r={STICK_RADIUS_PX}px | "
                    f"gravity_comp={'ON' if gravity_comp else 'OFF'}",
                    flush=True,
                )
                print(
                    "  base: continuous spin (reverse at MIT/soft rail only) "
                    + " ".join(
                        f"s{s}={base_dirs.get(s, 0):+.0f}" for s in sorted(base_dirs)
                    ),
                    flush=True,
                )
                print(f"  stream/pdb: {_bw_pdb_line()}", flush=True)
                if float(args.duration) > 0:
                    print(f"  duration={float(args.duration):.1f}s", flush=True)

                mstate = MouseTeleopState(
                    mode="joystick",
                    stick_radius_px=STICK_RADIUS_PX,
                    stick_deadzone=STICK_DEADZONE,
                    z_scale=1.0,
                )
                _attach_double_right(mstate)
                mouse_listener = pynput_mouse.Listener(
                    on_move=mstate.on_move,
                    on_click=mstate.on_click,
                    on_scroll=mstate.on_scroll,
                )
                mouse_listener.start()
                rt = MouseArmRuntime(arm_cmd)
                try:
                    session.set_led(TELEOP_LED, send=True)
                except Exception as exc:
                    print(f"  led warn: {exc}", flush=True)

                t_cruise0 = time.perf_counter()
                n = 0
                while not stop["flag"] and mouse_listener.running:
                    now = time.perf_counter()
                    if _cruise_guards(now, t_cruise0):
                        break
                    fb_arm = _read_arm(session)
                    faults = _arm_faults(session)
                    if _mouse_hard(faults):
                        print(f"HARD fault {faults} — stop", flush=True)
                        stop["flag"] = True
                        break
                    rt.step(mstate, fb=fb_arm, dt=dt_nom)
                    tick_base = _tick_base_and_dxl()
                    # Boost J2 kp only while driving it — always-on ×1.4 with
                    # residual lead buzzes once the arm is deep in CLEAR.
                    j2_scale = (
                        float(J2_KP_SCALE)
                        if J2 in rt.active
                        else float(J2_HOLD_KP_SCALE)
                    )
                    _write_plant(
                        session,
                        rt.cmd,
                        arm_dq=rt.dq,
                        arm_kp_scale=float(ENGAGE_KP),
                        j2_kp_scale=j2_scale,
                        gravity_comp=gravity_comp,
                        base_cmd=tick_base if tick_base else None,
                        base_vel=dict(base_vel) if tick_base else None,
                        base_proto=base_proto if tick_base else None,
                        base_gain_scale=1.0,
                        led=TELEOP_LED,
                    )
                    n += 1
                    if (now - last_status) >= float(args.status_s):
                        qshow = fb_arm if fb_arm is not None else rt.cmd
                        sx, sy = rt.stick
                        j2_err = float(rt.hold[J2] - qshow[J2])
                        print(
                            f"  mouse en={int(rt.deadman)} vert={rt.vert_mode} "
                            f"stick=({sx:+.2f},{sy:+.2f}) "
                            f"J2_hold/fb={rt.hold[J2]:+.3f}/{qshow[J2]:+.3f} "
                            f"J2_err={j2_err:+.3f} j2_kp×{j2_scale:.2f} "
                            f"faults={faults} | {_status_base_dxl(tick_base)}",
                            flush=True,
                        )
                        if args.record:
                            try:
                                hub.log_feedback()
                            except Exception:
                                pass
                        last_status = now
                    time.sleep(dt_nom)
            else:
                # Persistent direction bounce (no skip/flip-flop on unreachable CLEAR ends).
                # aim_lo/hi start at full CLEAR; only shrink when a leg is stuck.
                j2_dir = -1.0 if abs(float(arm_cmd[J2]) - j2_lo) >= abs(
                    float(arm_cmd[J2]) - j2_hi
                ) else 1.0
                aim_lo = float(j2_lo)
                aim_hi = float(j2_hi)
                last_reverse = 0.0
                reverse_anchor = float(arm_cmd[J2])
                ARRIVE_EPS = 0.10  # rad — FB near bound before reverse
                HYST = 0.15  # rad — min travel before arrive-reverse
                STUCK_S = 3.5
                MIN_REVERSE_S = 1.0  # debounce direction flips

                print(
                    "\n== J2 CLEAR + brace J1/J3-7 + DXL + base spin (Ctrl-C cleans) ==",
                    flush=True,
                )
                print(
                    f"  J2 dir={j2_dir:+.0f} start={float(arm_cmd[J2]):+.3f} "
                    f"CLEAR=[{j2_lo:+.3f},{j2_hi:+.3f}] (hysteresis bounce)",
                    flush=True,
                )
                print(
                    "  base: continuous spin (reverse at MIT/soft rail only) "
                    + " ".join(
                        f"s{s}={base_dirs.get(s, 0):+.0f}" for s in sorted(base_dirs)
                    ),
                    flush=True,
                )
                print(f"  stream/pdb: {_bw_pdb_line()}", flush=True)
                if float(args.duration) > 0:
                    print(f"  duration={float(args.duration):.1f}s", flush=True)
                if float(args.estop_after) > 0:
                    print(
                        f"  estop-after={float(args.estop_after):.1f}s "
                        f"(host ESTOP respect check)",
                        flush=True,
                    )

                t_leg = time.perf_counter()
                t_cruise0 = t_leg
                last_progress = t_leg
                last_fb_j2 = float(arm_cmd[J2])
                while not stop["flag"]:
                    now = time.perf_counter()
                    if _cruise_guards(now, t_cruise0):
                        break
                    fb_arm = _read_arm(session)
                    fb_j2 = (
                        float(fb_arm[J2]) if fb_arm is not None else float(arm_cmd[J2])
                    )

                    # Full CLEAR ends unless a prior stuck shrunk aim_lo/aim_hi.
                    target = aim_lo if j2_dir < 0 else aim_hi
                    cruise = (
                        float(args.cruise_down)
                        if j2_dir < 0
                        else float(args.cruise_up)
                    )

                    # Rate-limit toward target from cmd, glued near FB.
                    step = math.copysign(
                        min(abs(cruise) * dt_nom, abs(target - float(arm_cmd[J2]))),
                        target - float(arm_cmd[J2]),
                    )
                    j2_cmd = float(arm_cmd[J2]) + step
                    j2_cmd = float(np.clip(j2_cmd, j2_lo, j2_hi))
                    j2_cmd = _lead_clamp(j2_cmd, fb_j2)
                    j2_dq = float(step / max(dt_nom, 1e-3))

                    if fb_arm is not None:
                        for i in range(7):
                            if i == J2:
                                continue
                            arm_cmd[i] = (
                                0.98 * float(arm_cmd[i]) + 0.02 * float(fb_arm[i])
                            )
                    arm_cmd[J2] = j2_cmd
                    arm_dq = np.zeros(7, dtype=np.float32)
                    arm_dq[J2] = j2_dq

                    tick_base = _tick_base_and_dxl()
                    _write_plant(
                        session,
                        arm_cmd,
                        arm_dq=arm_dq,
                        arm_kp_scale=1.0,
                        gravity_comp=gravity_comp,
                        base_cmd=tick_base if tick_base else None,
                        base_vel=dict(base_vel) if tick_base else None,
                        base_proto=base_proto if tick_base else None,
                        base_gain_scale=1.0,
                    )

                    # Progress vs last sample (stuck detector). Ignore tiny FB noise.
                    if abs(fb_j2 - last_fb_j2) > 0.025:
                        last_progress = now
                        last_fb_j2 = fb_j2
                    traveled = abs(fb_j2 - reverse_anchor)
                    debounced = (
                        (now - last_reverse) >= MIN_REVERSE_S or last_reverse == 0.0
                    )
                    arrived = abs(fb_j2 - target) <= ARRIVE_EPS
                    stuck = (now - last_progress) >= STUCK_S and abs(
                        fb_j2 - target
                    ) > ARRIVE_EPS
                    # Arrive needs travel hysteresis; stuck may reverse without it.
                    do_arrive = arrived and debounced and (
                        traveled >= HYST or last_reverse == 0.0
                    )
                    do_stuck = stuck and debounced
                    if do_arrive or do_stuck:
                        if do_stuck:
                            if j2_dir < 0:
                                aim_lo = max(j2_lo, min(fb_j2, aim_hi - 0.25))
                            else:
                                aim_hi = min(j2_hi, max(fb_j2, aim_lo + 0.25))
                            print(
                                f"  J2 reverse stuck fb={fb_j2:+.3f} "
                                f"(wanted {target:+.3f}) aim=[{aim_lo:+.3f},{aim_hi:+.3f}]",
                                flush=True,
                            )
                        else:
                            print(
                                f"  J2 reverse arrived fb={fb_j2:+.3f} "
                                f"target={target:+.3f}",
                                flush=True,
                            )
                        j2_dir = -j2_dir
                        last_reverse = now
                        last_progress = now
                        reverse_anchor = fb_j2
                        last_fb_j2 = fb_j2
                        # Brief soft dwell at live pose (keep base/DXL moving).
                        dwell_end = now + 0.35
                        while time.perf_counter() < dwell_end and not stop["flag"]:
                            session.service_soft_kill()
                            fb_arm = _read_arm(session)
                            if fb_arm is not None:
                                for i in range(7):
                                    arm_cmd[i] = (
                                        0.9 * float(arm_cmd[i])
                                        + 0.1 * float(fb_arm[i])
                                    )
                            tick_base = _tick_base_and_dxl()
                            _write_plant(
                                session,
                                arm_cmd,
                                arm_kp_scale=1.0,
                                gravity_comp=gravity_comp,
                                base_cmd=tick_base if tick_base else None,
                                base_vel=dict(base_vel) if tick_base else None,
                                base_proto=base_proto if tick_base else None,
                            )
                            time.sleep(dt_nom)
                        last_fb_j2 = float(arm_cmd[J2])

                    if (now - last_status) >= float(args.status_s):
                        r = _read_arm_detail(session, J2)
                        faults = _arm_faults(session)
                        bw = _status_base_dxl(tick_base)
                        if r:
                            print(
                                f"  J2 dir={j2_dir:+.0f} cmd/fb={arm_cmd[J2]:+.3f}/"
                                f"{r[0]:+.3f} tau={r[2]:+.2f} faults={faults} | "
                                f"{bw}",
                                flush=True,
                            )
                            if (r[3] & 0xF) >= 8:
                                print("FAIL J2 hard fault — stop", flush=True)
                                stop["flag"] = True
                                break
                        else:
                            print(f"  status | {bw}", flush=True)
                        if args.record:
                            try:
                                hub.log_feedback()
                            except Exception:
                                pass
                        last_status = now
                    time.sleep(dt_nom)
        finally:
            if mouse_listener is not None:
                try:
                    mouse_listener.stop()
                except Exception:
                    pass
            if args.record:
                try:
                    hub.telemetry.stop_recording()
                    print(f"recording stopped: {record_path}", flush=True)
                except Exception as exc:
                    print(f"recording stop warning: {exc}", flush=True)
            _cleanup(session)

    print("done", flush=True)
    if estop_ok is False:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
