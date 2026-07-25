#!/usr/bin/env python3
"""Continuous YAM left arm + DXL neck + bus5/6 base (teleop / smoke pattern).

Arm (CH1 Damiao slots 0-6): **all-7 CFG enabled**, Goal=FB acquire, soft-engage
with teleop gains, J2 CLEAR bounce while J1/J3-J7 brace at FB. Solo-CFG J2
left wrists faulted (no clear/enable/MIT) — do not use that path.

DXL neck (host servo slots 0/1, IDs 1/2): torque-off discover → hold present →
gentle clear bounce (same as yam_rig_smoke_suite / yam_dxl_clear_teleop).

Base MCP 22-25: live-FB center + sine amp (proven RS path; no snap-to-0).

Telemetry: ``persist_telemetry=True`` → ``.deft_session/state.json``.
Dashboard: run without Connect COM to follow that file.

``killall -9`` does not stop CAN — Ctrl-C / ``_tmp_stop_can.py``.
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
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER  # noqa: E402
from deft_controls_sdk.link.exchange import (  # noqa: E402
    ACTUATOR_COUNT,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_scan_command,
    parse_probe_pdu,
    parse_servo_feedback,
)
from deft_controls_sdk.vbeta import PcbRobotSession  # noqa: E402
from deft_controls_sdk.vbeta.cfg import ensure_yam_left_arm_cfg, pause_plant_stream  # noqa: E402
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

STREAM_HZ = 20.0
J2 = 1
J2_ESC = 0x02
ARM_KP = tuple(float(x) for x in DEFAULT_ARM_KP)  # (40, 60, 90, 60, 25, 25, 20)
ARM_KD = float(DEFAULT_ARM_KD)
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
BASE_AMP = 0.60
# Constant-rate triangle about live center (RS teleop-style), not a fast sine.
BASE_RATE = math.pi / 4.0  # rad/s
RS_P_MIN = -12.57
RS_P_MAX = 12.57
RS_MARGIN = 0.35
BASE_LEAD = 0.55

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
    arm_kd: float = ARM_KD,
    base_cmd: Optional[Dict[int, float]] = None,
    base_proto: Optional[Dict[int, int]] = None,
    base_gain_scale: float = 1.0,
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
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(dq[i]),
            kp=float(ARM_KP[i]) * scale,
            kd=float(arm_kd),
            torque=0.0,
        )
    bscale = float(base_gain_scale)
    if base_cmd and base_proto:
        for slot, pos in base_cmd.items():
            proto = int(base_proto[slot])
            kp, kd = _gains_base(proto)
            d[slot] = ActuatorDesire(
                position=float(pos),
                velocity=0.0,
                kp=float(kp) * bscale,
                kd=float(kd) * bscale,
                torque=0.0,
            )
    session.set_actuators(d, send=False)


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


def _probe_base(hub) -> Dict[int, float]:
    found: Dict[int, float] = {}
    for slot, bus, proto, mid, _master, label in BASE_ROWS:
        if proto != PROTO_ROBSTRIDE:
            continue
        try:
            resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
        except Exception as exc:
            print(f"  probe {label}: {exc}", flush=True)
            continue
        if resp and resp.get("found"):
            q = float(resp["position"])
            found[slot] = q
            print(f"  probe {label} q={q:+.4f}", flush=True)
        else:
            print(f"  probe miss {label}", flush=True)
    try:
        dm_id = hub.debug.discover_damiao(bus=6, start=1, end=16, listen_ms=80)
    except Exception as exc:
        print(f"  CH6 Damiao discover: {exc}", flush=True)
        dm_id = None
    if dm_id is not None:
        print(f"  CH6 Damiao discover id=0x{int(dm_id):02X}", flush=True)
        mid = int(dm_id)
        master = (mid + 0x10) & 0xFF
        with pause_plant_stream(hub):
            hub.debug.cfg_set_slot(
                slot=25,
                bus=6,
                protocol=PROTO_DAMIAO,
                motor_id=mid,
                master_id=master,
                enabled=True,
                persist=False,
            )
        if 25 not in found:
            found[25] = 1e-6
    else:
        print("  CH6 Damiao discover miss", flush=True)
    return found


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
    args = ap.parse_args(list(argv) if argv is not None else None)

    port = args.port or find_cdc_port()
    j2_lo, j2_hi = float(CLEAR_LO[J2]), float(CLEAR_HI[J2])
    with_base = not bool(args.no_base)
    with_dxl = not bool(args.no_dxl)
    base_rate = float(args.base_rate if args.base_omega is None else args.base_omega)
    print(
        f"yam_continuous_all teleop-brace J2 CLEAR [{j2_lo:+.3f}, {j2_hi:+.3f}] "
        f"kp={ARM_KP} kd={ARM_KD} base={'ON rate='+f'{base_rate:.3f}' if with_base else 'OFF'} "
        f"dxl={'ON' if with_dxl else 'OFF'}",
        flush=True,
    )

    stop = {"flag": False}

    def _sig(_s, _f) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

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

        print("\n== KICK + DISCOVER (need ESC 0x02) ==", flush=True)
        j2_ok = False
        for attempt in range(5):
            _kick_fdcan1(hub)
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
            print("FAIL: J2 ESC 0x02 not on bus after kick/discover", flush=True)
            return 3

        # Progressive all-green: enable CH1 one-by-one with soft Goal=FB hold
        # (not a snap-home). Full teleop kp comes later in soft-engage.
        print("\n== PROGRESSIVE ARM LATCH (soft Goal=FB) ==", flush=True)
        ensure_yam_left_arm_cfg(hub, force=True)
        _cfg_arm_slots(hub, set())  # all off first
        hub.set_mcu_state(McuState.NORMAL, send=True)

        q0 = np.zeros(7, dtype=np.float32)
        armed: set = set()
        for i in range(7):
            if stop["flag"]:
                break
            armed.add(i)
            _cfg_arm_slots(hub, armed)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            # Idle-anchor at live FB only (kd light) — never dwell Goal=0.
            t_seed = time.perf_counter() + 0.8
            while time.perf_counter() < t_seed:
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
            ok = False
            t0_latch = time.perf_counter()
            t_latch_end = t0_latch + LATCH_RAMP_S + LATCH_HOLD_S
            while time.perf_counter() < t_latch_end:
                # Keep Goal glued to FB while gains come up (no chase/home).
                fb = _read_arm(session)
                if fb is not None:
                    for s in armed:
                        if abs(float(fb[s])) > 1e-3:
                            q0[s] = (
                                0.85 * float(q0[s]) + 0.15 * float(fb[s])
                                if abs(q0[s]) > 1e-3
                                else float(fb[s])
                            )
                u = (time.perf_counter() - t0_latch) / max(LATCH_RAMP_S, 1e-3)
                s_gain = min(1.0, max(0.0, u))
                s_gain = s_gain * s_gain * (3.0 - 2.0 * s_gain)
                desires = _blank()
                for s in armed:
                    desires[s] = ActuatorDesire(
                        position=float(q0[s]),
                        velocity=0.0,
                        kp=float(ARM_KP[s]) * LATCH_KP_SCALE * s_gain,
                        kd=ARM_KD,
                    )
                session.set_actuators(desires, send=False)
                faults = _arm_faults(session)
                if (
                    time.perf_counter() >= t0_latch + LATCH_RAMP_S
                    and all(faults[s] == 1 for s in armed)
                ):
                    ok = True
                    break
                time.sleep(0.05)
            print(
                f"  armed={sorted(armed)} faults={_arm_faults(session)} ok={ok}",
                flush=True,
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
                    kd=ARM_KD,
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
            print("\n== BASE CFG + SOFT SEED (no hard DIAG snap) ==", flush=True)
            # Keep arm at soft latch gains; CFG base without yanking mcu_state.
            _write_plant(session, q0, arm_kp_scale=LATCH_KP_SCALE)
            _cfg_base_on(hub)
            # Brief DIAG only for RS enable/probe (arm desires stay soft-held).
            hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
            time.sleep(0.15)
            probe_q = _probe_base(hub)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            for slot, q in probe_q.items():
                # Post-cali RS sit near 0 — still a valid live center.
                base_center[slot] = float(q)
                base_cmd[slot] = float(q)
                base_dirs[slot] = 1.0 if ((slot - 22) % 2 == 0) else -1.0
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
                    base_gain_scale=0.0,  # idle-anchored RS until soft-engage
                )
                if with_dxl and dxl_cmd is not None:
                    _write_dxl(session, dxl_cmd, torque=True)
                live = _read_base(session)
                for slot, q in live.items():
                    pq = probe_q.get(slot)
                    if slot not in base_center:
                        base_center[slot] = q
                        base_dirs.setdefault(
                            slot, 1.0 if ((slot - 22) % 2 == 0) else -1.0
                        )
                    elif pq is not None and abs(q - pq) > 0.5 and abs(pq) > 1e-3:
                        base_center[slot] = float(pq)
                    else:
                        base_center[slot] = q
                    base_cmd[slot] = base_center[slot]
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
        # Prefer starting toward the nearer CLEAR end, then bounce.
        targets = [j2_hi, j2_lo]
        if abs(arm_cmd[J2] - j2_lo) < abs(arm_cmd[J2] - j2_hi):
            targets = [j2_lo, j2_hi]
        tgt_i = 0
        last_status = time.perf_counter()
        dt_nom = 1.0 / max(float(args.stream_hz), 1.0)
        ARRIVE_EPS = 0.08  # rad — turn around near target / when stuck
        STUCK_S = 2.5

        def _tick_base_and_dxl() -> Dict[int, float]:
            """Advance base triangle + DXL; never blank RS (re-seed if lagging)."""
            live = _read_base(session)
            for slot, q in live.items():
                if slot not in base_center:
                    base_center[slot] = q
                    base_cmd[slot] = q
                    base_dirs[slot] = 1.0 if ((slot - 22) % 2 == 0) else -1.0
                    print(f"  base slot{slot} live FB={q:+.3f}", flush=True)
                # Slow center track only when near cmd (avoid dragging center while lagging).
                if abs(float(base_cmd.get(slot, q)) - q) < BASE_LEAD * 0.8:
                    base_center[slot] = 0.99 * base_center[slot] + 0.01 * q
            tick_base: Dict[int, float] = {}
            for slot, center in base_center.items():
                proto = base_proto[slot]
                amp = _amp_for_center(center, float(args.base_amp), proto=proto)
                if amp < 1e-3:
                    desire = _rail_clip(center, proto=proto)
                    if abs(desire) < 1e-6:
                        desire = 1e-6
                    tick_base[slot] = desire
                    base_cmd[slot] = desire
                    continue
                d = base_dirs.get(slot, 1.0)
                cmd = float(base_cmd.get(slot, center))
                cmd = cmd + d * base_rate * dt_nom
                lo, hi = center - amp, center + amp
                if cmd >= hi:
                    cmd = hi
                    base_dirs[slot] = -1.0
                elif cmd <= lo:
                    cmd = lo
                    base_dirs[slot] = 1.0
                desire = _rail_clip(cmd, proto=proto)
                fb_q = live.get(slot)
                if fb_q is not None:
                    err = desire - fb_q
                    if abs(err) > BASE_LEAD:
                        # Don't accumulate unreachable lead — walk from FB.
                        desire = fb_q + math.copysign(BASE_LEAD, err)
                        desire = _rail_clip(desire, proto=proto)
                        base_cmd[slot] = desire
                    else:
                        base_cmd[slot] = desire
                else:
                    base_cmd[slot] = desire
                if abs(desire) < 1e-6:
                    desire = 1e-6
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

        print(
            "\n== J2 CLEAR + brace J1/J3-7 + DXL + base (Ctrl-C cleans) ==",
            flush=True,
        )
        try:
            while not stop["flag"]:
                target = float(targets[tgt_i])
                # Always start from live FB — never from an unreachable CLEAR end.
                fb_arm = _read_arm(session)
                if fb_arm is not None:
                    for i in range(7):
                        if abs(float(fb_arm[i])) > 1e-3:
                            arm_cmd[i] = float(fb_arm[i])
                start = float(arm_cmd[J2])
                delta = target - start
                cruise = (
                    float(args.cruise_down) if delta < 0 else float(args.cruise_up)
                )
                if abs(delta) < ARRIVE_EPS:
                    print(
                        f"  J2 already at {start:+.3f} (~target {target:+.3f}) — skip",
                        flush=True,
                    )
                    tgt_i = 1 - tgt_i
                    continue
                print(
                    f"  J2 {start:+.3f} -> {target:+.3f} @ {cruise:.3f} rad/s "
                    f"(rate-limit + lead, no end snap) "
                    f"base={sorted(base_cmd)} dxl={dxl_cmd is not None}",
                    flush=True,
                )
                t_leg = time.perf_counter()
                last_progress = t_leg
                last_fb_j2 = start
                while not stop["flag"]:
                    now = time.perf_counter()
                    fb_arm = _read_arm(session)
                    fb_j2 = float(fb_arm[J2]) if fb_arm is not None else float(arm_cmd[J2])
                    # Rate-limit toward target from *current cmd*, glued near FB.
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
                        base_cmd=tick_base if tick_base else None,
                        base_proto=base_proto if tick_base else None,
                        base_gain_scale=1.0,
                    )

                    # Arrive on FB near target, or turn around if stuck (no snap).
                    if abs(fb_j2 - target) <= ARRIVE_EPS:
                        print(
                            f"  J2 arrived fb={fb_j2:+.3f} target={target:+.3f}",
                            flush=True,
                        )
                        break
                    if abs(fb_j2 - last_fb_j2) > 0.01:
                        last_progress = now
                        last_fb_j2 = fb_j2
                    elif (now - last_progress) >= STUCK_S:
                        print(
                            f"  J2 stuck fb={fb_j2:+.3f} (wanted {target:+.3f}) — reverse",
                            flush=True,
                        )
                        break
                    # Safety cap on leg duration
                    if (now - t_leg) > max(25.0, abs(delta) / max(cruise, 1e-3) * 2.5):
                        print(f"  J2 leg timeout fb={fb_j2:+.3f} — reverse", flush=True)
                        break

                    if (now - last_status) >= float(args.status_s):
                        r = _read_arm_detail(session, J2)
                        faults = _arm_faults(session)
                        live = _read_base(session)
                        btxt = " ".join(
                            f"s{s}={tick_base.get(s, float('nan')):+.2f}/"
                            f"{live.get(s, float('nan')):+.2f}"
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
                        if r:
                            print(
                                f"  J2 cmd/fb={arm_cmd[J2]:+.3f}/{r[0]:+.3f} "
                                f"tau={r[2]:+.2f} faults={faults} | {btxt}{dtxt}",
                                flush=True,
                            )
                            if (r[3] & 0xF) >= 8:
                                print("FAIL J2 hard fault — stop", flush=True)
                                stop["flag"] = True
                                break
                        last_status = now
                    time.sleep(dt_nom)

                # Soft dwell at *live* pose — never force CLEAR endpoint.
                fb_arm = _read_arm(session)
                if fb_arm is not None:
                    arm_cmd = fb_arm.copy()
                dwell_end = time.perf_counter() + 0.5
                while time.perf_counter() < dwell_end and not stop["flag"]:
                    fb_arm = _read_arm(session)
                    if fb_arm is not None:
                        for i in range(7):
                            arm_cmd[i] = (
                                0.9 * float(arm_cmd[i]) + 0.1 * float(fb_arm[i])
                            )
                    tick_base = _tick_base_and_dxl()
                    _write_plant(
                        session,
                        arm_cmd,
                        arm_kp_scale=1.0,
                        base_cmd=tick_base if tick_base else None,
                        base_proto=base_proto if tick_base else None,
                    )
                    time.sleep(dt_nom)
                tgt_i = 1 - tgt_i
        finally:
            _cleanup(session)

    print("done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
