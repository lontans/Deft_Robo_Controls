#!/usr/bin/env python3
"""Arm-only: continuous-style progressive latch, then slow J1 jog (J2 frozen).

fault==1 is MIT-green (not an error). fault==0 = not latched. >=8 = hard fault.
"""
from __future__ import annotations

import math
import sys
import time
from typing import Dict, List, Optional, Set

import numpy as np

from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_scan_command,
    parse_probe_pdu,
)
from deft_controls_sdk.vbeta import PcbRobotSession
from deft_controls_sdk.vbeta.cfg import ensure_yam_left_arm_cfg, pause_plant_stream
from deft_controls_sdk.vbeta.slots import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    LEFT_ARM_SLOTS,
    PROTO_DAMIAO,
    _DAMIAO_MASTER,
)
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI, CLEAR_LO

# Reuse continuous latch timings (proven on this rig).
LATCH_KP_SCALE = 0.35
LATCH_RAMP_S = 1.6
LATCH_HOLD_S = 1.2
ENGAGE_S = 2.0
ENGAGE_KP = 0.70  # full teleop after latch, but not slamming 1.0 for this smoke
STREAM_HZ = 40.0
J1 = 0
J2 = 1
J1_DELTA = 0.05  # rad each way
J1_RATE = 0.06  # rad/s — slower than continuous J2 cruise
JOG_INSET = 0.10


def _blank() -> Dict[int, ActuatorDesire]:
    return {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}


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


def _cfg_arm_slots(hub, enabled: Set[int]) -> None:
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
        if abs(float(st.position)) > 1e-3:
            q[i] = float(st.position)
            any_live = True
    return q if any_live else None


def _arm_faults(session: PcbRobotSession) -> List[int]:
    fb = session.latest_feedback()
    out = [0] * 7
    if fb is None:
        return out
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        st = fb.actuator(slot)
        if st is not None:
            out[i] = int(st.fault)
    return out


def _hard(faults: List[int]) -> bool:
    return any((f & 0xF) >= 8 for f in faults)


def _write_arm(
    session: PcbRobotSession,
    q: np.ndarray,
    *,
    kp_scale: float,
    dq: Optional[np.ndarray] = None,
) -> None:
    d = _blank()
    vel = np.zeros(7, dtype=np.float32) if dq is None else np.asarray(dq, dtype=np.float32)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        d[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(vel[i]),
            kp=float(DEFAULT_ARM_KP[i]) * float(kp_scale),
            kd=float(DEFAULT_ARM_KD[i]),
        )
    session.set_actuators(d, send=False)


def main() -> int:
    print("KP", DEFAULT_ARM_KP, flush=True)
    print("KD", DEFAULT_ARM_KD, flush=True)
    print(
        f"plan: progressive latch -> engage x{ENGAGE_KP} -> "
        f"J1 +/-{J1_DELTA} @{J1_RATE} rad/s; J2 frozen at FB",
        flush=True,
    )

    with PcbRobotSession.connect(
        "/dev/ttyACM0", apply_yam_cfg=False, stream_hz=STREAM_HZ
    ) as session:
        hub = session.hub

        print("\n== KICK + DISCOVER ==", flush=True)
        for attempt in range(5):
            _kick_fdcan1(hub)
            sweep = hub.debug.discover_damiao_all(bus=1, start=1, end=7, listen_ms=80)
            print(f"  attempt {attempt+1} sweep={sweep}", flush=True)
            if len(sweep) >= 5:
                break

        print("\n== PROGRESSIVE ARM LATCH (soft Goal=FB) ==", flush=True)
        ensure_yam_left_arm_cfg(hub, force=True)
        _cfg_arm_slots(hub, set())
        hub.set_mcu_state(McuState.NORMAL, send=True)

        q0 = np.zeros(7, dtype=np.float32)
        armed: Set[int] = set()

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
                        kp=float(DEFAULT_ARM_KP[s]) * LATCH_KP_SCALE * s_gain,
                        kd=float(DEFAULT_ARM_KD[s]),
                    )
                session.set_actuators(desires, send=False)
                faults = _arm_faults(session)
                if _hard(faults):
                    print(f"  HARD fault during latch {faults}", flush=True)
                    return False
                if (
                    time.perf_counter() >= t0_latch + ramp_s
                    and all(faults[s] == 1 for s in armed)
                ):
                    ok = True
                    break
                time.sleep(0.05)
            return ok

        def _recover_rearm() -> None:
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
                    f"ok={ok} try={attempt+1}/{attempts}",
                    flush=True,
                )
                if ok:
                    break
                print(f"  J{i+1} not green — recover + re-arm", flush=True)
                _recover_rearm()
            if not ok:
                print(f"WARN: J{i+1} still not green", flush=True)

        for pass_i in range(2):
            faults = _arm_faults(session)
            bad = [s for s in range(7) if faults[s] != 1]
            if not bad:
                break
            print(f"  final green pass {pass_i+1}: retry {bad}", flush=True)
            armed = set(range(7))
            _recover_rearm()
            _latch_armed(ramp_s=LATCH_RAMP_S + 0.8, hold_s=LATCH_HOLD_S + 1.0)

        # Refresh home from FB
        for _ in range(12):
            fb = _read_arm(session)
            if fb is not None:
                for s in range(7):
                    if abs(float(fb[s])) > 1e-3:
                        q0[s] = float(fb[s])
            _write_arm(session, q0, kp_scale=LATCH_KP_SCALE)
            time.sleep(0.05)

        faults = _arm_faults(session)
        print(
            f"arm home(FB)={[float(f'{x:+.3f}') for x in q0]} faults={faults}",
            flush=True,
        )
        if not all(f == 1 for f in faults):
            print("ABORT: not all joints MIT-green — will not jog", file=sys.stderr)
            session.set_actuators(_blank(), send=False)
            return 4
        if _hard(faults):
            print("ABORT: hard fault", file=sys.stderr)
            return 5

        # Soft-engage kp toward ENGAGE_KP while bracing at FB (incl. J2 hardstop).
        print(f"\n== SOFT ENGAGE -> kp x{ENGAGE_KP} ==", flush=True)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < ENGAGE_S:
            fb = _read_arm(session)
            if fb is not None:
                for s in range(7):
                    if abs(float(fb[s])) > 1e-3:
                        # brace track — keep J2 on hardstop FB, don't pull into CLEAR
                        q0[s] = 0.95 * float(q0[s]) + 0.05 * float(fb[s])
            u = (time.perf_counter() - t0) / ENGAGE_S
            s = u * u * (3.0 - 2.0 * u)
            scale = LATCH_KP_SCALE + (ENGAGE_KP - LATCH_KP_SCALE) * s
            _write_arm(session, q0, kp_scale=scale)
            time.sleep(0.025)
        print(f"engaged faults={_arm_faults(session)} q={np.array2string(q0, precision=3)}", flush=True)

        # J1 jog both directions; freeze others at live brace.
        lo = float(CLEAR_LO[J1]) + JOG_INSET
        hi = float(CLEAR_HI[J1]) - JOG_INSET
        for sign in (+1.0, -1.0):
            fb = _read_arm(session)
            if fb is not None:
                q0 = fb.copy()
            start = float(q0[J1])
            goal = float(np.clip(start + sign * J1_DELTA, lo, hi))
            if abs(goal - start) < 0.01:
                print(f"skip J1 {sign:+}: no room in CLEAR", flush=True)
                continue
            print(
                f"\n== JOG J1 {start:+.3f} -> {goal:+.3f} "
                f"(J2 frozen @ {float(q0[J2]):+.3f}) ==",
                flush=True,
            )
            cmd = q0.copy()
            t_leg = abs(goal - start) / J1_RATE
            t0 = time.perf_counter()
            while True:
                u = min(1.0, (time.perf_counter() - t0) / max(t_leg, 1e-3))
                s = u * u * (3.0 - 2.0 * u)
                cmd[J1] = start + (goal - start) * s
                fb = _read_arm(session)
                if fb is not None:
                    for s_i in range(7):
                        if s_i == J1:
                            continue
                        if abs(float(fb[s_i])) > 1e-3:
                            cmd[s_i] = 0.98 * float(cmd[s_i]) + 0.02 * float(fb[s_i])
                    # hard-freeze J2 to FB every tick
                    cmd[J2] = float(fb[J2]) if abs(float(fb[J2])) > 1e-3 else float(cmd[J2])
                dq = np.zeros(7, dtype=np.float32)
                dq[J1] = math.copysign(J1_RATE, goal - start) * (1.0 if u < 0.95 else 0.0)
                _write_arm(session, cmd, kp_scale=ENGAGE_KP, dq=dq)
                if _hard(_arm_faults(session)):
                    print("HARD fault during jog — stop", flush=True)
                    session.set_actuators(_blank(), send=False)
                    return 6
                if u >= 1.0:
                    break
                time.sleep(0.025)
            # settle
            t_set = time.perf_counter() + 1.0
            while time.perf_counter() < t_set:
                fb = _read_arm(session)
                if fb is not None:
                    cmd[J2] = float(fb[J2])
                    for s_i in range(7):
                        if s_i not in (J1, J2) and abs(float(fb[s_i])) > 1e-3:
                            cmd[s_i] = 0.98 * float(cmd[s_i]) + 0.02 * float(fb[s_i])
                cmd[J1] = goal
                _write_arm(session, cmd, kp_scale=ENGAGE_KP)
                time.sleep(0.025)
            fb = _read_arm(session)
            if fb is not None:
                print(
                    f"  after: J1={fb[J1]:+.4f} (err={fb[J1]-goal:+.4f}) "
                    f"J2={fb[J2]:+.4f} faults={_arm_faults(session)}",
                    flush=True,
                )
                q0 = fb.copy()

        print("\n== ZERO TORQUE / BLANK ==", flush=True)
        session.set_actuators(_blank(), send=False)
        time.sleep(0.4)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)

    print("\nJ1_TELEOP_OK", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
