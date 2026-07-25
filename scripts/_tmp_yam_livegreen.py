#!/usr/bin/env python3
"""Progressive arm with LIVE motion proof (ignore stale fault=1)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import PcbRobotSession, ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta import slots as yam_slots
from deft_controls_sdk.vbeta.cfg import pause_plant_stream

HZ = 12.0
KP = (50.0, 35.0, 70.0, 80.0, 60.0, 60.0, 120.0)
KD = 3.5
# J2 near stop — prove with torque, not displacement.
PROOF_DQ = (0.04, 0.0, 0.04, 0.06, 0.05, 0.05, 0.08)
PROOF_TAU = (0.3, 1.0, 0.3, 0.25, 0.2, 0.2, 0.4)


def cfg_en(hub, en: set[int]) -> None:
    for i in range(7):
        hub.debug.cfg_set_slot(
            slot=i,
            bus=1,
            protocol=yam_slots.PROTO_DAMIAO,
            motor_id=0x01 + i,
            master_id=yam_slots._DAMIAO_MASTER[i],
            enabled=(i in en),
            persist=False,
        )


def faults(session) -> list[int]:
    fb = session.latest_feedback()
    out = []
    for s in range(7):
        st = fb.actuator(s) if fb else None
        out.append(int(st.fault) if st else -1)
    return out


def qtau(session, slot: int) -> tuple[float, float, int]:
    fb = session.latest_feedback()
    st = fb.actuator(slot) if fb else None
    if st is None:
        return 0.0, 0.0, -1
    return float(st.position), float(st.torque), int(st.fault)


def hold_all(session, q: np.ndarray, armed: set[int], seconds: float) -> None:
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        for s in range(7):
            if s in armed:
                session.set_actuator(
                    s,
                    ActuatorDesire(position=float(q[s]), kp=float(KP[s]), kd=KD),
                    send=False,
                )
            else:
                session.set_actuator(s, ActuatorDesire(), send=False)
        time.sleep(0.08)


def prove_slot(session, q: np.ndarray, armed: set[int], slot: int) -> bool:
    """Micro-jog newly armed slot; accept dq or tau proof."""
    q0, _, f0 = qtau(session, slot)
    delta = 0.10 if slot != 1 else 0.08
    q_cmd = q.copy()
    q_cmd[slot] = q0 + delta
    hold_all(session, q_cmd, armed, 1.8)
    q1, tau1, f1 = qtau(session, slot)
    dq = q1 - q0
    ok = (f1 == 1) and (abs(dq) >= PROOF_DQ[slot] or abs(tau1) >= PROOF_TAU[slot])
    print(
        f"  prove J{slot+1}: dq={dq:+.4f} τ={tau1:+.2f} f={f1} ok={ok}",
        flush=True,
    )
    hold_all(session, q, armed, 0.8)
    return ok


def main() -> int:
    port = find_cdc_port()
    print("port", port, "hz", HZ)
    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=HZ, idle_first=True
    ) as session:
        hub = session.hub
        with pause_plant_stream(hub):
            ensure_yam_left_arm_cfg(hub, force=True)
            cfg_en(hub, set())
        hub.recover()
        time.sleep(0.3)

        armed: set[int] = set()
        q = np.zeros(7, dtype=np.float64)
        for i in range(7):
            armed.add(i)
            print(f"arm {sorted(armed)}", flush=True)
            with pause_plant_stream(hub):
                cfg_en(hub, armed)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            time.sleep(0.15)
            fb = session.latest_feedback()
            for s in armed:
                st = fb.actuator(s) if fb else None
                if st:
                    q[s] = float(st.position)
            hold_all(session, q, armed, 1.0)
            if not prove_slot(session, q, armed, i):
                print("  FAIL live prove — abort")
                break
            print(f"  faults={faults(session)}")

        print("\nSustain all-7 for 6s…")
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 6.0:
            hold_all(session, q, armed, 0.5)
            print(f"  t={time.perf_counter()-t0:.1f} faults={faults(session)}")

        # Jog J2 + J7 while all armed
        for slot, delta, name in ((1, 0.10, "J2"), (6, 0.15, "J7")):
            q0, _, _ = qtau(session, slot)
            qc = q.copy()
            qc[slot] = q0 + delta
            print(f"\nall-armed jog {name} +{delta}")
            hold_all(session, qc, armed, 2.0)
            q1, tau, f = qtau(session, slot)
            print(f"  dq={q1-q0:+.4f} τ={tau:+.2f} f={f} faults={faults(session)}")
            hold_all(session, q, armed, 1.0)

        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        for s in range(ACTUATOR_COUNT):
            session.set_actuator(s, ActuatorDesire(), send=False)
        session.send_once()
        hub.recover()
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
