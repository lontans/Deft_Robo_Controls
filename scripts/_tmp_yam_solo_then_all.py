#!/usr/bin/env python3
"""Validate green is live (not stale), then solo-prove J2/J4/J7, then all-7."""
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


def snap(session, label: str) -> None:
    fb = session.latest_feedback()
    rows = []
    for s in range(7):
        st = fb.actuator(s) if fb else None
        if st is None:
            rows.append(f"J{s+1}:?")
        else:
            rows.append(
                f"J{s+1}:f={int(st.fault)} q={float(st.position):+.3f} "
                f"τ={float(st.torque):+.2f}"
            )
    print(label, " | ".join(rows), flush=True)


def hold_slot(session, slot: int, q: float, *, kp: float, kd: float, seconds: float) -> None:
    # Blank other left slots so plant only spends TX on this one.
    for s in range(7):
        if s == slot:
            session.set_actuator(
                s, ActuatorDesire(position=q, kp=kp, kd=kd), send=False
            )
        else:
            session.set_actuator(s, ActuatorDesire(), send=False)
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        session.send_once()
        time.sleep(0.08)


def solo_jog(session, slot: int, *, kp: float, delta: float) -> None:
    with pause_plant_stream(session.hub):
        cfg_en(session.hub, {slot})
    session.hub.set_mcu_state(McuState.NORMAL, send=True)
    time.sleep(0.2)
    fb = session.latest_feedback()
    st = fb.actuator(slot) if fb else None
    q0 = float(st.position) if st else 0.0
    print(f"\n=== solo J{slot+1} q0={q0:+.4f} fault={int(st.fault) if st else -1} ===")
    hold_slot(session, slot, q0, kp=kp, kd=3.5, seconds=1.0)
    snap(session, "hold")
    hold_slot(session, slot, q0 + delta, kp=kp, kd=3.5, seconds=2.2)
    fb = session.latest_feedback()
    st = fb.actuator(slot)
    print(f"  +d dq={float(st.position)-q0:+.4f} f={int(st.fault)} τ={float(st.torque):+.2f}")
    hold_slot(session, slot, q0 - delta, kp=kp, kd=3.5, seconds=2.2)
    fb = session.latest_feedback()
    st = fb.actuator(slot)
    print(f"  -d dq={float(st.position)-q0:+.4f} f={int(st.fault)} τ={float(st.torque):+.2f}")
    hold_slot(session, slot, q0, kp=kp, kd=3.5, seconds=1.0)
    session.hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    for s in range(ACTUATOR_COUNT):
        session.set_actuator(s, ActuatorDesire(), send=False)
    session.send_once()
    time.sleep(0.1)
    session.hub.recover()
    time.sleep(0.25)


def main() -> int:
    port = find_cdc_port()
    print("port", port)
    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=HZ, idle_first=True
    ) as session:
        hub = session.hub
        with pause_plant_stream(hub):
            ensure_yam_left_arm_cfg(hub, force=True)
            cfg_en(hub, set())
        print("after CFG all-disabled (still DIAG):")
        time.sleep(0.3)
        snap(session, "disabled")
        hub.recover()
        time.sleep(0.3)
        snap(session, "post-recover")

        # Solo proves — true motion, not stale ERR
        solo_jog(session, 1, kp=45.0, delta=0.12)  # J2
        solo_jog(session, 6, kp=140.0, delta=0.20)  # J7
        solo_jog(session, 3, kp=100.0, delta=0.12)  # J4

        # All-7 soft hold; require torque response on a J1 nudge as bus sanity
        print("\n=== all-7 soft hold ===")
        with pause_plant_stream(hub):
            cfg_en(hub, set(range(7)))
        hub.set_mcu_state(McuState.NORMAL, send=True)
        fb = session.latest_feedback()
        q = np.array(
            [float(fb.actuator(s).position) for s in range(7)], dtype=np.float64
        )
        kp = (50, 30, 70, 60, 50, 50, 85)
        for _ in range(25):
            for s in range(7):
                session.set_actuator(
                    s,
                    ActuatorDesire(position=float(q[s]), kp=float(kp[s]), kd=3.5),
                    send=False,
                )
            time.sleep(0.08)
        snap(session, "all7")
        # nudge J1
        for _ in range(20):
            session.set_actuator(
                0, ActuatorDesire(position=float(q[0] + 0.08), kp=50, kd=3.5), send=False
            )
            for s in range(1, 7):
                session.set_actuator(
                    s,
                    ActuatorDesire(position=float(q[s]), kp=float(kp[s]), kd=3.5),
                    send=False,
                )
            time.sleep(0.08)
        fb = session.latest_feedback()
        print(f"J1 nudge dq={float(fb.actuator(0).position)-q[0]:+.4f}")
        snap(session, "after J1 nudge")

        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        for s in range(ACTUATOR_COUNT):
            session.set_actuator(s, ActuatorDesire(), send=False)
        session.send_once()
        hub.recover()
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
