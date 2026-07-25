#!/usr/bin/env python3
"""Low-Hz solo teleop for J2 and J7 after user repositioned J7."""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.vbeta import PcbRobotSession, ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta import slots as yam_slots
from deft_controls_sdk.vbeta.cfg import pause_plant_stream


def set_en(hub, en: set[int]) -> None:
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


def jog(session, slot: int, *, kp: float, kd: float, delta: float, hold_s: float) -> None:
    fb = session.latest_feedback()
    st = fb.actuator(slot) if fb else None
    q0 = float(st.position) if st else 0.0
    f0 = int(st.fault) if st else -1
    print(f"  J{slot+1} q0={q0:+.4f} fault={f0}")
    session.set_actuator(slot, ActuatorDesire(position=q0, kp=kp, kd=kd), send=False)
    t_end = time.perf_counter() + 0.8
    while time.perf_counter() < t_end:
        session.send_once()
        time.sleep(0.05)
    session.set_actuator(slot, ActuatorDesire(position=q0 + delta, kp=kp, kd=kd), send=False)
    t_end = time.perf_counter() + hold_s
    while time.perf_counter() < t_end:
        session.send_once()
        time.sleep(0.05)
    st1 = session.latest_feedback().actuator(slot)
    print(f"  +delta dq={float(st1.position)-q0:+.4f} fault={st1.fault}")
    session.set_actuator(slot, ActuatorDesire(position=q0 - delta, kp=kp, kd=kd), send=False)
    t_end = time.perf_counter() + hold_s
    while time.perf_counter() < t_end:
        session.send_once()
        time.sleep(0.05)
    st2 = session.latest_feedback().actuator(slot)
    print(f"  -delta dq={float(st2.position)-q0:+.4f} fault={st2.fault}")
    session.set_actuator(slot, ActuatorDesire(position=q0, kp=kp, kd=kd), send=False)
    t_end = time.perf_counter() + 1.0
    while time.perf_counter() < t_end:
        session.send_once()
        time.sleep(0.05)


def main() -> int:
    port = find_cdc_port()
    # Low stream — high Hz vibrates / faults wrist on this daisy.
    hz = 18.0
    print(f"port={port} stream_hz={hz}")
    with PcbRobotSession.connect(port, apply_yam_cfg=False, stream_hz=hz) as session:
        with pause_plant_stream(session.hub):
            ensure_yam_left_arm_cfg(session.hub, force=True)
            # All off first
            set_en(session.hub, set())
        session.hub.recover()
        time.sleep(0.4)

        print("\n=== J2 solo (mild) ===")
        with pause_plant_stream(session.hub):
            set_en(session.hub, {1})
        time.sleep(0.3)
        jog(session, 1, kp=25.0, kd=2.0, delta=0.05, hold_s=1.8)
        session.hub.recover()
        time.sleep(0.3)

        print("\n=== J7 solo ===")
        with pause_plant_stream(session.hub):
            set_en(session.hub, {6})
        time.sleep(0.3)
        jog(session, 6, kp=100.0, kd=2.0, delta=0.12, hold_s=2.0)
        session.hub.recover()
        time.sleep(0.3)

        print("\n=== J4 solo (sanity) ===")
        with pause_plant_stream(session.hub):
            set_en(session.hub, {3})
        time.sleep(0.3)
        jog(session, 3, kp=80.0, kd=2.0, delta=0.08, hold_s=1.5)
        session.hub.recover()
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
