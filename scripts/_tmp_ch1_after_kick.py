#!/usr/bin/env python3
"""Kick FDCAN1, full Damiao discover, soft-engage + J1 nudge with live FB."""
from __future__ import annotations

import time

import numpy as np

from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import PcbRobotSession, ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP, PROTO_DAMIAO, _DAMIAO_MASTER


def main() -> int:
    port = find_cdc_port()
    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=20.0, idle_first=True
    ) as session:
        hub = session.hub
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        try:
            hub.debug.discover_robstride(bus=1, start=0x01, end=0x01)
        except Exception:
            pass
        ids = hub.debug.discover_damiao_all(bus=1, start=1, end=7, listen_ms=80)
        print("discover", ids, flush=True)
        with pause_plant_stream(hub):
            ensure_yam_left_arm_cfg(hub, force=True)
            for i in range(7):
                hub.debug.cfg_set_slot(
                    slot=i,
                    bus=1,
                    protocol=PROTO_DAMIAO,
                    motor_id=0x01 + i,
                    master_id=_DAMIAO_MASTER[i],
                    enabled=True,
                    persist=False,
                )
        hub.set_mcu_state(McuState.NORMAL, send=True)
        d = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for s in range(7):
            d[s] = ActuatorDesire(position=0.0, kp=0.0, kd=float(DEFAULT_ARM_KD))
        session.set_actuators(d, send=False)
        q0 = None
        for _ in range(40):
            session.send_once()
            time.sleep(0.05)
            fb = session.latest_feedback()
            if not fb:
                continue
            qs = [
                float(fb.actuator(s).position) if fb.actuator(s) else 0.0
                for s in range(7)
            ]
            if max(abs(x) for x in qs[:6]) > 0.02:
                q0 = qs
        print("q0", None if q0 is None else [round(x, 3) for x in q0], flush=True)
        if q0 is None:
            return 2
        for step in range(25):
            u = step / 24.0
            sc = u * u * (3 - 2 * u)
            # track live
            fb = session.latest_feedback()
            if fb:
                for s in range(7):
                    st = fb.actuator(s)
                    if st:
                        q0[s] = float(st.position)
            d = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            for s in range(7):
                d[s] = ActuatorDesire(
                    position=q0[s],
                    kp=float(DEFAULT_ARM_KP[s]) * sc,
                    kd=float(DEFAULT_ARM_KD),
                )
            session.set_actuators(d, send=False)
            session.send_once()
            time.sleep(0.05)
        target = list(q0)
        target[0] = q0[0] + 0.12
        t0 = time.time()
        q = q0[0]
        while time.time() - t0 < 2.0:
            d = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            for s in range(7):
                d[s] = ActuatorDesire(
                    position=target[s],
                    kp=float(DEFAULT_ARM_KP[s]),
                    kd=float(DEFAULT_ARM_KD),
                )
            session.set_actuators(d, send=False)
            session.send_once()
            time.sleep(0.05)
            fb = session.latest_feedback()
            if fb and fb.actuator(0):
                q = float(fb.actuator(0).position)
        print(f"nudge dq={q - q0[0]:+.4f} q={q:+.4f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
