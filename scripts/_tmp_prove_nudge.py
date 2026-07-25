#!/usr/bin/env python3
"""Prove-style left-arm soft-engage + J1 nudge; print plant_block/dq."""
from __future__ import annotations

import time

import numpy as np

from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import PcbRobotSession, ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP


def main() -> int:
    port = find_cdc_port()
    print("port", port, flush=True)
    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=20.0, idle_first=True
    ) as session:
        hub = session.hub
        hub.set_rx_sim_mask(0)
        with pause_plant_stream(hub):
            ensure_yam_left_arm_cfg(hub, force=True)
        hub.set_mcu_state(McuState.NORMAL, send=True)

        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for s in range(7):
            desires[s] = ActuatorDesire(
                position=0.0, velocity=0.0, kp=0.0, kd=float(DEFAULT_ARM_KD)
            )
        session.set_actuators(desires, send=False)

        q0 = None
        for i in range(50):
            session.send_once()
            time.sleep(0.05)
            fb = session.latest_feedback()
            if fb is None:
                continue
            qs = [
                float(fb.actuator(s).position) if fb.actuator(s) else 0.0
                for s in range(7)
            ]
            faults = [
                int(fb.actuator(s).fault) & 0xFF if fb.actuator(s) else -1
                for s in range(7)
            ]
            pb = int(getattr(fb.plant_block, "value", fb.plant_block))
            if i % 10 == 0:
                print(
                    f"seed i={i} pb={pb} q={np.round(qs, 3)} f={faults}",
                    flush=True,
                )
            if max(abs(x) for x in qs) > 0.05:
                q0 = qs

        print("q0", None if q0 is None else [round(x, 4) for x in q0], flush=True)
        if q0 is None:
            print("FAIL no FB", flush=True)
            return 2

        for step in range(30):
            u = step / 29.0
            sc = u * u * (3.0 - 2.0 * u)
            d = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            for s in range(7):
                d[s] = ActuatorDesire(
                    position=q0[s],
                    velocity=0.0,
                    kp=float(DEFAULT_ARM_KP[s]) * sc,
                    kd=float(DEFAULT_ARM_KD),
                )
            session.set_actuators(d, send=False)
            session.send_once()
            time.sleep(0.05)

        target = list(q0)
        target[0] = q0[0] + 0.15
        print(f"nudge J1 {q0[0]:+.4f} -> {target[0]:+.4f}", flush=True)
        q = q0[0]
        pb = -1
        t0 = time.time()
        while time.time() - t0 < 2.5:
            d = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            for s in range(7):
                d[s] = ActuatorDesire(
                    position=target[s],
                    velocity=0.0,
                    kp=float(DEFAULT_ARM_KP[s]),
                    kd=float(DEFAULT_ARM_KD),
                )
            session.set_actuators(d, send=False)
            session.send_once()
            time.sleep(0.05)
            fb = session.latest_feedback()
            if fb and fb.actuator(0):
                q = float(fb.actuator(0).position)
                pb = int(getattr(fb.plant_block, "value", fb.plant_block))
                faults = [
                    int(fb.actuator(s).fault) & 0xFF if fb.actuator(s) else -1
                    for s in range(7)
                ]
        print(
            f"final q0={q:+.4f} pb={pb} dq={q - q0[0]:+.4f} faults={faults}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
