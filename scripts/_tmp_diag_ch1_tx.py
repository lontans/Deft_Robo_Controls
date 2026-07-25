#!/usr/bin/env python3
"""Diagnose CH1 plant_block / CFG / nudge response (no continuous)."""
from __future__ import annotations

import time

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import FeedbackImage
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, parse_feedback_header
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP


def drain(hub):
    out = []
    while True:
        raw = hub._connection.reader.pop()
        if raw is None:
            break
        out.append(raw)
    return out


def latest(hub):
    pb = mcu = None
    qs = [None] * 7
    faults = [-1] * 7
    for raw in drain(hub):
        h = parse_feedback_header(raw)
        if not h or h.get("is_debug"):
            continue
        fb = FeedbackImage(raw)
        pb = int(getattr(fb.plant_block, "value", fb.plant_block))
        mcu = int(fb.mcu_state)
        for s in range(7):
            st = fb.actuator(s)
            if st is not None:
                qs[s] = float(st.position)
                faults[s] = int(st.fault) & 0xFF
    return pb, mcu, qs, faults


def main() -> int:
    port = find_cdc_port()
    print("port", port, flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.NORMAL, send=True)

        t = hub.debug.cfg_get_table()
        for s in range(7):
            r = t[s]
            print(
                f"cfg s{s:02d} en={r.get('enabled')} bus={r.get('bus')} "
                f"proto={r.get('protocol')} id=0x{int(r.get('motor_id', 0)):02X} "
                f"master=0x{int(r.get('master_id', 0)):02X}",
                flush=True,
            )

        blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        hub._connection.set_actuators(blank, send=True)
        time.sleep(0.2)
        drain(hub)

        qs = [0.0] * 7
        pb = mcu = None
        faults = [-1] * 7
        for _ in range(20):
            hub._connection.set_actuators(blank, send=False)
            hub._connection.send_once()
            time.sleep(0.05)
            pb, mcu, qn, faults = latest(hub)
            for s, v in enumerate(qn):
                if v is not None:
                    qs[s] = v
        print(f"blank pb={pb} mcu={mcu} faults={faults}", flush=True)
        print(f"qs={[round(x, 4) for x in qs]}", flush=True)

        desires = dict(blank)
        target0 = qs[0] + 0.20
        for s in range(7):
            desires[s] = ActuatorDesire(
                position=target0 if s == 0 else qs[s],
                velocity=0.0,
                kp=float(DEFAULT_ARM_KP[s]),
                kd=float(DEFAULT_ARM_KD),
            )
        print(f"nudge s0 {qs[0]:+.4f} -> {target0:+.4f} kp={DEFAULT_ARM_KP[0]}", flush=True)

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 2.5:
            hub.set_mcu_state(McuState.NORMAL, send=False)
            hub._connection.set_actuators(desires, send=False)
            hub._connection.send_once()
            time.sleep(0.05)
            pb, mcu, qn, faults = latest(hub)
            if qn[0] is not None:
                qs[0] = qn[0]

        print(
            f"after pb={pb} mcu={mcu} s0={qs[0]:+.4f} faults={faults} "
            f"dq={qs[0] - (target0 - 0.20):+.4f}",
            flush=True,
        )

        hub._connection.set_actuators(blank, send=False)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        hub._connection.send_once()
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
