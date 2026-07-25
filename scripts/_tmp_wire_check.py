#!/usr/bin/env python3
"""Verify wire desires land and whether CH1 FB moves after CFG+nudge."""
from __future__ import annotations

import struct
import time

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import FeedbackImage
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, parse_actuator_feedback, parse_feedback_header
from deft_controls_sdk.link.exchange.pack import actuator_slot_offset
from deft_controls_sdk.vbeta import ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import PROTO_DAMIAO, _DAMIAO_MASTER


def main() -> int:
    port = find_cdc_port()
    print("port", port, flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        hub.set_rx_sim_mask(0)
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

        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        desires[0] = ActuatorDesire(position=0.5, velocity=0.0, kp=40.0, kd=1.0)
        hub._connection.set_actuators(desires, send=False)
        raw = hub._connection.build_command().to_bytes()
        off = actuator_slot_offset(0)
        pos, vel, kp, kd, tau = struct.unpack_from("<fffff", raw, off)
        print(
            f"wire s0 pos={pos} vel={vel} kp={kp} kd={kd} tau={tau} "
            f"sys={raw[12]:02x}{raw[13]:02x}{raw[14]:02x}{raw[15]:02x}",
            flush=True,
        )

        t0 = time.perf_counter()
        last = None
        while time.perf_counter() - t0 < 3.0:
            hub.set_mcu_state(McuState.NORMAL, send=False)
            hub._connection.set_actuators(desires, send=False)
            hub._connection.send_once()
            time.sleep(0.05)
            while True:
                r = hub._connection.reader.pop()
                if r is None:
                    break
                h = parse_feedback_header(r)
                if not h or h.get("is_debug"):
                    continue
                fb = FeedbackImage(r)
                st = fb.actuator(0)
                pb = int(getattr(fb.plant_block, "value", fb.plant_block))
                if st is not None:
                    last = (pb, float(st.position), int(st.fault) & 0xFF)
        print("last", last, flush=True)

        hub._connection.send_once()
        time.sleep(0.05)
        while True:
            r = hub._connection.reader.pop()
            if r is None:
                break
            h = parse_feedback_header(r)
            if not h or h.get("is_debug"):
                continue
            print("act0", parse_actuator_feedback(r, 0), flush=True)
            break

        blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        hub._connection.set_actuators(blank, send=False)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        hub._connection.send_once()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
