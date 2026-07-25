#!/usr/bin/env python3
"""Solo J2: FDCAN kick, discover, CFG only slot1, nudge inside CLEAR, print live dq."""
from __future__ import annotations

import time

import numpy as np

from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_scan_command,
    parse_probe_pdu,
)
from deft_controls_sdk.vbeta import PcbRobotSession, ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import PROTO_DAMIAO, _DAMIAO_MASTER
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI, CLEAR_LO

J2 = 1


def main() -> int:
    port = find_cdc_port()
    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=20.0, idle_first=True
    ) as session:
        hub = session.hub
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        conn = hub._connection
        for kind in (SESSION_BEGIN, SESSION_END):
            r = conn.exchange_raw(
                build_rs2_scan_command(0, kind, conn.next_seq(), bus=1),
                parse_probe_pdu,
                timeout_s=3.0,
                predicate=lambda p, k=kind: p.get("probe_kind") == k,
            )
            print("kick", kind, "ok" if r else "MISS", flush=True)
        ids = hub.debug.discover_damiao_all(bus=1, start=1, end=7, listen_ms=60)
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
                    enabled=(i == J2),
                    persist=False,
                )
        print("CFG: only J2 enabled", flush=True)

        hub.set_mcu_state(McuState.NORMAL, send=True)
        blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        # acquire
        q = None
        for _ in range(30):
            session.set_actuators(blank, send=False)
            # light poke only slot J2 at 0 then rewrite — use blank until fb
            time.sleep(0.05)
            fb = session.latest_feedback()
            if fb and fb.actuator(J2):
                p = float(fb.actuator(J2).position)
                if abs(p) > 1e-3:
                    q = p
                    break
        # better: kd hold at whatever
        for _ in range(20):
            d = dict(blank)
            # try read
            fb = session.latest_feedback()
            if fb and fb.actuator(J2) and abs(float(fb.actuator(J2).position)) > 1e-3:
                q = float(fb.actuator(J2).position)
            if q is None:
                d[J2] = ActuatorDesire(position=0.0, kp=0.0, kd=0.5)
            else:
                d[J2] = ActuatorDesire(position=q, kp=0.0, kd=0.5)
            session.set_actuators(d, send=False)
            time.sleep(0.05)
        if q is None:
            print("FAIL no FB", flush=True)
            return 2
        print(f"q0={q:+.4f} CLEAR=[{CLEAR_LO[J2]:+.3f},{CLEAR_HI[J2]:+.3f}]", flush=True)

        # soft engage
        for step in range(40):
            u = step / 39.0
            s = u * u * (3 - 2 * u)
            d = dict(blank)
            d[J2] = ActuatorDesire(position=q, velocity=0.0, kp=160.0 * s, kd=2.5)
            session.set_actuators(d, send=False)
            time.sleep(0.05)
        fb = session.latest_feedback()
        f0 = float(fb.actuator(J2).fault) if fb and fb.actuator(J2) else -1
        print(f"engaged fault={f0}", flush=True)

        target = float(np.clip(q - 0.25, CLEAR_LO[J2], CLEAR_HI[J2]))
        print(f"nudge {q:+.4f} -> {target:+.4f}", flush=True)
        t0 = time.time()
        while time.time() - t0 < 4.0:
            u = min(1.0, (time.time() - t0) / 3.0)
            s = u * u * (3 - 2 * u)
            cmd = q + (target - q) * s
            d = dict(blank)
            d[J2] = ActuatorDesire(
                position=cmd, velocity=(target - q) / 3.0, kp=160.0, kd=2.5, torque=-6.0
            )
            session.set_actuators(d, send=False)
            time.sleep(0.05)
        fb = session.latest_feedback()
        st = fb.actuator(J2) if fb else None
        q1 = float(st.position) if st else float("nan")
        print(
            f"result q={q1:+.4f} dq={q1 - q:+.4f} fault={int(st.fault)&0xff if st else -1} "
            f"vel={float(st.velocity) if st else float('nan'):+.3f} "
            f"tau={float(st.torque) if st else float('nan'):+.2f}",
            flush=True,
        )
        session.set_actuators(blank, send=False)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
