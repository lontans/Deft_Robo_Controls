#!/usr/bin/env python3
"""DIAG idle first → progressive all-7 green → jog J2 + J7 at low Hz."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import (
    PcbArmDriver,
    PcbRobotSession,
    ensure_yam_left_arm_cfg,
)
from deft_controls_sdk.vbeta import slots as yam_slots
from deft_controls_sdk.vbeta.cfg import pause_plant_stream

STREAM_HZ = 12.0
KP = (50.0, 30.0, 70.0, 60.0, 50.0, 50.0, 85.0)
KD = 3.5


def _cfg_enable(hub, enabled: set[int]) -> None:
    for i in range(7):
        hub.debug.cfg_set_slot(
            slot=i,
            bus=1,
            protocol=yam_slots.PROTO_DAMIAO,
            motor_id=0x01 + i,
            master_id=yam_slots._DAMIAO_MASTER[i],
            enabled=(i in enabled),
            persist=False,
        )


def _faults(arm: PcbArmDriver) -> list[int]:
    fb = arm._session.latest_feedback()  # noqa: SLF001
    out = []
    for slot in arm.slots:
        st = fb.actuator(slot) if fb else None
        out.append(int(st.fault) if st else -1)
    return out


def _hold(arm: PcbArmDriver, q: np.ndarray, seconds: float) -> None:
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        arm.write("Goal_Position", q.astype(np.float32))
        time.sleep(0.08)


def main() -> int:
    port = find_cdc_port()
    print("port", port, "stream_hz", STREAM_HZ)
    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=STREAM_HZ, idle_first=True
    ) as session:
        hub = session.hub
        print("MCU already DIAG_ONLY + idle (idle_first)")

        with pause_plant_stream(hub):
            ensure_yam_left_arm_cfg(hub, force=True)
            _cfg_enable(hub, set())  # all off while we progressive-arm
            print("CFG base applied; slots temporarily disabled for progressive arm")

        arm = PcbArmDriver(
            session,
            side="left",
            skip_home_on_connect=True,
            clamp_goals=False,
            kp=KP,
            kd=KD,
        )
        # Do not arm.connect() yet (it would MIT with CFG all-off / wrong state).
        arm.is_connected = True

        # Progressive: grow enabled set so enable-latch flood never hits 7 at once.
        armed: set[int] = set()
        q_hold = np.zeros(7, dtype=np.float64)
        for i in range(7):
            armed.add(i)
            print(f"arm slots {sorted(armed)} …", flush=True)
            with pause_plant_stream(hub):
                _cfg_enable(hub, armed)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            time.sleep(0.15)
            # Seed hold from live FB for newly armed slot(s)
            fb = session.latest_feedback()
            for s in armed:
                st = fb.actuator(s) if fb else None
                if st is not None:
                    q_hold[s] = float(st.position)
            arm._command_joint_pos(q_hold.astype(np.float32), send=False)  # noqa: SLF001
            ok = False
            for _ in range(40):
                _hold(arm, q_hold, 0.1)
                f = _faults(arm)
                if all(f[s] == 1 for s in armed):
                    ok = True
                    break
            print(f"  faults={_faults(arm)} ok={ok}")
            if not ok:
                print("  retry recover + re-arm subset")
                hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
                for s in range(ACTUATOR_COUNT):
                    hub.set_actuator(s, ActuatorDesire(), send=False)
                session.send_once()
                time.sleep(0.2)
                hub.recover()  # disable + reset latches → then NORMAL
                time.sleep(0.25)
                hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
                with pause_plant_stream(hub):
                    _cfg_enable(hub, armed)
                hub.set_mcu_state(McuState.NORMAL, send=True)
                for _ in range(50):
                    _hold(arm, q_hold, 0.1)
                    if all(_faults(arm)[s] == 1 for s in armed):
                        break
                print(f"  after retry faults={_faults(arm)}")

        print("ALL faults", _faults(arm), "q", np.array2string(q_hold, precision=3))
        q0 = np.asarray(arm.read("Position_Rad"), dtype=np.float64)
        print("live q", np.array2string(q0, precision=4))

        for j, name, delta in ((1, "J2", 0.10), (6, "J7", 0.12), (3, "J4", 0.08)):
            for sign, tag in ((+1, "+"), (-1, "-")):
                q = q0.copy()
                q[j] = q0[j] + sign * delta
                print(f"\njog {name} {tag}{delta} @ {STREAM_HZ}Hz")
                _hold(arm, q, 2.0)
                q1 = np.asarray(arm.read("Position_Rad"), dtype=np.float64)
                print(f"  dq={q1[j]-q0[j]:+.4f} faults={_faults(arm)}")
                _hold(arm, q0, 1.2)

        print("final faults", _faults(arm))
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        for s in range(ACTUATOR_COUNT):
            hub.set_actuator(s, ActuatorDesire(), send=False)
        session.send_once()
        time.sleep(0.15)
        hub.recover()
        arm.is_connected = False
    print("allgreen probe done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
