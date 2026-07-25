#!/usr/bin/env python3
"""Recover CH1 Damiao faults then proven i2rt soft-engage jogs (20 Hz)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire, LedDesire, McuState
from deft_controls_sdk.bench import damiao as dm
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.bench import (
    DM_PROBE_CLEAR_FAULT,
    DM_PROBE_ENABLE,
    DM_PROBE_MIT,
)
from deft_controls_sdk.vbeta import (
    PcbArmDriver,
    PcbRobotSession,
    ensure_yam_left_arm_cfg,
)
from deft_controls_sdk.vbeta import slots as yam_slots
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP

STREAM_HZ = 20.0
KP = tuple(float(x) for x in DEFAULT_ARM_KP)
KD = float(DEFAULT_ARM_KD)
ACTIVE = (0, 1, 2, 3, 4, 5)  # J1..J6
ARM_IDS = (1, 2, 3, 4, 5, 6)  # ESC — match ACTIVE
DM_QUIET_S = 3.2  # firmware PLANT_DIAG_DM_QUIET_MS = 3000


def cfg_no_j7(hub) -> None:
    ensure_yam_left_arm_cfg(hub, force=True)
    hub.debug.cfg_set_slot(
        slot=6,
        bus=1,
        protocol=yam_slots.PROTO_DAMIAO,
        motor_id=0x07,
        master_id=yam_slots._DAMIAO_MASTER[6],
        enabled=False,
        persist=False,
    )


def faults(arm: PcbArmDriver) -> list[int]:
    fb = arm._session.latest_feedback()  # noqa: SLF001
    out = []
    for s in arm.slots:
        if fb and fb.actuator(s):
            out.append(int(fb.actuator(s).fault) & 0xFF)  # low byte / ERR nibble view
        else:
            out.append(-1)
    return out


def write_active(session, arm, q, *, dq=None, kp_scale: float = 1.0) -> None:
    desires = {}
    vel = (
        np.zeros(7, dtype=np.float32)
        if dq is None
        else np.asarray(dq, dtype=np.float32).reshape(7)
    )
    scale = float(np.clip(kp_scale, 0.0, 1.0))
    for i, slot in enumerate(arm.slots):
        if i in ACTIVE:
            desires[slot] = ActuatorDesire(
                position=float(q[i]),
                velocity=float(vel[i]),
                kp=float(KP[i]) * scale,
                kd=KD,
            )
        else:
            desires[slot] = ActuatorDesire()
    session.set_actuators(desires, send=False)
    arm._setpoint = np.asarray(q, dtype=np.float32).reshape(7)  # noqa: SLF001


def soft_engage(session, arm, q, engage_s: float = 1.4) -> None:
    print(f"soft-engage MIT over {engage_s:.1f}s…", flush=True)
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / engage_s
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        write_active(session, arm, q, kp_scale=s)
        time.sleep(0.02)
    write_active(session, arm, q, kp_scale=1.0)


def go_to_active(session, arm, target, dt: float) -> None:
    start = arm._setpoint.copy()  # noqa: SLF001
    target = np.asarray(target, dtype=np.float32).reshape(7)
    delta = target - start
    dt = max(float(dt), 1e-3)
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / dt
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        ds_du = 6.0 * u * (1.0 - u)
        q = start + delta * np.float32(s)
        dq = (delta / np.float32(dt)) * np.float32(ds_du)
        write_active(session, arm, q, dq=dq, kp_scale=1.0)
        time.sleep(0.01)
    write_active(session, arm, target, kp_scale=1.0)


def recover_dm_enables(hub) -> None:
    """Bench clear+enable+MIT verify per ESC, then wait out DM quiet gate."""
    conn = hub._connection  # noqa: SLF001
    print("\n== DM CLEAR+ENABLE (bench) ==", flush=True)
    begin = dm._dm_session_begin(conn, 1)
    print(f"  SESSION_BEGIN {'ok' if begin else 'MISS'}", flush=True)
    ok_n = 0
    for mid in ARM_IDS:
        dm._send_probe(
            conn, mid, DM_PROBE_CLEAR_FAULT, bus=1, listen_ms=40, timeout_s=1.5
        )
        en = dm._send_probe(
            conn, mid, DM_PROBE_ENABLE, bus=1, listen_ms=60, timeout_s=2.0
        )
        mit = dm._send_probe(
            conn, mid, DM_PROBE_MIT, bus=1, listen_ms=50, timeout_s=1.5
        )
        err = None if mit is None else (int(mit.get("err", 0)) & 0xF)
        found = None if mit is None else mit.get("found")
        good = err == 1
        ok_n += int(good)
        print(
            f"  ESC 0x{mid:02X} enable_found={None if en is None else en.get('found')} "
            f"mit_found={found} err={err} {'OK' if good else 'BAD'}",
            flush=True,
        )
    dm._dm_session_end(conn, 1)
    print(
        f"  enabled {ok_n}/{len(ARM_IDS)} — waiting {DM_QUIET_S:.1f}s quiet gate…",
        flush=True,
    )
    # Blank desires during quiet so we don't fight the gate
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    hub._connection.set_actuators(  # noqa: SLF001
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
    )
    time.sleep(DM_QUIET_S)


def main() -> int:
    port = find_cdc_port()
    print(f"port={port} hz={STREAM_HZ} proven i2rt path + DM recover", flush=True)

    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=STREAM_HZ, idle_first=True
    ) as session:
        hub = session.hub
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        session.set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
            send=False,
        )
        session.send_once()

        with pause_plant_stream(hub):
            cfg_no_j7(hub)
        print("CFG: J1–J6 on, J7 off", flush=True)

        hub.recover()
        time.sleep(0.25)
        hub.set_mcu_state(McuState.NORMAL, send=True)

        # Stop streamer during bench DM (lease path)
        was = hub.is_streaming
        if was:
            hub.stop_streaming()
        recover_dm_enables(hub)
        if was:
            hub.start_streaming(hz=STREAM_HZ)

        hub.set_mcu_state(McuState.NORMAL, send=True)
        arm = PcbArmDriver(
            session,
            side="left",
            skip_home_on_connect=True,
            clamp_goals=False,
            kp=KP,
            kd=KD,
        )
        arm.is_connected = True

        q_seed = np.zeros(7, dtype=np.float32)
        write_active(session, arm, q_seed, kp_scale=0.0)
        q0 = None
        for _ in range(80):
            q = np.asarray(arm.read("Position_Rad"), dtype=np.float32)
            if float(np.max(np.abs(q[list(ACTIVE)]))) > 1e-3:
                q0 = q.copy()
                break
            time.sleep(0.05)
        if q0 is None:
            print("FAIL: no FB on J1–J6 after recover", flush=True)
            return 2

        print("frozen home", np.array2string(q0[list(ACTIVE)], precision=3), flush=True)
        write_active(session, arm, q0, kp_scale=0.0)
        time.sleep(0.2)
        soft_engage(session, arm, q0, engage_s=1.4)
        time.sleep(0.5)
        print("after engage faults(lo8)=", faults(arm), flush=True)

        for slot, delta, name, move_s in (
            (0, 0.10, "J1", 1.5),
            (1, 0.10, "J2", 1.8),
            (5, 0.10, "J6", 1.5),
        ):
            q = q0.copy()
            q[slot] = float(q0[slot] + delta)
            print(f"smooth {name} +{delta} over {move_s}s …", flush=True)
            go_to_active(session, arm, q, move_s)
            time.sleep(0.3)
            q1 = np.asarray(arm.read("Position_Rad"), dtype=np.float64)
            print(
                f"  dq={q1[slot]-float(q0[slot]):+.4f} faults(lo8)={faults(arm)}",
                flush=True,
            )
            go_to_active(session, arm, q0, move_s)
            time.sleep(0.25)

        print("final faults(lo8)=", faults(arm), flush=True)
        # Stop prescribing: blank + DIAG
        session.set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
            send=False,
        )
        session.send_once()
        arm.is_connected = False

    print("done — not prescribing motion", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
