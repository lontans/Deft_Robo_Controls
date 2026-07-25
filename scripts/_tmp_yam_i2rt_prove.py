#!/usr/bin/env python3
"""Smooth-ramp prove, J7 off. Soft-engage MIT (no full-kp snap at arming)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire, LedDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import (
    PcbArmDriver,
    PcbRobotSession,
    ensure_yam_left_arm_cfg,
)
from deft_controls_sdk.vbeta import slots as yam_slots
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP

STREAM_HZ = 20.0
KP = tuple(DEFAULT_ARM_KP)
KD = float(DEFAULT_ARM_KD)
ACTIVE = (0, 1, 2, 3, 4, 5)  # J1..J6


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
    return [
        int(fb.actuator(s).fault) if fb and fb.actuator(s) else -1 for s in arm.slots
    ]


def write_active(
    session,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    dq=None,
    kp_scale: float = 1.0,
) -> None:
    """Hold fixed q. kp_scale soft-engages (0→1) — never chase live FB here."""
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
                kd=KD,  # full damping even while kp ramps
            )
        else:
            desires[slot] = ActuatorDesire()
    session.set_actuators(desires, send=False)
    arm._setpoint = np.asarray(q, dtype=np.float32).reshape(7)  # noqa: SLF001


def soft_engage(session, arm: PcbArmDriver, q: np.ndarray, engage_s: float = 1.2) -> None:
    """Ramp kp 0→full at a *fixed* setpoint. Fixes start-of-run J6 buzz.

    Prior bug: first write_active(q0) applied full KP instantly after recover.
    """
    print(f"soft-engage MIT over {engage_s:.1f}s at fixed setpoint (no FB chase)…")
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / engage_s
        if u >= 1.0:
            break
        # smoothstep on kp scale
        s = u * u * (3.0 - 2.0 * u)
        write_active(session, arm, q, kp_scale=s)
        time.sleep(0.02)
    write_active(session, arm, q, kp_scale=1.0)


def go_to_active(session, arm: PcbArmDriver, target: np.ndarray, dt: float) -> None:
    """Smoothstep ramp from setpoint → target (setpoint-relative, not FB)."""
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


def main() -> int:
    port = find_cdc_port()
    print(f"port={port} hz={STREAM_HZ} J7=off kp={KP} kd={KD}")
    print(
        "audit: jogs use setpoint ramps; engage uses kp ramp at fixed q — "
        "no FB→Goal rewrite loop"
    )

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
        print("CFG: J1–J6 on, J7 off")

        # recover disables; stay DIAG until we have a pose seed ready
        hub.recover()
        time.sleep(0.25)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)

        arm = PcbArmDriver(
            session,
            side="left",
            skip_home_on_connect=True,
            clamp_goals=False,
            kp=KP,
            kd=KD,
        )
        arm.is_connected = True

        # Brief NORMAL + kd-only (kp=0) to get live FB without torque snap.
        hub.set_mcu_state(McuState.NORMAL, send=True)
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
            send=True,
        )
        # Seed positions at 0 with kp=0 until FB arrives — then freeze that FB once.
        q_seed = np.zeros(7, dtype=np.float32)
        write_active(session, arm, q_seed, kp_scale=0.0)
        q0 = None
        for _ in range(60):
            q = np.asarray(arm.read("Position_Rad"), dtype=np.float32)
            if float(np.max(np.abs(q[list(ACTIVE)]))) > 1e-3:
                q0 = q.copy()
                break
            time.sleep(0.05)
        if q0 is None:
            print("FAIL: no FB on J1–J6")
            return 2

        # Freeze pose once — do not re-read into Goal during engage.
        print(
            "frozen home",
            np.array2string(q0[list(ACTIVE)], precision=3),
        )
        write_active(session, arm, q0, kp_scale=0.0)
        time.sleep(0.2)
        soft_engage(session, arm, q0, engage_s=1.4)
        time.sleep(0.8)
        print("after engage faults", faults(arm))

        for slot, delta, name, move_s in (
            (0, 0.10, "J1", 1.2),
            (1, 0.12, "J2", 1.8),
            (5, 0.10, "J6", 1.5),
            (3, 0.10, "J4", 1.2),
        ):
            for sign, tag in ((+1, "+"), (-1, "-")):
                q = q0.copy()
                q[slot] = float(q0[slot] + sign * delta)
                print(f"smooth {name} {tag}{delta} over {move_s}s …", flush=True)
                go_to_active(session, arm, q, move_s)
                time.sleep(0.35)
                q1 = np.asarray(arm.read("Position_Rad"), dtype=np.float64)
                tau = np.asarray(arm.read("torque"), dtype=np.float64)
                print(
                    f"  dq={q1[slot]-float(q0[slot]):+.4f} τ={tau[slot]:+.2f} "
                    f"faults={faults(arm)}",
                    flush=True,
                )
                go_to_active(session, arm, q0, move_s)
                time.sleep(0.25)

        print("final faults", faults(arm))
        arm.is_connected = False

    print("done — cornflower idle")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
