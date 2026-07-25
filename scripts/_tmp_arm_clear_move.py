#!/usr/bin/env python3
"""Plant-only arm recover + gentle jog (no DM bench session — avoids 3s quiet gate)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import FeedbackImage
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, parse_feedback_header
from deft_controls_sdk.vbeta import ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    PROTO_DAMIAO,
    PROTO_ROBSTRIDE,
)
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI, CLEAR_LO

HZ = 40.0
LEFT = tuple(range(7))
ARM_KP = tuple(float(x) for x in DEFAULT_ARM_KP)
ARM_KD = float(DEFAULT_ARM_KD)
CRUISE = 0.08  # rad/s


def _conn(hub):
    return hub._connection


def _blank(hub):
    _conn(hub).set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
    )


def _read_arm(hub):
    last = None
    reader = _conn(hub).reader
    while True:
        raw = reader.pop()
        if raw is None:
            break
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        try:
            fb = FeedbackImage(raw)
        except Exception:
            continue
        q = np.zeros(7, dtype=np.float32)
        faults = [0] * 7
        ok = False
        for i, slot in enumerate(LEFT):
            st = fb.actuator(slot)
            if st is not None:
                q[i] = float(st.position)
                faults[i] = int(st.fault) & 0xFF
                ok = True
        if ok:
            last = (q, faults)
    return last


def _pace(next_t, dt):
    next_t += dt
    d = next_t - time.perf_counter()
    if d > 0:
        time.sleep(d)
    else:
        next_t = time.perf_counter()
    return next_t


def _stream(hub, q, *, kp_scale: float, seconds: float, label: str):
    dt = 1.0 / HZ
    next_t = time.perf_counter()
    t_end = next_t + seconds
    last_faults = [0] * 7
    print(f"  {label} {seconds:.1f}s kp_scale={kp_scale:.2f}", flush=True)
    while time.perf_counter() < t_end:
        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for i, slot in enumerate(LEFT):
            desires[slot] = ActuatorDesire(
                position=float(q[i]),
                velocity=0.0,
                kp=float(ARM_KP[i]) * kp_scale,
                kd=ARM_KD if kp_scale > 0 else 0.4,
            )
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).send_once()
        got = _read_arm(hub)
        if got:
            q[:], last_faults = got[0], got[1]
        next_t = _pace(next_t, dt)
    return q, last_faults


def main() -> int:
    port = find_cdc_port()
    print(f"port {port}", flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        time.sleep(0.2)
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _blank(hub)

        with pause_plant_stream(hub):
            ensure_yam_left_arm_cfg(hub, force=True)
            for slot, bus, proto, mid, master in (
                (22, 5, PROTO_ROBSTRIDE, 0x70, 0),
                (23, 5, PROTO_ROBSTRIDE, 0x74, 0),
                (24, 6, PROTO_ROBSTRIDE, 0x75, 0),
                (25, 6, PROTO_DAMIAO, 0x06, 0x16),
            ):
                hub.debug.cfg_set_slot(
                    slot=slot,
                    bus=bus,
                    protocol=proto,
                    motor_id=mid,
                    master_id=master,
                    enabled=True,
                    persist=False,
                )

        # Soft acquire (kd only) — plant will clear+enable once kp engages
        q = np.zeros(7, dtype=np.float32)
        print("\n== ACQUIRE ==", flush=True)
        q, faults = _stream(hub, q, kp_scale=0.0, seconds=2.0, label="soft kd")
        print(
            f"  q={np.array2string(q, precision=3)} faults={faults} "
            f"(1=enabled, ≥8=real fault, 0xD=timeout)",
            flush=True,
        )

        print("\n== ENGAGE (plant auto clear+enable) ==", flush=True)
        for scale in (0.25, 0.5, 0.75, 1.0):
            q, faults = _stream(
                hub, q, kp_scale=scale, seconds=1.2, label=f"ramp {scale:.2f}"
            )
            print(f"    faults={faults}", flush=True)

        enabled = sum(1 for f in faults if (f & 0xF) == 1)
        print(f"  enabled_count={enabled}/7", flush=True)
        if enabled < 4:
            print("  WARN: few joints reporting ERR=1 — still trying jogs", flush=True)

        home = q.copy()
        print("\n== JOG ==", flush=True)
        dt = 1.0 / HZ
        for joint, delta in ((0, 0.15), (1, 0.08), (5, 0.12)):
            lo, hi = float(CLEAR_LO[joint]), float(CLEAR_HI[joint])
            start = float(home[joint])
            target = start + delta
            if target > hi:
                target = start - delta
            target = float(np.clip(target, lo, hi))
            if abs(target - start) < 0.02:
                print(f"  skip J{joint+1}", flush=True)
                continue
            move_s = max(abs(target - start) / CRUISE, 1.2)
            print(
                f"  J{joint+1} {start:+.3f}->{target:+.3f} over {move_s:.1f}s",
                flush=True,
            )
            q_cmd = home.copy()
            for target_pos, tag in ((target, "out"), (float(home[joint]), "back")):
                t0 = time.perf_counter()
                start_j = float(q_cmd[joint])
                next_t = t0
                while True:
                    u = (time.perf_counter() - t0) / move_s
                    if u >= 1.0:
                        break
                    s = u * u * (3.0 - 2.0 * u)
                    q_cmd[joint] = start_j + (target_pos - start_j) * s
                    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
                    for i, slot in enumerate(LEFT):
                        desires[slot] = ActuatorDesire(
                            position=float(q_cmd[i]),
                            velocity=0.0,
                            kp=float(ARM_KP[i]),
                            kd=ARM_KD,
                        )
                    _conn(hub).set_actuators(desires, send=False)
                    _conn(hub).send_once()
                    got = _read_arm(hub)
                    if got:
                        faults = got[1]
                    next_t = _pace(next_t, dt)
                got = _read_arm(hub)
                fb_j = float(got[0][joint]) if got else float("nan")
                print(
                    f"    {tag} cmd={target_pos:+.3f} fb={fb_j:+.3f} "
                    f"Δ={fb_j-start:+.3f} faults={faults}",
                    flush=True,
                )

        print("\n== STOP ==", flush=True)
        _blank(hub)
        time.sleep(0.2)
        hub.recover()
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _blank(hub)
        print("done — board not prescribing motion", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
