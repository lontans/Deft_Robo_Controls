#!/usr/bin/env python3
"""Bench smoke: LED + neck DXL + left Damiao + optional MCP base slots.

Uses proven DXL path (ControlsPcbHub + send_once). Arm MIT @ 0.12 rad/s cruise.
Base MCP (not required until plugged):

  bus5 slot22 RS02 @ 0x70
  bus5 slot23 RS01 @ 0x74
  bus6 slot24 RS01 @ 0x75   (unplug OK — SKIP if no FB)
  bus6 slot25 Damiao @ 0x06 / master 0x16

Missing base FB → SKIP (not FAIL) unless --require-base.

    python yam_rig_smoke_suite.py --apply-cfg
    python yam_rig_smoke_suite.py --apply-cfg --pdu-kill   # needs Jetson pdb_uart_sim
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import (  # noqa: E402
    ActuatorDesire,
    ControlsPcbHub,
    LedDesire,
    McuState,
    ServoDesire,
)
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import (  # noqa: E402
    LED_MODE_FLASH,
    LED_MODE_IDLE_CORNFLOWER,
    LED_MODE_SOLID_GREEN,
    LED_MODE_SOLID_YELLOW,
)
from deft_controls_sdk.link.exchange import (  # noqa: E402
    ACTUATOR_COUNT,
    parse_feedback_header,
    parse_servo_feedback,
)
from deft_controls_sdk.pdb import KILL_SOFT_READY  # noqa: E402
from deft_controls_sdk.vbeta import ensure_yam_left_arm_cfg  # noqa: E402
from deft_controls_sdk.vbeta.cfg import pause_plant_stream  # noqa: E402
from deft_controls_sdk.vbeta.slots import (  # noqa: E402
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    PROTO_DAMIAO,
    PROTO_ROBSTRIDE,
)
from deft_controls_sdk.vbeta.yam_bench_clear_left import (  # noqa: E402
    CLEAR_HI,
    CLEAR_LO,
)

# --- suite knobs -------------------------------------------------------------
HZ = 40.0
ARM_CRUISE_RAD_S = 0.12
ARM_KP = tuple(float(x) for x in DEFAULT_ARM_KP)
ARM_KD = tuple(float(x) for x in DEFAULT_ARM_KD)  # per-joint, was a flat scalar
DXL_CRUISE_TICK_S = 350.0
RS_HOLD_KP = 20.0
RS_HOLD_KD = 1.0
DM_MCP_HOLD_KP = 15.0
DM_MCP_HOLD_KD = 1.0

# Operator clear (2026-07-24 paste), inset a little inside edges.
DXL_LO = (2153 + 40, 871 + 40)  # pitch, yaw
DXL_HI = (3058 - 40, 2622 - 40)
DXL_IDS = (1, 2)

LEFT_SLOTS = tuple(range(7))

# Plant-factory MCP map (CH5×2, CH6×2) — ready before HW is plugged.
# (slot, bus, protocol, motor_id, master_id, label)
BASE_MCP_ROWS: Tuple[Tuple[int, int, int, int, int, str], ...] = (
    (22, 5, PROTO_ROBSTRIDE, 0x70, 0, "CH5 RS02"),
    (23, 5, PROTO_ROBSTRIDE, 0x74, 0, "CH5 RS01"),
    (24, 6, PROTO_ROBSTRIDE, 0x75, 0, "CH6 RS01"),
    (25, 6, PROTO_DAMIAO, 0x06, 0x16, "CH6 Damiao"),
)


@dataclass
class Check:
    name: str
    ok: bool
    detail: str = ""


@dataclass
class SuiteResult:
    checks: List[Check] = field(default_factory=list)

    def add(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append(Check(name, ok, detail))
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {name}" + (f" — {detail}" if detail else ""), flush=True)

    def skip(self, name: str, detail: str = "") -> None:
        self.checks.append(Check(name, True, f"SKIP: {detail}"))
        print(f"  [SKIP] {name} — {detail}", flush=True)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)


def apply_left_arm_and_base_cfg(hub: ControlsPcbHub, *, force: bool = True) -> None:
    """Left CH1 Damiao + factory MCP slots 22–25 for upcoming base HW."""
    with pause_plant_stream(hub):
        ensure_yam_left_arm_cfg(hub, force=force)
        print("CFG: overlay base MCP slots 22–25 (bus5 RS02+RS01, bus6 RS01+Damiao)", flush=True)
        for slot, bus, proto, mid, master, label in BASE_MCP_ROWS:
            hub.debug.cfg_set_slot(
                slot=slot,
                bus=bus,
                protocol=proto,
                motor_id=mid,
                master_id=master,
                enabled=True,
                persist=False,
            )
            print(
                f"  slot {slot}: bus={bus} proto={proto} id=0x{mid:02X} "
                f"master=0x{master:02X} ({label})",
                flush=True,
            )


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _drain(hub: ControlsPcbHub):
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        yield frame


def _blank_actuators(hub: ControlsPcbHub) -> None:
    _conn(hub).set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )


def _set_leds(hub: ControlsPcbHub, mode: int, *, bright: int = 10) -> None:
    hub.set_led(LedDesire(mode=mode, master_brightness=bright), send=False)


def _set_dxl(hub: ControlsPcbHub, cmd: Sequence[float]) -> None:
    for i, sid in enumerate(DXL_IDS):
        lo, hi = DXL_LO[i], DXL_HI[i]
        goal = int(max(lo, min(hi, round(cmd[i]))))
        hub.set_servo(
            i,
            ServoDesire(
                servo_id=sid,
                native_step_position=goal,
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )


def _set_arm(
    hub: ControlsPcbHub,
    q: np.ndarray,
    *,
    dq: Optional[np.ndarray] = None,
    kp_scale: float = 1.0,
) -> None:
    q = np.asarray(q, dtype=np.float32).reshape(7)
    vel = (
        np.zeros(7, dtype=np.float32)
        if dq is None
        else np.asarray(dq, dtype=np.float32).reshape(7)
    )
    scale = float(np.clip(kp_scale, 0.0, 1.0))
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    for i, slot in enumerate(LEFT_SLOTS):
        desires[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(vel[i]),
            kp=float(ARM_KP[i]) * scale,
            kd=float(ARM_KD[i]),
        )
    _conn(hub).set_actuators(desires, send=False)


def _tx(hub: ControlsPcbHub) -> None:
    _conn(hub).send_once()


def _read_dxl_fb(hub: ControlsPcbHub) -> Tuple[Optional[int], Optional[int]]:
    out: List[Optional[int]] = [None, None]
    for raw in _drain(hub):
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        for slot in (0, 1):
            sv = parse_servo_feedback(raw, slot)
            if sv is None:
                continue
            pos = int(sv["present_position"]) & 0xFFFF
            if pos > 4095:
                pos &= 0x0FFF
            out[slot] = pos
    return out[0], out[1]


def _read_arm_fb(hub: ControlsPcbHub) -> Optional[np.ndarray]:
    last = None
    for raw in _drain(hub):
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        # parse via hub feedback image
        from deft_controls_sdk.link.api_types import FeedbackImage

        try:
            fb = FeedbackImage(raw)
        except Exception:
            continue
        q = np.zeros(7, dtype=np.float32)
        ok = False
        for i, slot in enumerate(LEFT_SLOTS):
            st = fb.actuator(slot)
            if st is not None:
                q[i] = float(st.position)
                ok = True
        if ok:
            last = q
    return last


def _pace(next_t: float, dt: float) -> float:
    next_t += dt
    sleep_for = next_t - time.perf_counter()
    if sleep_for > 0:
        time.sleep(sleep_for)
    else:
        next_t = time.perf_counter()
    return next_t


def phase_led(hub: ControlsPcbHub, result: SuiteResult, *, seconds: float = 3.0) -> None:
    print("\n== PHASE LED ==", flush=True)
    modes = (
        ("cornflower", LED_MODE_IDLE_CORNFLOWER),
        ("yellow", LED_MODE_SOLID_YELLOW),
        ("green", LED_MODE_SOLID_GREEN),
        ("flash", LED_MODE_FLASH),
        ("cornflower", LED_MODE_IDLE_CORNFLOWER),
    )
    dt = 1.0 / HZ
    per = seconds / len(modes)
    try:
        for name, mode in modes:
            t_end = time.perf_counter() + per
            next_t = time.perf_counter()
            while time.perf_counter() < t_end:
                _blank_actuators(hub)
                _conn(hub).clear_servos(send=False)
                _set_leds(hub, mode)
                _tx(hub)
                next_t = _pace(next_t, dt)
            print(f"  LED {name} ok", flush=True)
        result.add("led_cycle", True, f"{len(modes)} modes / {seconds:.1f}s")
    except Exception as exc:
        result.add("led_cycle", False, str(exc))


def phase_dxl(hub: ControlsPcbHub, result: SuiteResult, *, seconds: float = 8.0) -> None:
    print("\n== PHASE DXL (cmd-only sweep in operator clear) ==", flush=True)
    dt = 1.0 / HZ
    # Arm both: torque-off discover then hold
    present = [float((DXL_LO[i] + DXL_HI[i]) // 2) for i in range(2)]
    last: List[Optional[int]] = [None, None]
    t0 = time.perf_counter()
    next_t = t0
    while time.perf_counter() - t0 < 1.5:
        for i, sid in enumerate(DXL_IDS):
            if last[i] is None:
                hub.set_servo(
                    i,
                    ServoDesire(
                        servo_id=sid,
                        native_step_position=0,
                        torque_enable=False,
                        operating_mode=3,
                    ),
                    send=False,
                )
            else:
                hub.set_servo(
                    i,
                    ServoDesire(
                        servo_id=sid,
                        native_step_position=int(last[i]),
                        torque_enable=True,
                        operating_mode=3,
                    ),
                    send=False,
                )
        _blank_actuators(hub)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)
        p, y = _read_dxl_fb(hub)
        if p is not None:
            last[0] = p
        if y is not None:
            last[1] = y
        next_t = _pace(next_t, dt)

    if last[0] is None or last[1] is None:
        result.add("dxl_fb", False, f"no present after arm last={last}")
        return
    result.add("dxl_fb", True, f"pitch={last[0]} yaw={last[1]}")
    present = [float(last[0]), float(last[1])]
    cmd = list(present)
    dirs = [1.0, -1.0]
    fb_min = list(present)
    fb_max = list(present)
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    samples = 0
    while time.perf_counter() < t_end:
        for i in range(2):
            nxt = cmd[i] + dirs[i] * DXL_CRUISE_TICK_S * dt
            if nxt >= DXL_HI[i]:
                nxt = float(DXL_HI[i])
                dirs[i] = -1.0
            elif nxt <= DXL_LO[i]:
                nxt = float(DXL_LO[i])
                dirs[i] = 1.0
            cmd[i] = nxt
        _set_dxl(hub, cmd)
        _blank_actuators(hub)
        _set_leds(hub, LED_MODE_SOLID_GREEN, bright=8)
        _tx(hub)
        p, y = _read_dxl_fb(hub)
        if p is not None:
            fb_min[0] = min(fb_min[0], float(p))
            fb_max[0] = max(fb_max[0], float(p))
            samples += 1
        if y is not None:
            fb_min[1] = min(fb_min[1], float(y))
            fb_max[1] = max(fb_max[1], float(y))
        next_t = _pace(next_t, dt)

    moved = (fb_max[0] - fb_min[0] > 80) or (fb_max[1] - fb_min[1] > 80)
    result.add(
        "dxl_sweep",
        moved and samples > 10,
        f"samples={samples} pitchΔ={fb_max[0]-fb_min[0]:.0f} yawΔ={fb_max[1]-fb_min[1]:.0f}",
    )


def phase_arm(hub: ControlsPcbHub, result: SuiteResult, *, seconds: float = 12.0) -> None:
    print(
        f"\n== PHASE DAMIAO left arm (cruise {ARM_CRUISE_RAD_S} rad/s) ==",
        flush=True,
    )
    dt = 1.0 / HZ
    # Soft acquire: kp=0 kd light then engage
    q = np.zeros(7, dtype=np.float32)
    t0 = time.perf_counter()
    next_t = t0
    got = False
    while time.perf_counter() - t0 < 2.0:
        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for i, slot in enumerate(LEFT_SLOTS):
            desires[slot] = ActuatorDesire(
                position=float(q[i]), velocity=0.0, kp=0.0, kd=0.5
            )
        _conn(hub).set_actuators(desires, send=False)
        _conn(hub).clear_servos(send=False)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)
        fb = _read_arm_fb(hub)
        if fb is not None and float(np.max(np.abs(fb))) > 1e-3:
            q = fb.copy()
            got = True
        next_t = _pace(next_t, dt)

    if not got:
        result.add("arm_fb", False, "no Damiao FB")
        return
    result.add("arm_fb", True, np.array2string(q, precision=3))

    # Soft-engage
    t0 = time.perf_counter()
    next_t = t0
    while True:
        u = (time.perf_counter() - t0) / 1.4
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        _set_arm(hub, q, kp_scale=s)
        _conn(hub).clear_servos(send=False)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)
        next_t = _pace(next_t, dt)
    _set_arm(hub, q, kp_scale=1.0)
    _tx(hub)

    home = q.copy()
    # Jog joints that have clear span: J1, J2 (short), J6 — stay inside CLEAR_*
    plan = [
        (0, 0.15),  # J1
        (1, 0.10),  # J2 small
        (5, 0.12),  # J6
    ]
    moved_ok = True
    details = []
    for joint, delta in plan:
        lo = float(CLEAR_LO[joint])
        hi = float(CLEAR_HI[joint])
        start = float(home[joint])
        # Pick direction that stays in clear
        target = start + delta
        if target > hi:
            target = start - delta
        target = float(np.clip(target, lo, hi))
        dist = abs(target - start)
        if dist < 0.02:
            details.append(f"J{joint+1}:skip(span)")
            continue
        move_s = max(dist / ARM_CRUISE_RAD_S, 0.8)
        print(
            f"  jog J{joint+1} {start:+.3f}→{target:+.3f} over {move_s:.1f}s "
            f"@ {ARM_CRUISE_RAD_S} rad/s",
            flush=True,
        )
        t0 = time.perf_counter()
        next_t = t0
        q_cmd = home.copy()
        while True:
            u = (time.perf_counter() - t0) / move_s
            if u >= 1.0:
                break
            s = u * u * (3.0 - 2.0 * u)
            q_cmd[joint] = start + (target - start) * s
            ds = 6.0 * u * (1.0 - u)
            dq = np.zeros(7, dtype=np.float32)
            dq[joint] = ((target - start) / move_s) * ds
            _set_arm(hub, q_cmd, dq=dq, kp_scale=1.0)
            # Keep DXL held at mid-clear while arm moves (combined load)
            mid_dxl = [0.5 * (DXL_LO[i] + DXL_HI[i]) for i in range(2)]
            _set_dxl(hub, mid_dxl)
            _set_leds(hub, LED_MODE_SOLID_YELLOW, bright=8)
            _tx(hub)
            next_t = _pace(next_t, dt)
        _set_arm(hub, q_cmd, kp_scale=1.0)
        _tx(hub)
        time.sleep(0.3)
        fb = _read_arm_fb(hub)
        if fb is None:
            moved_ok = False
            details.append(f"J{joint+1}:no_fb")
        else:
            got_d = float(fb[joint] - start)
            details.append(f"J{joint+1}:Δ={got_d:+.3f}")
            if abs(got_d) < 0.03:
                moved_ok = False
        # return home
        t0 = time.perf_counter()
        start2 = float(q_cmd[joint])
        while True:
            u = (time.perf_counter() - t0) / move_s
            if u >= 1.0:
                break
            s = u * u * (3.0 - 2.0 * u)
            q_cmd[joint] = start2 + (float(home[joint]) - start2) * s
            _set_arm(hub, q_cmd, kp_scale=1.0)
            _set_dxl(hub, mid_dxl)
            _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
            _tx(hub)
            next_t = _pace(time.perf_counter(), dt)

    result.add("arm_jogs", moved_ok, "; ".join(details))


def _read_slot_fb(hub: ControlsPcbHub, slot: int) -> Optional[float]:
    last = None
    for raw in _drain(hub):
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        from deft_controls_sdk.link.api_types import FeedbackImage

        try:
            fb = FeedbackImage(raw)
        except Exception:
            continue
        st = fb.actuator(slot)
        if st is not None:
            last = float(st.position)
    return last


def phase_base_mcp(
    hub: ControlsPcbHub,
    result: SuiteResult,
    *,
    require: bool = False,
    hold_s: float = 2.0,
) -> None:
    """Soft-hold each base MCP slot if FB appears; SKIP if not plugged yet."""
    print("\n== PHASE BASE MCP (bus5 RS02+RS01, bus6 RS01+Damiao) ==", flush=True)
    dt = 1.0 / HZ
    seen: dict[int, float] = {}
    # Probe window: non-blank desires so MCP TX runs (blank skips SPI on CH4–6).
    t0 = time.perf_counter()
    next_t = t0
    while time.perf_counter() - t0 < 2.0:
        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for slot, bus, proto, mid, master, label in BASE_MCP_ROWS:
            if proto == PROTO_DAMIAO:
                desires[slot] = ActuatorDesire(
                    position=0.0, velocity=0.0, kp=DM_MCP_HOLD_KP, kd=DM_MCP_HOLD_KD
                )
            else:
                desires[slot] = ActuatorDesire(
                    position=0.0, velocity=0.0, kp=RS_HOLD_KP, kd=RS_HOLD_KD
                )
        _conn(hub).set_actuators(desires, send=False)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)
        for slot, *_rest in BASE_MCP_ROWS:
            pos = _read_slot_fb(hub, slot)
            # Unplugged slots usually sit at ~0; require non-trivial present.
            if pos is not None and abs(pos) > 1e-3:
                seen[slot] = pos
        next_t = _pace(next_t, dt)

    # Re-probe with hold-at-present for slots that answered
    if seen:
        t_end = time.perf_counter() + hold_s
        next_t = time.perf_counter()
        while time.perf_counter() < t_end:
            desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            for slot, bus, proto, mid, master, label in BASE_MCP_ROWS:
                q = float(seen.get(slot, 0.0))
                if proto == PROTO_DAMIAO:
                    desires[slot] = ActuatorDesire(
                        position=q, velocity=0.0, kp=DM_MCP_HOLD_KP, kd=DM_MCP_HOLD_KD
                    )
                else:
                    desires[slot] = ActuatorDesire(
                        position=q, velocity=0.0, kp=RS_HOLD_KP, kd=RS_HOLD_KD
                    )
            _conn(hub).set_actuators(desires, send=False)
            _set_leds(hub, LED_MODE_SOLID_GREEN, bright=8)
            _tx(hub)
            for slot, *_rest in BASE_MCP_ROWS:
                pos = _read_slot_fb(hub, slot)
                if pos is not None:
                    seen[slot] = pos
            next_t = _pace(next_t, dt)

    for slot, bus, proto, mid, master, label in BASE_MCP_ROWS:
        if slot in seen:
            result.add(
                f"base_slot{slot}",
                True,
                f"{label} bus={bus} id=0x{mid:02X} q={seen[slot]:+.4f}",
            )
        elif require:
            result.add(f"base_slot{slot}", False, f"{label} no FB (require-base)")
        else:
            result.skip(f"base_slot{slot}", f"{label} not plugged / no FB yet")


def phase_pdu_kill(hub: ControlsPcbHub, result: SuiteResult) -> None:
    """Observe USB kill_state / LED reaction to PDU UART peer (Jetson sim)."""
    print("\n== PHASE PDU UART kill / estop observe ==", flush=True)
    dt = 1.0 / HZ
    samples: List[str] = []
    t_end = time.perf_counter() + 2.5
    next_t = time.perf_counter()
    while time.perf_counter() < t_end:
        _blank_actuators(hub)
        # Host desire idle cornflower; FW may override from PDU kill.
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        hub.set_mcu_state(McuState.NORMAL, send=False)
        _tx(hub)
        st = hub.pdb_status()
        if st is not None:
            samples.append(st.kill_state_name)
        next_t = _pace(next_t, dt)

    if not samples:
        result.skip("pdu_kill_observe", "no pdb_status (sim not running?)")
        return

    last = samples[-1]
    uniq = sorted(set(samples))
    result.add(
        "pdu_kill_observe",
        True,
        f"last={last} seen={uniq} (LEDs follow PDU kill in FW)",
    )

    # If peer is already asking soft-kill, park and expect READY path.
    st = hub.pdb_status()
    if st is not None and st.soft_kill_req:
        print("  peer SOFT_KILL_REQ → soft_kill_park()…", flush=True)
        hub.soft_kill_park()
        time.sleep(0.4)
        st2 = hub.pdb_status()
        name = st2.kill_state_name if st2 else "?"
        result.add(
            "pdu_soft_kill_park",
            st2 is not None and (st2.soft_kill_ready or st2.kill_state == KILL_SOFT_READY),
            f"after park kill={name}",
        )
        hub.recover()
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _tx(hub)


def phase_combined(
    hub: ControlsPcbHub, result: SuiteResult, *, seconds: float = 6.0
) -> None:
    print("\n== PHASE COMBINED (LED flash + DXL bounce + arm hold) ==", flush=True)
    dt = 1.0 / HZ
    fb = _read_arm_fb(hub)
    if fb is None:
        # brief re-acquire
        q = np.zeros(7, dtype=np.float32)
    else:
        q = fb.copy()
    cmd_dxl = [float(DXL_LO[0]), float(DXL_HI[1])]
    dirs = [1.0, -1.0]
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    dxl_samples = 0
    try:
        while time.perf_counter() < t_end:
            for i in range(2):
                nxt = cmd_dxl[i] + dirs[i] * DXL_CRUISE_TICK_S * dt
                if nxt >= DXL_HI[i]:
                    nxt = float(DXL_HI[i])
                    dirs[i] = -1.0
                elif nxt <= DXL_LO[i]:
                    nxt = float(DXL_LO[i])
                    dirs[i] = 1.0
                cmd_dxl[i] = nxt
            _set_arm(hub, q, kp_scale=1.0)
            _set_dxl(hub, cmd_dxl)
            _set_leds(hub, LED_MODE_FLASH, bright=12)
            _tx(hub)
            p, y = _read_dxl_fb(hub)
            if p is not None or y is not None:
                dxl_samples += 1
            next_t = _pace(next_t, dt)
        result.add("combined", dxl_samples > 5, f"dxl_fb_ticks={dxl_samples}")
    except Exception as exc:
        result.add("combined", False, str(exc))


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--apply-cfg", action="store_true")
    ap.add_argument("--skip-arm", action="store_true")
    ap.add_argument("--skip-dxl", action="store_true")
    ap.add_argument("--skip-base", action="store_true", help="Skip MCP base probe")
    ap.add_argument(
        "--require-base",
        action="store_true",
        help="FAIL (not SKIP) if base MCP slots have no FB",
    )
    ap.add_argument(
        "--pdu-kill",
        action="store_true",
        help="Observe PDU UART kill_state (start pdb_uart_sim on Jetson first)",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    port = args.port or find_cdc_port()
    print(
        f"yam_rig_smoke_suite port={port} arm_cruise={ARM_CRUISE_RAD_S} "
        f"kp={ARM_KP} kd={ARM_KD}",
        flush=True,
    )
    print(f"arm clear lo={CLEAR_LO}", flush=True)
    print(f"arm clear hi={CLEAR_HI}", flush=True)
    print(f"dxl clear lo={DXL_LO} hi={DXL_HI}", flush=True)
    print(
        "base MCP ready: "
        + ", ".join(
            f"s{s}/CH{b}/0x{mid:02X}" for s, b, _p, mid, _m, _l in BASE_MCP_ROWS
        ),
        flush=True,
    )

    result = SuiteResult()
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        time.sleep(0.2)
        hub.set_rx_sim_mask(0)
        if args.apply_cfg:
            apply_left_arm_and_base_cfg(hub, force=True)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _blank_actuators(hub)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)
        time.sleep(0.2)

        phase_led(hub, result)
        if not args.skip_dxl:
            phase_dxl(hub, result)
        if not args.skip_arm:
            phase_arm(hub, result)
            phase_combined(hub, result)
        if not args.skip_base:
            phase_base_mcp(hub, result, require=bool(args.require_base))
        if args.pdu_kill:
            phase_pdu_kill(hub, result)

        # cleanup
        print("\n== CLEANUP ==", flush=True)
        _blank_actuators(hub)
        _conn(hub).clear_servos(send=False)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)
        time.sleep(0.15)
        hub.recover()
        time.sleep(0.1)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        _set_leds(hub, LED_MODE_IDLE_CORNFLOWER)
        _tx(hub)

    print("\n=== SUMMARY ===", flush=True)
    for c in result.checks:
        tag = "PASS" if c.ok and not c.detail.startswith("SKIP") else (
            "SKIP" if c.detail.startswith("SKIP") else "FAIL"
        )
        if c.ok and c.detail.startswith("SKIP"):
            tag = "SKIP"
        elif c.ok:
            tag = "PASS"
        else:
            tag = "FAIL"
        print(f"  {tag:4}  {c.name}: {c.detail}", flush=True)
    print("OVERALL", "PASS" if result.ok else "FAIL", flush=True)
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
