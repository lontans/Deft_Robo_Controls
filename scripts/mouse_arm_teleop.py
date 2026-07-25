#!/usr/bin/env python3
"""Laptop in-person teleop: Logitech M650 -> left Damiao arm (Controls PCB CDC).

Claude-2 per-joint KD + fast progressive latch (fault==1 = MIT green).

Controls (hold MIDDLE = enable / stick origin):
  Stick L/R                 J1
  Stick U/D                 J2
  Stick U/D + right hold    J3
  Stick U/D + left hold     J4  (thumbs too if OS reports them)
  Double-right (enabled)    J7 open/close
  Scroll                    J5
  Idle joints               hard position hold (no sag-track)
  Middle release            keep streaming hold (no settle→FB)
  Ctrl+C                    blank + exit
  LED                       cornflower idle (host non-OFF; not blink-red)

    python scripts/mouse_arm_teleop.py
    python scripts/mouse_arm_teleop.py --port COM5
    python scripts/mouse_arm_teleop.py --debug
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Optional, Set

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from deft_controls_sdk import ActuatorDesire, LedDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    SESSION_BEGIN,
    SESSION_END,
    build_rs2_scan_command,
    parse_probe_pdu,
)
from deft_controls_sdk.vbeta import PcbRobotSession
from deft_controls_sdk.vbeta.cfg import ensure_yam_left_arm_cfg, pause_plant_stream
from deft_controls_sdk.vbeta.slots import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    LEFT_ARM_SLOTS,
    PROTO_DAMIAO,
    _DAMIAO_MASTER,
)
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI, CLEAR_LO
from mouse_teleop import MouseTeleopState

try:
    from pynput import mouse
except ImportError as e:  # pragma: no cover
    print("Need pynput:  python -m pip install pynput", file=sys.stderr)
    raise SystemExit(2) from e

LATCH_KP_SCALE = 0.35
LATCH_RAMP_S = 0.70
LATCH_HOLD_S = 0.35
SEED_S = 0.28
ENGAGE_S = 1.0
ENGAGE_KP = 0.85
# J2 lift boost — only while J2 is actively driven. Always-on ×1.4 (~84 kp)
# with residual lead buzzes late in a session (deep CLEAR + gravity load).
J2_KP_SCALE = 1.40  # ~84 kp while raising/lowering J2
J2_HOLD_KP_SCALE = 1.00  # ~60 kp when bracing / enable-up
STREAM_HZ = 60.0

J1, J2, J3, J4, J5, J7 = 0, 1, 2, 3, 4, 6
# Full-stick rates (rad/s)
# ~+30% vs prior teleop rates
RATE_J1 = 0.36
RATE_J2 = 0.34
RATE_J3 = 0.29
RATE_J4 = 0.39
RATE_J5 = 0.31
RATE_J7 = 0.45  # open/close slew
JOG_INSET = 0.06
MAX_LEAD = 0.25
J2_MAX_LEAD = 0.18  # tighter than MAX_LEAD — large lead + high kp buzzes
J2_HARDSTOP_HI = -2.55
STICK_RADIUS_PX = 15.0
STICK_DEADZONE = 0.05
DOUBLE_RIGHT_S = 0.40
J7_SLEW_S = 0.7
# Driving-only buzz heuristic (idle gravity droop is normal: err≈tau/kp).
# Do not settle hold→FB on stick/enable release — that caused J2 sag.
BUZZ_ERR = 0.14
BUZZ_VEL = 0.20
TELEOP_LED = LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=10)


def _blank() -> Dict[int, ActuatorDesire]:
    return {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}


def _kick_fdcan1(hub) -> None:
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    conn = hub._connection  # noqa: SLF001
    for kind in (SESSION_BEGIN, SESSION_END):
        try:
            conn.exchange_raw(
                build_rs2_scan_command(0, kind, conn.next_seq(), bus=1),
                parse_probe_pdu,
                timeout_s=0.6,
                predicate=lambda p, k=kind: p.get("probe_kind") == k,
            )
        except Exception as exc:
            print(f"  kick {kind}: {exc}", flush=True)


def _cfg_arm_slots(hub, enabled: Set[int]) -> None:
    with pause_plant_stream(hub):
        for i in range(7):
            hub.debug.cfg_set_slot(
                slot=i,
                bus=1,
                protocol=PROTO_DAMIAO,
                motor_id=0x01 + i,
                master_id=_DAMIAO_MASTER[i],
                enabled=(i in enabled),
                persist=False,
            )


def _read_arm(session: PcbRobotSession) -> Optional[np.ndarray]:
    detail = _read_arm_detail(session)
    if detail is None:
        return None
    return detail[0]


def _read_arm_detail(
    session: PcbRobotSession,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, List[int]]]:
    """pos, vel, tau, faults — None if no live positions."""
    fb = session.latest_feedback()
    if fb is None:
        return None
    q = np.zeros(7, dtype=np.float32)
    v = np.zeros(7, dtype=np.float32)
    tau = np.zeros(7, dtype=np.float32)
    faults = [0] * 7
    any_live = False
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        st = fb.actuator(slot)
        if st is None:
            continue
        faults[i] = int(st.fault)
        v[i] = float(st.velocity)
        tau[i] = float(st.torque)
        if abs(float(st.position)) > 1e-3:
            q[i] = float(st.position)
            any_live = True
    if not any_live:
        return None
    return q, v, tau, faults


def _arm_faults(session: PcbRobotSession) -> List[int]:
    detail = _read_arm_detail(session)
    if detail is None:
        return [0] * 7
    return detail[3]


def _hard(faults: List[int]) -> bool:
    return any((f & 0xF) >= 8 for f in faults)


def _write_arm(
    session: PcbRobotSession,
    q: np.ndarray,
    *,
    kp_scale: float,
    dq: Optional[np.ndarray] = None,
    j2_boost: bool = False,
) -> None:
    d = _blank()
    vel = np.zeros(7, dtype=np.float32) if dq is None else np.asarray(dq, dtype=np.float32)
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        scale = float(kp_scale)
        if i == J2:
            scale = max(scale, J2_KP_SCALE if j2_boost else J2_HOLD_KP_SCALE)
        d[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(vel[i]),
            kp=float(DEFAULT_ARM_KP[i]) * scale,
            kd=float(DEFAULT_ARM_KD[i]),
        )
    session.set_actuators(d, send=False)
    # Keep non-OFF LED latched every frame — mode 0 lets PDB paint blink-red.
    session.set_led(TELEOP_LED, send=False)


def _clamp_joint(i: int, q: float) -> float:
    lo = float(CLEAR_LO[i]) + JOG_INSET
    hi = float(CLEAR_HI[i]) - JOG_INSET
    if i == J2:
        hi = max(hi, J2_HARDSTOP_HI)
    if lo >= hi:
        return float(q)
    return float(np.clip(q, lo, hi))


def _j7_ends() -> tuple[float, float]:
    return (
        float(CLEAR_LO[J7]) + JOG_INSET,
        float(CLEAR_HI[J7]) - JOG_INSET,
    )


def progressive_latch(session: PcbRobotSession) -> np.ndarray:
    hub = session.hub
    print("\n== BUS KICK ==", flush=True)
    t_kick = time.perf_counter()
    _kick_fdcan1(hub)
    print(f"  kick {time.perf_counter() - t_kick:.2f}s", flush=True)

    print("\n== PROGRESSIVE ARM LATCH (fast) ==", flush=True)
    print(f"  KP={DEFAULT_ARM_KP}", flush=True)
    print(f"  KD={DEFAULT_ARM_KD}", flush=True)
    t_latch = time.perf_counter()
    ensure_yam_left_arm_cfg(hub, force=True)
    _cfg_arm_slots(hub, set())
    hub.set_mcu_state(McuState.NORMAL, send=True)

    q0 = np.zeros(7, dtype=np.float32)
    armed: Set[int] = set()

    def _seed_hold(secs: float) -> None:
        t_end = time.perf_counter() + secs
        while time.perf_counter() < t_end:
            fb = _read_arm(session)
            if fb is not None:
                for s in armed:
                    if abs(float(fb[s])) > 1e-3:
                        q0[s] = float(fb[s])
            desires = _blank()
            for s in armed:
                desires[s] = ActuatorDesire(
                    position=float(q0[s]), velocity=0.0, kp=0.0, kd=0.3
                )
            session.set_actuators(desires, send=False)
            time.sleep(0.04)

    def _latch_armed(*, ramp_s: float, hold_s: float) -> bool:
        ok = False
        t0_latch = time.perf_counter()
        t_latch_end = t0_latch + ramp_s + hold_s
        while time.perf_counter() < t_latch_end:
            fb = _read_arm(session)
            if fb is not None:
                for s in armed:
                    if abs(float(fb[s])) > 1e-3:
                        q0[s] = (
                            0.85 * float(q0[s]) + 0.15 * float(fb[s])
                            if abs(q0[s]) > 1e-3
                            else float(fb[s])
                        )
            u = (time.perf_counter() - t0_latch) / max(ramp_s, 1e-3)
            s_gain = min(1.0, max(0.0, u))
            s_gain = s_gain * s_gain * (3.0 - 2.0 * s_gain)
            desires = _blank()
            for s in armed:
                desires[s] = ActuatorDesire(
                    position=float(q0[s]),
                    velocity=0.0,
                    kp=float(DEFAULT_ARM_KP[s]) * LATCH_KP_SCALE * s_gain,
                    kd=float(DEFAULT_ARM_KD[s]),
                )
            session.set_actuators(desires, send=False)
            faults = _arm_faults(session)
            if _hard(faults):
                return False
            if (
                time.perf_counter() >= t0_latch + ramp_s
                and all(faults[s] == 1 for s in armed)
            ):
                ok = True
                break
            time.sleep(0.04)
        return ok

    def _recover_rearm() -> None:
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        session.set_actuators(_blank(), send=False)
        time.sleep(0.12)
        try:
            hub.recover()
        except Exception as exc:
            print(f"  recover warn: {exc}", flush=True)
        time.sleep(0.2)
        _cfg_arm_slots(hub, armed)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        _seed_hold(SEED_S)

    for i in range(7):
        attempts = 2 if i == 3 else 1
        ok = False
        for attempt in range(attempts):
            armed.add(i)
            _cfg_arm_slots(hub, armed)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            _seed_hold(SEED_S + (0.12 if i == 3 else 0.0))
            ramp = LATCH_RAMP_S + (0.2 if i == 3 else 0.0)
            hold = LATCH_HOLD_S + (0.2 if i == 3 else 0.0)
            ok = _latch_armed(ramp_s=ramp, hold_s=hold)
            print(
                f"  armed={sorted(armed)} faults={_arm_faults(session)} "
                f"ok={ok} try={attempt+1}/{attempts}",
                flush=True,
            )
            if ok:
                break
            print(f"  J{i+1} not green — recover + re-arm", flush=True)
            _recover_rearm()
        if not ok:
            print(f"WARN: J{i+1} still not green", flush=True)

    faults = _arm_faults(session)
    bad = [s for s in range(7) if faults[s] != 1]
    if bad:
        print(f"  final green pass: retry {bad}", flush=True)
        armed = set(range(7))
        _recover_rearm()
        _latch_armed(ramp_s=LATCH_RAMP_S + 0.3, hold_s=LATCH_HOLD_S + 0.3)

    for _ in range(5):
        fb = _read_arm(session)
        if fb is not None:
            for s in range(7):
                if abs(float(fb[s])) > 1e-3:
                    q0[s] = float(fb[s])
        _write_arm(session, q0, kp_scale=LATCH_KP_SCALE)
        time.sleep(0.04)

    faults = _arm_faults(session)
    print(
        f"arm home(FB)={np.array2string(q0, precision=3)} faults={faults} "
        f"latch_s={time.perf_counter() - t_latch:.1f}",
        flush=True,
    )
    if not all(f == 1 for f in faults):
        print("ABORT: not all MIT-green", file=sys.stderr)
        raise SystemExit(4)
    if _hard(faults):
        print("ABORT: hard fault", file=sys.stderr)
        raise SystemExit(5)

    print(f"\n== SOFT ENGAGE -> kp x{ENGAGE_KP} ==", flush=True)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < ENGAGE_S:
        fb = _read_arm(session)
        if fb is not None:
            for s in range(7):
                if abs(float(fb[s])) > 1e-3:
                    q0[s] = 0.95 * float(q0[s]) + 0.05 * float(fb[s])
        u = (time.perf_counter() - t0) / ENGAGE_S
        s = u * u * (3.0 - 2.0 * u)
        scale = LATCH_KP_SCALE + (ENGAGE_KP - LATCH_KP_SCALE) * s
        _write_arm(session, q0, kp_scale=scale)
        time.sleep(0.02)
    try:
        session.set_led(TELEOP_LED, send=True)
    except Exception as exc:
        print(f"  led warn: {exc}", flush=True)
    return q0


def _attach_double_right(state: MouseTeleopState) -> None:
    """Monkey-patch click handler to flag double-right while middle held."""
    state.j7_toggle_pending = False  # type: ignore[attr-defined]
    state._right_press_t = 0.0  # type: ignore[attr-defined]
    orig = state.on_click

    def on_click(x: int, y: int, button: mouse.Button, pressed: bool) -> None:
        orig(x, y, button, pressed)
        if button != mouse.Button.right or not pressed:
            return
        with state.lock:
            if not state.middle:
                state._right_press_t = 0.0  # type: ignore[attr-defined]
                return
            now = time.time()
            prev = float(getattr(state, "_right_press_t", 0.0))
            if prev > 0.0 and (now - prev) <= DOUBLE_RIGHT_S:
                state.j7_toggle_pending = True  # type: ignore[attr-defined]
                state._right_press_t = 0.0  # type: ignore[attr-defined]
            else:
                state._right_press_t = now  # type: ignore[attr-defined]

    state.on_click = on_click  # type: ignore[method-assign]


class MouseArmRuntime:
    """One-tick M650 → arm hold/cmd (shared by mouse_arm_teleop + continuous).

    Middle gates motion only; hold stays frozen on release (no settle→FB).
    """

    def __init__(self, hold: np.ndarray) -> None:
        self.hold = np.asarray(hold, dtype=np.float32).copy()
        self.cmd = self.hold.copy()
        self.dq = np.zeros(7, dtype=np.float32)
        self.scroll_hold = 0.0
        self.last_mode = ""
        self.j7_slew_from: Optional[float] = None
        self.j7_slew_to: Optional[float] = None
        self.j7_slew_t0 = 0.0
        self.ignore_right_until = 0.0
        self.active: Set[int] = set()
        self.deadman = False
        self.vert_mode = "-"
        self.left_hold = False
        self.right_hold = False
        self.thumb = False

    def step(
        self,
        state: MouseTeleopState,
        *,
        fb: Optional[np.ndarray],
        dt: float,
        log_axis: bool = True,
    ) -> None:
        sample = state.sample()
        with state.lock:
            deadman = state.middle
            left_hold = state.left
            right_hold = state.right
            thumb = state.thumb_back or state.thumb_fwd
            scroll_notches = state.scroll_notches
            state.scroll_notches = 0.0
            j7_toggle = bool(getattr(state, "j7_toggle_pending", False))
            if j7_toggle:
                state.j7_toggle_pending = False  # type: ignore[attr-defined]

        self.deadman = deadman
        self.left_hold = left_hold
        self.right_hold = right_hold
        self.thumb = thumb

        axis_joint = None
        if deadman:
            if left_hold or thumb:
                axis_joint = J4
            elif right_hold and time.time() >= self.ignore_right_until:
                axis_joint = J3
            else:
                axis_joint = J2

        mode = (
            "J4"
            if axis_joint == J4
            else ("J3" if axis_joint == J3 else ("J2" if axis_joint == J2 else "-"))
        )
        if log_axis and mode != self.last_mode:
            print(
                f"  vert axis -> {mode}  "
                f"(left={int(left_hold)} right={int(right_hold)} "
                f"thumb={int(thumb)})",
                flush=True,
            )
            self.last_mode = mode
        self.vert_mode = mode

        self.dq[:] = 0.0
        active: Set[int] = set()
        self.cmd[:] = self.hold

        if j7_toggle and deadman and fb is not None:
            lo, hi = _j7_ends()
            cur = float(fb[J7])
            target = hi if abs(cur - lo) <= abs(cur - hi) else lo
            self.j7_slew_from = float(self.hold[J7])
            self.j7_slew_to = target
            self.j7_slew_t0 = time.perf_counter()
            self.ignore_right_until = time.time() + 0.45
            print(f"  J7 toggle {cur:+.3f} -> {target:+.3f}", flush=True)

        if self.j7_slew_to is not None and self.j7_slew_from is not None:
            u = min(
                1.0,
                (time.perf_counter() - self.j7_slew_t0) / max(J7_SLEW_S, 1e-3),
            )
            s = u * u * (3.0 - 2.0 * u)
            self.cmd[J7] = float(self.j7_slew_from) + (
                float(self.j7_slew_to) - float(self.j7_slew_from)
            ) * s
            self.cmd[J7] = _clamp_joint(J7, float(self.cmd[J7]))
            self.hold[J7] = float(self.cmd[J7])
            self.dq[J7] = math.copysign(
                RATE_J7,
                float(self.j7_slew_to) - float(self.j7_slew_from),
            ) * (1.0 if u < 0.95 else 0.0)
            active.add(J7)
            if u >= 1.0:
                self.j7_slew_from = None
                self.j7_slew_to = None

        if deadman and fb is not None:
            sx = float(np.clip(sample.r_stick[0], -1.0, 1.0))
            sy = float(np.clip(sample.r_stick[1], -1.0, 1.0))

            r1 = RATE_J1 * sx
            if abs(r1) > 1e-4:
                self.cmd[J1] = _clamp_joint(J1, float(self.hold[J1]) + r1 * dt)
                self.hold[J1] = float(self.cmd[J1])
                self.dq[J1] = r1
                active.add(J1)

            rates = {J2: RATE_J2, J3: RATE_J3, J4: RATE_J4}
            if axis_joint is not None and axis_joint in rates:
                sy_cmd = -sy if axis_joint == J2 else sy
                rj = rates[axis_joint] * sy_cmd
                if abs(rj) > 1e-4:
                    self.cmd[axis_joint] = _clamp_joint(
                        axis_joint, float(self.hold[axis_joint]) + rj * dt
                    )
                    lim = J2_MAX_LEAD if axis_joint == J2 else MAX_LEAD
                    lead = float(self.cmd[axis_joint]) - float(fb[axis_joint])
                    if abs(lead) > lim:
                        self.cmd[axis_joint] = float(fb[axis_joint]) + math.copysign(
                            lim, lead
                        )
                    self.hold[axis_joint] = float(self.cmd[axis_joint])
                    self.dq[axis_joint] = rj
                    active.add(axis_joint)

            if abs(scroll_notches) > 0:
                self.scroll_hold = float(np.clip(scroll_notches, -3.0, 3.0)) / 3.0
            self.scroll_hold *= 0.88
            r5 = RATE_J5 * self.scroll_hold
            if abs(r5) > 1e-4:
                self.cmd[J5] = _clamp_joint(J5, float(self.hold[J5]) + r5 * dt)
                self.hold[J5] = float(self.cmd[J5])
                self.dq[J5] = r5
                active.add(J5)

        for i in range(7):
            if i not in active:
                self.cmd[i] = float(self.hold[i])
                self.dq[i] = 0.0
        self.active = active
        self._last_sample = sample  # type: ignore[attr-defined]

    @property
    def stick(self) -> tuple[float, float]:
        s = getattr(self, "_last_sample", None)
        if s is None:
            return (0.0, 0.0)
        return (float(s.r_stick[0]), float(s.r_stick[1]))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None, help="CDC port (default: auto)")
    ap.add_argument("--hz", type=float, default=STREAM_HZ)
    ap.add_argument(
        "--debug",
        action="store_true",
        help="Print hold-fb error / vel / tau when a joint looks like it may buzz",
    )
    args = ap.parse_args(argv)

    port = args.port or find_cdc_port()
    if not port:
        print("No Controls PCB CDC found. Plug board into this laptop.", file=sys.stderr)
        return 2
    print(f"CDC port={port}", flush=True)

    state = MouseTeleopState(
        mode="joystick",
        stick_radius_px=STICK_RADIUS_PX,
        stick_deadzone=STICK_DEADZONE,
        z_scale=1.0,
    )
    _attach_double_right(state)

    listener = mouse.Listener(
        on_move=state.on_move,
        on_click=state.on_click,
        on_scroll=state.on_scroll,
    )
    listener.start()

    try:
        with PcbRobotSession.connect(
            port, apply_yam_cfg=False, stream_hz=float(args.hz)
        ) as session:
            q0 = progressive_latch(session)
            rt = MouseArmRuntime(q0)
            print(
                "\n== MOUSE JOYSTICK ==\n"
                "  MIDDLE = enable / stick center\n"
                "  L/R = J1 | U/D = J2 | U/D+RIGHT = J3 | U/D+LEFT = J4\n"
                "  (thumbs also map to J4 if OS reports them)\n"
                "  double-right = J7 open/close | scroll = J5 | Ctrl+C quit\n"
                f"  hold=always (middle only gates motion) | J2 kp x{J2_KP_SCALE} | "
                f"stick_r={STICK_RADIUS_PX}px",
                flush=True,
            )
            dt = 1.0 / max(float(args.hz), 1.0)
            n = 0
            last_status = 0.0
            try:
                while listener.running:
                    t0 = time.perf_counter()
                    detail = _read_arm_detail(session)
                    if detail is None:
                        fb = None
                        vel = np.zeros(7, dtype=np.float32)
                        tau = np.zeros(7, dtype=np.float32)
                        faults = [0] * 7
                    else:
                        fb, vel, tau, faults = detail
                    if _hard(faults):
                        print(f"HARD fault {faults} — blanking", flush=True)
                        session.set_actuators(_blank(), send=False)
                        return 6

                    rt.step(state, fb=fb, dt=dt)
                    _write_arm(
                        session,
                        rt.cmd,
                        kp_scale=ENGAGE_KP,
                        dq=rt.dq,
                        j2_boost=(J2 in rt.active),
                    )

                    n += 1
                    now = time.time()

                    # Only flag while actively driving + moving. Idle err≈tau/kp
                    # (~0.12 rad at ~10 Nm / kp84) is normal static droop, not buzz.
                    if args.debug and fb is not None and n % 15 == 0:
                        for i in (J1, J2, J3, J4, J7):
                            if i not in rt.active:
                                continue
                            err = float(rt.hold[i] - fb[i])
                            if abs(err) >= BUZZ_ERR and abs(float(vel[i])) >= BUZZ_VEL:
                                print(
                                    f"  BUZZ? J{i+1} err={err:+.3f} "
                                    f"vel={vel[i]:+.3f} tau={tau[i]:+.2f} "
                                    f"kp~{DEFAULT_ARM_KP[i]*(J2_KP_SCALE if i==J2 else ENGAGE_KP):.0f}",
                                    flush=True,
                                )

                    if now - last_status > 1.0:
                        last_status = now
                        qshow = fb if fb is not None else rt.cmd
                        sx, sy = rt.stick
                        j2_err = float(rt.hold[J2] - qshow[J2])
                        print(
                            f"#{n} en={int(rt.deadman)} vert={rt.vert_mode} "
                            f"L={int(rt.left_hold)} R={int(rt.right_hold)} "
                            f"th={int(rt.thumb)} "
                            f"stick=({sx:+.2f},{sy:+.2f}) "
                            f"J2_hold={rt.hold[J2]:+.3f} J2_fb={qshow[J2]:+.3f} "
                            f"J2_err={j2_err:+.3f} "
                            f"J4_hold={rt.hold[J4]:+.3f} J4_fb={qshow[J4]:+.3f} "
                            f"faults={faults}",
                            flush=True,
                        )
                    elapsed = time.perf_counter() - t0
                    time.sleep(max(0.0, dt - elapsed))
            except KeyboardInterrupt:
                print("\nCtrl+C — blanking", flush=True)

            session.set_actuators(_blank(), send=False)
            time.sleep(0.3)
            session.hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    finally:
        listener.stop()

    print("mouse_arm_teleop done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
