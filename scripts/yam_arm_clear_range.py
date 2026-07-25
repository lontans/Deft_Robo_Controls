#!/usr/bin/env python3
"""Operator-supervised left-arm clear-range characterization (bus 1 / 7 Damiao).

    python yam_arm_clear_range.py --apply-cfg
    python yam_arm_clear_range.py --apply-cfg --joint 5   # J6 only
    python yam_arm_clear_range.py --dry-run

Owns COM exclusively — close the dashboard / other scripts first.

Bring-up (exact path from ``_tmp_yam_i2rt_prove.py``):
  DIAG → CFG → recover → NORMAL → ``_write_mit`` kp=0 → freeze FB →
  soft-engage kp_scale 0→1 → ``_go_to`` jogs (setpoint + vel FF).
Uses ``session.set_actuators`` directly — not ``arm.write`` / ``arm.go_to``.

Conservative sweeps: Enter *before* contact; last safe FB kept; inset applied.
Skips Damiao discover by default (REG_SCAN flood faults drives). Optional
``--discover`` = one light ID_SWEEP only. Exit leaves DIAG + cornflower idle.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import date
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, LedDesire, McuState  # noqa: E402
from deft_controls_sdk.bench.soft_dfu import find_cdc_port  # noqa: E402
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT  # noqa: E402
from deft_controls_sdk.vbeta import (  # noqa: E402
    PcbArmDriver,
    PcbRobotSession,
    ensure_yam_left_arm_cfg,
)
from deft_controls_sdk.vbeta.cfg import pause_plant_stream  # noqa: E402
from deft_controls_sdk.vbeta.slots import DEFAULT_ARM_KD, DEFAULT_ARM_KP  # noqa: E402
from deft_controls_sdk.vbeta.yam_limits import (  # noqa: E402
    DEFAULT_CLEAR_INSET,
    apply_clear_inset,
)

_BENCH_MODULE = (
    _SCRIPTS / "deft_controls_sdk" / "vbeta" / "yam_bench_clear_left.py"
)
_SESSION_DIR = _SCRIPTS / ".deft_session"
_NOMINAL_ESC = tuple(range(0x01, 0x08))

# Loaded-bench gains (vbeta DEFAULT_ARM_* — J2 needs high kp under gravity).
_CLEAR_KP: Tuple[float, ...] = tuple(DEFAULT_ARM_KP)
_CLEAR_KD = float(DEFAULT_ARM_KD)
_DEFAULT_STREAM_HZ = 20.0
_J2_INDEX = 1


def _fmt_q(q: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(v):+.4f}" for v in q) + "]"


def apply_clear_inset_q7(
    edge_lo: Sequence[float],
    edge_hi: Sequence[float],
    home: Sequence[float],
    *,
    inset: float,
) -> Tuple[np.ndarray, np.ndarray]:
    lo = np.zeros(7, dtype=np.float64)
    hi = np.zeros(7, dtype=np.float64)
    for i in range(7):
        lo[i], hi[i] = apply_clear_inset(
            float(edge_lo[i]),
            float(edge_hi[i]),
            inset=inset,
            home=float(home[i]),
        )
    return lo, hi


def render_bench_module(
    *,
    lo: Sequence[float],
    hi: Sequence[float],
    home: Sequence[float],
    source: str,
    inset: float,
    step: float,
) -> str:
    def _t(vals: Sequence[float]) -> str:
        return "(" + ", ".join(f"{float(v):.6f}" for v in vals) + ")"

    return f'''"""Left-arm motor-frame clear envelope for the current bench (bus 1 / slots 0–6).

Filled by ``scripts/yam_arm_clear_range.py`` after operator-supervised sweeps.
Until ``CLEAR_ACTIVE`` is True, ``yam_limits`` ignores this module.
"""
from __future__ import annotations

from typing import Optional, Tuple

CLEAR_ACTIVE = True

CLEAR_LO: Tuple[float, ...] = {_t(lo)}
CLEAR_HI: Tuple[float, ...] = {_t(hi)}
HOME_Q: Tuple[float, ...] = {_t(home)}

SOURCE = {source!r}
INSET_RAD = {float(inset):.6f}
STEP_RAD = {float(step):.6f}


def clear_q7() -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    """Return ``(lo, hi)`` when active, else None."""
    if not CLEAR_ACTIVE:
        return None
    if len(CLEAR_LO) != 7 or len(CLEAR_HI) != 7:
        return None
    return tuple(CLEAR_LO), tuple(CLEAR_HI)
'''


def _wait_live_fb(arm: PcbArmDriver, *, timeout_s: float = 8.0) -> np.ndarray:
    """Require non-trivial motion FB on at least one joint (power + CFG).

    Read-only — does **not** write Goal from FB (that floats/buzzes wrists).
    """
    deadline = time.perf_counter() + timeout_s
    last = np.zeros(7, dtype=np.float32)
    while time.perf_counter() < deadline:
        q = np.asarray(arm.read("Position_Rad"), dtype=np.float32).reshape(7)
        last = q
        if float(np.max(np.abs(q))) > 1e-3:
            return q
        time.sleep(0.05)
    raise RuntimeError(
        "no live Damiao FB on left arm (all ~0) — check 24 V, CFG, ESC IDs. "
        f"last={_fmt_q(last)}"
    )


# Prove script only MIT-enables J1–J6; J7 blank. Clear-range historically enabled all 7.
_ACTIVE_PROVE = (0, 1, 2, 3, 4, 5)  # J1..J6
_ACTIVE_ALL7 = (0, 1, 2, 3, 4, 5, 6)


def _write_mit(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    dq: Optional[np.ndarray] = None,
    kp_scale: float = 1.0,
    kd: Optional[float] = None,
    active: Sequence[int] = _ACTIVE_ALL7,
    kp_scale_per_joint: Optional[Sequence[float]] = None,
) -> None:
    """Same as ``_tmp_yam_i2rt_prove.write_active`` — direct set_actuators, not arm.write.

    Inactive slots get blank ``ActuatorDesire()`` (prove blanks J7).
    Default ``kd``: full ``_CLEAR_KD`` when any kp>0; else 0 (non-commanding).
    Pass ``kd=_CLEAR_KD`` explicitly for kp=0 damping holds at a *known* q.
    """
    q = np.asarray(q, dtype=np.float32).reshape(7)
    vel = (
        np.zeros(7, dtype=np.float32)
        if dq is None
        else np.asarray(dq, dtype=np.float32).reshape(7)
    )
    scale = float(np.clip(kp_scale, 0.0, 1.0))
    active_set = {int(i) for i in active}
    desires = {}
    for i, slot in enumerate(arm.slots):
        if i not in active_set:
            desires[slot] = ActuatorDesire()
            continue
        js = scale
        if kp_scale_per_joint is not None:
            js = float(np.clip(kp_scale_per_joint[i], 0.0, 1.0))
        kp = float(_CLEAR_KP[i]) * js
        if kd is not None:
            kd_i = float(kd)
        else:
            kd_i = float(_CLEAR_KD) if kp > 0.01 else 0.0
        desires[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=float(vel[i]),
            kp=kp,
            kd=kd_i,
        )
    session.set_actuators(desires, send=False)
    arm._setpoint = q.copy()  # noqa: SLF001


def _blank_all(session: PcbRobotSession) -> None:
    session.set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )


def _acquire_home_fb(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    *,
    active: Sequence[int] = _ACTIVE_PROVE,
    settle_s: float = 0.8,
) -> np.ndarray:
    """Get live FB without holding Goal=0.

    Bug: ``write_mit(zeros, kp=0, kd=full)`` still *enables* Damiao (kd>0.01) and
    MIT-commands position=0 for the whole FB wait — J6 drifts/buzzes before E0.
    Instead: read FB first; only MIT-enable after a real pose is seen (Goal=FB).
    """
    q = np.zeros(7, dtype=np.float32)
    got = False
    deadline = time.perf_counter() + settle_s
    # Phase A: poke enable briefly only until first non-zero FB, then Goal=FB.
    while time.perf_counter() < deadline and not got:
        _write_mit(session, arm, q, kp_scale=0.0, kd=0.5, active=active)
        time.sleep(0.04)
        fb = np.asarray(arm.read("Position_Rad"), dtype=np.float32).reshape(7)
        if float(np.max(np.abs(fb[list(active)]))) > 1e-3:
            q = fb.copy()
            got = True
            break
    if not got:
        raise RuntimeError(
            "no live Damiao FB during acquire — check 24 V, CFG, ESC IDs. "
            f"last={_fmt_q(q)}"
        )
    # Phase B: hold Goal=FB (still kp=0); allow encoder to settle, no chase after.
    t1 = time.perf_counter() + 0.35
    while time.perf_counter() < t1:
        _write_mit(session, arm, q, kp_scale=0.0, kd=0.5, active=active)
        time.sleep(0.05)
    _write_mit(session, arm, q, kp_scale=0.0, kd=0.5, active=active)
    return q


def _hold_stream(
    arm: PcbArmDriver,
    session: PcbRobotSession,
    q: np.ndarray,
    hold_s: float,
    *,
    active: Sequence[int] = _ACTIVE_PROVE,
) -> None:
    """Hold fixed setpoint via prove-style MIT write (no FB re-seed)."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    t_end = time.perf_counter() + hold_s
    while time.perf_counter() < t_end:
        if session.service_soft_kill():
            raise RuntimeError("soft-kill park during characterize — aborting")
        _write_mit(
            session, arm, q, kp_scale=1.0, kd=_CLEAR_KD, active=active
        )
        time.sleep(0.05)


def _prompt_hold(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    prompt: str,
    *,
    refresh_mit: bool = True,
    kp_scale: float = 1.0,
    kd: Optional[float] = None,
    active: Sequence[int] = _ACTIVE_ALL7,
    kp_scale_per_joint: Optional[Sequence[float]] = None,
    blank: bool = False,
) -> None:
    """Wait for Enter; optionally refresh frozen MIT each tick."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    print(prompt, end="", flush=True)
    stop, _th = _stdin_stop_flag()
    while not stop.is_set():
        if session.service_soft_kill():
            raise RuntimeError("soft-kill while waiting at prompt — aborting")
        if blank:
            _blank_all(session)
        elif refresh_mit:
            _write_mit(
                session,
                arm,
                q,
                kp_scale=kp_scale,
                kd=kd,
                active=active,
                kp_scale_per_joint=kp_scale_per_joint,
            )
        time.sleep(0.05)
    print(flush=True)


def _stage_banner(n: object, title: str, detail: str) -> None:
    print(
        f"\n========== STAGE {n}: {title} ==========\n"
        f"  {detail}\n"
        f"  Listen for J6 buzz NOW. Enter = next stage.\n",
        flush=True,
    )


def _write_mit_except_j6(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    kp_scale: float = 1.0,
) -> None:
    """Full MIT on J1–J5,J7; J6 kp=0 (kd kept) — isolate whether J6 gain is the buzz."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    scale = float(np.clip(kp_scale, 0.0, 1.0))
    desires = {}
    for i, slot in enumerate(arm.slots):
        kp = 0.0 if i == 5 else float(_CLEAR_KP[i]) * scale
        desires[slot] = ActuatorDesire(
            position=float(q[i]),
            velocity=0.0,
            kp=kp,
            kd=_CLEAR_KD,
        )
    session.set_actuators(desires, send=False)
    arm._setpoint = q.copy()  # noqa: SLF001


def _ramp_kp(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    scale_from: float,
    scale_to: float,
    engage_s: float,
    active: Sequence[int],
    kp_scale_per_joint_to: Optional[Sequence[float]] = None,
) -> None:
    """Smoothstep kp_scale from→to at frozen q (prove soft_engage slice)."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / max(engage_s, 1e-3)
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        scale = float(scale_from) + (float(scale_to) - float(scale_from)) * s
        per = None
        if kp_scale_per_joint_to is not None:
            per = [
                float(scale_from) + (float(t) - float(scale_from)) * s
                for t in kp_scale_per_joint_to
            ]
        _write_mit(
            session,
            arm,
            q,
            kp_scale=scale,
            kd=_CLEAR_KD,
            active=active,
            kp_scale_per_joint=per,
        )
        time.sleep(0.02)
    _write_mit(
        session,
        arm,
        q,
        kp_scale=scale_to,
        kd=_CLEAR_KD,
        active=active,
        kp_scale_per_joint=kp_scale_per_joint_to,
    )


def _buzz_locate_stages(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    home: np.ndarray,
) -> None:
    """Enter-gated engage partitions — buzz was reported before post-engage locator."""
    q = np.asarray(home, dtype=np.float32).reshape(7)
    print(
        "\n*** J6 buzz locator (engage stages) ***\n"
        "Bring-up B-stages already ran. Tell me first STAGE where J6 buzzes.\n",
        flush=True,
    )

    # E0 — kd-only / kp=0 at frozen home (correct q — never zeros).
    _stage_banner(
        "E0",
        "kp=0 + full kd hold (J1–J6)",
        "Frozen home, kp=0, kd=full, J7 blank. Prove-style pre-engage hold.",
    )
    _write_mit(
        session, arm, q, kp_scale=0.0, kd=_CLEAR_KD, active=_ACTIVE_PROVE
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E0 — Enter when done… ",
        refresh_mit=True,
        kp_scale=0.0,
        kd=_CLEAR_KD,
        active=_ACTIVE_PROVE,
    )

    # E1 — prove path: J1–J6 only, ramp 0→1 (J7 blank).
    _stage_banner(
        "E1",
        "prove soft-engage J1–J6 (J7 blank)",
        "Exact tmp prove: ACTIVE=J1..J6, kp_scale 0→1 over 1.4s, J7 blank Desire.",
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E1 — Enter to START prove-style engage… ",
        refresh_mit=True,
        kp_scale=0.0,
        kd=_CLEAR_KD,
        active=_ACTIVE_PROVE,
    )
    print("  ramping J1–J6 kp 0→1 (J7 off)…", flush=True)
    _ramp_kp(
        session,
        arm,
        q,
        scale_from=0.0,
        scale_to=1.0,
        engage_s=1.4,
        active=_ACTIVE_PROVE,
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E1 — engaged J1–J6. Enter when done listening… ",
        refresh_mit=True,
        kp_scale=1.0,
        active=_ACTIVE_PROVE,
    )

    # E2 — J7 enable (KNOWN: snap full-kp J7 buzzes J6). Try soft ramp.
    _stage_banner(
        "E2",
        "soft-engage J7 (not snap)",
        "KNOWN: snap-enabling J7 @ full kp buzzes J6. Ramping J7 kp 0→1 over 2.5s.",
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E2 — Enter to soft-engage J7… ",
        refresh_mit=True,
        kp_scale=1.0,
        active=_ACTIVE_PROVE,
    )
    _soft_engage_j7(session, arm, q, engage_s=2.5)
    _prompt_hold(
        session,
        arm,
        q,
        "E2 — J7 soft-engaged. Enter when done (buzz? )… ",
        refresh_mit=True,
        kp_scale=1.0,
        active=_ACTIVE_ALL7,
    )
    print("  returning to J7-blank (prove hold)…", flush=True)
    _blank_j7_keep_arm(session, arm, q)
    _prompt_hold(
        session,
        arm,
        q,
        "E2b — J7 blank again. Enter to continue locator (or note E2)… ",
        refresh_mit=True,
        kp_scale=1.0,
        active=_ACTIVE_PROVE,
    )

    # E3 — J6 kp=0, others (incl J7) full.
    _stage_banner(
        "E3",
        "J6 kp=0 (others full)",
        "If buzz STOPS here, J6 gain/pack is the cause.",
    )
    scales = [1.0] * 7
    scales[5] = 0.0
    _write_mit(
        session,
        arm,
        q,
        kd=_CLEAR_KD,
        active=_ACTIVE_ALL7,
        kp_scale_per_joint=scales,
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E3 — Enter when done… ",
        refresh_mit=True,
        kd=_CLEAR_KD,
        active=_ACTIVE_ALL7,
        kp_scale_per_joint=scales,
    )

    # E4 — J6 at 0.5 kp.
    _stage_banner(
        "E4",
        "J6 kp×0.5",
        "Restore J6 to half DEFAULT_ARM_KP (35); others stay full.",
    )
    scales = [1.0] * 7
    scales[5] = 0.5
    _prompt_hold(
        session,
        arm,
        q,
        "E4 — Enter to set J6×0.5… ",
        refresh_mit=True,
        active=_ACTIVE_ALL7,
        kp_scale_per_joint=[1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0],
    )
    print("  setting J6×0.5…", flush=True)
    _write_mit(session, arm, q, active=_ACTIVE_ALL7, kp_scale_per_joint=scales)
    _prompt_hold(
        session,
        arm,
        q,
        "E4 — J6×0.5 holding. Enter when done… ",
        refresh_mit=True,
        active=_ACTIVE_ALL7,
        kp_scale_per_joint=scales,
    )

    # E5 — J6 full again.
    _stage_banner(
        "E5",
        "J6 kp×1.0 (full)",
        "Ramp J6 0.5→1.0 — classic full-kp buzz candidate.",
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E5 — Enter to ramp J6 to full… ",
        refresh_mit=True,
        active=_ACTIVE_ALL7,
        kp_scale_per_joint=scales,
    )
    scales_full = [1.0] * 7
    print("  ramping J6 → full…", flush=True)
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / 1.0
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        per = [1.0] * 7
        per[5] = 0.5 + 0.5 * s
        _write_mit(session, arm, q, active=_ACTIVE_ALL7, kp_scale_per_joint=per)
        time.sleep(0.02)
    _write_mit(session, arm, q, active=_ACTIVE_ALL7, kp_scale_per_joint=scales_full)
    _prompt_hold(
        session,
        arm,
        q,
        "E5 — full kp all 7. Enter when done… ",
        refresh_mit=True,
        active=_ACTIVE_ALL7,
        kp_scale_per_joint=scales_full,
    )

    # E6 — stream-only (no host rewrite) at full.
    _stage_banner(
        "E6",
        "stream-only at full kp",
        "Stop host rewrite; stream repeats last full-kp image.",
    )
    _prompt_hold(
        session,
        arm,
        q,
        "E6 — Enter when done (end of locator)… ",
        refresh_mit=False,
    )
    print(
        f"buzz-locator done. faults={_faults_q7(arm)}\n"
        "Report first STAGE where J6 buzzed (E0–E6).\n",
        flush=True,
    )


def _soft_engage(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    engage_s: float = 1.4,
    active: Sequence[int] = _ACTIVE_PROVE,
) -> None:
    """Exact copy of tmp prove soft_engage: kp_scale 0→1 at frozen q."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    print(
        f"soft-engage MIT over {engage_s:.1f}s at fixed setpoint (no FB chase)…",
        flush=True,
    )
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / max(engage_s, 1e-3)
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        _write_mit(session, arm, q, kp_scale=s, kd=_CLEAR_KD, active=active)
        time.sleep(0.02)
    _write_mit(session, arm, q, kp_scale=1.0, kd=_CLEAR_KD, active=active)


def _go_to(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    target: np.ndarray,
    dt: float,
    *,
    active: Sequence[int] = _ACTIVE_PROVE,
) -> None:
    """Exact copy of tmp prove go_to_active: smoothstep + vel ff from setpoint."""
    start = arm._setpoint.copy()  # noqa: SLF001
    target = np.asarray(target, dtype=np.float32).reshape(7)
    delta = target - start
    dt = max(float(dt), 1e-3)
    t0 = time.perf_counter()
    while True:
        if session.service_soft_kill():
            raise RuntimeError("soft-kill during go_to — aborting")
        u = (time.perf_counter() - t0) / dt
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        ds_du = 6.0 * u * (1.0 - u)
        q = start + delta * np.float32(s)
        dq = (delta / np.float32(dt)) * np.float32(ds_du)
        _write_mit(
            session, arm, q, dq=dq, kp_scale=1.0, kd=_CLEAR_KD, active=active
        )
        time.sleep(0.01)
    _write_mit(
        session, arm, target, kp_scale=1.0, kd=_CLEAR_KD, active=active
    )


def _soft_engage_j7(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
    *,
    engage_s: float = 2.5,
) -> None:
    """Ramp J7 kp 0→1 while J1–J6 stay full. Snap-enabling J7 buzzes J6 (E2)."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    print(f"soft-engage J7 over {engage_s:.1f}s (J1–J6 held full)…", flush=True)
    t0 = time.perf_counter()
    while True:
        u = (time.perf_counter() - t0) / max(engage_s, 1e-3)
        if u >= 1.0:
            break
        s = u * u * (3.0 - 2.0 * u)
        per = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, float(s)]
        _write_mit(
            session,
            arm,
            q,
            kd=_CLEAR_KD,
            active=_ACTIVE_ALL7,
            kp_scale_per_joint=per,
        )
        time.sleep(0.02)
    _write_mit(
        session,
        arm,
        q,
        kp_scale=1.0,
        kd=_CLEAR_KD,
        active=_ACTIVE_ALL7,
    )


def _blank_j7_keep_arm(
    session: PcbRobotSession,
    arm: PcbArmDriver,
    q: np.ndarray,
) -> None:
    """Drop J7 MIT (prove-style) after a J7 sweep — stops J6 buzz from E2 coupling."""
    q = np.asarray(q, dtype=np.float32).reshape(7)
    print("  blanking J7 MIT (J1–J6 stay)…", flush=True)
    _write_mit(session, arm, q, kp_scale=1.0, kd=_CLEAR_KD, active=_ACTIVE_PROVE)


def _faults_q7(arm: PcbArmDriver) -> List[int]:
    fb = arm._session.latest_feedback()  # noqa: SLF001
    out: List[int] = []
    for slot in arm.slots:
        st = fb.actuator(slot) if fb else None
        out.append(int(st.fault) if st else -1)
    return out


def _idle_mcu(hub, session: PcbRobotSession) -> None:
    """DIAG_ONLY + blank desires + cornflower (plant CAN gated)."""
    hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
    session.set_actuators(
        {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
    )
    hub.set_led(
        LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8), send=False
    )
    session.send_once()


# Slow return after a range edge — snap-to-home with MIT kp yanks the arm.
_DEFAULT_HOME_RATE_RAD_S = 0.12  # ~8 s for a 1 rad recovery
_HOME_RETURN_MIN_S = 3.0
_HOME_RETURN_MAX_S = 14.0


def _return_home_slow(
    arm: PcbArmDriver,
    session: PcbRobotSession,
    home: np.ndarray,
    *,
    rate_rad_s: float = _DEFAULT_HOME_RATE_RAD_S,
    active: Sequence[int] = _ACTIVE_PROVE,
) -> None:
    """Smoothstep ramp setpoint → home (tmp prove go_to), then hold."""
    start = np.asarray(arm._setpoint, dtype=np.float64).reshape(7)  # noqa: SLF001
    target = np.asarray(home, dtype=np.float64).reshape(7)
    dist = float(np.max(np.abs(target - start)))
    rate = max(0.04, float(rate_rad_s))
    dt = min(_HOME_RETURN_MAX_S, max(_HOME_RETURN_MIN_S, dist / rate))
    print(
        f"  returning home smoothly (Δmax={dist:.3f} rad over {dt:.1f}s)…",
        flush=True,
    )
    _go_to(session, arm, target.astype(np.float32), dt, active=active)
    _hold_stream(arm, session, target.astype(np.float32), 0.8, active=active)


def _stdin_stop_flag() -> Tuple[threading.Event, threading.Thread]:
    stop = threading.Event()

    def _reader() -> None:
        try:
            sys.stdin.readline()
        except Exception:
            pass
        stop.set()

    th = threading.Thread(target=_reader, daemon=True)
    th.start()
    return stop, th


def _jog_direction(
    arm: PcbArmDriver,
    session: PcbRobotSession,
    *,
    home: np.ndarray,
    joint: int,
    sign: int,
    step: float,
    dwell_s: float,
    max_steps: int,
    home_rate: float = _DEFAULT_HOME_RATE_RAD_S,
    active: Sequence[int] = _ACTIVE_PROVE,
) -> float:
    """Step joint until Enter; return last safe FB (before the stop step).

    Motion matches ``_tmp_yam_i2rt_prove.go_to_active``: setpoint-relative
    smoothstep + vel FF. Next goal advances from *command*, never live FB
    (FB-chase after each micro-step was exciting J6).
    """
    home = np.asarray(home, dtype=np.float32).reshape(7)
    q = home.copy()
    cmd = float(home[joint])
    last_safe = cmd
    direction = "plus" if sign > 0 else "minus"
    print(
        f"\n--- J{joint + 1} {direction}: stepping {sign * step:+.3f} rad. "
        f"Press Enter BEFORE contact (last safe kept). ---",
        flush=True,
    )
    stop, _th = _stdin_stop_flag()
    # Prove pace: J1 Δ0.10 over 1.2s ≈ 0.083 rad/s.
    step_dt = max(1.0, abs(float(step)) / 0.08)
    for i in range(max_steps):
        if stop.is_set():
            break
        cmd = cmd + sign * float(step)
        q[joint] = cmd
        _go_to(session, arm, q, step_dt, active=active)
        t_end = time.perf_counter() + max(0.25, float(dwell_s))
        while time.perf_counter() < t_end:
            if session.service_soft_kill():
                raise RuntimeError("soft-kill during jog — aborting")
            if stop.is_set():
                break
            _write_mit(
                session, arm, q, kp_scale=1.0, kd=_CLEAR_KD, active=active
            )
            time.sleep(0.05)
        fb = np.asarray(arm.read("Position_Rad"), dtype=np.float32).reshape(7)
        if stop.is_set():
            print(f"  stop @ step {i}: last_safe={last_safe:+.4f} fb={fb[joint]:+.4f}")
            break
        last_safe = float(fb[joint])
        print(f"  step {i + 1}: cmd={cmd:+.4f} fb={last_safe:+.4f}", flush=True)
    else:
        print(f"  hit max_steps={max_steps} without Enter — using last_safe={last_safe:+.4f}")

    _return_home_slow(arm, session, home, rate_rad_s=home_rate, active=active)
    return last_safe


def _light_discover_once(hub, *, bus: int = 1) -> List[int]:
    """One ID_SWEEP over 0x01..0x07 (known_ids first). Do **not** loop discover.

    Calling ``discover_damiao`` per-ID (7× SESSION + REG_SCAN) floods the daisy
    and can put Damiao drives into an error state — see docs/lessons.md
    \"Scan-order flood\". Prefer skipping discover and trusting YAM CFG.
    """
    hit = hub.debug.discover_damiao(
        bus=bus,
        start=0x01,
        end=0x07,
        known_ids=_NOMINAL_ESC,
    )
    if hit is None:
        return []
    return [int(hit) & 0xFF]


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None, help="CDC port (default: auto)")
    ap.add_argument("--serial", default=None, help="USB serial disambiguation")
    ap.add_argument(
        "--apply-cfg",
        action="store_true",
        help="RAM-apply left-arm-only YAM CFG (CH1 on, CH2+ off)",
    )
    ap.add_argument(
        "--discover",
        action="store_true",
        help=(
            "Optional: one light Damiao ID_SWEEP on CH1 (default: skip — "
            "trust CFG ESC 0x01..0x07; per-ID REG_SCAN floods can fault drives)"
        ),
    )
    ap.add_argument(
        "--joint",
        type=int,
        default=None,
        help="Characterize only this arm-local joint 0..6 (default: all)",
    )
    ap.add_argument("--step", type=float, default=0.04, help="Jog step rad (default 0.04)")
    ap.add_argument(
        "--inset",
        type=float,
        default=DEFAULT_CLEAR_INSET,
        help="Conservative inset after stop edges (default 0.08)",
    )
    ap.add_argument("--dwell", type=float, default=0.45, help="Seconds per step")
    ap.add_argument(
        "--home-rate",
        type=float,
        default=_DEFAULT_HOME_RATE_RAD_S,
        help="Max rad/s for slow return-to-home after each edge (default 0.12)",
    )
    ap.add_argument("--max-steps", type=int, default=80)
    ap.add_argument(
        "--stream-hz",
        type=float,
        default=_DEFAULT_STREAM_HZ,
        help=f"Plant stream Hz (default {_DEFAULT_STREAM_HZ:.0f}; high rates vibrate/fault)",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="No COM — demo inset math from fake edges",
    )
    ap.add_argument(
        "--write",
        action="store_true",
        default=True,
        help="Write yam_bench_clear_left.py + JSON artifact (default on)",
    )
    ap.add_argument("--no-write", action="store_true", help="Skip writing outputs")
    ap.add_argument(
        "--buzz-locate",
        action="store_true",
        help="Enter-gated J6 buzz locator (B/E stages). Default: skip — use prove path.",
    )
    ap.add_argument(
        "--buzz-locate-only",
        action="store_true",
        help="Run buzz locator only, then exit (implies --buzz-locate)",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)
    write = bool(args.write) and not bool(args.no_write)

    if args.joint is not None and not (0 <= args.joint <= 6):
        ap.error("--joint must be 0..6")

    joints = [args.joint] if args.joint is not None else list(range(7))

    if args.dry_run:
        home = np.array([0.1, 1.0, 1.0, 0.0, 0.0, 0.0, 1.5], dtype=np.float64)
        edge_lo = home - 0.4
        edge_hi = home + 0.4
        lo, hi = apply_clear_inset_q7(edge_lo, edge_hi, home, inset=args.inset)
        print("dry-run inset:")
        print(f"  home {_fmt_q(home)}")
        print(f"  edge_lo {_fmt_q(edge_lo)}")
        print(f"  edge_hi {_fmt_q(edge_hi)}")
        print(f"  clear_lo {_fmt_q(lo)}")
        print(f"  clear_hi {_fmt_q(hi)}")
        return 0

    port = args.port or find_cdc_port(serial=args.serial)
    print(f"port={port}", flush=True)

    with PcbRobotSession.connect(
        port,
        apply_yam_cfg=False,
        stream_hz=float(args.stream_hz),
        idle_first=True,
    ) as session:
        found: List[int] = list(_NOMINAL_ESC)
        print("MCU DIAG_ONLY + idle (idle_first) before CFG", flush=True)
        _idle_mcu(session.hub, session)
        # CFG under paused stream (I2RT has no CFG — here: all 7 left Damiao on).
        with pause_plant_stream(session.hub):
            if args.apply_cfg:
                ensure_yam_left_arm_cfg(session.hub, force=True)
            else:
                print("note: --apply-cfg not set; using whatever CFG is already in RAM", flush=True)

            if args.discover:
                print(
                    "Damiao light discover (one ID_SWEEP 0x01..0x07)…",
                    flush=True,
                )
                found = _light_discover_once(session.hub, bus=1)
                print(f"  first hit: {[hex(x) for x in found] or '(none)'}")
                if not found:
                    print(
                        "FAIL: discover found no Damiao — check power/termination; "
                        "or omit --discover and rely on CFG + live FB",
                        flush=True,
                    )
                    return 2
            else:
                print(
                    "skipping Damiao discover (use --discover only if needed); "
                    "trusting CFG ESC 0x01..0x07 — verifying via plant FB next",
                    flush=True,
                )

        # Bring-up was commanding Goal=0 + full kd before FB — that moved/buzzed J6
        # before E0. Now: Enter-gated B-stages, never hold position=0.
        print("bring-up: recover → Enter-gated B-stages (no Goal=0 seed)", flush=True)
        session.hub.recover()
        time.sleep(0.25)
        session.hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        _blank_all(session)

        arm = PcbArmDriver(
            session,
            side="left",
            skip_home_on_connect=True,
            clamp_goals=False,
            kp=_CLEAR_KP,
            kd=_CLEAR_KD,
        )
        home: Optional[np.ndarray] = None
        try:
            arm.is_connected = True
            session.hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=True,
            )

            do_locate = bool(args.buzz_locate or args.buzz_locate_only)
            session.hub.set_mcu_state(McuState.NORMAL, send=True)
            _blank_all(session)

            if do_locate:
                _stage_banner(
                    "B0",
                    "DIAG + blank desires",
                    "Plant gated. No MIT. Arm must be still/silent.",
                )
                session.hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
                _blank_all(session)
                _prompt_hold(
                    session,
                    arm,
                    np.zeros(7, dtype=np.float32),
                    "B0 — Enter when done… ",
                    blank=True,
                )
                _stage_banner(
                    "B1",
                    "NORMAL + blank desires",
                    "MCU NORMAL but blank ActuatorDesire.",
                )
                session.hub.set_mcu_state(McuState.NORMAL, send=True)
                _blank_all(session)
                _prompt_hold(
                    session,
                    arm,
                    np.zeros(7, dtype=np.float32),
                    "B1 — Enter when done… ",
                    blank=True,
                )
                _stage_banner(
                    "B2",
                    "acquire FB (no Goal=0 hold)",
                    "Goal tracks FB then FREEZES. Never parks at 0.",
                )
                _prompt_hold(
                    session,
                    arm,
                    np.zeros(7, dtype=np.float32),
                    "B2 — Enter to START FB acquire… ",
                    blank=True,
                )

            print("acquiring home FB (J1–J6, J7 blank)…", flush=True)
            q_live = _acquire_home_fb(
                session, arm, active=_ACTIVE_PROVE, settle_s=0.7
            )
            home = q_live.astype(np.float64)
            print(f"frozen home {_fmt_q(home)}", flush=True)
            print(f"faults={_faults_q7(arm)}", flush=True)

            if do_locate:
                _prompt_hold(
                    session,
                    arm,
                    q_live,
                    "B2 — frozen. Enter… ",
                    refresh_mit=True,
                    kp_scale=0.0,
                    kd=0.5,
                    active=_ACTIVE_PROVE,
                )
                _stage_banner(
                    "B3",
                    "raise kd to full at frozen home",
                    "Still kp=0; kd 0.5→full.",
                )
                _prompt_hold(
                    session,
                    arm,
                    q_live,
                    "B3 — Enter to raise kd… ",
                    refresh_mit=True,
                    kp_scale=0.0,
                    kd=0.5,
                    active=_ACTIVE_PROVE,
                )

            print("kd → full at frozen home…", flush=True)
            _write_mit(
                session,
                arm,
                q_live,
                kp_scale=0.0,
                kd=_CLEAR_KD,
                active=_ACTIVE_PROVE,
            )
            if do_locate:
                _prompt_hold(
                    session,
                    arm,
                    q_live,
                    "B3 — full kd. Enter for E0+… ",
                    refresh_mit=True,
                    kp_scale=0.0,
                    kd=_CLEAR_KD,
                    active=_ACTIVE_PROVE,
                )
            else:
                time.sleep(0.3)

            print(
                f"MIT kp={_CLEAR_KP} kd={_CLEAR_KD} stream_hz={args.stream_hz} "
                f"(J7 blank until J7 sweep)",
                flush=True,
            )

            if do_locate:
                _buzz_locate_stages(session, arm, q_live)
                if args.buzz_locate_only:
                    print("buzz-locate-only: done", flush=True)
                    return 0

            # Prove path: soft-engage J1–J6 only (J7 blank — E2 snap buzzes J6).
            _soft_engage(session, arm, q_live, engage_s=1.4, active=_ACTIVE_PROVE)
            time.sleep(0.5)

            edge_lo = home.copy()
            edge_hi = home.copy()

            print(
                "\nSupervised clear-range. Keep e-stop ready. "
                "J1–J6: J7 MIT blank (prove). J7: soft-engage then blank again.\n"
                "Enter = stop before contact.\n",
                flush=True,
            )
            j7_engaged = False
            for j in joints:
                step_j = float(args.step)
                dwell_j = float(args.dwell)
                if j == _J2_INDEX:
                    step_j = min(step_j, 0.02)
                    dwell_j = max(dwell_j, 0.5)
                elif j == 5:  # J6
                    step_j = min(step_j, 0.03)
                    dwell_j = max(dwell_j, 0.55)
                elif j == 6:  # J7
                    step_j = min(step_j, 0.03)
                    dwell_j = max(dwell_j, 0.6)

                active_j: Sequence[int] = (
                    _ACTIVE_ALL7 if j == 6 else _ACTIVE_PROVE
                )
                if j == 6 and not j7_engaged:
                    _prompt_hold(
                        session,
                        arm,
                        home.astype(np.float32),
                        "J7 next — Enter to soft-engage J7 (slow ramp)… ",
                        refresh_mit=True,
                        kp_scale=1.0,
                        active=_ACTIVE_PROVE,
                    )
                    _soft_engage_j7(
                        session, arm, home.astype(np.float32), engage_s=2.5
                    )
                    j7_engaged = True
                    time.sleep(0.4)

                _prompt_hold(
                    session,
                    arm,
                    home.astype(np.float32),
                    f"Ready for J{j + 1}? Press Enter to start PLUS sweep… ",
                    refresh_mit=True,
                    kp_scale=1.0,
                    active=active_j,
                )
                print(f"  faults={_faults_q7(arm)} active={list(active_j)}", flush=True)

                plus = _jog_direction(
                    arm,
                    session,
                    home=home.astype(np.float32),
                    joint=j,
                    sign=+1,
                    step=step_j,
                    dwell_s=dwell_j,
                    max_steps=int(args.max_steps),
                    home_rate=float(args.home_rate),
                    active=active_j,
                )
                _prompt_hold(
                    session,
                    arm,
                    home.astype(np.float32),
                    f"Ready for J{j + 1} MINUS? Press Enter… ",
                    refresh_mit=True,
                    kp_scale=1.0,
                    active=active_j,
                )
                minus = _jog_direction(
                    arm,
                    session,
                    home=home.astype(np.float32),
                    joint=j,
                    sign=-1,
                    step=step_j,
                    dwell_s=dwell_j,
                    max_steps=int(args.max_steps),
                    home_rate=float(args.home_rate),
                    active=active_j,
                )
                edge_lo[j] = min(minus, plus, float(home[j]))
                edge_hi[j] = max(minus, plus, float(home[j]))
                print(
                    f"J{j + 1} edges raw: lo={edge_lo[j]:+.4f} hi={edge_hi[j]:+.4f}",
                    flush=True,
                )
                if j == 6 and j7_engaged:
                    _blank_j7_keep_arm(session, arm, home.astype(np.float32))
                    j7_engaged = False

            # Joints not swept keep a tiny band around home.
            for j in range(7):
                if j not in joints:
                    edge_lo[j] = float(home[j]) - 0.05
                    edge_hi[j] = float(home[j]) + 0.05

            lo, hi = apply_clear_inset_q7(
                edge_lo, edge_hi, home, inset=float(args.inset)
            )
            source = (
                f"bench left CH1 supervised {date.today().isoformat()} "
                f"step={args.step} inset={args.inset} port={port} esc={found!r}"
            )
            print("\n=== clear envelope (motor frame, after inset) ===")
            print(f"  home     {_fmt_q(home)}")
            print(f"  edge_lo  {_fmt_q(edge_lo)}")
            print(f"  edge_hi  {_fmt_q(edge_hi)}")
            print(f"  clear_lo {_fmt_q(lo)}")
            print(f"  clear_hi {_fmt_q(hi)}")
            print(f"  source   {source}")

            if write:
                _SESSION_DIR.mkdir(parents=True, exist_ok=True)
                artifact = {
                    "date": date.today().isoformat(),
                    "port": port,
                    "found_esc": found,
                    "home": [float(x) for x in home],
                    "edge_lo": [float(x) for x in edge_lo],
                    "edge_hi": [float(x) for x in edge_hi],
                    "clear_lo": [float(x) for x in lo],
                    "clear_hi": [float(x) for x in hi],
                    "inset": float(args.inset),
                    "step": float(args.step),
                    "source": source,
                    "joints": joints,
                }
                json_path = (
                    _SESSION_DIR / f"yam_clear_left_{date.today().isoformat()}.json"
                )
                json_path.write_text(
                    json.dumps(artifact, indent=2) + "\n", encoding="utf-8"
                )
                print(f"wrote {json_path}")

                if set(joints) == set(range(7)):
                    _BENCH_MODULE.write_text(
                        render_bench_module(
                            lo=lo,
                            hi=hi,
                            home=home,
                            source=source,
                            inset=float(args.inset),
                            step=float(args.step),
                        ),
                        encoding="utf-8",
                    )
                    print(f"wrote {_BENCH_MODULE} (CLEAR_ACTIVE=True)")
                else:
                    print(
                        "partial joint set — JSON only; re-run all joints to "
                        "activate yam_bench_clear_left.py",
                        flush=True,
                    )
        finally:
            try:
                if home is not None and arm.is_connected:
                    _go_to(
                        session,
                        arm,
                        home.astype(np.float32),
                        2.0,
                        active=_ACTIVE_PROVE,
                    )
                print("cleanup: recover → DIAG + cornflower idle…", flush=True)
                session.hub.recover()
                time.sleep(0.2)
                _idle_mcu(session.hub, session)
                time.sleep(0.2)
                arm.is_connected = False
            except Exception as exc:
                print(f"cleanup warning: {exc}", flush=True)

    print("done — MCU left cornflower idle; review JSON / yam_bench_clear_left.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
