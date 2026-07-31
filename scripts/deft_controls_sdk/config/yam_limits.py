"""YAM Damiao arm soft limits (host-side clamps).

Port of ``scripts/pcb_lab/legacy/control_hub/yam_limits.py`` into the SDK surface.
J1–J6 come from MuJoCo ``External_Documentation/yam_arm_damiao/yam.xml``;
J7 is bench-provisional (motor frame). Left/right arms share the same
local soft ranges (identical hardware) until a left-bench clear table is
active (see ``yam_bench_clear_left``).

**Frame caveat:** XML ranges are model joint angles. Damiao feedback is motor
encoder angle until zeros are calibrated. When the left-bench clear table is
active, left-arm clamps use that **motor-frame** envelope (tighter than MuJoCo
on this setup). Absolute teleop still needs zeros (see docs/bringup.md).
"""
from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple, Union

import numpy as np

from deft_controls_sdk.config.actuator import arm_slots

_REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_YAM_XML = _REPO_ROOT / "External_Documentation" / "yam_arm_damiao" / "yam.xml"

ARM_JOINT_COUNT = 7
SOFT_MARGIN = 0.05
DEFAULT_DELTA_FRAC = 0.08
DEFAULT_DELTA_CAP = 0.35
DEFAULT_CLEAR_INSET = 0.08

# Bench-derived EE (motor frame; not in yam.xml) — see legacy yam_limits.
J7_MOTOR_LO = 1.10
J7_MOTOR_HI = 2.80
J7_SOURCE = "bench-derived (2026-07; motor frame; not in yam.xml)"

# Embedded J1–J6 from yam.xml (fallback if XML missing on a thin checkout).
_EMBEDDED_J1_J6: Tuple[Tuple[str, float, float], ...] = (
    ("joint1", -2.61799, 3.13),
    ("joint2", 0.0, 3.65),
    ("joint3", 0.0, 3.13),
    ("joint4", -1.5708, 1.5708),
    ("joint5", -1.5708, 1.5708),
    ("joint6", -2.0944, 2.0944),
)


@dataclass(frozen=True)
class JointLimit:
    """One joint soft range (global joint index 1..14 or arm-local via helpers)."""

    joint: int
    name: str
    lo: float
    hi: float
    source: str
    force_lo: float = -10.0
    force_hi: float = 10.0

    @property
    def span(self) -> float:
        return max(0.0, self.hi - self.lo)

    @property
    def mid(self) -> float:
        return 0.5 * (self.lo + self.hi)

    def soft(self, margin: float = SOFT_MARGIN) -> Tuple[float, float]:
        m = min(margin, 0.45 * self.span) if self.span > 0 else 0.0
        return (self.lo + m, self.hi - m)


def arm_local_joint(joint: int) -> int:
    return ((int(joint) - 1) % ARM_JOINT_COUNT) + 1


def slot_to_joint(slot: int) -> int:
    return int(slot) + 1


def joint_to_slot(joint: int) -> int:
    return int(joint) - 1


def _parse_range_attr(range_attr: str) -> Tuple[float, float]:
    parts = range_attr.strip().split()
    if len(parts) != 2:
        raise ValueError(f"bad joint range={range_attr!r}")
    return float(parts[0]), float(parts[1])


def _parse_force_attr(attr: Optional[str]) -> Tuple[float, float]:
    if not attr:
        return -10.0, 10.0
    parts = attr.strip().split()
    if len(parts) != 2:
        return -10.0, 10.0
    return float(parts[0]), float(parts[1])


def _arm1_base_from_embedded() -> Dict[int, JointLimit]:
    out: Dict[int, JointLimit] = {}
    for i, (name, lo, hi) in enumerate(_EMBEDDED_J1_J6, start=1):
        out[i] = JointLimit(joint=i, name=name, lo=lo, hi=hi, source="embedded:yam.xml")
    out[7] = JointLimit(
        joint=7,
        name="joint7_ee",
        lo=J7_MOTOR_LO,
        hi=J7_MOTOR_HI,
        source=J7_SOURCE,
    )
    return out


def _mirror_arm2(arm1: Dict[int, JointLimit]) -> Dict[int, JointLimit]:
    out = dict(arm1)
    for j in range(1, ARM_JOINT_COUNT + 1):
        base = arm1[j]
        mirrored = j + ARM_JOINT_COUNT
        out[mirrored] = JointLimit(
            joint=mirrored,
            name=f"{base.name}_arm2",
            lo=base.lo,
            hi=base.hi,
            source=f"{base.source} (arm2 mirror)",
            force_lo=base.force_lo,
            force_hi=base.force_hi,
        )
    return out


def load_yam_limits(xml_path: Optional[Path] = None) -> Dict[int, JointLimit]:
    """Load J1–J6 + provisional J7, mirror onto arm2 (J8–J14)."""
    path = Path(xml_path) if xml_path is not None else DEFAULT_YAM_XML
    if not path.is_file():
        return _mirror_arm2(_arm1_base_from_embedded())

    root = ET.parse(path).getroot()
    out: Dict[int, JointLimit] = {}
    for joint_el in root.iter("joint"):
        name = joint_el.get("name") or ""
        m = re.fullmatch(r"joint(\d+)", name)
        if not m:
            continue
        j = int(m.group(1))
        if j < 1 or j > 6:
            continue
        lo, hi = _parse_range_attr(joint_el.get("range") or "0 0")
        flo, fhi = _parse_force_attr(joint_el.get("actuatorfrcrange"))
        out[j] = JointLimit(
            joint=j,
            name=name,
            lo=lo,
            hi=hi,
            source=str(path.as_posix()),
            force_lo=flo,
            force_hi=fhi,
        )

    if set(out.keys()) != {1, 2, 3, 4, 5, 6}:
        # Corrupt/partial XML → fall back rather than crash smokes.
        return _mirror_arm2(_arm1_base_from_embedded())

    out[7] = JointLimit(
        joint=7,
        name="joint7_ee",
        lo=J7_MOTOR_LO,
        hi=J7_MOTOR_HI,
        source=J7_SOURCE,
    )
    return _mirror_arm2(out)


def limits_for_side(
    side: str,
    limits: Optional[Dict[int, JointLimit]] = None,
) -> Tuple[JointLimit, ...]:
    """Seven ``JointLimit``s in arm-local order (index 0 = J1 / gripper=J7)."""
    table = limits if limits is not None else load_yam_limits()
    slots = arm_slots(side)
    return tuple(table[slot_to_joint(s)] for s in slots)


def apply_clear_inset(
    edge_lo: float,
    edge_hi: float,
    *,
    inset: float = DEFAULT_CLEAR_INSET,
    home: Optional[float] = None,
) -> Tuple[float, float]:
    """Conservative clear window from operator stop edges (last-safe FB).

    Inset is ``min(inset, 10% of half-span)``. Ensures ``lo <= hi``; if the
    window collapses, keeps a tiny band around ``home`` (or the midpoint).
    """
    lo_e = float(edge_lo)
    hi_e = float(edge_hi)
    if hi_e < lo_e:
        lo_e, hi_e = hi_e, lo_e
    span = hi_e - lo_e
    half = 0.5 * span
    m = float(inset)
    if half > 1e-6:
        m = min(m, 0.10 * half)
    lo = lo_e + m
    hi = hi_e - m
    if lo <= hi:
        return lo, hi
    mid = float(home) if home is not None else 0.5 * (lo_e + hi_e)
    band = max(0.02, 0.25 * max(span, 0.04))
    return mid - band, mid + band


def load_bench_clear_left(
    *,
    force: Optional[bool] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Motor-frame clear ``(lo[7], hi[7])`` for left arm, or None if inactive.

    ``force``: True/False overrides module flag; None uses ``CLEAR_ACTIVE``
    (and ``YAM_BENCH_CLEAR_LEFT=0`` disables).
    """
    env = os.environ.get("YAM_BENCH_CLEAR_LEFT", "").strip().lower()
    if env in ("0", "false", "off", "no"):
        return None
    try:
        from deft_controls_sdk.config import yam_bench_clear_left as bench
    except ImportError:
        return None
    active = bool(bench.CLEAR_ACTIVE) if force is None else bool(force)
    if not active:
        return None
    got = bench.clear_q7()
    if got is None:
        return None
    lo_t, hi_t = got
    lo = np.asarray(lo_t, dtype=np.float64).reshape(7)
    hi = np.asarray(hi_t, dtype=np.float64).reshape(7)
    return lo, hi


def soft_limits_q7(
    side: str,
    *,
    margin: float = SOFT_MARGIN,
    limits: Optional[Dict[int, JointLimit]] = None,
    use_bench_clear: bool = True,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(lo[7], hi[7])`` soft windows for one arm side.

    Left arm: when a bench clear table is active, use that motor-frame window
    (already inset) intersected with MuJoCo soft — bench is typically tighter.
    """
    lims = limits_for_side(side, limits)
    lo = np.zeros(7, dtype=np.float64)
    hi = np.zeros(7, dtype=np.float64)
    for i, lim in enumerate(lims):
        lo[i], hi[i] = lim.soft(margin)

    if use_bench_clear and side.strip().lower() in ("left", "l", "can_deft_l"):
        bench = load_bench_clear_left()
        if bench is not None:
            blo, bhi = bench
            lo = np.maximum(lo, blo)
            hi = np.minimum(hi, bhi)
            # If frames disagree badly, prefer bench (motor-frame truth here).
            bad = hi < lo
            if np.any(bad):
                lo = np.where(bad, blo, lo)
                hi = np.where(bad, bhi, hi)
    return lo, hi


def clamp_absolute(pos: float, lim: JointLimit, margin: float = SOFT_MARGIN) -> float:
    lo, hi = lim.soft(margin)
    return min(hi, max(lo, float(pos)))


def clamp_q7(
    q: Union[Sequence[float], np.ndarray],
    side: str = "left",
    *,
    margin: float = SOFT_MARGIN,
    limits: Optional[Dict[int, JointLimit]] = None,
    use_bench_clear: bool = True,
) -> np.ndarray:
    """Clamp a length-7 joint vector to the side's effective soft limits."""
    arr = np.asarray(q, dtype=np.float64).reshape(7)
    lo, hi = soft_limits_q7(
        side, margin=margin, limits=limits, use_bench_clear=use_bench_clear
    )
    out = np.minimum(hi, np.maximum(lo, arr))
    return out.astype(np.float32)


def clamp_delta(
    start: float,
    delta: float,
    lim: JointLimit,
    *,
    margin: float = SOFT_MARGIN,
    absolute: bool = False,
    lo_hi: Optional[Tuple[float, float]] = None,
) -> Tuple[float, float, str]:
    """Return ``(clamped_delta, target, note)`` — same policy as legacy."""
    if lo_hi is not None:
        lo, hi = float(lo_hi[0]), float(lo_hi[1])
        span = max(0.0, hi - lo)
    else:
        lo, hi = lim.soft(margin)
        span = lim.span
    span_cap = min(DEFAULT_DELTA_CAP, max(0.05, DEFAULT_DELTA_FRAC * span))

    if arm_local_joint(lim.joint) == 7 and start <= J7_MOTOR_LO + 0.15 and delta < 0.0:
        mag = min(abs(delta), span_cap)
        return mag, start + mag, (
            f"J{lim.joint} near motor-frame hard-stop (~{J7_MOTOR_LO:.2f}); "
            f"flipped delta to +{mag:.3f}"
        )

    if not absolute:
        if start < lo - 0.25 or start > hi + 0.25:
            mag = min(abs(delta), span_cap)
            d = math.copysign(mag, delta) if delta != 0.0 else 0.0
            return d, start + d, (
                f"fb outside soft [{lo:.3f},{hi:.3f}] — relative-only, |delta|<={span_cap:.3f}"
            )

    target = start + delta
    clamped_t = min(hi, max(lo, target))
    d = clamped_t - start
    note = ""
    if abs(d - delta) > 1e-6:
        note = f"clamped delta {delta:+.3f} -> {d:+.3f} (soft [{lo:.3f},{hi:.3f}])"
    if abs(d) < 1e-4 and abs(delta) > 1e-4:
        room_up = hi - start
        room_dn = start - lo
        if room_up >= room_dn and room_up > 1e-3:
            d = min(abs(delta), span_cap, room_up)
            note = f"against soft wall; auto +{d:.3f}"
        elif room_dn > 1e-3:
            d = -min(abs(delta), span_cap, room_dn)
            note = f"against soft wall; auto {d:.3f}"
    return d, start + d, note


def plan_hold_q7(
    q_fb: Union[Sequence[float], np.ndarray],
    side: str = "left",
    *,
    margin: float = SOFT_MARGIN,
) -> np.ndarray:
    """Soft-clamped hold setpoint from present FB (or last known)."""
    return clamp_q7(q_fb, side, margin=margin)


def plan_jog_q7(
    q_fb: Union[Sequence[float], np.ndarray],
    side: str = "left",
    *,
    joint: int = 0,
    delta: float = 0.05,
    margin: float = SOFT_MARGIN,
    limits: Optional[Dict[int, JointLimit]] = None,
) -> Tuple[np.ndarray, str]:
    """Relative jog on one arm-local joint index (0..6); returns (q_cmd, note)."""
    if joint < 0 or joint > 6:
        raise ValueError(f"joint must be 0..6, got {joint}")
    q0 = np.asarray(q_fb, dtype=np.float64).reshape(7)
    lim = limits_for_side(side, limits)[joint]
    lo, hi = soft_limits_q7(side, margin=margin, limits=limits)
    d, _target, note = clamp_delta(
        float(q0[joint]),
        float(delta),
        lim,
        margin=margin,
        lo_hi=(float(lo[joint]), float(hi[joint])),
    )
    q = q0.copy()
    q[joint] = float(q0[joint]) + d
    q_cmd = clamp_q7(q, side, margin=margin, limits=limits)
    return q_cmd, note
