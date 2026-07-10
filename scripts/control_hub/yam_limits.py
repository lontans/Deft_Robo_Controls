"""YAM arm joint limits — MuJoCo ``yam.xml`` (J1–J6) + bench-derived J7.

``yam.xml`` is a 6-DOF collision model (no end-effector joint). Slot/joint map for
the Damiao CH1 daisy is 0-based plant slot == joint_index - 1 (J1→slot0 … J7→slot6).

**Frame caveat:** XML ranges are model joint angles. Damiao feedback is motor
encoder angle. Absolute clamps are only safe after zero alignment; until then
prefer relative jogs limited by span (see ``clamp_delta`` / hello-world).
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

# Repo-relative default (works from repo root or scripts/).
_REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_YAM_XML = _REPO_ROOT / "External_Documentation" / "yam_arm_damiao" / "yam.xml"

# Jul 2026 bench (motor frame, ESC 0x07 / slot 6):
# - At fb≈+1.11 rad, MIT −delta held ~−8 Nm with moved≈0.01 → hard stop / fold limit.
# - MIT +jog tracked cleanly up to ~+2.50 rad; open-loop torque reached ~+6.4 (not a stop).
# EE stroke is not in yam.xml; keep soft and slightly wider than the proven MIT window.
J7_MOTOR_LO = 1.10
J7_MOTOR_HI = 2.80
J7_SOURCE = "bench-derived (2026-07; motor frame; not in yam.xml)"

# Soft margin inside published limits for host jogs (rad).
SOFT_MARGIN = 0.05
# Default hello-world |delta| as a fraction of joint span (capped).
DEFAULT_DELTA_FRAC = 0.08
DEFAULT_DELTA_CAP = 0.35


@dataclass(frozen=True)
class JointLimit:
    joint: int  # 1..7
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


def load_yam_limits(xml_path: Optional[Path] = None) -> Dict[int, JointLimit]:
    """Load J1–J6 from MuJoCo XML; attach provisional J7 motor-frame soft range."""
    path = Path(xml_path) if xml_path is not None else DEFAULT_YAM_XML
    if not path.is_file():
        raise FileNotFoundError(f"yam.xml not found: {path}")

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
        missing = sorted({1, 2, 3, 4, 5, 6} - set(out.keys()))
        raise ValueError(f"{path}: missing joints {missing}")

    out[7] = JointLimit(
        joint=7,
        name="joint7_ee",
        lo=J7_MOTOR_LO,
        hi=J7_MOTOR_HI,
        source=J7_SOURCE,
        force_lo=-10.0,
        force_hi=10.0,
    )
    return out


def limit_for_slot(slot: int, limits: Optional[Dict[int, JointLimit]] = None) -> JointLimit:
    table = limits if limits is not None else load_yam_limits()
    joint = slot_to_joint(slot)
    if joint not in table:
        raise KeyError(f"no limit for slot {slot} (joint {joint})")
    return table[joint]


def clamp_absolute(pos: float, lim: JointLimit, margin: float = SOFT_MARGIN) -> float:
    lo, hi = lim.soft(margin)
    return min(hi, max(lo, pos))


def clamp_delta(
    start: float,
    delta: float,
    lim: JointLimit,
    *,
    margin: float = SOFT_MARGIN,
    absolute: bool = False,
) -> Tuple[float, float, str]:
    """Return ``(clamped_delta, target, note)``.

    If ``absolute`` is False (default), only shrink |delta| so ``start+delta`` stays
    inside soft limits when ``start`` is already inside; if ``start`` is outside
    (uncalibrated motor frame), fall back to span-capped relative delta and note it.
    """
    lo, hi = lim.soft(margin)
    span_cap = min(DEFAULT_DELTA_CAP, max(0.05, DEFAULT_DELTA_FRAC * lim.span))

    # J7 bench: hard-stop when commanding more negative near ~+1.11 motor-frame.
    if lim.joint == 7 and start <= J7_MOTOR_LO + 0.15 and delta < 0.0:
        mag = min(abs(delta), span_cap)
        return mag, start + mag, (
            f"J7 near motor-frame hard-stop (~{J7_MOTOR_LO:.2f}); "
            f"flipped delta to +{mag:.3f} (avoid negative)"
        )

    if not absolute:
        # Uncalibrated / outside published window: do not trust absolute edges.
        if start < lo - 0.25 or start > hi + 0.25:
            mag = min(abs(delta), span_cap)
            d = math_copysign(mag, delta) if delta != 0.0 else 0.0
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
        # Against a soft wall: flip toward interior if possible.
        room_up = hi - start
        room_dn = start - lo
        if room_up >= room_dn and room_up > 1e-3:
            d = min(abs(delta), span_cap, room_up)
            note = f"against lo/hi; auto +{d:.3f} toward free space"
        elif room_dn > 1e-3:
            d = -min(abs(delta), span_cap, room_dn)
            note = f"against lo/hi; auto {d:.3f} toward free space"
    return d, start + d, note


def math_copysign(mag: float, ref: float) -> float:
    return mag if ref >= 0.0 else -mag


def format_limits_table(limits: Optional[Dict[int, JointLimit]] = None) -> str:
    table = limits if limits is not None else load_yam_limits()
    lines = [
        "joint  slot   lo_rad    hi_rad    span    tmax    source",
        "-----  ----  --------  --------  ------  ------  ------",
    ]
    for j in sorted(table):
        lim = table[j]
        lines.append(
            f"J{j:<4}  {joint_to_slot(j):<4}  {lim.lo:+8.4f}  {lim.hi:+8.4f}  "
            f"{lim.span:6.3f}  {lim.force_hi:5.1f}  {lim.source}"
        )
    return "\n".join(lines)


def reasonableness_notes() -> str:
    """Short review of yam.xml ranges vs bench (for docs / dry-run)."""
    return "\n".join(
        [
            "YAM limit review (yam.xml + J7 bench):",
            "  J1 [-2.62, 3.13] asymmetric base yaw - OK for floor-mounted arm.",
            "  J2 [0, 3.65] and J3 [0, 3.13] one-sided - typical folded pitch; large (>pi).",
            "  J4/J5 +/-pi/2 wrist - standard.",
            "  J6 +/-2.09 (+/-120 deg) wrist roll - standard.",
            "  actuatorfrcrange +/-10 Nm on all - matches host Damiao TMAX; OK as soft cap",
            "    (DM-J4340 can exceed; keep host clamp for bring-up).",
            "  No joint7 in XML (6-link model + tcp/grasp sites only) - expected.",
            f"  J7 provisional motor-frame [{J7_MOTOR_LO:.2f}, {J7_MOTOR_HI:.2f}] from hard-stop",
            "    at ~+1.11 (neg fight) and safe MIT travel to ~+2.5; EE not strict.",
            "  Do not treat motor fb as MuJoCo qpos until zeros are calibrated.",
        ]
    )
