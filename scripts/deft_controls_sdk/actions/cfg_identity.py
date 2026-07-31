"""CFG / NVM identity helpers — pure checks over ``cfg_get_table`` rows.

Used by the Assembly workshop nudge gate; dashboard / ROS can reuse the same
predicates without importing the suite TUI.
"""
from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from deft_controls_sdk.config.typed_profiles import ActuatorProfile


def table_by_slot(table: Sequence[Optional[Mapping[str, Any]]]) -> Dict[int, dict]:
    """Index CFG table rows by slot (skips None entries)."""
    out: Dict[int, dict] = {}
    for i, row in enumerate(table):
        if row is None:
            continue
        slot = int(row.get("slot", i))
        out[slot] = dict(row)
    return out


def cfg_row_matches(
    expected: Mapping[str, Any],
    live: Optional[Mapping[str, Any]],
    *,
    require_enabled: bool = True,
) -> bool:
    """True when live NVM/RAM CFG matches an expected ``as_cfg_row`` dict."""
    if live is None:
        return False
    if require_enabled and not bool(live.get("enabled", False)):
        return False
    if int(live.get("bus", -1)) != int(expected["bus"]):
        return False
    if int(live.get("protocol", -1)) != int(expected["protocol"]):
        return False
    if (int(live.get("motor_id", -1)) & 0xFF) != (int(expected["motor_id"]) & 0xFF):
        return False
    if (int(live.get("master_id", 0)) & 0xFF) != (int(expected.get("master_id", 0)) & 0xFF):
        return False
    return True


def profile_cfg_status(
    profile: "ActuatorProfile",
    table: Sequence[Optional[Mapping[str, Any]]],
) -> List[Tuple[int, bool, str]]:
    """Per-slot match status for ``profile.as_cfg_rows()`` against live table.

    Returns ``(slot, ok, detail)``. Slots without a profile cfg entry are
    reported as ``ok=False`` with detail ``no_profile_cfg`` (operator must
    confirm live CFG manually).
    """
    by_slot = table_by_slot(table)
    rows = profile.as_cfg_rows()
    expected_by_slot = {int(r["slot"]): r for r in rows}
    out: List[Tuple[int, bool, str]] = []
    for slot in profile.slots:
        s = int(slot)
        live = by_slot.get(s)
        exp = expected_by_slot.get(s)
        if exp is None:
            if live is None:
                out.append((s, False, "no_profile_cfg; empty live"))
            elif not bool(live.get("enabled", False)):
                out.append((s, False, "no_profile_cfg; live disabled"))
            else:
                out.append(
                    (
                        s,
                        False,
                        "no_profile_cfg; live "
                        f"bus={live.get('bus')} proto={live.get('protocol')} "
                        f"id=0x{int(live.get('motor_id', 0)) & 0xFF:02X}",
                    )
                )
            continue
        if cfg_row_matches(exp, live):
            out.append((s, True, "match"))
        else:
            if live is None:
                detail = "missing live row"
            else:
                detail = (
                    f"expected bus={exp['bus']} proto={exp['protocol']} "
                    f"id=0x{int(exp['motor_id']) & 0xFF:02X}; "
                    f"live en={bool(live.get('enabled'))} bus={live.get('bus')} "
                    f"proto={live.get('protocol')} "
                    f"id=0x{int(live.get('motor_id', 0)) & 0xFF:02X}"
                )
            out.append((s, False, detail))
    return out


def profile_in_nvm(
    profile: "ActuatorProfile",
    table: Sequence[Optional[Mapping[str, Any]]],
) -> bool:
    """True when every profile slot has CFG and matches live (enabled)."""
    status = profile_cfg_status(profile, table)
    if not status:
        return False
    if not profile.as_cfg_rows():
        return False
    return all(ok for _, ok, _ in status)


def format_slot_cfg_lines(
    slots: Sequence[int],
    table: Sequence[Optional[Mapping[str, Any]]],
) -> List[str]:
    """Compact CFG peek lines for selected slots (show --cfg style)."""
    by_slot = table_by_slot(table)
    lines = [f"{'slot':>4}  {'en':>3}  {'bus':>3}  {'protocol':>8}  {'motor_id':>8}"]
    for slot in slots:
        s = int(slot)
        live = by_slot.get(s)
        if live is None:
            lines.append(f"{s:4d}  {'?':>3}  {'-':>3}  {'(none)':>8}  {'-':>8}")
            continue
        mid = int(live.get("motor_id", 0)) & 0xFF
        lines.append(
            f"{s:4d}  {'Y' if live.get('enabled') else '.':>3}  "
            f"{int(live.get('bus', 0)):3d}  "
            f"{int(live.get('protocol', 0)):8d}  "
            f"0x{mid:02X}"
        )
    return lines


__all__ = [
    "cfg_row_matches",
    "format_slot_cfg_lines",
    "profile_cfg_status",
    "profile_in_nvm",
    "table_by_slot",
]
