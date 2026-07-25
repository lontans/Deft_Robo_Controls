"""Host-side mirror of ``App/Inc/host/pdb_vi_limits.h`` — PDU V/I
acceptability thresholds for the Controls soft-kill overlay.

Belt-and-suspenders: firmware (``pdb_link.c`` ``pdb_vi_reject_reason`` /
``pdb_link_eval_kill``) overlays ``SOFT_KILL_REQ`` onto the USB kill mirror
when V/I is bad and the peer reports ``NORMAL``, so
``hub.soft_kill_park_if_requested()`` already catches it on flashed boards.
This module lets the *host* independently recompute the same policy from the
raw ``pdb[]`` mirror and park proactively — works even before that firmware
lands, and as defense-in-depth afterward.

Keep these counts aligned with ``App/Inc/host/pdb_vi_limits.h``.
Provisional; tune after the first real PDB capture (docs/pdb-uart-v1.md).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from deft_controls_sdk.pdb.frame import (
    KILL_REASON_NAMES,
    KILL_REASON_NONE,
    KILL_REASON_OVERCURRENT,
    KILL_REASON_UNDERVOLTAGE,
)

# Counts, 10 mV / 10 mA per count (docs/pdb-uart-v1.md placeholder scales) —
# bit-exact values to App/Inc/host/pdb_vi_limits.h.
PACK_V_MIN_COUNTS = 4000  # 40.00 V
PACK_V_MAX_COUNTS = 5500  # 55.00 V
RAIL48_V_MIN_COUNTS = 4200  # 42.00 V
RAIL48_V_MAX_COUNTS = 5200  # 52.00 V
I_ABS_MAX_COUNTS = 3000  # 30.00 A

PACK_V_MIN_V = PACK_V_MIN_COUNTS / 100.0
PACK_V_MAX_V = PACK_V_MAX_COUNTS / 100.0
RAIL48_V_MIN_V = RAIL48_V_MIN_COUNTS / 100.0
RAIL48_V_MAX_V = RAIL48_V_MAX_COUNTS / 100.0
I_ABS_MAX_A = I_ABS_MAX_COUNTS / 100.0


def pdb_vi_reject_reason(pdb: Dict[str, Any]) -> int:
    """Bit-exact mirror of ``pdb_vi_reject_reason()`` in
    ``App/Src/host/pdb_link.c``.

    ``pdb`` is shaped like ``deft_controls_sdk.pdb.parse_feedback()`` output
    (``pack_v``/``rail_v``/``pack_i``/``rail_i`` as 4-int count sequences,
    plus ``contactor_state``). Returns a ``KILL_REASON_*`` int,
    ``KILL_REASON_NONE`` if nothing is out of range.

    Only ``rail_v[0]`` (central 48 V) has a voltage window; rails 1-3 are
    current-checked only when active. A channel is "active" if its pack
    voltage / rail voltage is nonzero or its contactor bit is set —
    unpopulated (all-zero) channels never trip a violation.
    """
    contactor = int(pdb["contactor_state"])
    reason = KILL_REASON_NONE
    for i in range(4):
        pv = int(pdb["pack_v"][i])
        pi = int(pdb["pack_i"][i])
        rv = int(pdb["rail_v"][i])
        ri = int(pdb["rail_i"][i])
        pack_on = pv != 0
        rail_on = ((contactor & (1 << i)) != 0) or (rv != 0)
        if pack_on and pi > I_ABS_MAX_COUNTS:
            return KILL_REASON_OVERCURRENT
        if rail_on and ri > I_ABS_MAX_COUNTS:
            return KILL_REASON_OVERCURRENT
        if pack_on and (pv < PACK_V_MIN_COUNTS or pv > PACK_V_MAX_COUNTS):
            reason = KILL_REASON_UNDERVOLTAGE
        if i == 0 and rail_on and (
            rv < RAIL48_V_MIN_COUNTS or rv > RAIL48_V_MAX_COUNTS
        ):
            reason = KILL_REASON_UNDERVOLTAGE
    return reason


@dataclass(frozen=True)
class ViCheck:
    """Result of evaluating a ``PdbStatus``'s raw ``pdb[]`` mirror against
    the shared V/I limits."""

    reason: int
    reason_name: str

    @property
    def violated(self) -> bool:
        return self.reason != KILL_REASON_NONE


def check_status(status: Any) -> Optional[ViCheck]:
    """Evaluate ``status.pdb`` (from ``hub.pdb_status()``) against the shared
    V/I limits. Returns ``None`` if there's no fresh ``pdb[]`` mirror to
    check — absence of data is not a violation.
    """
    if status is None or status.pdb is None:
        return None
    reason = pdb_vi_reject_reason(status.pdb)
    return ViCheck(
        reason=reason,
        reason_name=KILL_REASON_NAMES.get(reason, f"unknown({reason})"),
    )


__all__ = [
    "I_ABS_MAX_A",
    "I_ABS_MAX_COUNTS",
    "PACK_V_MAX_COUNTS",
    "PACK_V_MAX_V",
    "PACK_V_MIN_COUNTS",
    "PACK_V_MIN_V",
    "RAIL48_V_MAX_COUNTS",
    "RAIL48_V_MAX_V",
    "RAIL48_V_MIN_COUNTS",
    "RAIL48_V_MIN_V",
    "ViCheck",
    "check_status",
    "pdb_vi_reject_reason",
]
