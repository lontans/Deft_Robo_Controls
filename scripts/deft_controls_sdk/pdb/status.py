"""Typed USB accessors for PDB kill/status mirrored into the plant image.

USB plant feedback carries:
  - system.kill_state / kill_reason / estop_sense at SYSTEM_FB_OFF+14
  - pdb[64] verbatim PDBF mirror at PDB_OFF (parse via frame.parse_feedback)

Freshness contract (firmware pdb_link): stale UART peer ⇒
``kill_state=HARD_ESTOP`` and ``kill_reason=COMMS_LOSS`` on the USB system
bytes (not a separate freshness flag). See docs/pdb-uart-v1.md.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

from deft_controls_sdk.link.exchange.wire_layout import (
    HOST_PDB_PAYLOAD_BYTES,
    IMAGE_BYTES,
    PDB_OFF,
    SYSTEM_FB_OFF,
)
from deft_controls_sdk.pdb.frame import (
    KILL_HARD_ESTOP,
    KILL_NORMAL,
    KILL_REASON_COMMS_LOSS,
    KILL_REASON_NAMES,
    KILL_SOFT_READY,
    KILL_SOFT_REQ,
    KILL_STATE_NAMES,
    STALE_MS,
    parse_feedback,
)

# Placeholder scales pending real sensor resolution (docs/pdb-uart-v1.md).
VOLTAGE_SCALE_V_PER_COUNT = 0.01  # 10 mV / count
CURRENT_SCALE_A_PER_COUNT = 0.01  # 10 mA / count


def counts_to_volts(counts: int) -> float:
    """Convert a placeholder u16 voltage count to volts (10 mV/count)."""
    return float(counts) * VOLTAGE_SCALE_V_PER_COUNT


def counts_to_amps(counts: int) -> float:
    """Convert a placeholder u16 current count to amps (10 mA/count)."""
    return float(counts) * CURRENT_SCALE_A_PER_COUNT


def volts_to_counts(volts: float) -> int:
    return int(round(float(volts) / VOLTAGE_SCALE_V_PER_COUNT)) & 0xFFFF


def amps_to_counts(amps: float) -> int:
    return int(round(float(amps) / CURRENT_SCALE_A_PER_COUNT)) & 0xFFFF


def counts_vec_to_volts(counts: Sequence[int]) -> Tuple[float, ...]:
    return tuple(counts_to_volts(c) for c in counts)


def counts_vec_to_amps(counts: Sequence[int]) -> Tuple[float, ...]:
    return tuple(counts_to_amps(c) for c in counts)


def parse_system_kill(frame: bytes) -> Optional[Dict[str, Any]]:
    """Parse USB ``system.kill_state/reason/estop_sense`` at SYSTEM_FB_OFF+14."""
    if len(frame) < SYSTEM_FB_OFF + 17:
        return None
    kill_state, kill_reason, estop_sense = struct.unpack_from(
        "<BBB", frame, SYSTEM_FB_OFF + 14
    )
    return {
        "kill_state": int(kill_state),
        "kill_reason": int(kill_reason),
        "estop_sense": int(estop_sense),
        "kill_state_name": KILL_STATE_NAMES.get(int(kill_state), f"unknown({kill_state})"),
        "kill_reason_name": KILL_REASON_NAMES.get(
            int(kill_reason), f"unknown({kill_reason})"
        ),
        # Stale peer is reported as HARD + COMMS_LOSS (MCU contract).
        "stale_failsafe": (
            int(kill_state) == KILL_HARD_ESTOP
            and int(kill_reason) == KILL_REASON_COMMS_LOSS
        ),
    }


def parse_pdb_mirror(frame: bytes) -> Optional[Dict[str, Any]]:
    """Parse the 64 B ``pdb[]`` mirror from a plant feedback image."""
    if len(frame) < PDB_OFF + HOST_PDB_PAYLOAD_BYTES:
        return None
    return parse_feedback(frame[PDB_OFF : PDB_OFF + HOST_PDB_PAYLOAD_BYTES])


@dataclass(frozen=True)
class PdbStatus:
    """Typed snapshot of USB-mirrored PDB status (system + optional pdb[64])."""

    kill_state: int
    kill_reason: int
    estop_sense: int
    kill_state_name: str
    kill_reason_name: str
    stale_failsafe: bool
    pdb: Optional[Dict[str, Any]] = None

    @property
    def soft_kill_req(self) -> bool:
        return self.kill_state == KILL_SOFT_REQ

    @property
    def soft_kill_ready(self) -> bool:
        return self.kill_state == KILL_SOFT_READY

    @property
    def hard_estop(self) -> bool:
        return self.kill_state == KILL_HARD_ESTOP

    @property
    def normal(self) -> bool:
        return self.kill_state == KILL_NORMAL

    @property
    def pack_v_V(self) -> Optional[Tuple[float, ...]]:
        if self.pdb is None:
            return None
        return counts_vec_to_volts(self.pdb["pack_v"])

    @property
    def rail_v_V(self) -> Optional[Tuple[float, ...]]:
        if self.pdb is None:
            return None
        return counts_vec_to_volts(self.pdb["rail_v"])

    @property
    def pack_i_A(self) -> Optional[Tuple[float, ...]]:
        if self.pdb is None:
            return None
        return counts_vec_to_amps(self.pdb["pack_i"])

    @property
    def rail_i_A(self) -> Optional[Tuple[float, ...]]:
        if self.pdb is None:
            return None
        return counts_vec_to_amps(self.pdb["rail_i"])

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "kill_state": self.kill_state,
            "kill_reason": self.kill_reason,
            "estop_sense": self.estop_sense,
            "kill_state_name": self.kill_state_name,
            "kill_reason_name": self.kill_reason_name,
            "stale_failsafe": self.stale_failsafe,
            "pdb": self.pdb,
            "pack_v_V": self.pack_v_V,
            "rail_v_V": self.rail_v_V,
            "pack_i_A": self.pack_i_A,
            "rail_i_A": self.rail_i_A,
        }
        return out


def pdb_status_from_frame(frame: bytes) -> Optional[PdbStatus]:
    """Build ``PdbStatus`` from a full plant/DEBUG feedback image."""
    if len(frame) != IMAGE_BYTES:
        return None
    sys_kill = parse_system_kill(frame)
    if sys_kill is None:
        return None
    # Mirror optional: zeros / non-PDBF still yield system kill fields.
    return PdbStatus(
        kill_state=sys_kill["kill_state"],
        kill_reason=sys_kill["kill_reason"],
        estop_sense=sys_kill["estop_sense"],
        kill_state_name=sys_kill["kill_state_name"],
        kill_reason_name=sys_kill["kill_reason_name"],
        stale_failsafe=sys_kill["stale_failsafe"],
        pdb=parse_pdb_mirror(frame),
    )


__all__ = [
    "CURRENT_SCALE_A_PER_COUNT",
    "KILL_HARD_ESTOP",
    "KILL_NORMAL",
    "KILL_SOFT_READY",
    "KILL_SOFT_REQ",
    "PdbStatus",
    "STALE_MS",
    "VOLTAGE_SCALE_V_PER_COUNT",
    "amps_to_counts",
    "counts_to_amps",
    "counts_to_volts",
    "counts_vec_to_amps",
    "counts_vec_to_volts",
    "parse_pdb_mirror",
    "parse_system_kill",
    "pdb_status_from_frame",
    "volts_to_counts",
]
