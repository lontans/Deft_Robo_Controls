"""Gen2 26-actuator product map — 8DoF arms (7DoF + gripper), 4-actuator
mixed-protocol torso, 6-actuator base.

Distinct from ``profile.py``'s ``LEFT_ARM_SLOTS``/``RIGHT_ARM_SLOTS`` (7 slots,
no gripper, single ``LIFT_SLOT``) — that map is the older product shape and
does not fit this topology. Slot layout here::

    0-7   left arm  (CH1 / bus=1): J1-J4 CubeMars AKH70-48, J5-J6 RobStride
          RS-03, J7 RobStride RS-02, J8 (gripper) RobStride RS-05
    8-15  right arm (CH2 / bus=2): mirrors left arm
    16-19 torso     (CH3 / bus=3): yaw = CubeMars AKH70-48 (first in
          sequence); L1, L2, pitch = ZeroErr — the one bus that genuinely
          mixes CANopen (std ID) with MIT-mode CubeMars (also std ID — see
          note below) at real load. Matches the live board CFG order
          confirmed via `show --cfg` (slot16=cubemars, 17-19=zeroerr).

          NOTE: docs/rfc-cubemars-servo-current.md claims AKH70-48 has no
          MIT mode, but that claim doesn't hold up against the actual PDF
          text (§5.3 MIT Mode is documented in the same "Generalised" doc
          the RFC cites, with no per-model exclusion, and neither CubeMars
          PDF in this repo mentions "AKH" anywhere) — see the correction
          note added to that RFC. MIT mode for AKH70-48 (yaw here, and arm
          J1-J4 below) is the working assumption unless bench-disproven;
          the still-real gap is CUBEMARS_MIT_DEFAULT_MODEL (AK80-9) being
          used for every slot regardless of actual model (no AKH70-48 entry
          in k_ak_limits[], cubemars_set_model() never called) — that's a
          wrong-limits-table issue, not a wrong-protocol one.
    20-25 base       (CH4-6 / bus=4,5,6): 2x RobStride RS-06 per rail
          (swerve, drive) — same protocol per rail, matches the existing
          product's steer/drive pairing and SPI_POLL_TX_MAX=2 MCP tuning

Motor IDs below are bench placeholders (small, unique per (bus, protocol)
group) — replace with each actuator's actual configured CAN node ID before
flashing CFG to real hardware.

CAN ID / frame-type note (checked against cubemars.c, not assumed): CubeMars
MIT Power Mode — the only mode the firmware currently speaks — packs
STANDARD (11-bit) IDs (`cubemars_pack_cmd`/`cubemars_pack_mit` both set
`id_type = CAN_ID_STD`, TX/RX id = motor_id). Only the compiled-out Servo
Mode path (`CUBEMARS_ENABLE_SERVO_MODE=0`, not on the hot path) uses
extended IDs. RobStride is genuinely extended-ID
(`robstride_build_ext_id`'s `comm_mode<<24|...` floors at 16,777,216), so
CH1/CH2's CubeMars-vs-RobStride separation *is* real IDE-bit separation —
but CH3's CubeMars-vs-ZeroErr split is NOT frame-type-disjoint: both are
std-ID and share the same 11-bit numeric space on that bus. Today's
placeholder IDs (CubeMars torso motor_id=1 -> CAN ID 0x001; ZeroErr node
1/2/3 -> CANopen COB-IDs 0x081/0x181/0x201/0x581/0x601/0x701 etc, all >=
0x080) don't collide, but that's a numeric-range check that has to be
re-verified whenever real motor/node IDs replace these placeholders — not a
structural guarantee like the arm buses have.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

from .actuator import PROTO_CUBEMARS, PROTO_ROBSTRIDE, PROTO_ZEROERR
from .typed_profiles import ActuatorProfile, CfgSlotSpec

# --- slot constants -----------------------------------------------------

GEN2_LEFT_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 8))
GEN2_RIGHT_ARM_SLOTS: Tuple[int, ...] = tuple(range(8, 16))
GEN2_TORSO_SLOTS: Tuple[int, ...] = tuple(range(16, 20))
GEN2_BASE_SLOTS: Tuple[int, ...] = tuple(range(20, 26))
# (bus, (swerve_slot, drive_slot)) per rail — mirrors profile.py's
# BASE_WHEEL_*_SLOTS naming, CH4/5/6 this time (Gen2 has no CH1-3 base use).
GEN2_BASE_WHEEL_SLOTS: Dict[int, Tuple[int, int]] = {
    4: (20, 21),
    5: (22, 23),
    6: (24, 25),
}

# Per-arm joint order, both arms identical protocol/model shape.
_ARM_JOINT_PROTO: Tuple[int, ...] = (
    PROTO_CUBEMARS,  # J1
    PROTO_CUBEMARS,  # J2
    PROTO_CUBEMARS,  # J3
    PROTO_CUBEMARS,  # J4
    PROTO_ROBSTRIDE,  # J5 (RS-03)
    PROTO_ROBSTRIDE,  # J6 (RS-03)
    PROTO_ROBSTRIDE,  # J7 (RS-02)
    PROTO_ROBSTRIDE,  # J8 gripper (RS-05)
)
# Per-protocol motor_id counters, restarting per arm — CubeMars J1-J4 get
# 1-4, RobStride J5-J8 get their own independent 1-4 (disjoint ID ranges
# per protocol, see module docstring).
_ARM_JOINT_MOTOR_ID: Tuple[int, ...] = (1, 2, 3, 4, 1, 2, 3, 4)

_TORSO_JOINT_PROTO: Tuple[int, ...] = (
    PROTO_CUBEMARS,  # yaw (AKH70-48) — first in sequence
    PROTO_ZEROERR,   # L1
    PROTO_ZEROERR,   # L2
    PROTO_ZEROERR,   # pitch
)
_TORSO_JOINT_MOTOR_ID: Tuple[int, ...] = (1, 1, 2, 3)


def gen2_product_rows() -> List[Tuple[int, bool, int, int, int]]:
    """(bus, enabled, protocol, motor_id, master_id) per slot 0..25.

    Mirrors ``actuator.yam_product_rows()``'s shape so it plugs into the same
    ``ActuatorProfile``/``CfgSlotSpec`` machinery. master_id is unused by
    CubeMars/RobStride/ZeroErr (Damiao-only field) — left 0 throughout.
    """
    rows: List[Tuple[int, bool, int, int, int]] = []

    for bus in (1, 2):  # left arm, right arm
        for proto, mid in zip(_ARM_JOINT_PROTO, _ARM_JOINT_MOTOR_ID):
            rows.append((bus, True, proto, mid, 0))

    for proto, mid in zip(_TORSO_JOINT_PROTO, _TORSO_JOINT_MOTOR_ID):
        rows.append((3, True, proto, mid, 0))

    for bus in (4, 5, 6):
        rows.append((bus, True, PROTO_ROBSTRIDE, 1, 0))  # swerve
        rows.append((bus, True, PROTO_ROBSTRIDE, 2, 0))  # drive

    if len(rows) != ACTUATOR_COUNT:
        raise RuntimeError(
            f"Gen2 CFG rows={len(rows)} != ACTUATOR_COUNT={ACTUATOR_COUNT}"
        )
    return rows


def gen2_slots_by_bus() -> Dict[int, List[int]]:
    """bus (1..6) -> enabled slot list, for the bandwidth-matrix scenario map."""
    rows = gen2_product_rows()
    by_bus: Dict[int, List[int]] = {b: [] for b in range(1, 7)}
    for slot, (bus, enabled, _p, _m, _mas) in enumerate(rows):
        if enabled and 1 <= bus <= 6:
            by_bus[bus].append(slot)
    return by_bus


def _cfg_from_gen2_rows(slots: Tuple[int, ...]) -> Tuple[CfgSlotSpec, ...]:
    rows = gen2_product_rows()
    out: List[CfgSlotSpec] = []
    for slot in slots:
        bus, enabled, proto, mid, master = rows[slot]
        out.append(
            CfgSlotSpec(
                bus=bus, protocol=proto, motor_id=mid, master_id=master,
                enabled=enabled,
            )
        )
    return tuple(out)


def gen2_left_arm_profile() -> ActuatorProfile:
    return ActuatorProfile(
        name="gen2_left_arm",
        slots=GEN2_LEFT_ARM_SLOTS,
        cfg=_cfg_from_gen2_rows(GEN2_LEFT_ARM_SLOTS),
    )


def gen2_right_arm_profile() -> ActuatorProfile:
    return ActuatorProfile(
        name="gen2_right_arm",
        slots=GEN2_RIGHT_ARM_SLOTS,
        cfg=_cfg_from_gen2_rows(GEN2_RIGHT_ARM_SLOTS),
    )


def gen2_torso_profile() -> ActuatorProfile:
    """The mixed-protocol bus: 3x ZeroErr (CANopen) + 1x CubeMars (MIT) on CH3."""
    return ActuatorProfile(
        name="gen2_torso",
        slots=GEN2_TORSO_SLOTS,
        cfg=_cfg_from_gen2_rows(GEN2_TORSO_SLOTS),
    )


def gen2_base_wheel_profile(bus: int) -> ActuatorProfile:
    """Base wheel module on rail ``bus`` in {4,5,6} -> (swerve, drive) slots."""
    try:
        slots = GEN2_BASE_WHEEL_SLOTS[int(bus)]
    except KeyError as exc:
        raise ValueError(f"gen2 base wheel bus must be 4|5|6, got {bus!r}") from exc
    return ActuatorProfile(
        name=f"gen2_base_ch{bus}",
        slots=slots,
        cfg=_cfg_from_gen2_rows(slots),
    )


def gen2_all_profile() -> ActuatorProfile:
    return ActuatorProfile(
        name="gen2_all",
        slots=tuple(range(ACTUATOR_COUNT)),
        cfg=_cfg_from_gen2_rows(tuple(range(ACTUATOR_COUNT))),
    )


__all__ = [
    "GEN2_BASE_SLOTS",
    "GEN2_BASE_WHEEL_SLOTS",
    "GEN2_LEFT_ARM_SLOTS",
    "GEN2_RIGHT_ARM_SLOTS",
    "GEN2_TORSO_SLOTS",
    "gen2_all_profile",
    "gen2_base_wheel_profile",
    "gen2_left_arm_profile",
    "gen2_product_rows",
    "gen2_right_arm_profile",
    "gen2_slots_by_bus",
    "gen2_torso_profile",
]
