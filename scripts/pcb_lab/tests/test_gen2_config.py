"""Offline tests for the Gen2 26-actuator slot/protocol map (no hardware).

Mirrors test_bench_load_matrix.py's style/imports. Covers: row count matches
ACTUATOR_COUNT, arm/torso/base protocol assignments match the spec discussed
in bringup (CubeMars J1-4 + RobStride J5-J8 per arm, ZeroErr x3 + CubeMars x1
torso on CH3, RobStride x2 per base rail), no motor_id collisions within a
(bus, protocol) group, and the ch3 bandwidth scenario resolves to the
mixed-protocol torso slots.
"""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from collections import defaultdict

from deft_controls_sdk.config import PROTO_CUBEMARS, PROTO_ROBSTRIDE, PROTO_ZEROERR
from deft_controls_sdk.config.gen2 import (
    GEN2_BASE_SLOTS,
    GEN2_LEFT_ARM_SLOTS,
    GEN2_RIGHT_ARM_SLOTS,
    GEN2_TORSO_SLOTS,
    gen2_all_profile,
    gen2_base_wheel_profile,
    gen2_left_arm_profile,
    gen2_product_rows,
    gen2_right_arm_profile,
    gen2_slots_by_bus,
    gen2_torso_profile,
)
from deft_controls_sdk.debug.suite.bandwidth_matrix import scenario_slots
from deft_controls_sdk.debug.suite.gen2_bandwidth import (
    GEN2_PRODUCT_BY_BUS,
    gen2_default_scenarios_for_matrix,
)
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

ROWS = gen2_product_rows()


def test_row_count_matches_actuator_count() -> None:
    assert len(ROWS) == ACTUATOR_COUNT == 26


def test_arm_slots_are_cubemars_then_robstride() -> None:
    for arm_slots in (GEN2_LEFT_ARM_SLOTS, GEN2_RIGHT_ARM_SLOTS):
        protos = [ROWS[s][2] for s in arm_slots]
        assert protos == [
            PROTO_CUBEMARS, PROTO_CUBEMARS, PROTO_CUBEMARS, PROTO_CUBEMARS,
            PROTO_ROBSTRIDE, PROTO_ROBSTRIDE, PROTO_ROBSTRIDE, PROTO_ROBSTRIDE,
        ]


def test_arms_are_on_ch1_ch2_respectively() -> None:
    assert all(ROWS[s][0] == 1 for s in GEN2_LEFT_ARM_SLOTS)
    assert all(ROWS[s][0] == 2 for s in GEN2_RIGHT_ARM_SLOTS)


def test_torso_is_mixed_protocol_on_ch3() -> None:
    """CubeMars (yaw) first, then ZeroErr x3 (L1/L2/pitch) — matches the
    live board CFG order confirmed via `show --cfg`."""
    protos = [ROWS[s][2] for s in GEN2_TORSO_SLOTS]
    assert protos == [PROTO_CUBEMARS, PROTO_ZEROERR, PROTO_ZEROERR, PROTO_ZEROERR]
    assert all(ROWS[s][0] == 3 for s in GEN2_TORSO_SLOTS)


def test_base_is_two_robstride_per_mcp_rail() -> None:
    assert len(GEN2_BASE_SLOTS) == 6
    buses = [ROWS[s][0] for s in GEN2_BASE_SLOTS]
    assert buses == [4, 4, 5, 5, 6, 6]
    assert all(ROWS[s][2] == PROTO_ROBSTRIDE for s in GEN2_BASE_SLOTS)


def test_no_motor_id_collisions_within_a_bus_protocol_group() -> None:
    """Arm buses (CubeMars vs RobStride) are safe by CAN frame type alone —
    CubeMars MIT and RobStride are both std-ID vs ext-ID respectively (see
    plugin id_type checks). Torso (CubeMars MIT vs ZeroErr CANopen) is NOT
    frame-type-disjoint — both are std-ID on CH3 — so that pairing is only
    safe because today's placeholder IDs (CubeMars=1, ZeroErr nodes=1-3 ->
    COB-IDs >=0x080) happen not to land in the same numeric slot; see the
    CAN ID note in gen2.py's module docstring. Either way, the real hazard
    checked here is same-protocol nodes on the same bus sharing a motor_id."""
    groups: dict[tuple[int, int], list[int]] = defaultdict(list)
    for bus, enabled, proto, motor_id, _master in ROWS:
        if enabled:
            groups[(bus, proto)].append(motor_id)
    for key, ids in groups.items():
        assert len(ids) == len(set(ids)), f"duplicate motor_id in bus/protocol {key}: {ids}"


def test_gen2_slots_by_bus_matches_rows() -> None:
    by_bus = gen2_slots_by_bus()
    assert by_bus[1] == list(GEN2_LEFT_ARM_SLOTS)
    assert by_bus[2] == list(GEN2_RIGHT_ARM_SLOTS)
    assert by_bus[3] == list(GEN2_TORSO_SLOTS)
    assert by_bus[4] == [20, 21]
    assert by_bus[5] == [22, 23]
    assert by_bus[6] == [24, 25]


def test_ch3_scenario_resolves_to_mixed_protocol_torso() -> None:
    """The scenario that actually matters for mixed-protocol bandwidth/function
    testing — CH3 under the Gen2 map, unlike the old product map where it's
    empty (see PRODUCT_BY_BUS's `3: []` in bandwidth_matrix.py)."""
    assert scenario_slots("ch3", GEN2_PRODUCT_BY_BUS) == list(GEN2_TORSO_SLOTS)


def test_all_scenario_covers_every_slot() -> None:
    assert scenario_slots("all", GEN2_PRODUCT_BY_BUS) == list(range(26))


def test_default_scenarios_include_ch3() -> None:
    assert "ch3" in gen2_default_scenarios_for_matrix()


def test_actuator_profiles_build_without_error() -> None:
    for profile in (
        gen2_left_arm_profile(),
        gen2_right_arm_profile(),
        gen2_torso_profile(),
        gen2_base_wheel_profile(4),
        gen2_all_profile(),
    ):
        rows = profile.as_cfg_rows()
        assert len(rows) == len(profile.slots)


def test_base_wheel_profile_rejects_fdcan_bus() -> None:
    import pytest

    with pytest.raises(ValueError):
        gen2_base_wheel_profile(1)
