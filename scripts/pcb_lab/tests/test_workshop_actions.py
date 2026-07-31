"""Offline tests: NVM gate helpers, assembly mutators, actions teleop/operate, board verify."""
from __future__ import annotations

import os
import sys

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.actions import (  # noqa: E402
    TeleopEngine,
    build_actuator_specs,
    cfg_row_matches,
    make_teleop_engine,
    neck_cruise,
    profile_cfg_status,
    profile_in_table,
    spin_jog,
    stop_servos,
)
from deft_controls_sdk.actions import table_by_slot  # noqa: E402
from deft_controls_sdk.config import (  # noqa: E402
    PROTO_ROBSTRIDE,
    assembly_put_actuator,
    assembly_remove_actuator,
    single_profile,
    yam_product_assembly,
)


def _live_row(*, slot=22, bus=5, protocol=PROTO_ROBSTRIDE, motor_id=0x70, enabled=True):
    return {
        "slot": slot,
        "bus": bus,
        "protocol": protocol,
        "motor_id": motor_id,
        "master_id": 0,
        "enabled": enabled,
    }


def test_cfg_row_matches_and_profile_in_table():
    prof = single_profile(22, protocol="robstride", motor_id=0x70, bus=5)
    table = [None] * 26
    table[22] = _live_row()
    assert profile_in_table(prof, table) is True
    assert cfg_row_matches(prof.as_cfg_row(), table[22]) is True

    table[22] = _live_row(enabled=False)
    assert profile_in_table(prof, table) is False

    table[22] = _live_row(motor_id=0x71)
    status = profile_cfg_status(prof, table)
    assert status[0][0] == 22 and status[0][1] is False


def test_profile_cfg_status_without_cfg_rows():
    from deft_controls_sdk.config import ActuatorProfile

    bare = ActuatorProfile(name="bare", slots=(22,), cfg=())
    table = [_live_row()]
    # table_by_slot uses row.slot; index-only table needs slot key
    table = [None] * 26
    table[22] = _live_row()
    status = profile_cfg_status(bare, table)
    assert status[0][1] is False
    assert "no_profile_cfg" in status[0][2]
    assert profile_in_table(bare, table) is False


def test_assembly_put_and_remove():
    asm = yam_product_assembly()
    # Overlap product left_arm slot 0
    dup = single_profile(0, protocol="dm", motor_id=1, bus=1, name="dup0")
    with pytest.raises(ValueError, match="overlaps"):
        assembly_put_actuator(asm, dup)

    extra = single_profile(22, protocol="rs", motor_id=0x70, bus=5, name="ch5")
    asm2 = assembly_put_actuator(asm, extra)
    assert asm2.actuator("ch5").slots == (22,)
    asm3 = assembly_remove_actuator(asm2, "ch5")
    assert "ch5" not in asm3.actuators


def test_profile_with_cfg_helpers():
    from deft_controls_sdk.config import (
        ActuatorProfile,
        CfgSlotSpec,
        PROTO_ROBSTRIDE,
        cfg_specs_from_table,
        cfg_specs_from_yam_slots,
        wheel_profile,
    )

    bare = wheel_profile(name="base", bench=True)
    assert bare.as_cfg_rows() == []

    yam = bare.with_yam_cfg()
    # bench slots 22-25 are spare in product rows → still empty or sparse
    specs = cfg_specs_from_yam_slots((0, 1))
    assert specs[0] is not None and specs[0].protocol != 0

    table = [None] * 26
    table[22] = {
        "slot": 22,
        "bus": 5,
        "protocol": PROTO_ROBSTRIDE,
        "motor_id": 0x70,
        "master_id": 0,
        "enabled": True,
    }
    filled = bare.with_cfg_from_table(table)
    rows = filled.as_cfg_rows()
    assert len(rows) == 1 and rows[0]["slot"] == 22 and rows[0]["motor_id"] == 0x70

    manual = bare.with_cfg(
        (
            CfgSlotSpec(bus=5, protocol=PROTO_ROBSTRIDE, motor_id=0x70),
            CfgSlotSpec(bus=5, protocol=PROTO_ROBSTRIDE, motor_id=0x71),
            None,
            None,
        )
    )
    assert len(manual.as_cfg_rows()) == 2
    assert isinstance(manual, ActuatorProfile)
    assert cfg_specs_from_table((22,), table)[0].motor_id == 0x70


def test_actions_teleop_import_and_spin_jog():
    specs = build_actuator_specs("bench")
    assert 22 in specs and specs[22].verified

    held = {}

    class _Hub:
        def set_actuator(self, slot, desire, *, send=False):
            held[slot] = desire

        def set_servo(self, *a, **k):
            pass

    hub = _Hub()
    engine = make_teleop_engine(lambda: hub, hz=50.0)
    assert isinstance(engine, TeleopEngine)
    spin_jog(
        engine,
        slots=(22,),
        specs=specs,
        seeds={22: 0.0},
        direction=1,
        cruise=0.2,
    )
    snap = engine.snapshot()
    assert 22 in snap["actuators"]
    engine.stop()


def test_neck_cruise_engages_servos():
    held = {}

    class _Hub:
        def set_actuator(self, *a, **k):
            pass

        def set_servo(self, slot, desire, *, send=False):
            held[slot] = desire

    hub = _Hub()
    engine = make_teleop_engine(lambda: hub, hz=50.0)
    # Targets must sit inside teleop DXL rails (pitch lo=2193).
    neck_cruise(engine, pitch=2400, yaw=1500, cruise=100)
    snap = engine.snapshot()
    assert 0 in snap["servos"] and 1 in snap["servos"]
    assert snap["servos"][0]["target"] == 2400
    assert snap["servos"][1]["target"] == 1500
    stop_servos(engine)
    engine.stop()


def test_table_by_slot():
    rows = [None, {"slot": 1, "bus": 2}]
    assert table_by_slot(rows)[1]["bus"] == 2


def test_board_verify_cruise_specs_and_slot_parse():
    from deft_controls_sdk.debug.suite.board_verify import (
        _cruise_specs_for_slots,
        _parse_slot_list,
    )

    assert _parse_slot_list("22,23") == [22, 23]
    specs = _cruise_specs_for_slots([22, 0])
    assert 22 in specs and specs[22].lo is not None
    assert 0 in specs  # synthesized or arm/bench entry
    with pytest.raises(ValueError, match="range"):
        _parse_slot_list("99")
