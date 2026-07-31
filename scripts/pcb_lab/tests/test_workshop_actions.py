"""Offline tests: NVM gate helpers, assembly mutators, actions teleop/operate."""
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
    profile_cfg_status,
    profile_in_nvm,
    spin_jog,
)
from deft_controls_sdk.actions.cfg_identity import table_by_slot  # noqa: E402
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


def test_cfg_row_matches_and_profile_in_nvm():
    prof = single_profile(22, protocol="robstride", motor_id=0x70, bus=5, kind="wheel")
    table = [None] * 26
    table[22] = _live_row()
    assert profile_in_nvm(prof, table) is True
    assert cfg_row_matches(prof.as_cfg_row(), table[22]) is True

    table[22] = _live_row(enabled=False)
    assert profile_in_nvm(prof, table) is False

    table[22] = _live_row(motor_id=0x71)
    status = profile_cfg_status(prof, table)
    assert status[0][0] == 22 and status[0][1] is False


def test_profile_cfg_status_without_cfg_rows():
    from deft_controls_sdk.config import ActuatorProfile

    bare = ActuatorProfile(name="bare", slots=(22,), kind="wheel", cfg=())
    table = [_live_row()]
    # table_by_slot uses row.slot; index-only table needs slot key
    table = [None] * 26
    table[22] = _live_row()
    status = profile_cfg_status(bare, table)
    assert status[0][1] is False
    assert "no_profile_cfg" in status[0][2]
    assert profile_in_nvm(bare, table) is False


def test_assembly_put_and_remove():
    asm = yam_product_assembly()
    # Overlap product left_arm slot 0
    dup = single_profile(0, protocol="dm", motor_id=1, bus=1, kind="joint", name="dup0")
    with pytest.raises(ValueError, match="overlaps"):
        assembly_put_actuator(asm, dup)

    extra = single_profile(22, protocol="rs", motor_id=0x70, bus=5, kind="wheel", name="ch5")
    asm2 = assembly_put_actuator(asm, extra)
    assert asm2.actuator("ch5").slots == (22,)
    asm3 = assembly_remove_actuator(asm2, "ch5")
    assert "ch5" not in asm3.actuators


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


def test_table_by_slot():
    rows = [None, {"slot": 1, "bus": 2}]
    assert table_by_slot(rows)[1]["bus"] == 2
