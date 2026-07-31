"""Offline preset helpers for deft_controls_sdk.debug.suite."""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug.suite.presets import (
    ARM_PRESETS,
    apply_actuator_preset_to_rows,
    parse_slot_list,
)


def test_parse_slot_list_order_and_reverse():
    assert parse_slot_list("0,1,2,3,4,5,6", expect=7) == (0, 1, 2, 3, 4, 5, 6)
    assert parse_slot_list("6 5 4 3 2 1 0", expect=7) == (6, 5, 4, 3, 2, 1, 0)
    assert parse_slot_list("0,3,2,9,10,12,13", expect=7) == (0, 3, 2, 9, 10, 12, 13)
    with pytest.raises(ValueError):
        parse_slot_list("0,1,1,2,3,4,5", expect=7)


def test_apply_yam_arm_custom_slots():
    rows = [
        {
            "slot": s,
            "enabled": False,
            "bus": 1,
            "protocol": 0,
            "motor_id": 0,
            "master_id": 0,
        }
        for s in range(14)
    ]
    dirty = apply_actuator_preset_to_rows(
        rows, ARM_PRESETS["yam"], (6, 5, 4, 3, 2, 1, 0)
    )
    assert dirty == [6, 5, 4, 3, 2, 1, 0]
    by = {r["slot"]: r for r in rows}
    # joint0 (id 0x01) landed on slot 6
    assert by[6]["motor_id"] == 0x01 and by[6]["enabled"]
    assert by[0]["motor_id"] == 0x07
