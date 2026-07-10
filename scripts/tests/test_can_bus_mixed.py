"""Mixed std/ext FDCAN bus policy — no hardware."""
from __future__ import annotations

import pytest

from control_hub.link import PlantRuntimeError, assert_plant_teleop_slot
from controls_pcb_host.protocol.can_bus import (
    FDCAN_MIXED_BUSES,
    is_fdcan_bus,
    is_fdcan_mixed_bus,
)


def test_ch3_is_mixed_fdcan_bus() -> None:
    assert is_fdcan_mixed_bus(3)
    assert 3 in FDCAN_MIXED_BUSES
    assert is_fdcan_bus(3)


def test_assert_plant_teleop_robstride_ch3_ok() -> None:
    assert_plant_teleop_slot(0, 3, "robstride")


def test_assert_plant_teleop_damiao_ch1_ok() -> None:
    assert_plant_teleop_slot(2, 1, "damiao")


def test_assert_plant_teleop_damiao_mcp_rejected() -> None:
    with pytest.raises(PlantRuntimeError, match="FDCAN"):
        assert_plant_teleop_slot(2, 4, "damiao")


def test_assert_plant_teleop_robstride_invalid_bus() -> None:
    with pytest.raises(ValueError):
        assert_plant_teleop_slot(0, 99, "robstride")
