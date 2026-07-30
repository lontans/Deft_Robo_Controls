"""Offline tests for suite bandwidth_matrix helpers (no hardware)."""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug.suite.bandwidth_matrix import (
    PRODUCT_BY_BUS,
    parse_hz_list,
    render_report,
    scenario_slots,
)

BY_BUS = PRODUCT_BY_BUS


def test_parse_hz_list_splits_and_casts() -> None:
    assert parse_hz_list("40,100, 200,500") == [40.0, 100.0, 200.0, 500.0]


def test_parse_hz_list_rejects_empty() -> None:
    with pytest.raises(ValueError):
        parse_hz_list("")


def test_scenario_slots_idle_is_empty() -> None:
    assert scenario_slots("idle", BY_BUS) == []


def test_scenario_slots_single_bus() -> None:
    assert scenario_slots("ch1", BY_BUS) == [0, 1, 2, 3, 4, 5, 6]
    assert scenario_slots("ch3", BY_BUS) == []


def test_scenario_slots_mcp_is_ch4_6_union() -> None:
    assert scenario_slots("mcp", BY_BUS) == [14, 15, 16, 17, 18, 19]


def test_scenario_slots_all_is_every_enabled_slot() -> None:
    assert scenario_slots("all", BY_BUS) == list(range(20))


def test_scenario_slots_rejects_unknown_name() -> None:
    with pytest.raises(ValueError):
        scenario_slots("ch9", BY_BUS)


def test_render_report_aggregates_by_rate() -> None:
    results = [
        {
            "hz": 40.0,
            "raw_fb_hz": 500.0,
            "ack_lag_max": 0,
            "lap_ms_mean": 1.0,
            "lap_max_ms": 13,
            "periph_lap_ms_mean": 0.0,
            "periph_lap_max_ms": 14,
            "ok": True,
        },
        {
            "hz": 40.0,
            "raw_fb_hz": 480.0,
            "ack_lag_max": 1,
            "lap_ms_mean": 1.2,
            "lap_max_ms": 14,
            "periph_lap_ms_mean": 0.0,
            "periph_lap_max_ms": 14,
            "ok": True,
        },
        {
            "hz": 500.0,
            "raw_fb_hz": 250.0,
            "ack_lag_max": 3,
            "lap_ms_mean": 2.0,
            "lap_max_ms": 18,
            "periph_lap_ms_mean": 0.1,
            "periph_lap_max_ms": 15,
            "ok": False,
        },
    ]
    text = render_report("all", results)
    assert "## Scenario: all" in text
    assert "| 40 | 2 |" in text
    assert "| 500 | 1 |" in text
    assert "2/2" in text  # 40 Hz both trials ok
    assert "0/1" in text  # 500 Hz trial not ok
