"""Gen2 26-actuator bus map for bandwidth_matrix's scenario helpers.

Reuses ``scenario_slots()``/``render_report()``/``render_matrix_summary()``
from :mod:`bandwidth_matrix` unchanged — those already accept a ``by_bus``
override, so this module is just the Gen2 bus->slots map + scenario blurbs,
not a parallel implementation.

CH3 ("ch3" scenario) is the one that matters most here: under the Gen2 map
it resolves to the torso slots (16-19), which mix ZeroErr (CANopen) and
CubeMars (MIT) on one physical bus — the real mixed-protocol-per-bus case,
not just adjacency. Include it explicitly in the default scenario list
rather than leaving it implicit alongside ch1/ch2/etc.
"""
from __future__ import annotations

from typing import Dict, List

from deft_controls_sdk.config.gen2 import gen2_slots_by_bus

GEN2_PRODUCT_BY_BUS: Dict[int, List[int]] = gen2_slots_by_bus()

GEN2_SCENARIO_BLURBS: Dict[str, str] = {
    "idle": "no actuator hold (USB / empty plant)",
    "ch1": "left arm FDCAN (slots 0-7: CubeMars x4, RobStride x4)",
    "ch2": "right arm FDCAN (slots 8-15: CubeMars x4, RobStride x4)",
    "ch3": "torso FDCAN (slots 16-19) — MIXED PROTOCOL: ZeroErr x3 + CubeMars x1",
    "ch4": "base MCP rail CH4 (RobStride swerve+drive)",
    "ch5": "base MCP rail CH5 (RobStride swerve+drive)",
    "ch6": "base MCP rail CH6 (RobStride swerve+drive)",
    "fdcan": "CH1-3 union — both arms + mixed-protocol torso",
    "mcp": "CH4-6 union — SPI-CAN critical path, 2 actuators/rail",
    "arms": "CH1+CH2 arms only",
    "all": "every Gen2 slot (26)",
}


def gen2_default_scenarios_for_matrix() -> List[str]:
    """Scenarios worth comparing for Gen2 — ch3 called out explicitly since
    it's the only bus with genuine cross-protocol traffic, not just adjacency."""
    return ["idle", "ch1", "ch3", "mcp", "arms", "all"]


__all__ = [
    "GEN2_PRODUCT_BY_BUS",
    "GEN2_SCENARIO_BLURBS",
    "gen2_default_scenarios_for_matrix",
]
