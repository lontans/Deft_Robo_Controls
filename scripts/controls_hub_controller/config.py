"""Actuator slot configuration — re-exports the host mirror of plant_config.c.

controls_pcb_host.actuator_config already carries this table (kept in sync by hand
with the C source; there is no live config-readback PDU yet — see the design doc's
gap list). This module just gives controls_hub_controller callers a stable import
path that does not pull in controls_pcb_host's RS2/DM/DXL bench CLI surface.
"""
from __future__ import annotations

from controls_pcb_host.actuator_config import (
    ActuatorSlotConfig,
    Protocol,
    default_table,
    host_table,
    slot_config,
)

__all__ = ["ActuatorSlotConfig", "Protocol", "default_table", "host_table", "slot_config"]
