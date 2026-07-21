"""Actuator slot configuration — re-exports the host mirror of plant_config.c.

When a USB session is available, controls_pcb_host `config` commands read/write
the MCU table via the CFG PDU (with optional flash NVM persist).
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
