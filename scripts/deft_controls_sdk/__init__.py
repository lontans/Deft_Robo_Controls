"""deft_controls_sdk — USB host SDK for the Deft controls PCB.

Software:
    from deft_controls_sdk import ControlsPcbHub, find_cdc_port, flash_firmware

    with ControlsPcbHub.connect() as hub:  # auto-picks 0483:5740
        hub.start_streaming()
        hub.debug.cfg_get_table()

Flash (no ST-Link):
    python soft_dfu_flash.py
    # or: flash_firmware(confirm=True)

Canonical docs: docs/api.md (call surface), docs/host-exchange-v2.md (672 B wire).
This package does not import scripts/legacy/.
"""
from .controls_pcb_hub import ControlsPcbHub
from .bench import (
    DebugAPI,
    enter_bootloader,
    find_cdc_port,
    flash_firmware,
    leave_bootloader,
    list_cdc_ports,
)
from .link import ActuatorDesire, LedDesire, McuState, ServoDesire
from .telemetry import SessionState, TelemetryCache

__all__ = [
    "ActuatorDesire",
    "ControlsPcbHub",
    "DebugAPI",
    "LedDesire",
    "McuState",
    "ServoDesire",
    "SessionState",
    "TelemetryCache",
    "enter_bootloader",
    "find_cdc_port",
    "flash_firmware",
    "leave_bootloader",
    "list_cdc_ports",
]
