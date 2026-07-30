"""deft_controls_sdk — USB host SDK for the Deft controls PCB.

Shape:
    Hub (controls_pcb_hub) → wire / COM
    HostProxy              → component demux (preferred plant API)
    vbeta/                 → YAM drivers on HostProxy
    debug/                 → hub.debug (CFG / discover / Soft-DFU)
    link/                  → USB bytes + types (+ CubeMars MIT helpers)
    telemetry/, pdb/       → FB cache / PDB helpers
    debug_dashboard/       → human UI (owns COM while open)

    from deft_controls_sdk import HostProxy, ControlsPcbHub, ActuatorDesire

Lab app (outside this package): ``python -m pcb_lab doctor``

Canonical docs: docs/host-contract.md, docs/integration.md.
"""
from .controls_pcb_hub import ControlsPcbHub
from .debug import (
    DebugAPI,
    enter_bootloader,
    find_cdc_port,
    flash_firmware,
    leave_bootloader,
    list_cdc_ports,
)
from .host_proxy import (
    ComponentView,
    HostProxy,
    Profile,
    bench_continuous_profile,
    yam_product_profile,
)
from .link import ActuatorDesire, LedDesire, McuState, ServoDesire
from .telemetry import SessionState, TelemetryCache

__all__ = [
    "ActuatorDesire",
    "ComponentView",
    "ControlsPcbHub",
    "DebugAPI",
    "HostProxy",
    "LedDesire",
    "McuState",
    "Profile",
    "ServoDesire",
    "SessionState",
    "TelemetryCache",
    "bench_continuous_profile",
    "enter_bootloader",
    "find_cdc_port",
    "flash_firmware",
    "leave_bootloader",
    "list_cdc_ports",
    "yam_product_profile",
]
