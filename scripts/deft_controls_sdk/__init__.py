"""deft_controls_sdk — USB host SDK for the Deft controls PCB.

Pillars:
    actions/   — plant CMDH behaviour (ActuatorAction, LedAction, ServoAction, …)
    config/    — profiles, slot maps, CFG row builders, LED/servo/pdu identity
    debug/     — discover, inventory, CFG wire RPC, cal, Soft-DFU, suite
    telemetry/ — FB cache / recorder
    link/      — Connection, wire layout, Desire types

Facades:
    ControlsPcbHub — board USB (slots, hub.debug, telemetry)
    HostProxy      — profile demux over hub (preferred plant API)
    vbeta/         — YAM drivers on HostProxy

    from deft_controls_sdk import HostProxy, ControlsPcbHub, ActuatorDesire
    from deft_controls_sdk.actions import ActuatorAction
    from deft_controls_sdk.config import yam_product_profile

Lab app (outside this package): ``python -m pcb_lab``

Canonical docs: docs/host-contract.md, docs/integration.md.
"""
from .actions import ActuatorAction
from .config import (
    Profile,
    bench_continuous_profile,
    yam_product_profile,
)
from .controls_pcb_hub import ControlsPcbHub
from .debug import (
    DebugAPI,
    enter_bootloader,
    find_cdc_port,
    flash_firmware,
    leave_bootloader,
    list_cdc_ports,
)
from .host_proxy import HostProxy
from .link import ActuatorDesire, LedDesire, McuState, ServoDesire
from .telemetry import SessionState, TelemetryCache

__all__ = [
    "ActuatorAction",
    "ActuatorDesire",
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
