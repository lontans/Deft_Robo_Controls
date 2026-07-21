"""deft_controls_sdk — USB host SDK for the Deft controls PCB.

Software:
    from deft_controls_sdk import ControlsPcbHub

    with ControlsPcbHub.connect("COM5") as hub:
        hub.start_streaming()
        with hub.debug.lease(bus=2):
            hub.debug.discover_robstride(bus=2)

Human (localhost telemetry UI):
    python -m deft_controls_sdk.debug_dashboard --port COM5

Layout: controls_pcb_hub (façade) + link/ (exchange pack/parse/bench + Connection)
+ bench/ (DEBUG-mode discover/config, under a lease on the same Connection)
+ telemetry/ + debug_dashboard/ (UI only; never opened by hub).

This package is insular — it does not import scripts/legacy packages.
"""
from .controls_pcb_hub import ControlsPcbHub
from .bench import DebugAPI
from .link import ActuatorDesire, McuState
from .telemetry import SessionState, TelemetryCache

__all__ = [
    "ActuatorDesire",
    "ControlsPcbHub",
    "DebugAPI",
    "McuState",
    "SessionState",
    "TelemetryCache",
]
