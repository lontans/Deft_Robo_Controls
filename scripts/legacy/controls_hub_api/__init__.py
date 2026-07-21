"""controls_hub_api — top-level ControlsHub façade (plant / debug / health).

App code should import from here and pick a mode, not build 562 B frames or
choose `pdu` tags directly. See docs/plan-host-api-streamline.md and
docs/architecture.md §Host API modes.

    from controls_hub_api import ControlsHub, ActuatorDesire

    with ControlsHub.connect("COM5") as hub:
        hub.plant.set_actuator(3, ActuatorDesire(position=0.0, kp=8.0, kd=0.45))
        print(hub.health.snapshot())

`hub.debug` is a bench/diag lease on the same connection (not a second COM
open) — see ControlsHub/DebugAPI docstrings in `hub.py`. `controls_pcb_host`
/ `PcbSession` remain the richer bench CLI path (discover/calibrate/config)
and must not run concurrently against a port a ControlsHub already holds.
"""
from controls_hub_controller import ActuatorDesire, McuState, PlantBlockReason  # re-export

from .hub import ControlsHub, DebugAPI, HealthAPI, HealthSnapshot, PlantAPI

__all__ = [
    "ActuatorDesire",
    "ControlsHub",
    "DebugAPI",
    "HealthAPI",
    "HealthSnapshot",
    "McuState",
    "PlantAPI",
    "PlantBlockReason",
]
