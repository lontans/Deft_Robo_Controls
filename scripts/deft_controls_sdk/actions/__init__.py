"""actions — plant-mode behaviour (CMDH desires): actuators, LED, servos, PDU, teleop.

Debug board RPC lives in ``debug``; when a debug tool needs normal behaviour
it should call into this package.

Hierarchy::

    PlantAction
    ├── ActuatorAction   (profile / slot groups)
    ├── ServoAction
    ├── LedAction
    └── PduLinkAction

    TeleopEngine — cruise/jog slew (shared; dashboard may import later)
"""
from __future__ import annotations

from .actuator import ActuatorAction
from .led import (
    LedAction,
    led_follow,
    led_idle,
    led_off,
    led_pdu,
    set_led,
)
from .pdu_link import PduLinkAction
from .plant import PlantAction
from .servo import ServoAction
from .sink import LedSink, PlantSink, ServoSink
from .cfg_identity import (
    cfg_row_matches,
    format_slot_cfg_lines,
    profile_cfg_status,
    profile_in_nvm,
)
from .operate import (
    make_teleop_engine,
    move_arm_cruise,
    specs_for_cfg_map,
    spin_jog,
    stop_slots,
)
from .teleop import (
    SlotSpec,
    TeleopEngine,
    build_actuator_specs,
    build_servo_specs,
)

__all__ = [
    "ActuatorAction",
    "LedAction",
    "LedSink",
    "PduLinkAction",
    "PlantAction",
    "PlantSink",
    "ServoAction",
    "ServoSink",
    "SlotSpec",
    "TeleopEngine",
    "build_actuator_specs",
    "build_servo_specs",
    "cfg_row_matches",
    "format_slot_cfg_lines",
    "led_follow",
    "led_idle",
    "led_off",
    "led_pdu",
    "make_teleop_engine",
    "move_arm_cruise",
    "profile_cfg_status",
    "profile_in_nvm",
    "set_led",
    "specs_for_cfg_map",
    "spin_jog",
    "stop_slots",
]
