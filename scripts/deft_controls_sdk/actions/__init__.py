"""actions — plant CMDH behaviour (desires), not CFG/discover RPC.

Primary session API::

    a = proxy.actions
    a.mount(a.actuator(slots=(22, 23)).hold([0.0, 0.0]))
    a.apply()
    a.clear()

Hierarchy::

    Actions  (mount / apply / clear / pending)
    └── PlantAction children: ActuatorAction, ServoAction, LedAction, PduLinkAction

    TeleopEngine / operate — shared cruise/jog slew (dashboard + board_verify)
    cfg_identity          — pure table checks (not wire RPC)

Not here: discover, CFG GET/SET/SAVE/LOAD, cal → ``hub.debug`` / ``debug.config``.
"""
from __future__ import annotations

from .actuator import ActuatorAction
from .cfg_identity import (
    cfg_row_matches,
    format_slot_cfg_lines,
    profile_cfg_status,
    profile_in_nvm,
    profile_in_table,
    table_by_slot,
)
from .facade import Actions
from .led import (
    LedAction,
    led_follow,
    led_idle,
    led_off,
    led_pdu,
    set_led,
)
from .mounted import MountedAction, merge_mounted
from .operate import (
    feedback_positions_from_proxy,
    make_teleop_engine,
    move_arm_cruise,
    neck_cruise,
    seed_for_slot,
    specs_for_cfg_map,
    spin_jog,
    stop_servos,
    stop_slots,
)
from .pdu_link import PduLinkAction
from .plant import PlantAction
from .servo import ServoAction
from .sink import LedSink, PlantSink, ServoSink
from .teleop import (
    SlotSpec,
    TeleopEngine,
    build_actuator_specs,
    build_servo_specs,
)

__all__ = [
    # Session façade
    "Actions",
    "MountedAction",
    "merge_mounted",
    # Plant actions
    "ActuatorAction",
    "LedAction",
    "PduLinkAction",
    "PlantAction",
    "ServoAction",
    # Sinks / protocols
    "LedSink",
    "PlantSink",
    "ServoSink",
    # LED helpers
    "led_follow",
    "led_idle",
    "led_off",
    "led_pdu",
    "set_led",
    # Teleop / operate
    "SlotSpec",
    "TeleopEngine",
    "build_actuator_specs",
    "build_servo_specs",
    "feedback_positions_from_proxy",
    "make_teleop_engine",
    "move_arm_cruise",
    "neck_cruise",
    "seed_for_slot",
    "specs_for_cfg_map",
    "spin_jog",
    "stop_servos",
    "stop_slots",
    # CFG identity (table checks only)
    "cfg_row_matches",
    "format_slot_cfg_lines",
    "profile_cfg_status",
    "profile_in_nvm",
    "profile_in_table",
    "table_by_slot",
]
