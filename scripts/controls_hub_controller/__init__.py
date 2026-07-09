"""controls_hub_controller — normal-operation motor control API.

Production surface for commanding actuators via actuator_commands[] + mcu_state
over the 562 B host_exchange_schema.h contract. No teleop, ramping, homing, or
arrow-key logic; see docs/controls_hub-controller.md and
docs/controls_hub-controller-design.md.

    from controls_hub_controller import PlantSession, ActuatorDesire, McuState

    with PlantSession.connect("COM5") as session:
        session.set_actuator(3, ActuatorDesire(position=0.0, kp=8.0, kd=0.45))
"""
from .command import ActuatorDesire, CommandImage, IDLE, McuState
from .config import ActuatorSlotConfig, Protocol, host_table, slot_config
from .exceptions import (
    ControlsHubError,
    FeedbackTimeoutError,
    InvalidFrameError,
    InvalidSlotError,
    NotConnectedError,
)
from .feedback import FeedbackImage, FeedbackState, PlantBlockReason
from .hub_session import HubSession, LedCommand, ServoCommand
from .session import HOST_STALE_MS, PlantSession

__all__ = [
    "ActuatorDesire",
    "ActuatorSlotConfig",
    "CommandImage",
    "ControlsHubError",
    "FeedbackImage",
    "FeedbackState",
    "FeedbackTimeoutError",
    "HOST_STALE_MS",
    "HubSession",
    "IDLE",
    "InvalidFrameError",
    "InvalidSlotError",
    "LedCommand",
    "McuState",
    "NotConnectedError",
    "PlantBlockReason",
    "PlantSession",
    "Protocol",
    "ServoCommand",
    "host_table",
    "slot_config",
]
