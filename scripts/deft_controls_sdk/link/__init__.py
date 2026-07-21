"""Link layer: Connection (socket + held desires + streaming/telemetry) + exchange
bytes + typed API objects. No separate "session" class — see connection.py
docstring for why streaming/telemetry-publish live on Connection directly."""
from .api_types import (
    IDLE,
    ActuatorDesire,
    CommandImage,
    FeedbackImage,
    FeedbackState,
    McuState,
    PlantBlockReason,
)
from .connection import HOST_STALE_MS, Connection
from .exceptions import (
    ControlsHubError,
    FeedbackTimeoutError,
    InvalidFrameError,
    InvalidSlotError,
    NotConnectedError,
)

__all__ = [
    "IDLE",
    "ActuatorDesire",
    "CommandImage",
    "Connection",
    "ControlsHubError",
    "FeedbackImage",
    "FeedbackState",
    "FeedbackTimeoutError",
    "HOST_STALE_MS",
    "InvalidFrameError",
    "InvalidSlotError",
    "McuState",
    "NotConnectedError",
    "PlantBlockReason",
]
