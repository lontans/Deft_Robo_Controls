"""Exceptions for Connection / command image."""
from __future__ import annotations


class ControlsHubError(Exception):
    """Base exception for the connection / command path."""


class NotConnectedError(ControlsHubError):
    """Raised when a Connection method is used before open()/connect()."""


class InvalidSlotError(ControlsHubError, ValueError):
    """Raised when an actuator slot index is outside 0..ACTUATOR_COUNT-1."""


class InvalidFrameError(ControlsHubError, ValueError):
    """Raised when a received frame fails header/magic/size validation."""


class FeedbackTimeoutError(ControlsHubError, TimeoutError):
    """Raised when read_feedback() times out waiting for a frame."""
