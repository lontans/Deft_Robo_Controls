"""Compatibility shim — use ``deft_controls_sdk.debug.suite.cli``."""
from __future__ import annotations

from deft_controls_sdk.debug.suite.cli import *  # noqa: F403
from deft_controls_sdk.debug.suite.cli import _build_parser, main
