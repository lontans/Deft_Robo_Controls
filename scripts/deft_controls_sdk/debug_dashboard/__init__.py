"""Localhost controller + telemetry UI.

This is now the one COM owner while connected (AppState opens/closes a
ControlsPcbHub in response to browser Connect/Disconnect) — it does import
serial transitively via ControlsPcbHub. That's a deliberate boundary change
from the read-only-viewer-only design: see app.py's module docstring."""
