"""Deft controls PCB host tools — protocol, session, plugins, CLI."""
from .cli.main import main
from .transport import list_serial_ports

__all__ = ["main", "list_serial_ports"]
