"""Firmware image path helpers (identity); flash execution stays in debug.soft_dfu."""
from __future__ import annotations

from pathlib import Path


def default_firmware_elf() -> Path:
    """Newest Release/Debug ELF for Soft-DFU (delegates to debug.soft_dfu)."""
    from deft_controls_sdk.debug.soft_dfu import default_firmware_elf as _pick

    return _pick()


__all__ = ["default_firmware_elf"]
