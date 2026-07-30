"""LED identity — policy presets (definitions only; apply via actions.LedAction)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from deft_controls_sdk.link import LED_MODE_IDLE_CORNFLOWER


@dataclass(frozen=True)
class LedPreset:
    name: str
    policy: str  # follow | debug | pdu
    pattern: int
    brightness: int
    blurb: str = ""


LED_PRESETS: Dict[str, LedPreset] = {
    "gen_2": LedPreset(
        name="gen_2",
        policy="follow",
        pattern=0,
        brightness=8,
        blurb="gen-2: NVM default cornflower + LedDesire follow",
    ),
    "idle": LedPreset(
        name="idle",
        policy="debug",
        pattern=LED_MODE_IDLE_CORNFLOWER,
        brightness=8,
        blurb="LedDesire debug cornflower + NVM default_mode=8",
    ),
    "pdu": LedPreset(
        name="pdu",
        policy="pdu",
        pattern=0,
        brightness=8,
        blurb="LedDesire pdu — force listen_pdu + traffic-light",
    ),
    "follow": LedPreset(
        name="follow",
        policy="follow",
        pattern=0,
        brightness=8,
        blurb="LedDesire follow — MCU uses NVM listen_pdu bit",
    ),
}

__all__ = ["LedPreset", "LED_PRESETS", "LED_MODE_IDLE_CORNFLOWER"]
