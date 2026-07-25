#!/usr/bin/env python3
"""Set MCU DIAG_ONLY + blank actuators + cornflower idle LED (leave streaming)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link import FeedbackImage
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT


def main() -> int:
    port = find_cdc_port()
    print("port", port)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        for s in range(ACTUATOR_COUNT):
            hub.set_actuator(s, ActuatorDesire(), send=False)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
            send=False,
        )
        hub.send_once()
        hub.start_streaming(hz=5.0)
        time.sleep(0.8)
        raw = hub._connection._latest_fb_raw  # noqa: SLF001
        if raw:
            img = FeedbackImage(raw)
            print("mcu_state", int(img.mcu_state), "(2=DIAG_ONLY)")
        print(
            "DIAG_ONLY + cornflower desire held at 5 Hz.\n"
            "If strip stays red: PDB override (HARD/stale/estop_sense) wins over host mode 8 — "
            "not an MCU-state miss."
        )
        # Keep stream so LED/state hold; caller closes hub on exit which stops stream.
        # Sleep a bit so user can see cornflower if PDB allows it.
        time.sleep(1.0)
        hub.stop_streaming()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
