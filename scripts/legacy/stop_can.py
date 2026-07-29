#!/usr/bin/env python3
"""Blank all actuator desires + DIAG — stops CAN MIT after kill -9 leftover."""
from __future__ import annotations

import time

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT


def main() -> int:
    port = find_cdc_port()
    print("stop_can", port, flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        time.sleep(0.1)
        blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        for _ in range(12):
            hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
            hub._connection.set_actuators(blank, send=False)
            hub._connection.clear_servos(send=False)
            hub.set_led(
                LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8),
                send=False,
            )
            hub._connection.send_once()
            time.sleep(0.05)
    print("CAN blanked + DIAG", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
