#!/usr/bin/env python3
"""Force Damiao DIAG discover on schematic bus 1."""
from __future__ import annotations

from deft_controls_sdk import ControlsPcbHub, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port


def main() -> int:
    port = find_cdc_port()
    print("port", port, flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        hub.set_rx_sim_mask(0)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        print("discover_damiao bus=1 ids 1..7", flush=True)
        ids = hub.debug.discover_damiao(bus=1, start=1, end=7, listen_ms=50)
        print("found", [f"0x{i:02X}" for i in ids], flush=True)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
