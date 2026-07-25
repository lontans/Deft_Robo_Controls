#!/usr/bin/env python3
"""Kick FDCAN1 via RS2 discover (SESSION_BEGIN restarts FDCAN), then Damiao discover."""
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

        print("RS2 discover bus=1 (forces FDCAN1 restart)", flush=True)
        try:
            hit = hub.debug.discover_robstride(bus=1, start=0x01, end=0x01)
            print("  rs hit", hit, flush=True)
        except Exception as e:
            print("  rs discover (expected miss)", type(e).__name__, e, flush=True)

        print("Damiao discover CH1 after kick", flush=True)
        ids = hub.debug.discover_damiao_all(bus=1, start=1, end=7, listen_ms=60)
        print("found", ids, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
