#!/usr/bin/env python3
"""Compare CH1 Damiao discover vs CH5/CH6 RobStride/Damiao presence."""
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

        print("\n== CH1 Damiao 1..7 ==", flush=True)
        ids = hub.debug.discover_damiao_all(bus=1, start=1, end=7, listen_ms=50)
        print("CH1 found", ids, flush=True)

        print("\n== CH5 RobStride 0x70/0x74 ==", flush=True)
        for mid in (0x70, 0x74):
            hit = hub.debug.discover_robstride(bus=5, start=mid, end=mid)
            print(f"  0x{mid:02X} -> {hit}", flush=True)

        print("\n== CH6 RobStride 0x75 + Damiao 0x06 ==", flush=True)
        hit = hub.debug.discover_robstride(bus=6, start=0x75, end=0x75)
        print(f"  RS 0x75 -> {hit}", flush=True)
        ids6 = hub.debug.discover_damiao_all(bus=6, start=6, end=6, listen_ms=80)
        print("  DM CH6", ids6, flush=True)

        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
