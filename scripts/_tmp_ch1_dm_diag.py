#!/usr/bin/env python3
from __future__ import annotations

import time

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.bench import damiao as dm
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import FeedbackImage
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, parse_feedback_header
from deft_controls_sdk.link.exchange.bench import (
    DM_PROBE_CLEAR_FAULT,
    DM_PROBE_ENABLE,
    DM_PROBE_MIT,
)


def main() -> int:
    port = find_cdc_port()
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        hub.set_mcu_state(McuState.NORMAL, send=True)
        hub.set_rx_sim_mask(0)
        hub._connection.set_actuators(  # noqa: SLF001
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )
        for _ in range(15):
            hub._connection.send_once()  # noqa: SLF001
            time.sleep(0.05)
            while True:
                raw = hub._connection.reader.pop()  # noqa: SLF001
                if raw is None:
                    break
                hdr = parse_feedback_header(raw)
                if hdr:
                    print(
                        "hdr",
                        {
                            k: hdr.get(k)
                            for k in ("mcu_state", "plant_block", "tick")
                        },
                    )
                try:
                    fb = FeedbackImage(raw)
                    st = fb.actuator(0)
                    if st:
                        print(
                            f"  s0 pos={st.position:+.4f} fault=0x{int(st.fault):08X}"
                        )
                except Exception:
                    pass

        print("\ndiscover CH1")
        hit = hub.debug.discover_damiao(bus=1, start=1, end=8)
        print("HIT", None if hit is None else hex(hit))

        conn = hub._connection  # noqa: SLF001
        print("\nprobes")
        print("begin", bool(dm._dm_session_begin(conn, 1)))
        for mid in (1, 2, 6):
            for kind, name in (
                (DM_PROBE_CLEAR_FAULT, "CLR"),
                (DM_PROBE_ENABLE, "EN"),
                (DM_PROBE_MIT, "MIT"),
            ):
                r = dm._send_probe(
                    conn, mid, kind, bus=1, listen_ms=80, timeout_s=2.0
                )
                print(
                    mid,
                    name,
                    None
                    if r is None
                    else {
                        k: r.get(k)
                        for k in (
                            "found",
                            "err",
                            "raw_frames",
                            "master_id",
                            "position",
                        )
                    },
                )
        dm._dm_session_end(conn, 1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
