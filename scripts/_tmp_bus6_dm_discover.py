#!/usr/bin/env python3
"""Damiao-only discover on CH6 — same path as YAM arm discover_damiao()."""
from __future__ import annotations

import sys
import time
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import PROTO_DAMIAO, PROTO_ROBSTRIDE


def main() -> int:
    port = find_cdc_port()
    print(f"port {port}", flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        time.sleep(0.3)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        # Blank desires only — do not quiet CFG siblings.
        hub._connection.set_actuators(  # noqa: SLF001
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

        print("\n== CH6 Damiao discover (arm-style, no known_ids) ==", flush=True)
        hit = hub.debug.discover_damiao(bus=6, start=1, end=16)
        print(f"result: {None if hit is None else f'0x{hit:02X}'}", flush=True)

        print("\n== CH5 RS discover_all (sanity) ==", flush=True)
        rs5 = hub.debug.discover_robstride_all(bus=5, start=0x70, end=0x7F)
        print("CH5", [f"0x{i:02X}" for i in rs5], flush=True)

        print("\n== CH6 RS discover_all (expect empty if unplugged) ==", flush=True)
        rs6 = hub.debug.discover_robstride_all(bus=6, start=0x70, end=0x7F)
        print("CH6 RS", [f"0x{i:02X}" for i in rs6], flush=True)

        # Apply accurate base CFG from what we see
        ch5_a = 0x70 if 0x70 in rs5 else (rs5[0] if rs5 else None)
        ch5_b = next((x for x in rs5 if x != ch5_a), None)
        ch6_rs = rs6[0] if rs6 else None
        ch6_dm = hit
        ch6_dm_m = ((ch6_dm + 0x10) & 0xFF) if ch6_dm is not None else 0x11

        print("\n== APPLY CFG 22–25 (RAM) ==", flush=True)
        with pause_plant_stream(hub):
            table = hub.debug.cfg_get_table()
            for s, row in enumerate(table):
                if not row.get("enabled"):
                    continue
                b = int(row.get("bus", 0))
                if b in (5, 6) and s not in (22, 23, 24, 25):
                    hub.debug.cfg_set_slot(
                        slot=s,
                        bus=b,
                        protocol=int(row.get("protocol", 0)),
                        motor_id=int(row.get("motor_id", 0)),
                        master_id=int(row.get("master_id", 0)),
                        enabled=False,
                        persist=False,
                    )
            if ch5_a is not None:
                hub.debug.cfg_set_slot(
                    slot=22, bus=5, protocol=PROTO_ROBSTRIDE,
                    motor_id=ch5_a, master_id=0, enabled=True, persist=False,
                )
            if ch5_b is not None:
                hub.debug.cfg_set_slot(
                    slot=23, bus=5, protocol=PROTO_ROBSTRIDE,
                    motor_id=ch5_b, master_id=0, enabled=True, persist=False,
                )
            if ch6_rs is not None:
                hub.debug.cfg_set_slot(
                    slot=24, bus=6, protocol=PROTO_ROBSTRIDE,
                    motor_id=ch6_rs, master_id=0, enabled=True, persist=False,
                )
            else:
                hub.debug.cfg_set_slot(
                    slot=24, bus=6, protocol=PROTO_ROBSTRIDE,
                    motor_id=0x75, master_id=0, enabled=False, persist=False,
                )
            if ch6_dm is not None:
                hub.debug.cfg_set_slot(
                    slot=25, bus=6, protocol=PROTO_DAMIAO,
                    motor_id=ch6_dm, master_id=ch6_dm_m, enabled=True, persist=False,
                )
            else:
                hub.debug.cfg_set_slot(
                    slot=25, bus=6, protocol=PROTO_DAMIAO,
                    motor_id=0x01, master_id=0x11, enabled=False, persist=False,
                )

        print("\n== CFG 22–25 ==", flush=True)
        table = hub.debug.cfg_get_table()
        for i in (22, 23, 24, 25):
            r = table[i]
            print(
                f"  slot{i} en={r.get('enabled')} bus={r.get('bus')} "
                f"proto={r.get('protocol')} id=0x{int(r.get('motor_id', 0)):02X} "
                f"master=0x{int(r.get('master_id', 0)):02X}",
                flush=True,
            )

        ok = hit is not None
        print("\nOVERALL", "PASS" if ok else "FAIL (no Damiao HIT)", flush=True)
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
