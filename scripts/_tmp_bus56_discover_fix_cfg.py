#!/usr/bin/env python3
"""Discover CH5/CH6 motors and rewrite base CFG slots 22–25 (RAM)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.bench.metrics import measure_hold
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import PROTO_DAMIAO, PROTO_ROBSTRIDE
from rs02_channel_bringup import sample_position, seed_idle_at_fb

PROTO_NAMES = {0: "none", 1: "RS", 2: "ZE", 3: "DM", 4: "CM"}


def _print_table(table, *, force_slots=()):
    for i, r in enumerate(table):
        bus = int(r.get("bus", 0))
        if not (r.get("enabled") or i in force_slots or bus in (5, 6)):
            continue
        print(
            f"  slot{i:02d} en={r.get('enabled')} bus={bus} "
            f"proto={PROTO_NAMES.get(int(r.get('protocol', 0)), r.get('protocol'))} "
            f"id=0x{int(r.get('motor_id', 0)):02X} "
            f"master=0x{int(r.get('master_id', 0)):02X}"
        )


def main() -> int:
    port = find_cdc_port()
    print(f"port {port}", flush=True)
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        hub.recover()
        time.sleep(0.2)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        hub._connection.set_actuators(  # noqa: SLF001
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

        print("\n== CFG TABLE (before) ==", flush=True)
        table = hub.debug.cfg_get_table()
        _print_table(table, force_slots=(22, 23, 24, 25))

        print("\n== DISCOVER CH5 RS ==", flush=True)
        rs5 = hub.debug.discover_robstride_all(bus=5, start=0x70, end=0x7F)
        print("CH5 RS hits", [f"0x{i:02X}" for i in rs5], flush=True)

        print("\n== DISCOVER CH6 RS ==", flush=True)
        rs6 = hub.debug.discover_robstride_all(bus=6, start=0x70, end=0x7F)
        print("CH6 RS hits", [f"0x{i:02X}" for i in rs6], flush=True)

        print("\n== DISCOVER CH6 Damiao ==", flush=True)
        dm6 = hub.debug.discover_damiao_all(bus=6, start=1, end=16)
        print("CH6 DM hits", [f"0x{i:02X}" for i in dm6], flush=True)

        ch5_70 = 0x70 if 0x70 in rs5 else (rs5[0] if rs5 else None)
        ch5_other = next((x for x in rs5 if x != ch5_70), None)
        ch6_rs = 0x70 if 0x70 in rs6 else (rs6[0] if rs6 else None)
        ch6_dm = dm6[0] if dm6 else None
        ch6_dm_master = ((ch6_dm + 0x10) & 0xFF) if ch6_dm is not None else 0x11

        print("\n== APPLY BASE CFG (RAM) ==", flush=True)
        print(
            f" plan: s22 CH5 RS 0x{ch5_70:02X}"
            if ch5_70 is not None
            else " plan: s22 missing",
            flush=True,
        )
        print(
            f" plan: s23 CH5 RS 0x{ch5_other:02X}"
            if ch5_other is not None
            else " plan: s23 missing",
            flush=True,
        )
        print(
            f" plan: s24 CH6 RS 0x{ch6_rs:02X}"
            if ch6_rs is not None
            else " plan: s24 missing",
            flush=True,
        )
        print(
            f" plan: s25 CH6 DM 0x{ch6_dm:02X}/m0x{ch6_dm_master:02X}"
            if ch6_dm is not None
            else " plan: s25 missing",
            flush=True,
        )

        with pause_plant_stream(hub):
            for s, row in enumerate(table):
                if not row.get("enabled"):
                    continue
                b = int(row.get("bus", 0))
                mid = int(row.get("motor_id", 0))
                proto = int(row.get("protocol", 0))
                if b in (5, 6) and s not in (22, 23, 24, 25):
                    print(
                        f"  disable stray slot{s} bus={b} id=0x{mid:02X}",
                        flush=True,
                    )
                    hub.debug.cfg_set_slot(
                        slot=s,
                        bus=b,
                        protocol=proto,
                        motor_id=mid,
                        master_id=int(row.get("master_id", 0)),
                        enabled=False,
                        persist=False,
                    )

            if ch5_70 is not None:
                hub.debug.cfg_set_slot(
                    slot=22,
                    bus=5,
                    protocol=PROTO_ROBSTRIDE,
                    motor_id=ch5_70,
                    master_id=0,
                    enabled=True,
                    persist=False,
                )
            if ch5_other is not None:
                hub.debug.cfg_set_slot(
                    slot=23,
                    bus=5,
                    protocol=PROTO_ROBSTRIDE,
                    motor_id=ch5_other,
                    master_id=0,
                    enabled=True,
                    persist=False,
                )
            elif ch5_70 is not None:
                hub.debug.cfg_set_slot(
                    slot=23,
                    bus=5,
                    protocol=PROTO_ROBSTRIDE,
                    motor_id=0x74,
                    master_id=0,
                    enabled=False,
                    persist=False,
                )
            if ch6_rs is not None:
                hub.debug.cfg_set_slot(
                    slot=24,
                    bus=6,
                    protocol=PROTO_ROBSTRIDE,
                    motor_id=ch6_rs,
                    master_id=0,
                    enabled=True,
                    persist=False,
                )
            if ch6_dm is not None:
                hub.debug.cfg_set_slot(
                    slot=25,
                    bus=6,
                    protocol=PROTO_DAMIAO,
                    motor_id=ch6_dm,
                    master_id=ch6_dm_master,
                    enabled=True,
                    persist=False,
                )
            else:
                # Don't leave factory RS junk enabled on the Damiao slot.
                hub.debug.cfg_set_slot(
                    slot=25,
                    bus=6,
                    protocol=PROTO_DAMIAO,
                    motor_id=0x01,
                    master_id=0x11,
                    enabled=False,
                    persist=False,
                )

        print("\n== CFG TABLE (after) ==", flush=True)
        table = hub.debug.cfg_get_table()
        _print_table(table, force_slots=(22, 23, 24, 25))

        if ch5_70 is not None:
            resp = hub.debug.probe_robstride(bus=5, motor_id=ch5_70)
            pos = float(resp["position"]) if resp and resp.get("found") else 0.0
            seed_idle_at_fb(hub, 22, pos)
            hub.refresh_feedback(slots=[22], seconds=0.4, hz=40)
            q = sample_position(hub, 22) or pos
            m = measure_hold(
                hub,
                "tag_check",
                {22: ActuatorDesire(position=q, kp=8.0, kd=0.5)},
                seconds=1.5,
                hz=40.0,
            )
            print("\n== METRICS TAG CHECK ==", flush=True)
            print(
                f" ok={m.get('ok')} ok_plant_tag={m.get('ok_plant_tag')} "
                f"pdu_tags={m.get('pdu_tags')} fb_hz={m.get('raw_fb_hz')}",
                flush=True,
            )

        ok = bool(rs5) and bool(rs6) and bool(dm6)
        print("\nOVERALL", "PASS" if ok else "PARTIAL/FAIL", flush=True)
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
