#!/usr/bin/env python3
"""Single Damiao (DM0) actuator bringup on an existing bus/slot.

  cd scripts
  python damiao_channel_bringup.py --bus 2 --slot 8 --motor-id 0x01
  python damiao_channel_bringup.py --bus 2 --slot 9 --motor-id 0x02 --known-ids 0x01,0x02,0x03

Steps: discover/verify id -> CFG slot (add-only) -> metrics hold -> tiny teleop.
No calibration step: Damiao (MIT protocol) has no RS02-style encoder cali.
SDK-only (no scripts/legacy).

Unlike rs02_channel_bringup.py, this does NOT disable other slots on the bus:
a Damiao bus is typically a daisy chain of several motors (e.g. one arm on
CH2), and touching sibling slots would kick other legitimate joints off the
bus. --slot is required for the same reason -- there is no single canonical
per-bus slot the way RS02 has one motor per bus.
"""
from __future__ import annotations

import argparse
import sys
from typing import Dict, List, Optional

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from deft_controls_sdk.bench.metrics import measure_hold
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from rs02_channel_bringup import sample_position, tiny_teleop

PROTO_DAMIAO = 3
DEFAULT_MASTER_ID_OFFSET = 0x10  # docs/lessons.md: "Master ID: typically ESC_ID + 0x10"


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _parse_motor_id(raw: str) -> int:
    text = raw.strip().lower()
    if text.startswith("0x"):
        return int(text, 16) & 0xFF
    return int(text, 0) & 0xFF


def _parse_known_ids(raw: Optional[str]) -> List[int]:
    if not raw:
        return []
    return [_parse_motor_id(tok) for tok in raw.split(",") if tok.strip()]


def assign_damiao_slot(
    hub: ControlsPcbHub,
    *,
    bus: int,
    slot: int,
    motor_id: int,
    master_id: int,
    persist: bool,
) -> None:
    """Enable ``slot`` for this one Damiao motor. Does not touch other slots
    on the bus -- a Damiao daisy chain expects multiple enabled motors."""
    print(
        f"CFG: enable slot {slot} on CH{bus} id=0x{motor_id:02X} "
        f"master=0x{master_id:02X} (persist={persist}) -- other slots on this "
        "bus left untouched (daisy chain)"
    )
    hub.debug.cfg_set_slot(
        slot=slot,
        bus=bus,
        protocol=PROTO_DAMIAO,
        motor_id=motor_id,
        master_id=master_id,
        enabled=True,
        persist=persist,
    )
    table = hub.debug.cfg_get_table()
    row = table[slot]
    print(
        f"  verify slot {slot}: bus={row.get('bus')} proto={row.get('protocol')} "
        f"id=0x{int(row.get('motor_id', 0)):02X} master=0x{int(row.get('master_id', 0)):02X} "
        f"en={row.get('enabled')}"
    )


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Damiao (DM0) single-actuator bringup on an existing bus (SDK-only)"
    )
    ap.add_argument("--bus", type=int, required=True, help="CAN bus this motor is wired to")
    ap.add_argument("--slot", type=int, required=True, help="CFG slot for this motor (no canonical default -- daisy chain)")
    ap.add_argument("--port", default=None, help="CDC port (default: auto-find)")
    ap.add_argument("--motor-id", default=None, help="ESC id, e.g. 0x01 (default: discover)")
    ap.add_argument("--master-id", default=None, help="Feedback CAN id (default: motor_id + 0x10, per docs/lessons.md)")
    ap.add_argument("--known-ids", default=None, help="Comma list of already-configured ids on this bus (scan-order safety)")
    ap.add_argument("--id-start", type=int, default=1, help="Discover sweep start id")
    ap.add_argument("--id-end", type=int, default=16, help="Discover sweep end id")
    ap.add_argument("--listen-ms", type=int, default=40, help="Per-probe listen window (ms)")
    ap.add_argument("--seconds", type=float, default=3.0, help="Metrics hold window")
    ap.add_argument("--hz", type=float, default=40.0, help="Host plant TX rate")
    ap.add_argument("--persist", action="store_true", help="CFG flash SAVE for the test slot")
    ap.add_argument("--delta", type=float, default=0.15, help="Teleop step (rad)")
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--kd", type=float, default=0.5)
    args = ap.parse_args(argv)

    bus = int(args.bus)
    slot = int(args.slot)
    if not (0 <= slot < ACTUATOR_COUNT):
        print(f"bad --slot {slot} (need 0..{ACTUATOR_COUNT - 1})", file=sys.stderr)
        return 2

    known_ids = _parse_known_ids(args.known_ids)
    motor_id = _parse_motor_id(args.motor_id) if args.motor_id is not None else None

    results: Dict[str, bool] = {}
    notes: List[str] = []

    print(f"Opening port={args.port or 'auto'}  bus=CH{bus}  slot={slot}")
    with ControlsPcbHub.connect(args.port) as hub:
        hub.recover()

        # --- discover / verify ---
        if motor_id is None:
            print(f"\n--- discover on CH{bus} ---")
            motor_id = hub.debug.discover_damiao(
                bus=bus,
                start=args.id_start,
                end=args.id_end,
                listen_ms=args.listen_ms,
                known_ids=known_ids,
            )
            if motor_id is None:
                print("FAIL: no Damiao motor found -- check cable/power/id range")
                return 1
        else:
            # No single-id probe exists for Damiao; a 1-wide discover sweep
            # against the known id serves the same purpose without duplicating
            # protocol logic.
            print(f"\n--- verify id=0x{motor_id:02X} on CH{bus} ---")
            hit = hub.debug.discover_damiao(
                bus=bus,
                start=motor_id,
                end=motor_id,
                listen_ms=args.listen_ms,
                known_ids=[motor_id],
            )
            if hit is None:
                print("FAIL: motor did not respond -- check bus/id")
                return 1
        notes.append(f"motor_id=0x{motor_id:02X}")

        master_id = (
            _parse_motor_id(args.master_id)
            if args.master_id is not None
            else (motor_id + DEFAULT_MASTER_ID_OFFSET) & 0xFF
        )
        notes.append(f"master_id=0x{master_id:02X} (confirm via regs 0x08/0x07 if unsure)")

        # --- CFG (add-only; sibling slots on this bus untouched) ---
        print("\n--- CFG slot (add-only) ---")
        assign_damiao_slot(
            hub, bus=bus, slot=slot, motor_id=motor_id, master_id=master_id, persist=bool(args.persist)
        )

        # --- metrics ---
        print("\n--- metrics hold ---")
        desire = ActuatorDesire(position=0.0, kp=float(args.kp), kd=float(args.kd))
        pos = sample_position(hub, slot)
        if pos is not None:
            desire = ActuatorDesire(position=pos, kp=float(args.kp), kd=float(args.kd))
            print(f"  holding current fb pos={pos:+.4f}")
        metrics = measure_hold(
            hub,
            f"CH{bus}_slot{slot}",
            {slot: desire},
            seconds=float(args.seconds),
            hz=float(args.hz),
        )
        results["metrics"] = bool(metrics.get("ok"))
        notes.append(
            f"fb_hz={metrics.get('raw_fb_hz')} ack_max={metrics.get('ack_lag_max')} "
            f"lap_max={metrics.get('lap_max_ms')}"
        )
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

        # --- no calibration step: Damiao/MIT has no RS02-style encoder cali ---
        print("\n--- cali: n/a (Damiao has no RS02-style encoder calibration) ---")
        results["cali"] = True
        notes.append("cali=n/a")

        # --- tiny teleop ---
        print("\n--- tiny teleop ---")
        ok_tele, tele_msg = tiny_teleop(
            hub,
            slot=slot,
            delta=float(args.delta),
            kp=float(args.kp),
            kd=float(args.kd),
        )
        results["teleop"] = ok_tele
        notes.append(tele_msg)
        print(f"  {tele_msg}")
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=True
        )

    print("\n=== SUMMARY ===")
    for name, ok in results.items():
        print(f"  {name}: {'PASS' if ok else 'FAIL'}")
    for n in notes:
        print(f"  - {n}")
    overall = all(results.values()) if results else False
    print(f"OVERALL: {'PASS' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
