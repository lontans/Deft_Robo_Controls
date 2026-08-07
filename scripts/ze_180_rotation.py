#!/usr/bin/env python3
"""One-off test: slow, smooth 180 deg rotation on a ZeroErr (CSP mode).

Reads the actual current position first (SDO, no enable needed), then
streams a smoothstep-eased ramp from there to +180 deg over --duration
seconds. Ctrl+C / any exit path blanks the desire and disarms before
closing the port.

    python scripts/ze_180_rotation.py --port COM5 --bus 1 --node-id 1
    python scripts/ze_180_rotation.py --port COM5 --bus 1 --node-id 1 --degrees -180 --duration 10

Only one process may own the CDC port at a time.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.actions import ActuatorAction  # noqa: E402
from deft_controls_sdk.config.actuator import PROTO_NONE  # noqa: E402
from deft_controls_sdk.debug.zeroerr import PROTO_ZEROERR  # noqa: E402
from deft_controls_sdk.host_proxy import HostProxy  # noqa: E402

# plant_config_nvm.c's plant_config_load_factory_defaults(): CH1 slots 0..7
# default to PROTO_ROBSTRIDE / motor_id 0x01..0x08 / enabled=true on every
# boot (RAM-only CFG overrides don't survive reflash). robstride_apply_cycle
# idle-anchors an enabled slot with a pararead poll every ~50ms even at
# kp=0 — with nothing physically on those IDs, that's real CH1 CAN traffic
# competing with the ZeroErr node the moment plant control is armed. Clear
# them (RAM-only, no NVM touch) before we arm anything.
_CH1_FACTORY_SLOTS = range(8)

# CiA402 statusword state decode (DS402 6.3): mask 0x006F against the raw
# statusword. Bench-confirmed the drive can drop straight to
# switch_on_disabled on a following-error trip WITHOUT ever setting the
# Fault bit (0x0008) — so "still operation_enabled" is the thing to watch,
# not just "no fault".
_STATE_MASK = 0x006F
_OPERATION_ENABLED = 0x0027


def _smoothstep(u: float) -> float:
    """Ease-in/ease-out, 0..1 -> 0..1. Zero velocity at both ends."""
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="CDC port (COM5 / /dev/ttyACM0)")
    p.add_argument("--bus", type=int, required=True, help="CAN bus 1..6")
    p.add_argument("--node-id", type=int, required=True, help="ZeroErr CANopen node id")
    p.add_argument("--slot", type=int, default=0, help="plant CFG slot to use (RAM only)")
    p.add_argument("--degrees", type=float, default=180.0, help="relative move, degrees (default 180)")
    # Bench-confirmed (2026-08-06): 180deg/8s/20Hz (peak ~0.59 rad/s) made the
    # drive silently drop operation_enabled -> switch_on_disabled mid-ramp
    # WITHOUT ever setting the Fault bit (so firmware's fault-detect never
    # caught it) — likely a following-error trip. 180deg/20s/50Hz (peak
    # ~0.24 rad/s) ran clean start to finish, zero faults/disables, tracking
    # error settling under 0.01 rad. Scale --duration up for larger --degrees
    # if you push past 180.
    p.add_argument("--duration", type=float, default=20.0, help="ramp duration, seconds (default 20, bench-confirmed clean for 180deg)")
    p.add_argument("--rate-hz", type=float, default=50.0, help="command update rate (default 50 Hz)")
    p.add_argument("--kp", type=float, default=20.0)
    p.add_argument("--kd", type=float, default=1.0)
    args = p.parse_args(argv)

    delta_rad = math.radians(args.degrees)
    period_s = 1.0 / args.rate_hz

    print(f"ze_180_rotation  bus={args.bus}  node=0x{args.node_id:02X}  "
          f"delta={args.degrees:+.1f}deg  duration={args.duration:g}s  rate={args.rate_hz:g}Hz")
    print("Note: only one process may own the CDC port — close dashboard/pcb_lab.debug first.")

    proxy = HostProxy.connect(args.port, stream_hz=50.0, telemetry_hz=50.0, armed=False, mode="debug")
    try:
        start_pos = proxy.hub.debug.read_position_zeroerr(bus=args.bus, node_id=args.node_id)
        if start_pos is None:
            print("FAIL: could not read current position — aborting before enabling anything.")
            return 1
        target_pos = start_pos + delta_rad
        print(f"start={start_pos:+.4f} rad  target={target_pos:+.4f} rad")

        if args.bus == 1:
            for s in _CH1_FACTORY_SLOTS:
                if s == args.slot:
                    continue
                proxy.hub.debug.cfg_set_slot(
                    slot=s, bus=1, protocol=PROTO_NONE, motor_id=0,
                    enabled=False, persist=False,
                )
            print(f"cleared factory-default CH1 slots {[s for s in _CH1_FACTORY_SLOTS if s != args.slot]}")

        proxy.hub.debug.cfg_set_slot(
            slot=args.slot, bus=args.bus, protocol=PROTO_ZEROERR,
            motor_id=args.node_id, persist=False,
        )
        action = ActuatorAction.from_slots(proxy, [args.slot], name="ze_180_rotation")

        proxy.ensure_plant_control(enable=True)

        # Hold at the current (already-confirmed) position first — this is
        # what actually drives the boot/enable walk, at zero position error.
        action.hold([start_pos], kp=args.kp, kd=args.kd, send=True)
        print("enabling (boot/enable walk) ...")
        time.sleep(2.0)  # NMT+SDO sequence + 500ms settle, generous margin

        t0 = time.monotonic()
        tick = 0
        while True:
            u = (time.monotonic() - t0) / args.duration
            if u >= 1.0:
                break
            pos = start_pos + delta_rad * _smoothstep(u)
            action.hold([pos], kp=args.kp, kd=args.kd, send=True)
            time.sleep(period_s)
            tick += 1

            # Bail out the moment the drive leaves operation_enabled instead
            # of blindly streaming a growing target into a disabled drive
            # for the rest of --duration (that's exactly what happened in
            # the failing 180deg/8s/20Hz run — it disabled at ~2s and this
            # loop kept going for 6 more seconds regardless).
            fb = proxy.latest_feedback()
            st = fb.actuator(args.slot) if fb is not None else None
            if st is not None and (int(st.fault) & _STATE_MASK) != _OPERATION_ENABLED:
                print(
                    f"STOP  t={u * args.duration:.2f}s  left operation_enabled "
                    f"(sw=0x{int(st.fault):04X})  actual_pos={st.position:+.4f}  "
                    f"cmd_pos={pos:+.4f}  — too fast for this drive/gearing, "
                    f"try a longer --duration or lower --degrees"
                )
                return 1
            if tick % max(1, int(args.rate_hz)) == 0 and st is not None:
                print(f"  t={u * args.duration:5.2f}s  cmd={pos:+.4f}  act={st.position:+.4f}  "
                      f"err={pos - st.position:+.4f}")

        action.hold([target_pos], kp=args.kp, kd=args.kd, send=True)
        print(f"done  target={target_pos:+.4f} rad")
        time.sleep(1.0)

        final_pos = proxy.hub.debug.read_position_zeroerr(bus=args.bus, node_id=args.node_id)
        print(f"final actual position: {final_pos}")
        return 0
    finally:
        try:
            action = locals().get("action")
            if action is not None:
                action.blank(send=True)
        except Exception as exc:  # noqa: BLE001
            print(f"(blank on exit failed: {exc})")
        try:
            proxy.ensure_plant_control(enable=False)
        except Exception as exc:  # noqa: BLE001
            print(f"(disarm on exit failed: {exc})")
        proxy.close()


if __name__ == "__main__":
    raise SystemExit(main())
