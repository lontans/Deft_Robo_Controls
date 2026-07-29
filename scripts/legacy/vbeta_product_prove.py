#!/usr/bin/env python3
"""Combined YAM-product-CFG prove: left arm hold + right-arm discover + base
6-RobStride smoke, all in **one** PcbRobotSession (exclusive CDC).

Unlike ``vbeta_arm_smoke.py`` / ``vbeta_base_smoke.py`` (separate one-shot
CLIs, each opening/closing its own session), this exercises the actual
``yam_product_rows()`` slot map — arms 0-13, base steer/drive 14-19 on
CH4-6 RobStride IDs ``0x01``/``0x02`` — as a single continuous session, so
"left arm + base tracking" can be evaluated together over one clock, and so
a base ID mismatch (bench spare IDs vs. product IDs) shows up as an
explicit found/not-found probe result rather than a silent zero.

Does not touch FW, does not Soft-DFU, does not edit continuous's bench CFG
(``yam_continuous_all.py``'s spare-slot map is untouched — this applies the
*product* CFG in RAM only, on the shared board, and yields CDC on exit).

Usage (Jetson):
    python3 vbeta_product_prove.py --port /dev/ttyACM0 --hold-s 30
"""
from __future__ import annotations

import argparse
import time
from typing import Optional

import numpy as np

from deft_controls_sdk.vbeta import PcbArmDriver, PcbPlatformClient, PcbRobotSession
from deft_controls_sdk.vbeta.slots import BASE_DRIVE_SLOTS, BASE_STEER_SLOTS
from deft_controls_sdk.vbeta.yam_limits import plan_jog_q7

# Product base map: CH4/CH5/CH6, one steer (id 0x01) + one drive (id 0x02) each.
PRODUCT_BASE_BUSES = (4, 5, 6)
PRODUCT_STEER_ID = 0x01
PRODUCT_DRIVE_ID = 0x02


def _fault(session: PcbRobotSession, slot: int) -> Optional[int]:
    fb = session.latest_feedback()
    if fb is None:
        return None
    st = fb.actuator(slot)
    return None if st is None else int(st.fault) & 0xFF


def _probe_base_product_ids(hub) -> dict:
    """Direct RobStride probe at the *product* IDs on CH4/5/6 — proves (or
    disproves) that the physically-wired base motors answer to 0x01/0x02
    rather than whatever bench spare-slot IDs they were last CFG'd with."""
    from deft_controls_sdk.vbeta.cfg import pause_plant_stream

    found = {}
    with pause_plant_stream(hub):
        for bus in PRODUCT_BASE_BUSES:
            for role, mid in (("steer", PRODUCT_STEER_ID), ("drive", PRODUCT_DRIVE_ID)):
                try:
                    resp = hub.debug.probe_robstride(bus=bus, motor_id=mid)
                except Exception as exc:
                    print(f"  probe CH{bus} {role} id=0x{mid:02X}: EXC {exc}", flush=True)
                    found[(bus, role)] = False
                    continue
                ok = bool(resp and resp.get("found"))
                found[(bus, role)] = ok
                pos = resp.get("position") if resp else None
                print(
                    f"  probe CH{bus} {role} id=0x{mid:02X}: "
                    f"{'FOUND pos=' + f'{pos:+.4f}' if ok else 'not found'}",
                    flush=True,
                )
    return found


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--hold-s", type=float, default=30.0, help="Combined arm+base window (s)")
    ap.add_argument("--base-creep", type=float, default=0.2, help="Bp* rad/s product-ID drive test")
    ap.add_argument("--status-s", type=float, default=2.0)
    ap.add_argument("--no-right-arm", action="store_true")
    ap.add_argument("--jog-joint", type=int, default=1, help="Arm-local joint index 0..6 for the track-proof jog")
    ap.add_argument("--jog-delta", type=float, default=0.15, help="Clamped jog delta (rad) to prove Goal_Position tracking")
    args = ap.parse_args(argv)

    with PcbRobotSession.connect(
        args.port, apply_yam_cfg=True, force_cfg=True, stream_hz=40.0
    ) as session:
        hub = session.hub

        # ---- left arm: required prove ----
        print("\n== LEFT ARM (CH1, product CFG slots 0-6) ==", flush=True)
        left = PcbArmDriver(session, side="left", skip_home_on_connect=True)
        left.connect()
        q0 = left.read("Position_Rad")
        print(f"left present Position_Rad={np.array2string(q0, precision=3)}", flush=True)
        left_live = bool(np.max(np.abs(q0)) > 1e-3)
        print(f"left_live(any joint |q|>1e-3)={left_live}", flush=True)

        # ---- right arm: discover only, or same hold if answering ----
        print("\n== RIGHT ARM (CH2, product CFG slots 7-13) ==", flush=True)
        right: Optional[PcbArmDriver] = None
        right_live = False
        if not args.no_right_arm:
            try:
                sweep = hub.debug.discover_damiao_all(bus=2, start=1, end=7, listen_ms=80)
            except Exception as exc:
                print(f"  CH2 discover EXC: {exc}", flush=True)
                sweep = []
            print(f"  CH2 Damiao discover sweep={sweep}", flush=True)
            if sweep:
                right = PcbArmDriver(session, side="right", skip_home_on_connect=True)
                right.connect()
                qr0 = right.read("Position_Rad")
                print(f"right present Position_Rad={np.array2string(qr0, precision=3)}", flush=True)
                right_live = bool(np.max(np.abs(qr0)) > 1e-3)
                print(f"right_live(any joint |q|>1e-3)={right_live}", flush=True)
            else:
                print("  CH2 empty — right arm CFG'd but not physically present/powered "
                      "this session; discover-only, no hold attempted.", flush=True)

        # ---- base: direct product-ID probe (ground truth before trusting FB) ----
        print("\n== BASE (CH4-6, product IDs steer=0x01 drive=0x02, slots 14-19) ==", flush=True)
        found = _probe_base_product_ids(hub)
        any_base_found = any(found.values())
        plat = PcbPlatformClient(session, use_neck=False)
        plat.connect()
        plat.send_target_state(
            {"BwC": 0.0, "BwR": 0.0, "BwL": 0.0},
            {"BpC": 0.0, "BpR": 0.0, "BpL": 0.0},
        )

        # ---- track-proof jog: clamped Goal_Position delta, must converge ----
        q_hold = left.read("Position_Rad")
        q_jog, jog_note = plan_jog_q7(
            q_hold, side="left", joint=args.jog_joint, delta=args.jog_delta
        )
        print(
            f"\n== LEFT ARM JOG (joint{args.jog_joint} delta={args.jog_delta:+.3f}, "
            f"clamp note={jog_note!r}) ==",
            flush=True,
        )
        print(f"  from={np.array2string(q_hold, precision=3)}", flush=True)
        print(f"  to  ={np.array2string(q_jog, precision=3)}", flush=True)
        left.write("Goal_Position", q_jog)
        for _ in range(20):
            session.send_once()
            time.sleep(0.05)
        q_after_jog = left.read("Position_Rad")
        jog_err = float(np.abs(q_after_jog[args.jog_joint] - q_jog[args.jog_joint]))
        print(
            f"  after 1.0s: q={np.array2string(q_after_jog, precision=3)} "
            f"joint{args.jog_joint} err={jog_err:.4f} rad "
            f"({'CONVERGED' if jog_err < 0.05 else 'still moving/lagging'})",
            flush=True,
        )

        # ---- combined window: left arm hold + base creep, one session clock ----
        print(
            f"\n== COMBINED WINDOW {args.hold_s:.0f}s "
            f"(left arm hold at jog target + base creep={args.base_creep:.2f} rad/s) ==",
            flush=True,
        )
        plat.send_target_state(
            {"BwC": 0.0, "BwR": 0.0, "BwL": 0.0},
            {"BpC": args.base_creep, "BpR": args.base_creep, "BpL": args.base_creep},
        )
        t_end = time.perf_counter() + float(args.hold_s)
        last_status = 0.0
        left_tracked_any_motion = jog_err < 0.05
        base_tracked_any_motion = False
        q_ref = q_after_jog.copy()
        while time.perf_counter() < t_end:
            if session.service_soft_kill():
                print("  SOFT_KILL_REQ -> parked, ending window early", flush=True)
                break
            session.send_once()
            now = time.perf_counter()
            if now - last_status >= args.status_s or last_status == 0.0:
                last_status = now
                ql = left.read("Position_Rad")
                if float(np.max(np.abs(ql - q_ref))) > 0.01:
                    left_tracked_any_motion = True
                st = plat.get_state()
                if abs(st["bpc_velocity"]) > 0.01 or abs(st["bwc_angle"]) > 0.01:
                    base_tracked_any_motion = True
                lf = [
                    _fault(session, s) for s in (0, 1, 2, 3, 4, 5, 6)
                ]
                print(
                    f"  t={now - (t_end - args.hold_s):5.1f}s left_q={np.array2string(ql, precision=2)} "
                    f"faults={lf} base bwc={st['bwc_angle']:+.3f} bpc_v={st['bpc_velocity']:+.3f}",
                    flush=True,
                )

        plat.send_command(("base_cmd", 0.0, 0.0, 0.0))
        left.write("Zero_Torque", True)
        session.send_once()
        time.sleep(0.2)
        left.disconnect()
        if right is not None:
            right.write("Zero_Torque", True)
            session.send_once()
            time.sleep(0.2)
            right.disconnect()
        plat.disconnect()

        print("\n== SUMMARY ==", flush=True)
        print(f"left_live={left_live} left_tracked_motion={left_tracked_any_motion}", flush=True)
        print(f"right_ch2_present={bool(right is not None)} right_live={right_live}", flush=True)
        print(
            f"base_product_id_found={found} any_base_found={any_base_found} "
            f"base_tracked_motion={base_tracked_any_motion}",
            flush=True,
        )
        if not any_base_found:
            print(
                "REMAP GAP: no RobStride answered product IDs 0x01(steer)/0x02(drive) "
                "on CH4/5/6 — physical base wheels are still on bench spare IDs "
                "(0x70/0x74/0x75 RobStride + 0x06 Damiao per docs/peripherals/base-robstride-mcp.md "
                "and base-damiao-ch6.md), not the YAM product map. Do not fall back to slots 22-25 "
                "inside this product-CFG path — see docs/vbeta-pcb-adapter.md remap note.",
                flush=True,
            )

    print("vbeta_product_prove done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
