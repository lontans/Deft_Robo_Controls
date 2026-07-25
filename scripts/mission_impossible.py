#!/usr/bin/env python3
"""Mission Impossible â€” stress failure-point hunt (main path).

    cd scripts
    python mission_impossible.py preflight --port /dev/ttyACM0
    python mission_impossible.py m1 --port /dev/ttyACM0
    python mission_impossible.py all --port /dev/ttyACM0

Appends results to docs/mission_impossible_findings.md (repo root relative).
Exclusive CDC owner while RUNNING. On FAIL/BLOCKED, append a follow-up block.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, LedDesire, McuState, ServoDesire
from deft_controls_sdk.bench.metrics import measure_hold
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import PcbRobotSession
from deft_controls_sdk.vbeta.cfg import (
    ensure_yam_left_arm_cfg,
    ensure_yam_product_cfg,
    pause_plant_stream,
)
from deft_controls_sdk.vbeta.slots import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    LEFT_ARM_SLOTS,
    PROTO_DAMIAO,
)
from deft_controls_sdk.vbeta.slots import _DAMIAO_MASTER  # noqa: SLC001 â€” same table continuous uses
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI, CLEAR_LO
from bench_load_matrix import parse_hz_list, render_report, run_matrix

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
FINDINGS = REPO / "docs" / "mission_impossible_findings.md"

ARM_KP = tuple(float(x) for x in DEFAULT_ARM_KP)
ARM_KD = float(DEFAULT_ARM_KD)


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")


def _append_findings(text: str) -> None:
    FINDINGS.parent.mkdir(parents=True, exist_ok=True)
    with FINDINGS.open("a", encoding="utf-8") as f:
        f.write(text if text.endswith("\n") else text + "\n")
    print(f"[findings] appended -> {FINDINGS}", flush=True)


def _set_status(mission: str, status: str) -> None:
    if not FINDINGS.exists():
        return
    body = FINDINGS.read_text(encoding="utf-8")
    key = {
        "M1": "M1 TX bandwidth",
        "M2": "M2 PDU soft-kill / V/I",
        "M3": "M3 Multi-joint CLEAR arm",
        "M4": "M4 Faster base + DXL",
        "M5": "M5 Soft-DFU stress",
    }.get(mission, mission)
    lines = []
    for line in body.splitlines(keepends=True):
        if line.startswith("| " + key + " |"):
            lines.append(f"| {key} | {status} | {_utc()} |\n")
        else:
            lines.append(line)
    FINDINGS.write_text("".join(lines), encoding="utf-8")


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _blank(hub: ControlsPcbHub) -> None:
    blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    for _ in range(8):
        hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
        _conn(hub).set_actuators(blank, send=False)
        try:
            _conn(hub).clear_servos(send=False)
        except Exception:
            pass
        hub.set_led(
            LedDesire(mode=LED_MODE_IDLE_CORNFLOWER, master_brightness=8), send=False
        )
        _conn(hub).send_once()
        time.sleep(0.04)


def _followup_note(mission: str, signature: str, layer: str, excerpt: str) -> str:
    return (
        f"\n### Follow-up\n"
        f"mission: {mission}\n"
        f"signature: {signature}\n"
        f"suspect layer: {layer}\n"
        f"log excerpt:\n```\n{excerpt[:1200]}\n```\n"
    )


# ---------------------------------------------------------------------------
# preflight
# ---------------------------------------------------------------------------


def cmd_preflight(args: argparse.Namespace) -> int:
    port = args.port or find_cdc_port()
    print(f"preflight port={port}", flush=True)
    with ControlsPcbHub.connect(port) as hub:
        hub.recover()
        time.sleep(0.2)
        table = hub.debug.cfg_get_table()
        enabled = sum(1 for r in table if r.get("enabled"))
        print(f"  cfg enabled slots={enabled}", flush=True)
        _blank(hub)
    _append_findings(
        f"\n## Preflight {_utc()}\n\n"
        f"- port: `{port}`\n- cfg enabled slots: {enabled}\n- CDC open/close: OK\n"
    )
    print("preflight PASS", flush=True)
    return 0


# ---------------------------------------------------------------------------
# M1 â€” TX bandwidth
# ---------------------------------------------------------------------------


def cmd_m1(args: argparse.Namespace) -> int:
    mission = "M1"
    _set_status(mission, "RUNNING")
    port = args.port or find_cdc_port()
    hz_list = parse_hz_list(args.hz)
    lines: List[str] = [f"\n## {mission} TX bandwidth â€” {_utc()}\n"]
    live_notes: List[str] = []

    with ControlsPcbHub.connect(port) as hub:
        hub.recover()
        results = run_matrix(
            hub,
            hz_list=hz_list,
            scenario="all",
            trials=args.trials,
            seconds=args.seconds,
        )
        lines.append(render_report("all", results))

        # Live stream sample at a few rates (telemetry path).
        for stream_hz in (20.0, 40.0, 100.0):
            hub.start_streaming(hz=stream_hz, auto_soft_kill=False)
            t_end = time.perf_counter() + 5.0
            while time.perf_counter() < t_end:
                hub.send_once()
                time.sleep(1.0 / stream_hz)
            snap = hub.telemetry.snapshot()
            note = (
                f"stream@{stream_hz:g}: fb={snap.fb_hz} tx={snap.stream_tx_hz} "
                f"ack={snap.stream_ack_lag} gap95={snap.stream_tx_gap_p95_ms} "
                f"lap={snap.lap_ms}"
            )
            print(note, flush=True)
            live_notes.append(note)
            hub.stop_streaming()
            time.sleep(0.2)
        _blank(hub)

    hard = min(hz_list)
    hard_ok = all(r["ok"] for r in results if r["hz"] == hard)
    # Capability note at 500 â€” do not fail suite solely on 500.
    ok_100 = all(r["ok"] for r in results if r["hz"] == 100.0) if 100.0 in hz_list else True
    verdict = "PASS" if hard_ok else "FAIL"
    if hard_ok and not ok_100:
        verdict = "PASS_WITH_NOTES"  # 40 ok, 100 stressed

    lines.append("\n### Live stream samples\n")
    for n in live_notes:
        lines.append(f"- `{n}`\n")
    lines.append(f"\n**Verdict: {verdict}** (hard gate {hard:g} Hz ok={hard_ok})\n")
    if verdict == "FAIL":
        excerpt = "\n".join(live_notes + [render_report("all", results)])
        lines.append(
            _followup_note(mission, "40Hz hard gate failed", "USB", excerpt)
        )
    _append_findings("".join(lines))
    _set_status(mission, verdict)
    return 0 if verdict.startswith("PASS") else 1


# ---------------------------------------------------------------------------
# M2 â€” PDU soft-kill / V/I
# ---------------------------------------------------------------------------


def _start_pdb_sim(
    *,
    pack_v: Sequence[int],
    pack_i: Sequence[int],
    force_kill: Optional[int] = None,
    simulate_kill_after: Optional[float] = None,
    wander: bool = True,
) -> Optional[subprocess.Popen]:
    """Best-effort local Jetson UART sim. Returns Popen or None if not Jetson.

    Do **not** pass ``force_kill=0`` for handshake tests â€” that pins NORMAL and
    suppresses ``--simulate-kill-after`` (KillSim never wins).
    """
    if not Path("/dev/ttyTHS1").exists():
        return None
    cmd = [
        sys.executable,
        "-u",
        str(SCRIPTS / "pdb_uart_sim.py"),
        "--port",
        "/dev/ttyTHS1",
        "--hz",
        "20",
        "--estop-sense",
        "1",
        "--pack-v",
        *[str(x) for x in pack_v],
        "--rail-v",
        "4800",
        "1900",
        "1200",
        "500",
        "--pack-i",
        *[str(x) for x in pack_i],
        "--rail-i",
        "90",
        "70",
        "40",
        "25",
    ]
    if force_kill is not None:
        cmd.extend(["--force-kill-state", str(force_kill)])
    if wander:
        cmd.append("--wander")
    if simulate_kill_after is not None:
        cmd.extend(["--simulate-kill-after", str(simulate_kill_after)])
    log = open("/tmp/mi_pdb_sim.log", "w", encoding="utf-8")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(SCRIPTS))


def _kill_pdb_sim() -> None:
    subprocess.run(["pkill", "-9", "-f", "pdb_uart_sim.py"], check=False)
    time.sleep(1.0)


def _stop_can_quiet(port: str) -> None:
    """Blank + DIAG so the next mission does not inherit ESTOP/MIT latches."""
    try:
        with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
            hub.recover()
            _blank(hub)
    except Exception as exc:
        print(f"stop_can warn: {exc!r}", flush=True)


def _pdb_seq(st) -> Optional[int]:
    if st is None or st.pdb is None:
        return None
    try:
        return int(st.pdb.get("seq", -1))
    except Exception:
        return None


def _wait_pdb_live(
    hub: ControlsPcbHub,
    *,
    timeout_s: float = 8.0,
    require_normal: bool = True,
) -> Tuple[bool, object]:
    """Stream until PDU mirror advances and COMMS_LOSS stale clears."""
    t0 = time.perf_counter()
    last_seq: Optional[int] = None
    advances = 0
    st = None
    while time.perf_counter() - t0 < timeout_s:
        hub.send_once()
        st = hub.pdb_status()
        seq = _pdb_seq(st)
        if (
            st is not None
            and not st.stale_failsafe
            and seq is not None
            and (not require_normal or st.normal)
        ):
            if last_seq is not None and seq != last_seq:
                advances += 1
                if advances >= 2:
                    return True, st
            last_seq = seq
        time.sleep(0.05)
    return False, st


def _wait_park(
    hub: ControlsPcbHub,
    *,
    timeout_s: float,
    desires: Optional[Dict[int, ActuatorDesire]] = None,
) -> Tuple[bool, object]:
    """Poll until soft-kill park hooks fire (requested or bad V/I)."""
    t_end = time.perf_counter() + timeout_s
    parked = False
    while time.perf_counter() < t_end:
        if desires is not None:
            _conn(hub).set_actuators(desires, send=False)
        if hub.soft_kill_park_if_requested(send=False) or hub.soft_kill_park_if_bad_vi(
            send=False
        ):
            parked = True
            hub.send_once()
            break
        st = hub.pdb_status()
        if st is not None and int(st.kill_state) == 1:
            hub.soft_kill_park(send=False)
            parked = True
            hub.send_once()
            break
        hub.send_once()
        time.sleep(0.05)
    return parked, hub.pdb_status()


def cmd_m2(args: argparse.Namespace) -> int:
    mission = "M2"
    _set_status(mission, "RUNNING")
    port = args.port or find_cdc_port()
    lines: List[str] = [f"\n## {mission} PDU soft-kill / V/I â€” {_utc()}\n"]
    steps: List[Tuple[str, bool, str]] = []

    if not Path("/dev/ttyTHS1").exists():
        lines.append(
            "- **BLOCKED:** `/dev/ttyTHS1` absent â€” run M2 on Jetson (or wire PDU UART sim).\n"
        )
        lines.append(
            _followup_note(
                mission,
                "no Jetson UART sim path on this host",
                "PDU",
                "ttyTHS1 missing",
            )
        )
        lines.append("\n**Verdict: BLOCKED**\n")
        _append_findings("".join(lines))
        _set_status(mission, "BLOCKED")
        return 1

    _kill_pdb_sim()
    _stop_can_quiet(port)

    # Warm PDU path: benign sim until MCU leaves COMMS_LOSS (M1 often leaves gap).
    warm = _start_pdb_sim(
        pack_v=(4800, 4800, 0, 0),
        pack_i=(180, 140, 0, 0),
        force_kill=None,
        wander=False,
    )
    try:
        with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
            hub.recover()
            hub.start_streaming(hz=40.0, auto_soft_kill=False)
            live, st = _wait_pdb_live(hub, timeout_s=10.0)
            lines.append(f"- warm PDU live={live} pdb={st}\n")
            print("M2 warm", live, st, flush=True)
            hub.stop_streaming()
            if not live:
                steps.append(("PDU warm (clear COMMS_LOSS)", False, f"pdb={st}"))
    finally:
        if warm is not None:
            warm.terminate()
            try:
                warm.wait(timeout=2)
            except Exception:
                warm.kill()
        _kill_pdb_sim()

    if steps and not steps[0][1]:
        verdict = "FAIL"
        for name, good, detail in steps:
            lines.append(f"- {'PASS' if good else 'FAIL'}: {name} â€” {detail}\n")
        lines.append("\n**Verdict: FAIL**\n")
        lines.append(
            _followup_note(
                mission,
                "PDU UART never left COMMS_LOSS after sim warm",
                "PDU",
                str(steps),
            )
        )
        _append_findings("".join(lines))
        _set_status(mission, verdict)
        if Path("/dev/ttyTHS1").exists():
            _start_pdb_sim(
                pack_v=(4800, 4800, 0, 0),
                pack_i=(180, 140, 0, 0),
                force_kill=None,
                wander=False,
            )
        return 1

    # Phase A: KillSim handshake (no force_kill â€” that pins NORMAL)
    sim = _start_pdb_sim(
        pack_v=(4800, 4800, 0, 0),
        pack_i=(180, 140, 0, 0),
        force_kill=None,
        simulate_kill_after=6.0,
        wander=True,
    )
    parked_sim = False
    try:
        with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
            hub.recover()
            ensure_yam_left_arm_cfg(hub, quiet=True)
            hub.start_streaming(hz=40.0, auto_soft_kill=True)
            live, st = _wait_pdb_live(hub, timeout_s=6.0)
            lines.append(f"- M2A pre-kill live={live} pdb={st}\n")
            desires = {
                s: ActuatorDesire(position=0.0, velocity=0.0, kp=8.0, kd=0.5)
                for s in LEFT_ARM_SLOTS
            }
            parked_sim, st = _wait_park(hub, timeout_s=16.0, desires=desires)
            detail = f"parked={parked_sim} live={live} pdb={st}"
            steps.append(("simulate-kill-after handshake", parked_sim, detail))
            print("M2A", detail, flush=True)
            _blank(hub)
            hub.stop_streaming()
    finally:
        if sim is not None:
            sim.terminate()
            try:
                sim.wait(timeout=2)
            except Exception:
                sim.kill()
        _kill_pdb_sim()
        _stop_can_quiet(port)

    # Phase B: UV trip
    sim = _start_pdb_sim(
        pack_v=(3900, 3900, 0, 0),
        pack_i=(180, 140, 0, 0),
        force_kill=None,
        wander=False,
    )
    parked_uv = False
    try:
        with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
            hub.recover()
            hub.start_streaming(hz=40.0, auto_soft_kill=True)
            # Wait until mirror shows UV pack (not leftover 48 V) or kill overlays.
            t_end = time.perf_counter() + 5.0
            saw_uv = False
            while time.perf_counter() < t_end:
                hub.send_once()
                st = hub.pdb_status()
                if st is not None and st.pdb is not None:
                    pv0 = int(st.pdb["pack_v"][0])
                    if pv0 and pv0 < 4000:
                        saw_uv = True
                        break
                    if int(st.kill_state) == 1:
                        saw_uv = True
                        break
                time.sleep(0.05)
            parked_uv, st = _wait_park(hub, timeout_s=8.0)
            detail = f"parked={parked_uv} saw_uv_mirror={saw_uv} pdb={st}"
            steps.append(("UV pack_v=3900", parked_uv, detail))
            print("M2B UV", detail, flush=True)
            _blank(hub)
            hub.stop_streaming()
    finally:
        if sim is not None:
            sim.terminate()
        _kill_pdb_sim()
        _stop_can_quiet(port)

    # Phase C: OC trip
    sim = _start_pdb_sim(
        pack_v=(4800, 4800, 0, 0),
        pack_i=(3100, 100, 0, 0),
        force_kill=None,
        wander=False,
    )
    parked_oc = False
    try:
        with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
            hub.recover()
            hub.start_streaming(hz=40.0, auto_soft_kill=True)
            live, _ = _wait_pdb_live(hub, timeout_s=6.0, require_normal=False)
            parked_oc, st = _wait_park(hub, timeout_s=8.0)
            detail = f"parked={parked_oc} liveish={live} pdb={st}"
            steps.append(("OC pack_i=3100", parked_oc, detail))
            print("M2C OC", detail, flush=True)
            _blank(hub)
            hub.stop_streaming()
    finally:
        if sim is not None:
            sim.terminate()
        _kill_pdb_sim()
        _stop_can_quiet(port)

    # Restore benign sim for other agents
    _start_pdb_sim(
        pack_v=(4800, 4800, 0, 0),
        pack_i=(180, 140, 0, 0),
        force_kill=None,
        wander=False,
    )

    ok = all(s[1] for s in steps)
    verdict = "PASS" if ok else "FAIL"
    for name, good, detail in steps:
        lines.append(f"- {'PASS' if good else 'FAIL'}: {name} â€” {detail}\n")
    lines.append(f"\n**Verdict: {verdict}**\n")
    if not ok:
        lines.append(
            _followup_note(
                mission,
                "one or more V/I or handshake parks missed",
                "PDU",
                "\n".join(f"{a}: {b} {c}" for a, b, c in steps),
            )
        )

    _append_findings("".join(lines))
    _set_status(mission, verdict)
    return 0 if verdict == "PASS" else 1


# ---------------------------------------------------------------------------
# M3 â€” multi-joint CLEAR (progressive MIT latch then bounce)
# ---------------------------------------------------------------------------


def _m3_arm_faults(session: PcbRobotSession) -> List[int]:
    fb = session.latest_feedback()
    out = [-1] * 7
    if fb is None:
        return out
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        st = fb.actuator(slot)
        if st is not None:
            out[i] = int(st.fault) & 0xFF
    return out


def _m3_arm_q(session: PcbRobotSession) -> List[float]:
    fb = session.latest_feedback()
    q = [0.0] * 7
    if fb is None:
        return q
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        st = fb.actuator(slot)
        if st is not None:
            q[i] = float(st.position)
    return q


def _m3_cfg_armed(hub: ControlsPcbHub, armed: set) -> None:
    # Must pause plant stream â€” CFG replies get stolen otherwise (TimeoutError).
    with pause_plant_stream(hub):
        for i in range(7):
            hub.debug.cfg_set_slot(
                slot=i,
                bus=1,
                protocol=PROTO_DAMIAO,
                motor_id=0x01 + i,
                master_id=_DAMIAO_MASTER[i],
                enabled=(i in armed),
                persist=False,
            )


def cmd_m3(args: argparse.Namespace) -> int:
    mission = "M3"
    _set_status(mission, "RUNNING")
    port = args.port or find_cdc_port()
    active = [1, 2, 3]  # J2â€“J4 first
    rate_up = float(args.cruise_up)
    rate_down = float(args.cruise_down)
    hold_s = float(args.hold_s)
    latch_scale = 0.35
    lines: List[str] = [
        f"\n## {mission} Multi-joint CLEAR â€” {_utc()}\n",
        f"- active joints (0-based): {active}\n",
        f"- rates up/down: {rate_up}/{rate_down} rad/s\n",
        f"- window: {hold_s}s\n",
    ]

    hard_fault = False
    clear_breach = False
    stage2 = False
    last_faults = [-1] * 7
    cmd = [0.0] * 7
    samples = 0

    # M2 OC park leaves ESTOP; blank/recover before CFG+stream.
    _stop_can_quiet(port)

    with PcbRobotSession.connect(
        port, apply_yam_cfg=False, stream_hz=40.0
    ) as session:
        hub = session.hub
        hub.recover()
        ensure_yam_left_arm_cfg(hub, force=True, quiet=True)
        _m3_cfg_armed(hub, set())
        hub.set_mcu_state(McuState.NORMAL, send=True)

        q0 = [0.0] * 7
        armed: set = set()

        def seed_hold(secs: float) -> None:
            t_end = time.perf_counter() + secs
            while time.perf_counter() < t_end:
                q = _m3_arm_q(session)
                for s in armed:
                    if abs(q[s]) > 1e-3:
                        q0[s] = q[s]
                desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
                for s in armed:
                    desires[LEFT_ARM_SLOTS[s]] = ActuatorDesire(
                        position=float(q0[s]), velocity=0.0, kp=0.0, kd=0.3
                    )
                session.set_actuators(desires, send=False)
                session.send_once()
                time.sleep(0.05)

        def latch_armed(ramp_s: float, hold_s: float) -> bool:
            ok = False
            t0 = time.perf_counter()
            t_end = t0 + ramp_s + hold_s
            while time.perf_counter() < t_end:
                q = _m3_arm_q(session)
                for s in armed:
                    if abs(q[s]) > 1e-3:
                        q0[s] = (
                            0.85 * q0[s] + 0.15 * q[s] if abs(q0[s]) > 1e-3 else q[s]
                        )
                u = (time.perf_counter() - t0) / max(ramp_s, 1e-3)
                s_gain = min(1.0, max(0.0, u))
                s_gain = s_gain * s_gain * (3.0 - 2.0 * s_gain)
                desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
                for s in armed:
                    desires[LEFT_ARM_SLOTS[s]] = ActuatorDesire(
                        position=float(q0[s]),
                        velocity=0.0,
                        kp=float(ARM_KP[s]) * latch_scale * s_gain,
                        kd=ARM_KD,
                    )
                session.set_actuators(desires, send=False)
                session.send_once()
                faults = _m3_arm_faults(session)
                if time.perf_counter() >= t0 + ramp_s and all(
                    faults[s] == 1 for s in armed
                ):
                    ok = True
                    break
                time.sleep(0.05)
            return ok

        # Progressive latch all 7 (same pattern as continuous)
        for i in range(7):
            armed.add(i)
            _m3_cfg_armed(hub, armed)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            seed_hold(0.6)
            ok = latch_armed(1.2 if i == 3 else 0.8, 1.2 if i == 3 else 0.8)
            faults = _m3_arm_faults(session)
            print(f"  latch J{i+1} ok={ok} faults={faults}", flush=True)
            lines.append(f"- latch J{i+1}: ok={ok} faults={faults}\n")
            if not ok:
                # one recover retry
                hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
                session.set_actuators(
                    {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
                )
                session.send_once()
                time.sleep(0.2)
                hub.recover()
                _m3_cfg_armed(hub, armed)
                hub.set_mcu_state(McuState.NORMAL, send=True)
                seed_hold(0.5)
                ok = latch_armed(1.0, 1.0)
                faults = _m3_arm_faults(session)
                lines.append(f"- latch J{i+1} retry: ok={ok} faults={faults}\n")
                if not ok:
                    break

        last_faults = _m3_arm_faults(session)
        green = sum(1 for f in last_faults if f == 1)
        lines.append(f"- post-latch green={green}/7 faults={last_faults}\n")
        if green < 5:
            lines.append("\n**Verdict: FAIL** (MIT latch incomplete)\n")
            lines.append(
                _followup_note(
                    mission,
                    f"latch green={green}/7 faults={last_faults}",
                    "motor",
                    str(last_faults),
                )
            )
            _blank(hub)
            _append_findings("".join(lines))
            _set_status(mission, "FAIL")
            return 1

        # Seed cmd from FB, clamp into CLEAR
        cmd = _m3_arm_q(session)
        for i in range(7):
            cmd[i] = min(CLEAR_HI[i], max(CLEAR_LO[i], cmd[i]))
        dirs = {i: 1.0 for i in active}

        t_end = time.perf_counter() + hold_s
        while time.perf_counter() < t_end:
            dt = 0.05
            for i in active:
                lo, hi = CLEAR_LO[i], CLEAR_HI[i]
                step = (rate_up if dirs[i] > 0 else -rate_down) * dt
                nxt = cmd[i] + step
                if nxt >= hi:
                    nxt = hi
                    dirs[i] = -1.0
                elif nxt <= lo:
                    nxt = lo
                    dirs[i] = 1.0
                cmd[i] = nxt
                if cmd[i] < lo - 1e-3 or cmd[i] > hi + 1e-3:
                    clear_breach = True
            desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            for i, slot in enumerate(LEFT_ARM_SLOTS):
                desires[slot] = ActuatorDesire(
                    position=float(cmd[i]),
                    velocity=0.0,
                    kp=float(ARM_KP[i]) * 0.5,
                    kd=ARM_KD,
                )
            session.set_actuators(desires, send=False)
            session.send_once()
            session.service_soft_kill()
            last_faults = _m3_arm_faults(session)
            samples += 1
            if any((f & 0xF) >= 8 for f in last_faults if f >= 0):
                hard_fault = True
                break
            time.sleep(dt)

        if not hard_fault and not clear_breach:
            stage2 = True
            active = list(range(7))
            dirs = {i: 1.0 for i in active}
            t_end = time.perf_counter() + min(8.0, hold_s)
            while time.perf_counter() < t_end:
                dt = 0.05
                for i in active:
                    lo, hi = CLEAR_LO[i], CLEAR_HI[i]
                    step = (rate_up if dirs[i] > 0 else -rate_down) * dt
                    nxt = cmd[i] + step
                    if nxt >= hi:
                        nxt = hi
                        dirs[i] = -1.0
                    elif nxt <= lo:
                        nxt = lo
                        dirs[i] = 1.0
                    cmd[i] = nxt
                desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
                for i, slot in enumerate(LEFT_ARM_SLOTS):
                    desires[slot] = ActuatorDesire(
                        position=float(cmd[i]),
                        velocity=0.0,
                        kp=float(ARM_KP[i]) * 0.5,
                        kd=ARM_KD,
                    )
                session.set_actuators(desires, send=False)
                session.send_once()
                last_faults = _m3_arm_faults(session)
                samples += 1
                if any((f & 0xF) >= 8 for f in last_faults if f >= 0):
                    hard_fault = True
                    break
                time.sleep(dt)

        _blank(hub)

    green = sum(1 for f in last_faults if f == 1)
    lines.append(f"- cruise samplesâ‰ˆ{samples} faults_end={last_faults} green={green}/7\n")
    lines.append(
        f"- stage2_all7={stage2} hard_fault={hard_fault} clear_breach={clear_breach}\n"
    )

    if hard_fault or clear_breach or green < 5:
        verdict = "FAIL"
    else:
        verdict = "PASS"

    lines.append(f"\n**Verdict: {verdict}**\n")
    if verdict == "FAIL":
        lines.append(
            _followup_note(
                mission,
                f"hard_fault={hard_fault} breach={clear_breach} faults={last_faults}",
                "motor",
                f"cmd_end={cmd}",
            )
        )
    _append_findings("".join(lines))
    _set_status(mission, verdict)
    return 0 if verdict == "PASS" else 1


# ---------------------------------------------------------------------------
# M4 â€” faster base + DXL via continuous short cruise
# ---------------------------------------------------------------------------


def cmd_m4(args: argparse.Namespace) -> int:
    mission = "M4"
    _set_status(mission, "RUNNING")
    duration = float(args.duration)
    base_rate = float(args.base_rate)
    lines: List[str] = [
        f"\n## {mission} Faster base + DXL â€” {_utc()}\n",
        f"- yam_continuous_all --duration {duration} --base-rate {base_rate}\n",
    ]
    log_path = Path("/tmp/mi_m4_cont.log")
    cmd = [
        sys.executable,
        "-u",
        str(SCRIPTS / "yam_continuous_all.py"),
        "--cruise-up",
        "0.18",
        "--cruise-down",
        "0.12",
        "--engage-s",
        "2.4",
        "--base-rate",
        str(base_rate),
        "--record",
        "--duration",
        str(duration),
    ]
    if args.port:
        cmd.extend(["--port", args.port])

    # Ensure no leftover continuous
    subprocess.run(["pkill", "-9", "-f", "yam_continuous_all.py"], check=False)
    time.sleep(0.5)
    # stop_can if present
    stop = SCRIPTS / "stop_can.py"
    if stop.is_file():
        subprocess.run([sys.executable, str(stop)], cwd=str(SCRIPTS), check=False)

    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            cmd, cwd=str(SCRIPTS), stdout=log, stderr=subprocess.STDOUT
        )
        try:
            proc.wait(timeout=duration + 90.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            lines.append("- TIMEOUT â€” killed continuous\n")

    text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.exists() else ""
    # Soft-kill early via flag if still running
    sess = SCRIPTS / ".deft_session" / "soft_kill_request"
    try:
        sess.parent.mkdir(parents=True, exist_ok=True)
        sess.write_text(f"{time.time():.3f}\n", encoding="utf-8")
        time.sleep(3.0)
    except Exception:
        pass
    subprocess.run(["pkill", "-9", "-f", "yam_continuous_all.py"], check=False)
    if stop.is_file():
        subprocess.run([sys.executable, str(stop)], cwd=str(SCRIPTS), check=False)

    has_dxl = "DXL present" in text and "WARN: DXL present incomplete" not in text
    has_base = "base: continuous" in text or "probe CH5" in text
    faults_ok = "faults=[1, 1, 1, 1, 1, 1, 1]" in text
    done = "done" in text or "duration reached" in text or "soft_kill" in text
    # Look for a status line with dxl= and s22=
    tracking = "dxl=" in text and "s22=" in text

    lines.append(f"- DXL present phase: {'yes' if has_dxl else 'no/weak'}\n")
    lines.append(f"- base cruise armed: {'yes' if has_base else 'no'}\n")
    lines.append(f"- MIT green seen: {'yes' if faults_ok else 'no'}\n")
    lines.append(f"- tracking lines: {'yes' if tracking else 'no'}\n")
    lines.append(f"- clean exit-ish: {'yes' if done else 'no'}\n")
    lines.append("\n### Log tail\n```\n")
    lines.append("\n".join(text.splitlines()[-40:]))
    lines.append("\n```\n")

    ok = faults_ok and tracking and has_base
    verdict = "PASS" if ok else "FAIL"
    if not Path("/dev/ttyACM0").exists() and not args.port and sys.platform.startswith("win"):
        # continuous may have failed to find port
        if "Port" not in text and "telemetry" not in text:
            verdict = "BLOCKED"
    lines.append(f"\n**Verdict: {verdict}**\n")
    if verdict != "PASS":
        lines.append(
            _followup_note(
                mission,
                "base/DXL elevated-rate cruise did not meet gates",
                "motor",
                "\n".join(text.splitlines()[-30:]),
            )
        )
    _append_findings("".join(lines))
    _set_status(mission, verdict)
    return 0 if verdict == "PASS" else 1


# ---------------------------------------------------------------------------
# M5 â€” Soft-DFU stress
# ---------------------------------------------------------------------------


def cmd_m5(args: argparse.Namespace) -> int:
    mission = "M5"
    _set_status(mission, "RUNNING")
    lines: List[str] = [f"\n## {mission} Soft-DFU stress â€” {_utc()}\n"]
    flash = SCRIPTS / "soft_dfu_flash.py"
    image = args.image
    if not image:
        for cand in (
            REPO / "Debug" / "DeftRoboticsControlsPCB.elf",
            REPO / "Release" / "DeftRoboticsControlsPCB.elf",
        ):
            if cand.is_file():
                image = str(cand)
                break

    # 1) scan
    r = subprocess.run(
        [sys.executable, str(flash), "scan"],
        cwd=str(SCRIPTS),
        capture_output=True,
        text=True,
    )
    scan_out = (r.stdout or "") + (r.stderr or "")
    lines.append("### scan\n```\n" + scan_out[:1500] + "\n```\n")
    has_dfu = "DF11" in scan_out and "no" not in scan_out.lower().split("DF11")[-1][:40]
    # parse gently
    usb_dfu = "0483:DF11" in scan_out and "DFU 0483:DF11: no" not in scan_out

    if not image or not Path(image).is_file():
        lines.append("- **BLOCKED:** no ELF image found for flash\n")
        lines.append(
            _followup_note(mission, "missing ELF", "DFU", scan_out[:500])
        )
        _append_findings("".join(lines))
        _set_status(mission, "BLOCKED")
        return 1

    lines.append(f"- image: `{image}`\n")
    lines.append(f"- USB DFU present at scan: {usb_dfu}\n")

    # Jetson/Linux known caveat: soft-enter drops CDC but DF11 often never
    # appears â€” do NOT soft-enter here or we orphan the port. SWD recover is
    # a laptop/ST-Link job; mark BLOCKED and stop.
    if (not usb_dfu) and sys.platform.startswith("linux"):
        lines.append(
            "- **BLOCKED (safe abort):** no 0483:DF11 on Linux host â€” refusing "
            "soft-enter to avoid CDC orphan (Jetson Soft-DFU caveat).\n"
        )
        lines.append(
            _followup_note(
                mission,
                "Jetson Soft-DFU: DF11 never enumerates; need laptop USB DFU or SWD",
                "DFU",
                scan_out[:800],
            )
        )
        lines.append("\n**Verdict: BLOCKED**\n")
        _append_findings("".join(lines))
        _set_status(mission, "BLOCKED")
        return 1

    # 2) timed flash
    t0 = time.perf_counter()
    r2 = subprocess.run(
        [sys.executable, "-u", str(flash), "--image", image],
        cwd=str(SCRIPTS),
        capture_output=True,
        text=True,
        timeout=180,
    )
    elapsed = time.perf_counter() - t0
    flash_out = (r2.stdout or "") + (r2.stderr or "")
    lines.append(f"- flash wall-clock: {elapsed:.1f}s exit={r2.returncode}\n")
    lines.append("### flash log\n```\n" + flash_out[-2000:] + "\n```\n")

    # 3) reclaim + cfg ping
    time.sleep(2.0)
    reclaim_ok = False
    try:
        port = args.port or find_cdc_port()
        with ControlsPcbHub.connect(port) as hub:
            hub.recover()
            table = hub.debug.cfg_get_table()
            reclaim_ok = len(table) > 0
            lines.append(f"- post-flash CDC `{port}` cfg rows={len(table)}\n")
            _blank(hub)
    except Exception as exc:
        lines.append(f"- post-flash reclaim FAIL: {exc}\n")

    # 4) second scan / enter-leave light touch
    r3 = subprocess.run(
        [sys.executable, str(flash), "scan"],
        cwd=str(SCRIPTS),
        capture_output=True,
        text=True,
    )
    scan2 = (r3.stdout or "") + (r3.stderr or "")
    lines.append("### scan after flash\n```\n" + scan2[:800] + "\n```\n")

    flash_ok = r2.returncode == 0
    if (not usb_dfu) and (
        "SWD" in flash_out.upper()
        or "st-link" in flash_out.lower()
        or "Cube" in flash_out
    ):
        lines.append("- note: likely SWD fallback path (Jetson USB-DFU caveat)\n")

    if flash_ok and reclaim_ok:
        verdict = "PASS"
    elif not usb_dfu and flash_ok and reclaim_ok:
        verdict = "PASS"
    elif not usb_dfu and not flash_ok:
        verdict = "BLOCKED"
        lines.append(
            _followup_note(
                mission,
                "USB DFU unavailable / flash failed on this host",
                "DFU",
                flash_out[-800:],
            )
        )
    else:
        verdict = "FAIL"
        lines.append(
            _followup_note(
                mission,
                f"flash_ok={flash_ok} reclaim_ok={reclaim_ok}",
                "DFU",
                flash_out[-800:],
            )
        )

    lines.append(f"\n**Verdict: {verdict}**\n")
    _append_findings("".join(lines))
    _set_status(mission, verdict)
    return 0 if verdict == "PASS" else 1


# ---------------------------------------------------------------------------
# all
# ---------------------------------------------------------------------------


def cmd_all(args: argparse.Namespace) -> int:
    rc = 0
    order: List[Tuple[str, Callable[[argparse.Namespace], int]]] = [
        ("preflight", cmd_preflight),
        ("m1", cmd_m1),
        ("m2", cmd_m2),
        ("m3", cmd_m3),
        ("m4", cmd_m4),
        ("m5", cmd_m5),
    ]
    port = args.port or find_cdc_port()
    for name, fn in order:
        print(f"\n======== MISSION {name} ========\n", flush=True)
        if name in ("m2", "m3", "m4"):
            _stop_can_quiet(port)
        try:
            r = fn(args)
        except Exception as exc:
            _append_findings(
                f"\n## {name} EXCEPTION â€” {_utc()}\n\n`{exc!r}`\n"
                + _followup_note(name.upper(), repr(exc), "plant", repr(exc))
            )
            key = name.upper()
            if key in ("M1", "M2", "M3", "M4", "M5"):
                _set_status(key, "FAIL")
            r = 1
        if r != 0:
            rc = r
            if not args.continue_on_fail and name != "preflight":
                print(f"stopping after {name} rc={r}", flush=True)
                break
    return rc


def _add_port(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", default=None, help="CDC port (default: auto-find)")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("preflight")
    _add_port(p)
    p.set_defaults(func=cmd_preflight)

    p = sub.add_parser("m1")
    _add_port(p)
    p.add_argument("--hz", default="40,100,200")
    p.add_argument("--trials", type=int, default=2)
    p.add_argument("--seconds", type=float, default=3.0)
    p.set_defaults(func=cmd_m1)

    p = sub.add_parser("m2")
    _add_port(p)
    p.set_defaults(func=cmd_m2)

    p = sub.add_parser("m3")
    _add_port(p)
    p.add_argument("--cruise-up", type=float, default=0.18)
    p.add_argument("--cruise-down", type=float, default=0.12)
    p.add_argument("--hold-s", type=float, default=20.0)
    p.set_defaults(func=cmd_m3)

    p = sub.add_parser("m4")
    _add_port(p)
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--base-rate", type=float, default=1.0)
    p.set_defaults(func=cmd_m4)

    p = sub.add_parser("m5")
    _add_port(p)
    p.add_argument("--image", default=None)
    p.set_defaults(func=cmd_m5)

    p = sub.add_parser("all")
    _add_port(p)
    p.add_argument("--hz", default="40,100,200")
    p.add_argument("--trials", type=int, default=2)
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--cruise-up", type=float, default=0.18)
    p.add_argument("--cruise-down", type=float, default=0.12)
    p.add_argument("--hold-s", type=float, default=20.0)
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--base-rate", type=float, default=1.0)
    p.add_argument("--image", default=None)
    p.add_argument("--continue-on-fail", action="store_true")
    p.set_defaults(func=cmd_all)

    args = ap.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
