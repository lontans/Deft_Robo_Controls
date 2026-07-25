#!/usr/bin/env python3
"""Integrated PDU kill-state × plant motion test (COM5 + Jetson pdb_uart_sim).

Cycles Jetson PDU peer through NORMAL → SOFT_KILL_REQ → SOFT_KILL_READY →
HARD_ESTOP → NORMAL while streaming neck DXL + bus6 RS02. Host implements the
product park reaction (firmware LEDs follow kill; actuators/servos freeze when
host enters ESTOP on fault kill — see docs/pdb-uart-v1.md soft-kill park).

Motion (NORMAL only):
  slot 0 — continuous bounce in plant_config range @ π/4 rad/s
  slot 1 — init to present, then ±SLOT1_WIGGLE_TICKS only (not full-range)
  RS02   — soft hold at present (kp>0); tiny optional wiggle

Metrics per phase: plant_fb_hz, cmd lag, act_lap / periph_lap, servo spans,
RS delta, LED mode, kill_state, PDB pack/rail V/I.

  set JETSON_PASS=...
  python scripts/_tmp_pdb_plant_integ_test.py --port COM5
"""
from __future__ import annotations

import argparse
import math
import os
import statistics
import struct
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import paramiko

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import (  # noqa: E402
    ActuatorDesire,
    ControlsPcbHub,
    LedDesire,
    McuState,
    ServoDesire,
)
from deft_controls_sdk.link.exchange import (  # noqa: E402
    ACTUATOR_COUNT,
    parse_actuator_feedback,
    parse_feedback_header,
    parse_servo_feedback,
)
from deft_controls_sdk.link.exchange.wire_layout import LED_CMD_OFF  # noqa: E402
from deft_controls_sdk.pdb import KILL_STATE_NAMES  # noqa: E402
from deft_controls_sdk.pdb.status import pdb_status_from_frame  # noqa: E402
from deft_controls_sdk.bench.rs02_motion import sample_position, seed_idle_at_fb  # noqa: E402
from deft_controls_sdk.bench.servo_fb import sample_servo_fb  # noqa: E402
from rs02_channel_bringup import CANONICAL_SLOT, assign_single_slot  # noqa: E402

JETSON = "192.168.50.48"
USER = "deft-robotics"
REMOTE_SCRIPTS = "/home/deft-robotics/controls_pcb/scripts"

SERVO_CFG = (
    {"slot": 0, "id": 1, "pos_min": 1024, "pos_max": 3072},
    {"slot": 1, "id": 2, "pos_min": 700, "pos_max": 2500},
)
DXL_TICKS_PER_REV = 4096.0
SLOT0_RATE_RAD_S = math.pi / 4.0
# Slot1: small wiggle around initial present (~±7°), not full-range bounce.
SLOT1_WIGGLE_TICKS = 80
SLOT1_RATE_RAD_S = math.pi / 16.0  # slower than slot0

# PDU force-kill phases (Jetson --force-kill-state).
PHASES: Tuple[Tuple[int, str, float], ...] = (
    (0, "NORMAL", 5.0),
    (1, "SOFT_KILL_REQ", 4.0),
    (2, "SOFT_KILL_READY", 4.0),
    (3, "HARD_ESTOP", 4.0),
    (0, "NORMAL_RESTORE", 5.0),
)

EXPECT_LED = {0: 8, 1: 6, 2: 5, 3: 7}  # NORMAL → IDLE_CORNFLOWER


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


def _mean(xs: List[float]) -> Optional[float]:
    return statistics.mean(xs) if xs else None


def _mode_int(xs: List[int], default: int = -1) -> int:
    if not xs:
        return default
    return Counter(xs).most_common(1)[0][0]


def _ssh(password: str) -> paramiko.SSHClient:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(JETSON, username=USER, password=password, timeout=15)
    return c


def _run(c: paramiko.SSHClient, cmd: str, timeout: float = 20.0) -> str:
    transport = c.get_transport()
    assert transport is not None
    chan = transport.open_session()
    chan.settimeout(timeout)
    print("JETSON>>>", cmd[:120] + ("…" if len(cmd) > 120 else ""))
    chan.exec_command(cmd)
    out = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if chan.recv_ready():
            out += chan.recv(4096)
        if chan.recv_stderr_ready():
            out += chan.recv_stderr(4096)
        if chan.exit_status_ready() and not chan.recv_ready() and not chan.recv_stderr_ready():
            break
        time.sleep(0.05)
    try:
        chan.recv_exit_status()
    except Exception:
        pass
    text = out.decode("utf-8", "replace").strip()
    if text:
        print(text)
    return text


def _deploy_sim(c: paramiko.SSHClient) -> None:
    local = Path(__file__).resolve().parent / "pdb_uart_sim.py"
    remote = f"{REMOTE_SCRIPTS}/pdb_uart_sim.py"
    sftp = c.open_sftp()
    sftp.put(str(local), remote)
    sftp.chmod(remote, 0o755)
    sftp.close()


def _start_sim(c: paramiko.SSHClient, kill_state: int) -> None:
    _run(c, "pkill -f pdb_uart_sim.py 2>/dev/null || true; sleep 0.35", timeout=8.0)
    _run(
        c,
        f"cd {REMOTE_SCRIPTS} && rm -f /tmp/pdb_uart_sim.log && "
        f"setsid nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        f"--force-kill-state {kill_state} --estop-sense 1 "
        f"--pack-v 4800 4800 0 0 --rail-v 4800 1900 1200 500 "
        f"--pack-i 120 80 0 0 --rail-i 50 30 20 10 "
        f"</dev/null >/tmp/pdb_uart_sim.log 2>&1 & echo PID=$!",
        timeout=5.0,
    )
    time.sleep(1.0)
    _run(c, "tail -n 4 /tmp/pdb_uart_sim.log || echo NO_LOG", timeout=5.0)


def _wait_kill(
    hub: ControlsPcbHub, want: int, *, timeout_s: float = 3.0, hz: float = 40.0
) -> bool:
    """Poll USB system.kill_state until it matches want (PDU fresh)."""
    dt = 1.0 / hz
    deadline = time.perf_counter() + timeout_s
    last = -1
    while time.perf_counter() < deadline:
        hub.send_once()
        time.sleep(0.005)
        st = hub.pdb_status()
        if st is not None:
            last = int(st.kill_state)
            if last == want:
                print(f"  kill sync ok (sys.kill={last})")
                return True
        time.sleep(dt)
    print(f"  WARN kill sync timeout want={want} last={last}")
    return False


def _stop_sim(c: paramiko.SSHClient) -> None:
    _run(c, "pkill -f pdb_uart_sim.py 2>/dev/null || true; sleep 0.4", timeout=8.0)


def _servo_step(
    *,
    pos: float,
    direction: float,
    dt: float,
    rate_steps_s: float,
    lo: float,
    hi: float,
) -> Tuple[float, float]:
    nxt = pos + direction * rate_steps_s * dt
    if nxt >= hi:
        return hi, -1.0
    if nxt <= lo:
        return lo, 1.0
    return nxt, direction


def _sample_servo_present(hub: ControlsPcbHub, *, hz: float = 40.0) -> Dict[int, int]:
    """Discover present pose per slot (same path as bus6 / _tmp_dxl_one)."""
    last: Dict[int, int] = {}
    for c in SERVO_CFG:
        fb = sample_servo_fb(
            hub, c["slot"], servo_id=c["id"], timeout_s=2.5, hz=hz
        )
        if fb is None:
            raise RuntimeError(
                f"no servo FB slot={c['slot']} id={c['id']} "
                "(check DXL power/bus)"
            )
        # Soft hold at present before next slot discover.
        sample_servo_fb(
            hub,
            c["slot"],
            servo_id=c["id"],
            hold_pos=int(fb),
            timeout_s=0.35,
            hz=hz,
        )
        last[c["slot"]] = int(fb)
        print(f"  slot{c['slot']} id={c['id']} present={fb}")
    return last


def _leave_idle(hub: ControlsPcbHub, holds: Dict[int, int]) -> None:
    try:
        hub.set_rx_sim_mask(0)
        for c in SERVO_CFG:
            pos = int(holds.get(c["slot"], (c["pos_min"] + c["pos_max"]) // 2))
            hub.set_servo(
                c["slot"],
                ServoDesire(
                    servo_id=c["id"],
                    native_step_position=pos,
                    torque_enable=False,
                    operating_mode=3,
                ),
                send=False,
            )
        hub.set_led(LedDesire(mode=0, master_brightness=0, led_count=0), send=False)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        hub.send_once()
        time.sleep(0.08)
        _conn(hub).clear_servos(send=False)
        hub.send_once()
        print("leave_idle: torque-off + clear")
    except Exception as exc:  # noqa: BLE001
        print(f"leave_idle WARN: {exc}")


def run_phase(
    hub: ControlsPcbHub,
    *,
    kill_force: int,
    phase_name: str,
    seconds: float,
    hz: float,
    servo_start: Dict[int, int],
    rs_slot: int,
    rs_start: float,
    fault: bool,
) -> Dict[str, Any]:
    """Stream one phase. fault=True → host ESTOP (freeze); else NORMAL motion."""
    dt = 1.0 / max(hz, 1.0)
    start_pose = dict(servo_start)

    if fault:
        # Product park path (typed helper) — FW acks SOFT_KILL_READY when peer
        # is still SOFT_KILL_REQ (force-kill phases keep peer state forced).
        hub.soft_kill_park(send=True)
        time.sleep(0.05)
    else:
        was_fault = _conn(hub).mcu_state in (McuState.ESTOP, McuState.RECOVERY)
        if was_fault:
            hub.recover()
            time.sleep(0.08)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        if was_fault:
            # Re-discover present only after ESTOP — keeps PDU link warm on
            # the initial NORMAL phase (long dual-slot sample can go stale).
            try:
                start_pose = _sample_servo_present(hub, hz=hz)
                print(
                    f"  re-arm present s0={start_pose[0]} s1={start_pose[1]}"
                )
            except RuntimeError as exc:
                print(f"  WARN re-arm present: {exc} — using prior holds")

    s0 = float(start_pose[0])
    s1_center = float(start_pose[1])
    s1_lo = max(float(SERVO_CFG[1]["pos_min"]), s1_center - SLOT1_WIGGLE_TICKS)
    s1_hi = min(float(SERVO_CFG[1]["pos_max"]), s1_center + SLOT1_WIGGLE_TICKS)
    s0_dir = -1.0 if s0 >= 0.5 * (SERVO_CFG[0]["pos_min"] + SERVO_CFG[0]["pos_max"]) else 1.0
    s1_dir = 1.0
    s0_pos, s1_pos = s0, s1_center
    rate0 = abs(SLOT0_RATE_RAD_S) / (2.0 * math.pi) * DXL_TICKS_PER_REV
    rate1 = abs(SLOT1_RATE_RAD_S) / (2.0 * math.pi) * DXL_TICKS_PER_REV

    servo_seen: Dict[int, List[int]] = {0: [], 1: []}
    rs_seen: List[float] = []
    ack_lags: List[int] = []
    act_laps: List[float] = []
    act_peaks: List[float] = []
    periph_laps: List[float] = []
    led_modes: List[int] = []
    kill_states: List[int] = []
    local_estops: List[int] = []
    pdb_samples: List[dict] = []
    last_sent: Optional[int] = None
    plant_fb_n = 0
    reader = _conn(hub).reader
    tf0 = reader.total_frames
    mcu_cmd = "ESTOP" if fault else "NORMAL"

    t0 = time.perf_counter()
    t_end = t0 + seconds
    t_span_arm = t0 + 0.6  # ignore settle window for motion-span checks
    next_t = t0

    while time.perf_counter() < t_end:
        if fault:
            # Keep ESTOP latched; SDK clears desires/servos on set.
            hub.set_mcu_state(McuState.ESTOP, send=False)
        else:
            hub.set_mcu_state(McuState.NORMAL, send=False)
            s0_pos, s0_dir = _servo_step(
                pos=s0_pos,
                direction=s0_dir,
                dt=dt,
                rate_steps_s=rate0,
                lo=float(SERVO_CFG[0]["pos_min"]),
                hi=float(SERVO_CFG[0]["pos_max"]),
            )
            s1_pos, s1_dir = _servo_step(
                pos=s1_pos,
                direction=s1_dir,
                dt=dt,
                rate_steps_s=rate1,
                lo=s1_lo,
                hi=s1_hi,
            )
            hub.set_servo(
                0,
                ServoDesire(
                    servo_id=1,
                    native_step_position=int(round(s0_pos)),
                    torque_enable=True,
                    operating_mode=3,
                ),
                send=False,
            )
            hub.set_servo(
                1,
                ServoDesire(
                    servo_id=2,
                    native_step_position=int(round(s1_pos)),
                    torque_enable=True,
                    operating_mode=3,
                ),
                send=False,
            )
            desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            desires[rs_slot] = ActuatorDesire(
                position=rs_start, velocity=0.0, kp=2.0, kd=0.5, torque=0.0
            )
            _conn(hub).set_actuators(desires, send=False)
            # LED owned by PDU override; host OFF desire is fine.
            hub.set_led(LedDesire(mode=0, master_brightness=0, led_count=0), send=False)

        while True:
            raw = reader.pop()
            if raw is None:
                break
            hdr = parse_feedback_header(raw)
            if hdr is None or hdr.get("is_debug"):
                continue
            plant_fb_n += 1
            ack = int(hdr["last_cmd_seq"]) & 0xFF
            if last_sent is not None:
                lag = (last_sent - ack) & 0xFF
                if lag <= 128:
                    ack_lags.append(lag)
            act = hdr.get("act_lap_ms", hdr.get("lap_ms"))
            if act is not None:
                act_laps.append(float(act))
            pk = hdr.get("act_lap_peak_ms", hdr.get("lap_max_ms"))
            if pk is not None:
                act_peaks.append(float(pk))
            if hdr.get("periph_lap_ms") is not None:
                periph_laps.append(float(hdr["periph_lap_ms"]))

            if len(raw) >= LED_CMD_OFF + 2:
                led_modes.append(struct.unpack_from("<H", raw, LED_CMD_OFF)[0] & 0x1F)
            st = pdb_status_from_frame(raw)
            if st is not None:
                kill_states.append(int(st.kill_state))
                local_estops.append(int(st.estop_sense))
                if st.pdb is not None:
                    pdb_samples.append(st.pdb)

            now_fb = time.perf_counter()
            for slot in (0, 1):
                sv = parse_servo_feedback(raw, slot)
                if sv is not None and now_fb >= t_span_arm:
                    servo_seen[slot].append(int(sv["present_position"]))
            act_fb = parse_actuator_feedback(raw, rs_slot)
            if act_fb is not None and now_fb >= t_span_arm:
                rs_seen.append(float(act_fb["position"]))

        hub.send_once()
        sent = _conn(hub)._last_sent_seq  # noqa: SLF001
        last_sent = (sent & 0xFF) if sent is not None else None

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    elapsed = max(time.perf_counter() - t0, 1e-6)
    raw_fb = reader.total_frames - tf0

    def span(samples: List[int]) -> int:
        return (max(samples) - min(samples)) if len(samples) >= 2 else 0

    rs_delta = (max(rs_seen) - min(rs_seen)) if len(rs_seen) >= 2 else 0.0
    last_pdb = pdb_samples[-1] if pdb_samples else None
    # Average last ~1 s of PDB V/I if available
    recent = pdb_samples[-max(1, int(hz)) :] if pdb_samples else []

    def avg_vec(key: str) -> Optional[List[float]]:
        if not recent:
            return None
        cols = list(zip(*(r[key] for r in recent)))
        return [statistics.mean(c) / 100.0 for c in cols]  # 10mV→V, 10mA→A

    kill_mode = _mode_int(kill_states)
    led_mode = _mode_int(led_modes)
    expect_led = EXPECT_LED[kill_force]

    ok_kill = kill_mode == kill_force
    ok_led = led_mode == expect_led
    if fault:
        ok_motion = span(servo_seen[0]) < 40 and span(servo_seen[1]) < 40 and rs_delta < 0.15
        ok_host = mcu_cmd == "ESTOP"
    else:
        ok_motion = span(servo_seen[0]) > 200 and span(servo_seen[1]) <= SLOT1_WIGGLE_TICKS * 2 + 40
        ok_host = mcu_cmd == "NORMAL"

    row = {
        "phase": phase_name,
        "kill_force": kill_force,
        "kill_name": KILL_STATE_NAMES.get(kill_force, str(kill_force)),
        "seconds": seconds,
        "host_mcu": mcu_cmd,
        "fault": fault,
        "kill_mode": kill_mode,
        "led_mode": led_mode,
        "expect_led": expect_led,
        "local_estop_mode": _mode_int(local_estops),
        "plant_fb_hz": plant_fb_n / elapsed,
        "raw_fb_hz": raw_fb / elapsed,
        "lag_mean": _mean([float(x) for x in ack_lags]),
        "lag_p95": _pct([float(x) for x in ack_lags], 95),
        "lag_max": max(ack_lags) if ack_lags else None,
        "act_lap_mean": _mean(act_laps),
        "act_lap_peak_max": max(act_peaks) if act_peaks else None,
        "periph_lap_mean": _mean(periph_laps),
        "servo0_span": span(servo_seen[0]),
        "servo1_span": span(servo_seen[1]),
        "rs_delta": rs_delta,
        "rs_n": len(rs_seen),
        "pack_v_V": avg_vec("pack_v"),
        "rail_v_V": avg_vec("rail_v"),
        "pack_i_A": avg_vec("pack_i"),
        "rail_i_A": avg_vec("rail_i"),
        "pdb_magic_ok": last_pdb is not None and last_pdb.get("kill_state") == kill_force,
        "ok_kill": ok_kill,
        "ok_led": ok_led,
        "ok_motion": ok_motion,
        "ok_host": ok_host,
        "pass": ok_kill and ok_led and ok_motion and ok_host,
    }
    return row


def _fmt_vec(v: Optional[List[float]], unit: str) -> str:
    if v is None:
        return "n/a"
    return "[" + ", ".join(f"{x:.2f}{unit}" for x in v) + "]"


def _print_row(r: Dict[str, Any]) -> None:
    tag = "PASS" if r["pass"] else "FAIL"
    lag = r["lag_mean"]
    lap = r["act_lap_mean"]
    lag_s = f"{lag:.3f}" if lag is not None else "n/a"
    lap_s = f"{lap:.3f}" if lap is not None else "n/a"
    print(
        f"  [{tag}] {r['phase']}: kill={r['kill_mode']}({r['kill_name']}) "
        f"led={r['led_mode']} (want {r['expect_led']}) host={r['host_mcu']} "
        f"fb={r['plant_fb_hz']:.1f}Hz lag_mean={lag_s} act_lap={lap_s} "
        f"s0_span={r['servo0_span']} s1_span={r['servo1_span']} "
        f"rs_d={r['rs_delta']:.3f}"
    )
    print(
        f"         pack_v={_fmt_vec(r['pack_v_V'], 'V')} "
        f"rail_v={_fmt_vec(r['rail_v_V'], 'V')} "
        f"pack_i={_fmt_vec(r['pack_i_A'], 'A')} "
        f"rail_i={_fmt_vec(r['rail_i_A'], 'A')}"
    )


def _write_report(path: Path, rows: List[Dict[str, Any]], meta: dict) -> None:
    lines = [
        "# PDU × plant integrated test",
        "",
        f"Date: {meta.get('date')}",
        f"Port: {meta.get('port')}  hz={meta.get('hz')}  RS slot={meta.get('rs_slot')} "
        f"id=0x{meta.get('motor_id'):02X}",
        "",
        "Host reacts to PDU kill: NORMAL → stream motion; kill≠0 → `McuState.ESTOP` "
        "(servos cleared / RS desires cleared). LEDs follow PDU via firmware override.",
        "",
        "Motion: slot0 full-range bounce @ π/4; slot1 ±"
        f"{SLOT1_WIGGLE_TICKS} ticks around initial present @ π/16; RS02 soft-hold.",
        "",
        "| Phase | kill | led | host | fb_hz | lag_mean | act_lap | s0 | s1 | rsΔ | pack_v | result |",
        "|-------|-----:|----:|------|------:|---------:|--------:|---:|---:|----:|--------|--------|",
    ]
    for r in rows:
        pack = _fmt_vec(r["pack_v_V"], "")
        lines.append(
            f"| {r['phase']} | {r['kill_mode']} | {r['led_mode']} | {r['host_mcu']} | "
            f"{r['plant_fb_hz']:.1f} | {r['lag_mean'] if r['lag_mean'] is not None else 'n/a'} | "
            f"{r['act_lap_mean'] if r['act_lap_mean'] is not None else 'n/a'} | "
            f"{r['servo0_span']} | {r['servo1_span']} | {r['rs_delta']:.3f} | {pack} | "
            f"{'PASS' if r['pass'] else 'FAIL'} |"
        )
    lines.extend(["", "## Per-phase detail", ""])
    for r in rows:
        lines.append(f"### {r['phase']} (`{r['kill_name']}`)")
        lines.append("")
        for k in (
            "ok_kill",
            "ok_led",
            "ok_motion",
            "ok_host",
            "lag_p95",
            "lag_max",
            "act_lap_peak_max",
            "periph_lap_mean",
            "raw_fb_hz",
            "rail_v_V",
            "pack_i_A",
            "rail_i_A",
            "local_estop_mode",
        ):
            lines.append(f"- `{k}`: {r.get(k)}")
        lines.append("")
    fails = sum(1 for r in rows if not r["pass"])
    lines.append(f"## Summary: failed={fails}/{len(rows)}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"report → {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--bus", type=int, default=6)
    ap.add_argument("--motor-id", type=lambda s: int(s, 0), default=0x70)
    ap.add_argument("--skip-deploy", action="store_true")
    ap.add_argument("--skip-cfg", action="store_true")
    ap.add_argument(
        "--out",
        default=str(
            Path(__file__).resolve().parents[1]
            / "docs"
            / "bench-pdb-plant-integ-2026-07-23.md"
        ),
    )
    args = ap.parse_args()

    pw = os.environ.get("JETSON_PASS", "")
    if not pw:
        print("set JETSON_PASS", file=sys.stderr)
        return 2

    rs_slot = CANONICAL_SLOT[args.bus]
    rows: List[Dict[str, Any]] = []
    holds: Dict[int, int] = {}

    print(f"COM5 integ port={args.port}  RS CH{args.bus}/0x{args.motor_id:02X} slot={rs_slot}")
    c = _ssh(pw)
    try:
        if not args.skip_deploy:
            print("deploy pdb_uart_sim.py")
            _deploy_sim(c)

        with ControlsPcbHub.connect(args.port, persist_telemetry=False) as hub:
            hub.set_rx_sim_mask(0)
            hub.set_mcu_state(McuState.NORMAL, send=True)

            if not args.skip_cfg:
                assign_single_slot(
                    hub,
                    bus=args.bus,
                    slot=rs_slot,
                    motor_id=args.motor_id,
                    persist=False,
                )

            print("discover neck present…")
            holds = _sample_servo_present(hub, hz=args.hz)
            print(f"  servo0={holds[0]}  servo1={holds[1]} (slot1 wiggle ±{SLOT1_WIGGLE_TICKS})")

            # Bring up RS02 under NORMAL peer first.
            _start_sim(c, 0)
            time.sleep(0.5)
            seed_idle_at_fb(hub, rs_slot, 0.0)
            # Soft hold with kp so MIT is live; sample present.
            for _ in range(20):
                desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
                desires[rs_slot] = ActuatorDesire(
                    position=0.0, velocity=0.0, kp=2.0, kd=0.5
                )
                _conn(hub).set_actuators(desires, send=False)
                for slot, pos in holds.items():
                    hub.set_servo(
                        slot,
                        ServoDesire(
                            servo_id=SERVO_CFG[slot]["id"],
                            native_step_position=pos,
                            torque_enable=True,
                            operating_mode=3,
                        ),
                        send=False,
                    )
                hub.send_once()
                time.sleep(1.0 / args.hz)
            rs_pos = sample_position(hub, rs_slot, timeout_s=1.5)
            if rs_pos is None:
                print("WARN: no RS02 FB — continuing with rs_start=0")
                rs_start = 0.0
            else:
                rs_start = float(rs_pos)
                print(f"  RS02 present={rs_start:+.4f} rad")
                seed_idle_at_fb(hub, rs_slot, rs_start)

            for kill_force, phase_name, seconds in PHASES:
                fault = kill_force != 0
                print(
                    f"\n=== {phase_name} force-kill={kill_force} "
                    f"({KILL_STATE_NAMES.get(kill_force, '?')}) "
                    f"{'FREEZE' if fault else 'MOTION'} {seconds:.1f}s ==="
                )
                _start_sim(c, kill_force)
                time.sleep(0.4)
                _wait_kill(hub, kill_force, timeout_s=4.0, hz=args.hz)
                row = run_phase(
                    hub,
                    kill_force=kill_force,
                    phase_name=phase_name,
                    seconds=seconds,
                    hz=args.hz,
                    servo_start=holds,
                    rs_slot=rs_slot,
                    rs_start=rs_start,
                    fault=fault,
                )
                rows.append(row)
                _print_row(row)

            _leave_idle(hub, holds)
            print("\n=== leave NORMAL peer running ===")
            _start_sim(c, 0)
    finally:
        c.close()

    out = Path(args.out)
    _write_report(
        out,
        rows,
        meta={
            "date": time.strftime("%Y-%m-%d %H:%M"),
            "port": args.port,
            "hz": args.hz,
            "rs_slot": rs_slot,
            "motor_id": args.motor_id,
        },
    )

    fails = sum(1 for r in rows if not r["pass"])
    print(f"\n--- summary failed={fails}/{len(rows)} ---")
    for r in rows:
        print(
            f"  {'PASS' if r['pass'] else 'FAIL'} {r['phase']}: "
            f"kill={r['kill_mode']} led={r['led_mode']} "
            f"s0={r['servo0_span']} s1={r['servo1_span']} rsΔ={r['rs_delta']:.3f}"
        )
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
