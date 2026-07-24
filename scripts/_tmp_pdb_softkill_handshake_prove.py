#!/usr/bin/env python3
"""Prove soft-kill handshake via Jetson --simulate-kill-after (not force-kill).

Flow:
  1. Start paced pdb_uart_sim on Jetson ttyTHS1 with --simulate-kill-after N
  2. Under NORMAL: visibly wiggle neck DXL + bus6 RS02 (0x70)
  3. Stream COM5 until USB kill_state == SOFT_KILL_REQ
  4. hub.soft_kill_park() → ESTOP → FW plant_recovery_all + SOFT_KILL_READY ack
  5. Wait until USB/PDBF kill_state == SOFT_KILL_READY (sim saw the ack)
  6. Recover host + leave a NORMAL PDU peer running so LEDs stay green
     (killing the sim without a peer → stale → blink-red)

  set JETSON_PASS=...
  python scripts/_tmp_pdb_softkill_handshake_prove.py --port COM5
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import paramiko

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deft_controls_sdk import (  # noqa: E402
    ActuatorDesire,
    ControlsPcbHub,
    LedDesire,
    McuState,
    ServoDesire,
)
from deft_controls_sdk.link.api_types import LED_MODE_OFF  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT  # noqa: E402
from deft_controls_sdk.link.exchange.wire_layout import LED_CMD_OFF  # noqa: E402
from deft_controls_sdk.pdb import (  # noqa: E402
    KILL_NORMAL,
    KILL_SOFT_READY,
    KILL_SOFT_REQ,
    KILL_STATE_NAMES,
)
import struct  # noqa: E402
from _tmp_dxl_one import sample_servo_fb  # noqa: E402
from rs02_channel_bringup import (  # noqa: E402
    CANONICAL_SLOT,
    assign_single_slot,
    sample_position,
    seed_idle_at_fb,
)

JETSON = "192.168.50.48"
USER = "deft-robotics"
REMOTE_SCRIPTS = "/home/deft-robotics/controls_pcb/scripts"

SERVO_CFG = (
    {"slot": 0, "id": 1, "pos_min": 1024, "pos_max": 3072},
    {"slot": 1, "id": 2, "pos_min": 700, "pos_max": 2500},
)

# Visible RS02 travel before kill (rad). ~±25° around present.
RS_WIGGLE_RAD = 0.45
RS_RATE_RAD_S = math.pi / 6.0  # 30°/s — easy to see


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
    print("JETSON>>>", cmd[:140] + ("…" if len(cmd) > 140 else ""))
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


def _start_sim(
    c: paramiko.SSHClient,
    *,
    kill_after: float | None = None,
    force_kill: int | None = None,
) -> None:
    _run(c, "pkill -f pdb_uart_sim.py 2>/dev/null || true; sleep 0.35", timeout=8.0)
    extra = ""
    if kill_after is not None:
        extra += f" --simulate-kill-after {kill_after}"
    if force_kill is not None:
        extra += f" --force-kill-state {force_kill}"
    _run(
        c,
        f"cd {REMOTE_SCRIPTS} && rm -f /tmp/pdb_uart_sim.log && "
        f"setsid nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        f"--estop-sense 1 "
        f"--pack-v 4800 4800 0 0 --rail-v 4800 1900 1200 500 "
        f"--pack-i 120 80 0 0 --rail-i 50 30 20 10{extra} "
        f"</dev/null >/tmp/pdb_uart_sim.log 2>&1 & echo PID=$!",
        timeout=5.0,
    )
    time.sleep(1.0)
    _run(c, "tail -n 6 /tmp/pdb_uart_sim.log || echo NO_LOG", timeout=5.0)


def _wait_kill(hub: ControlsPcbHub, want: int, *, timeout_s: float, hz: float) -> bool:
    dt = 1.0 / hz
    deadline = time.perf_counter() + timeout_s
    last = -1
    while time.perf_counter() < deadline:
        hub.send_once()
        time.sleep(0.005)
        st = hub.pdb_status()
        if st is not None:
            last = st.kill_state
            if last == want:
                print(
                    f"  kill sync ok (sys.kill={last} "
                    f"{KILL_STATE_NAMES.get(last, '?')})"
                )
                return True
        time.sleep(dt)
    print(f"  WARN kill sync timeout want={want} last={last}")
    return False


def _led_mode(hub: ControlsPcbHub) -> int | None:
    st = hub.pdb_status()
    raw = hub._connection._latest_fb_raw  # noqa: SLF001
    if raw is None or len(raw) < LED_CMD_OFF + 2:
        return None
    return int(struct.unpack_from("<H", raw, LED_CMD_OFF)[0] & 0x1F)


def _restore_normal_peer(c: paramiko.SSHClient, hub: ControlsPcbHub, *, hz: float) -> None:
    """Leave a fresh NORMAL PDU peer so LEDs go green (not stale blink-red)."""
    print("restore NORMAL PDU peer (keep running — do not leave stale)…")
    hub.recover()
    hub.set_mcu_state(McuState.NORMAL, send=True)
    hub.set_led(LedDesire(mode=LED_MODE_OFF, master_brightness=0, led_count=0), send=False)
    hub.send_once()
    _start_sim(c, force_kill=0)
    if not _wait_kill(hub, KILL_NORMAL, timeout_s=4.0, hz=hz):
        print("  WARN kill did not return NORMAL")
    # Pump a bit so PDB override paints SOLID_GREEN.
    for _ in range(int(hz)):
        hub.send_once()
        time.sleep(1.0 / hz)
    st = hub.pdb_status()
    led = _led_mode(hub)
    print(
        f"  bench leave: kill={st.kill_state if st else '?'} "
        f"led_mode={led} (want kill=0 led=8 idle cornflower)"
    )


def _run_visible_motion(
    hub: ControlsPcbHub,
    *,
    holds: dict[int, int],
    rs_slot: int,
    rs_start: float,
    seconds: float,
    hz: float,
) -> None:
    """Bounce RS02 ±RS_WIGGLE_RAD and DXL slot0; slot1 small wiggle."""
    dt = 1.0 / max(hz, 1.0)
    t0 = time.perf_counter()
    t_end = t0 + seconds
    s0 = float(holds.get(0, 2048))
    s1 = float(holds.get(1, 1600))
    s0_lo, s0_hi = float(SERVO_CFG[0]["pos_min"]), float(SERVO_CFG[0]["pos_max"])
    s1_lo = max(float(SERVO_CFG[1]["pos_min"]), s1 - 80)
    s1_hi = min(float(SERVO_CFG[1]["pos_max"]), s1 + 80)
    s0_dir = 1.0
    s1_dir = 1.0
    rate0 = (math.pi / 4.0) / (2.0 * math.pi) * 4096.0
    rate1 = (math.pi / 16.0) / (2.0 * math.pi) * 4096.0

    rs_lo = rs_start - RS_WIGGLE_RAD
    rs_hi = rs_start + RS_WIGGLE_RAD
    rs_pos = rs_start
    rs_dir = 1.0
    rs_min_seen = rs_start
    rs_max_seen = rs_start

    print(
        f"  MOTION {seconds:.1f}s: RS02 ±{RS_WIGGLE_RAD:.2f} rad around "
        f"{rs_start:+.3f} @ {RS_RATE_RAD_S:.2f} rad/s — watch the shaft"
    )
    next_t = time.perf_counter()
    while time.perf_counter() < t_end:
        s0 = s0 + s0_dir * rate0 * dt
        if s0 >= s0_hi:
            s0, s0_dir = s0_hi, -1.0
        elif s0 <= s0_lo:
            s0, s0_dir = s0_lo, 1.0
        s1 = s1 + s1_dir * rate1 * dt
        if s1 >= s1_hi:
            s1, s1_dir = s1_hi, -1.0
        elif s1 <= s1_lo:
            s1, s1_dir = s1_lo, 1.0

        rs_pos = rs_pos + rs_dir * RS_RATE_RAD_S * dt
        if rs_pos >= rs_hi:
            rs_pos, rs_dir = rs_hi, -1.0
        elif rs_pos <= rs_lo:
            rs_pos, rs_dir = rs_lo, 1.0
        rs_min_seen = min(rs_min_seen, rs_pos)
        rs_max_seen = max(rs_max_seen, rs_pos)

        desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
        desires[rs_slot] = ActuatorDesire(
            position=rs_pos, velocity=0.0, kp=8.0, kd=0.8, torque=0.0
        )
        hub._connection.set_actuators(desires, send=False)  # noqa: SLF001
        hub.set_servo(
            0,
            ServoDesire(
                servo_id=1,
                native_step_position=int(round(s0)),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )
        hub.set_servo(
            1,
            ServoDesire(
                servo_id=2,
                native_step_position=int(round(s1)),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )
        hub.set_led(LedDesire(mode=LED_MODE_OFF), send=False)
        hub.send_once()
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    print(
        f"  motion done: RS cmd span [{rs_min_seen:+.3f} .. {rs_max_seen:+.3f}] "
        f"(Δ={rs_max_seen - rs_min_seen:.3f} rad)"
    )
    # Soft-hold at center before kill so park is clean.
    seed_idle_at_fb(hub, rs_slot, rs_start)
    desires = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    desires[rs_slot] = ActuatorDesire(
        position=rs_start, velocity=0.0, kp=2.0, kd=0.5
    )
    hub._connection.set_actuators(desires, send=True)  # noqa: SLF001


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--bus", type=int, default=6)
    ap.add_argument("--motor-id", type=lambda s: int(s, 0), default=0x70)
    # Longer default so RS wiggle is visible before kill injects.
    ap.add_argument("--kill-after", type=float, default=8.0)
    ap.add_argument("--motion-s", type=float, default=6.0)
    ap.add_argument("--skip-deploy", action="store_true")
    ap.add_argument("--skip-motion", action="store_true")
    ap.add_argument(
        "--leave-stale",
        action="store_true",
        help="stop Jetson sim at end (LEDs will blink-red on stale)",
    )
    args = ap.parse_args()

    pw = os.environ.get("JETSON_PASS", "")
    if not pw:
        print("set JETSON_PASS", file=sys.stderr)
        return 2

    rs_slot = CANONICAL_SLOT[args.bus]
    print(
        f"handshake prove port={args.port} RS CH{args.bus}/0x{args.motor_id:02X} "
        f"slot={rs_slot} kill_after={args.kill_after}s motion={args.motion_s}s"
    )

    c = _ssh(pw)
    rc = 1
    try:
        if not args.skip_deploy:
            print("deploy pdb_uart_sim.py")
            _deploy_sim(c)

        with ControlsPcbHub.connect(args.port, persist_telemetry=False) as hub:
            hub.set_rx_sim_mask(0)
            hub.set_mcu_state(McuState.NORMAL, send=True)
            assign_single_slot(
                hub,
                bus=args.bus,
                slot=rs_slot,
                motor_id=args.motor_id,
                persist=False,
            )

            # kill_after must cover discover + motion window.
            kill_after = max(args.kill_after, args.motion_s + 2.0)
            _start_sim(c, kill_after=kill_after)

            holds: dict[int, int] = {}
            rs_start = 0.0
            if not args.skip_motion:
                print("discover present (DXL + RS02)…")
                for cfg in SERVO_CFG:
                    fb = sample_servo_fb(
                        hub, cfg["slot"], servo_id=cfg["id"], timeout_s=2.5, hz=args.hz
                    )
                    if fb is None:
                        print(f"  WARN no servo FB slot={cfg['slot']}")
                        continue
                    holds[cfg["slot"]] = int(fb)
                    print(f"  servo slot{cfg['slot']} present={fb}")
                seed_idle_at_fb(hub, rs_slot, 0.0)
                rs_pos = sample_position(hub, rs_slot, timeout_s=1.5)
                if rs_pos is None:
                    print("  WARN no RS02 FB — using 0.0")
                    rs_start = 0.0
                else:
                    rs_start = float(rs_pos)
                    print(f"  RS02 present={rs_start:+.4f} rad")
                    seed_idle_at_fb(hub, rs_slot, rs_start)

                _run_visible_motion(
                    hub,
                    holds=holds,
                    rs_slot=rs_slot,
                    rs_start=rs_start,
                    seconds=args.motion_s,
                    hz=args.hz,
                )

            print("wait SOFT_KILL_REQ…")
            if not _wait_kill(
                hub, KILL_SOFT_REQ, timeout_s=kill_after + 5.0, hz=args.hz
            ):
                _run(c, "tail -n 20 /tmp/pdb_uart_sim.log || true", timeout=5.0)
                return 1

            print("soft_kill_park()…")
            hub.soft_kill_park(send=True)
            st = hub.pdb_status()
            print(f"  post-park status={st.to_dict() if st else None}")

            print("wait SOFT_KILL_READY (sim ack path)…")
            ok_ready = _wait_kill(
                hub, KILL_SOFT_READY, timeout_s=6.0, hz=args.hz
            )
            _run(c, "tail -n 30 /tmp/pdb_uart_sim.log || true", timeout=5.0)
            if not ok_ready:
                return 1

            print("PASS soft-kill handshake")
            rc = 0

            if args.leave_stale:
                print("leave-stale: stopping Jetson sim (LEDs → blink-red)")
                _run(c, "pkill -f pdb_uart_sim.py 2>/dev/null || true", timeout=8.0)
            else:
                _restore_normal_peer(c, hub, hz=args.hz)
            return rc
    finally:
        # Only tear down SSH; NORMAL peer intentionally left running unless --leave-stale.
        try:
            c.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
