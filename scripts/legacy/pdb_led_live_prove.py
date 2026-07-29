#!/usr/bin/env python3
"""COM5 prove: Jetson PDU kill_state drives Controls factory LEDs (no USB LedDesire lap).

Mapping (UART4=PDB, firmware led_mode_from_pdb):
  NORMAL + fresh     → IDLE_CORNFLOWER (8)  # cornflower 500/500
  SOFT_KILL_REQ      → BLINK_YELLOW_SLOW (6)
  SOFT_KILL_READY    → SOLID_RED (5)
  HARD_ESTOP / stale → BLINK_RED_FAST (7)
  estop_sense wire 0 → BLINK_RED_FAST (7)

Laptop COM5 = Controls CDC. Jetson /dev/ttyTHS1 = paced pdb_uart_sim peer.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from typing import Optional

import paramiko

from deft_controls_sdk import ControlsPcbHub, McuState
from deft_controls_sdk.link.api_types import LedDesire, LED_MODE_OFF
from deft_controls_sdk.link.exchange.wire_layout import LED_CMD_OFF, PDB_OFF
from deft_controls_sdk.pdb import KILL_STATE_NAMES, MAGIC_FB
from deft_controls_sdk.pdb.status import pdb_status_from_frame

JETSON = "192.168.50.48"
USER = "deft-robotics"
REMOTE_SCRIPTS = "/home/deft-robotics/controls_pcb/scripts"

# kill_state → expected effective LED mode_readback
EXPECT = {
    0: 8,  # NORMAL → IDLE_CORNFLOWER (500/500)
    1: 6,  # SOFT_KILL_REQ → BLINK_YELLOW_SLOW
    2: 5,  # SOFT_KILL_READY → SOLID_RED
    3: 7,  # HARD_ESTOP → BLINK_RED_FAST
}


def _led_mode(raw: bytes) -> int:
    word = struct.unpack_from("<H", raw, LED_CMD_OFF)[0]
    return int(word & 0x1F)


def _sample(hub: ControlsPcbHub, seconds: float = 1.2) -> dict:
    last: Optional[bytes] = None
    t0 = time.time()
    while time.time() - t0 < seconds:
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            last = fb.raw
            hub._connection.publish_feedback(fb)
        time.sleep(0.02)
    if last is None:
        raise RuntimeError("no USB feedback frames")
    status = pdb_status_from_frame(last)
    if status is None:
        raise RuntimeError("pdb_status parse failed")
    magic = struct.unpack_from("<I", last, PDB_OFF)[0]
    return {
        "kill_state": status.kill_state,
        "kill_reason": status.kill_reason,
        "estop_sense": status.estop_sense,
        "led_mode": _led_mode(last),
        "pdb": status.pdb,
        "magic": magic,
        "status": status,
    }


def _ssh(password: str) -> paramiko.SSHClient:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(JETSON, username=USER, password=password, timeout=15)
    return c


def _run(c: paramiko.SSHClient, cmd: str, timeout: float = 20.0) -> str:
    """Non-blocking-friendly SSH exec (paramiko file.read can hang on bg jobs)."""
    transport = c.get_transport()
    assert transport is not None
    chan = transport.open_session()
    chan.settimeout(timeout)
    print("JETSON>>>", cmd)
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
    local = os.path.join(os.path.dirname(__file__), "pdb_uart_sim.py")
    remote = f"{REMOTE_SCRIPTS}/pdb_uart_sim.py"
    sftp = c.open_sftp()
    sftp.put(local, remote)
    sftp.chmod(remote, 0o755)
    sftp.close()


def _start_sim(
    c: paramiko.SSHClient, kill_state: int, *, estop_sense: int = 1
) -> None:
    _run(c, "pkill -f pdb_uart_sim.py 2>/dev/null || true; sleep 0.4", timeout=8.0)
    # Detach only — do not sleep/tail in the same remote shell (job wait hangs SSH).
    _run(
        c,
        f"cd {REMOTE_SCRIPTS} && rm -f /tmp/pdb_uart_sim.log && "
        f"setsid nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        f"--force-kill-state {kill_state} --estop-sense {estop_sense} "
        f"</dev/null >/tmp/pdb_uart_sim.log 2>&1 & echo PID=$!",
        timeout=5.0,
    )
    time.sleep(1.2)
    _run(c, "ps aux | grep -v grep | grep pdb_uart_sim || echo NO_PROC", timeout=5.0)
    _run(c, "tail -n 8 /tmp/pdb_uart_sim.log || echo NO_LOG", timeout=5.0)


def _stop_sim(c: paramiko.SSHClient) -> None:
    _run(c, "pkill -f pdb_uart_sim.py 2>/dev/null || true; sleep 0.5", timeout=8.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--hold-s", type=float, default=2.0, help="visual hold per kill phase")
    ap.add_argument("--skip-deploy", action="store_true")
    args = ap.parse_args()

    pw = os.environ.get("JETSON_PASS", "")
    if not pw:
        print("set JETSON_PASS (Jetson SSH password)", file=sys.stderr)
        return 2

    results: list[str] = []
    failed = 0

    print(f"COM5 prove port={args.port} — PDU kill → LED (host LedDesire=OFF)")
    c = _ssh(pw)
    try:
        if not args.skip_deploy:
            print("deploy pdb_uart_sim.py → Jetson")
            _deploy_sim(c)

        with ControlsPcbHub.connect(args.port, persist_telemetry=False) as hub:
            hub.set_mcu_state(McuState.NORMAL, send=True)
            # Host must not drive modes; PDB override owns the strip.
            hub.set_led(LedDesire(mode=LED_MODE_OFF, master_brightness=0, led_count=0), send=True)
            time.sleep(0.2)

            for kill, expect_led in EXPECT.items():
                print(f"\n=== force-kill-state={kill} ({KILL_STATE_NAMES.get(kill, '?')}) "
                      f"expect led_mode={expect_led} ===")
                _start_sim(c, kill)
                time.sleep(0.8)
                s = _sample(hub, max(1.2, args.hold_s))
                print(
                    f"  sys.kill={s['kill_state']} led_mode={s['led_mode']} "
                    f"estop_sense={s['estop_sense']} magic=0x{s['magic']:08X}"
                )
                if s["pdb"] is not None:
                    print(
                        f"  PDBF kill={s['pdb']['kill_state']} "
                        f"estop={s['pdb']['estop_sense']}"
                    )
                # Local PB7 (sys.estop_sense) may be stuck-low on this bench;
                # LED maps kill_state + PDBF peer estop, not the GPIO.
                ok = (
                    s["magic"] == MAGIC_FB
                    and s["kill_state"] == kill
                    and s["led_mode"] == expect_led
                )
                tag = "PASS" if ok else "FAIL"
                if not ok:
                    failed += 1
                line = (
                    f"kill_{kill}_led={tag} "
                    f"(kill={s['kill_state']} led={s['led_mode']} want_led={expect_led} "
                    f"local_estop={s['estop_sense']})"
                )
                results.append(line)
                print(line)
                time.sleep(args.hold_s)

            print("\n=== stop Jetson sim (stale → HARD_ESTOP / led 7) ===")
            _stop_sim(c)
            time.sleep(0.6)
            s = _sample(hub, 1.5)
            print(
                f"  sys.kill={s['kill_state']} led_mode={s['led_mode']} "
                f"estop_sense={s['estop_sense']}"
            )
            ok_stale = s["kill_state"] == 3 and s["led_mode"] == 7
            tag = "PASS" if ok_stale else "FAIL"
            if not ok_stale:
                failed += 1
            results.append(
                f"stale_hard_estop_led={tag} "
                f"(kill={s['kill_state']} led={s['led_mode']})"
            )
            print(results[-1])

            print("\n=== PDBF estop_sense=0 with NORMAL kill → led 7 ===")
            _start_sim(c, 0, estop_sense=0)
            time.sleep(0.8)
            s = _sample(hub, 1.5)
            print(
                f"  sys.kill={s['kill_state']} led_mode={s['led_mode']} "
                f"local_estop={s['estop_sense']} "
                f"pdb_estop={None if s['pdb'] is None else s['pdb']['estop_sense']}"
            )
            ok_peer = s["kill_state"] == 0 and s["led_mode"] == 7
            if s["pdb"] is not None:
                ok_peer = ok_peer and s["pdb"]["estop_sense"] == 0
            tag = "PASS" if ok_peer else "FAIL"
            if not ok_peer:
                failed += 1
            results.append(
                f"peer_estop_led={tag} "
                f"(kill={s['kill_state']} led={s['led_mode']})"
            )
            print(results[-1])

            print("\n=== restore NORMAL + peer estop released ===")
            _start_sim(c, 0, estop_sense=1)
            time.sleep(1.0)
            s = _sample(hub, 1.2)
            ok_r = s["kill_state"] == 0 and s["led_mode"] == 8
            results.append(
                f"restore_normal={'PASS' if ok_r else 'FAIL'} "
                f"(kill={s['kill_state']} led={s['led_mode']} "
                f"local_estop={s['estop_sense']})"
            )
            print(results[-1])
            if not ok_r:
                failed += 1

    finally:
        c.close()

    print("\n--- summary ---")
    for line in results:
        print(line)
    print(f"failed={failed}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
