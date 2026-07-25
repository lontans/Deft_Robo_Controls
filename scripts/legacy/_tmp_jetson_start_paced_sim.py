#!/usr/bin/env python3
"""Start paced pdb_uart_sim on Jetson ttyTHS1 (Track B hygiene/prove)."""
from __future__ import annotations

import argparse
import os
import sys
import time

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = "deft-robotics"
REMOTE = "/home/deft-robotics/controls_pcb/scripts"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--force-kill-state", type=int, default=None)
    ap.add_argument("--simulate-kill-after", type=float, default=None)
    args = ap.parse_args()
    pw = os.environ.get("JETSON_PASS", "")
    if not pw:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=pw, timeout=15)

    def run(cmd: str, timeout: float = 20.0) -> str:
        print("JETSON>>>", cmd[:160])
        transport = c.get_transport()
        assert transport is not None
        chan = transport.open_session()
        chan.settimeout(timeout)
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

    extra = ""
    if args.force_kill_state is not None:
        extra += f" --force-kill-state {args.force_kill_state}"
    if args.simulate_kill_after is not None:
        extra += f" --simulate-kill-after {args.simulate_kill_after}"

    run("pkill -f pdb_uart_sim.py 2>/dev/null || true; sleep 0.35", timeout=8.0)
    run(
        f"cd {REMOTE} && rm -f /tmp/pdb_uart_sim.log && "
        f"setsid nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz {args.hz} "
        f"--estop-sense 1 --pack-v 4800 4800 0 0 --rail-v 4800 1900 1200 500 "
        f"--pack-i 120 80 0 0 --rail-i 50 30 20 10{extra} "
        f"</dev/null >/tmp/pdb_uart_sim.log 2>&1 & echo PID=$!",
        timeout=5.0,
    )
    time.sleep(1.2)
    run("pgrep -af pdb_uart_sim.py || true", timeout=5.0)
    run("tail -n 12 /tmp/pdb_uart_sim.log || echo NO_LOG", timeout=5.0)
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
