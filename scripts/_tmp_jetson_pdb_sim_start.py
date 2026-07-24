#!/usr/bin/env python3
"""Start pdb_uart_sim on Jetson in background; print first log lines."""
from __future__ import annotations

import os
import sys
import time

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "")
REPO = os.environ.get("JETSON_REPO", "/home/deft-robotics/controls_pcb")
PORT = os.environ.get("JETSON_UART", "/dev/ttyTHS1")
LOG = "/tmp/pdb_uart_sim.log"


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    def run(cmd: str, timeout: int = 60) -> tuple[int, str, str]:
        print(">>>", cmd)
        _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        code = stdout.channel.recv_exit_status()
        if out:
            print(out, end="" if out.endswith("\n") else "\n")
        if err:
            print("STDERR:", err, end="" if err.endswith("\n") else "\n")
        print("exit", code)
        return code, out, err

    run("groups; id")
    run(f"python3 {REPO}/scripts/pdb_uart_sim.py --help | head -80")
    # Kill any prior sim
    run("pkill -f 'pdb_uart_sim.py' || true")
    time.sleep(0.5)
    # Start without --random first for a clean NORMAL prove; gpio-estop on pin 16.
    # nohup + redirect so SSH can disconnect.
    start = (
        f"cd {REPO}/scripts && nohup python3 -u pdb_uart_sim.py "
        f"--port {PORT} --hz 20 --gpio-estop 16 --seed 1 "
        f"> {LOG} 2>&1 & echo PID=$!"
    )
    run(start)
    time.sleep(2.0)
    run(f"ps aux | grep -v grep | grep pdb_uart_sim || true")
    run(f"tail -n 40 {LOG} || true")
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
