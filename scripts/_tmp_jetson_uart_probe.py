#!/usr/bin/env python3
"""Upload jetson_uart_listen.py, kill sims, listen on all Jetson UARTs."""
from __future__ import annotations

import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")
HOST = "192.168.50.48"
LOCAL = os.path.join(os.path.dirname(__file__), "jetson_uart_listen.py")
REMOTE = "/home/deft-robotics/controls_pcb/scripts/jetson_uart_listen.py"


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    sftp.put(LOCAL, REMOTE)
    sftp.chmod(REMOTE, 0o755)
    sftp.close()

    def run(cmd: str, timeout: float = 60.0) -> int:
        print(">>>", cmd)
        _, o, e = c.exec_command(cmd, timeout=timeout)
        sys.stdout.write(o.read().decode("utf-8", "replace"))
        err = e.read().decode("utf-8", "replace")
        if err:
            sys.stdout.write(err)
        return o.channel.recv_exit_status()

    run(
        "pkill -f pdb_uart_sim.py 2>/dev/null || true; "
        "pkill -f jetson_uart_listen.py 2>/dev/null || true; "
        "sleep 0.3"
    )
    # Show which kernel nodes map where
    run(
        "ls -l /dev/ttyTHS* /dev/ttyTCU* /dev/ttyS* 2>/dev/null; "
        "echo '---'; "
        "ls -l /sys/class/tty/ttyTHS*/device/of_node 2>/dev/null | head"
    )
    # Fast path first: 115200 on THS1/THS2 only (full sweep is slow).
    code = run(f"python3 {REMOTE} --seconds 3 --ports /dev/ttyTHS1 /dev/ttyTHS2", timeout=45)
    if code != 0:
        run(f"python3 {REMOTE} --seconds 2 --baud-sweep --ports /dev/ttyTHS1 /dev/ttyTHS2", timeout=120)
    c.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
