#!/usr/bin/env python3
"""One-shot listen after user rewires — prints THS1/THS2 histogram."""
from __future__ import annotations

import os
import sys

import paramiko

PW = os.environ.get("JETSON_PASS", "")


def main() -> int:
    if not PW:
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)
    # ensure latest listener
    sftp = c.open_sftp()
    local = os.path.join(os.path.dirname(__file__), "jetson_uart_listen.py")
    sftp.put(local, "/home/deft-robotics/controls_pcb/scripts/jetson_uart_listen.py")
    sftp.close()
    cmd = (
        "pkill -f pdb_uart_sim.py 2>/dev/null || true; "
        "python3 /home/deft-robotics/controls_pcb/scripts/jetson_uart_listen.py "
        "--seconds 3 --ports /dev/ttyTHS1 /dev/ttyTHS2"
    )
    print(">>>", cmd)
    _, o, e = c.exec_command(cmd, timeout=40)
    sys.stdout.write(o.read().decode() + e.read().decode())
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
