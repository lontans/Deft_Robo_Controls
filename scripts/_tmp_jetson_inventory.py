#!/usr/bin/env python3
"""One-shot Jetson scripts inventory for Track B UART hygiene."""
from __future__ import annotations

import os
import sys

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = "deft-robotics"
PW = os.environ.get("JETSON_PASS", "")


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    cmds = [
        "ls -la /home/deft-robotics/controls_pcb/scripts/ | head -80",
        "ls -la /home/deft-robotics/controls_pcb/scripts/*pdb* "
        "/home/deft-robotics/controls_pcb/scripts/*uart* 2>/dev/null || true",
        "ls /home/deft-robotics/controls_pcb/scripts/_tmp_* 2>/dev/null | head -50 || true",
        "pgrep -af pdb_uart || true",
        "find /home/deft-robotics/controls_pcb/scripts -name '__pycache__' "
        "-o -name '*.pyc' 2>/dev/null | head -30 || true",
    ]
    for cmd in cmds:
        print(">>>", cmd)
        _, o, e = c.exec_command(cmd, timeout=30)
        sys.stdout.write(o.read().decode("utf-8", "replace"))
        sys.stdout.write(e.read().decode("utf-8", "replace"))
        print()
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
