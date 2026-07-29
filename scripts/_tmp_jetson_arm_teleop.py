#!/usr/bin/env python3
"""Upload + run short arm teleop on Jetson via paramiko."""
from __future__ import annotations

import os
import sys

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASSWORD = "4565"
LOCAL = os.path.join(os.path.dirname(__file__), "_tmp_arm_teleop_remote.py")
REMOTE = "/home/deft-robotics/controls_pcb/scripts/_tmp_arm_teleop_remote.py"


def run(c: paramiko.SSHClient, cmd: str, timeout: float = 300) -> int:
    print(f"\n$ {cmd}", flush=True)
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    # Stream output live (ascii-safe for Windows consoles)
    while True:
        line = stdout.readline()
        if not line:
            break
        safe = line.encode("ascii", errors="replace").decode("ascii")
        print(safe, end="", flush=True)
    err = stderr.read().decode(errors="replace")
    code = stdout.channel.recv_exit_status()
    if err:
        safe_err = err.encode("ascii", errors="replace").decode("ascii")
        print(safe_err, end="" if safe_err.endswith("\n") else "\n", file=sys.stderr, flush=True)
    print(f"[exit {code}]", flush=True)
    return code


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASSWORD, timeout=15)
    try:
        run(
            c,
            "pkill -f yam_continuous_all.py || true; "
            "pkill -f _tmp_arm_teleop || true; "
            "fuser /dev/ttyACM0 2>/dev/null || echo CDC_FREE",
            timeout=30,
        )
        sftp = c.open_sftp()
        sftp.put(LOCAL, REMOTE)
        sftp.chmod(REMOTE, 0o755)
        sftp.close()
        print(f"uploaded {REMOTE}", flush=True)

        run(
            c,
            "cd ~/controls_pcb/scripts && PYTHONPATH=. python3 -u stop_can.py || true",
            timeout=60,
        )
        code = run(
            c,
            "cd ~/controls_pcb/scripts && PYTHONPATH=. python3 -u _tmp_arm_teleop_remote.py",
            timeout=180,
        )
        run(
            c,
            "cd ~/controls_pcb/scripts && PYTHONPATH=. python3 -u stop_can.py || true; "
            "fuser /dev/ttyACM0 2>/dev/null || echo CDC_FREE",
            timeout=60,
        )
        return code
    finally:
        c.close()


if __name__ == "__main__":
    raise SystemExit(main())
