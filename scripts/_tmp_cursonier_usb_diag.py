"""Diagnose Jetson USB after Soft-DFU soft-enter failure."""
from __future__ import annotations

import os
import time

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "4565")


def run(c, cmd, timeout=30.0):
    print("JETSON>>>", cmd)
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    print(out, end="" if not out or out.endswith("\n") else "\n")
    if err.strip():
        print("STDERR:", err)
    return out


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)
    for cmd in [
        "lsusb | grep -i -E 'STMicro|0483|dfu|ACM' || lsusb",
        "ls -l /dev/ttyACM* 2>/dev/null || echo 'no ttyACM'",
        "cd ~/controls_pcb && python3 scripts/soft_dfu_flash.py scan",
        "echo '4565' | sudo -S dfu-util -l 2>&1 | tail -30",
        "dmesg | tail -40",
    ]:
        run(c, cmd)
        time.sleep(0.3)
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
