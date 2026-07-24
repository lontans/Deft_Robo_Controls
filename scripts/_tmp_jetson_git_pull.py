#!/usr/bin/env python3
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
    cmd = (
        "cd /home/deft-robotics/controls_pcb && "
        "git status -sb && git pull --ff-only && git log -1 --oneline && "
        "ls -l /proc/device-tree/bus@0/serial@3100000/ 2>/dev/null | head -30"
    )
    print(">>>", cmd)
    _, o, e = c.exec_command(cmd, timeout=60)
    sys.stdout.write(o.read().decode("utf-8", "replace"))
    sys.stdout.write(e.read().decode("utf-8", "replace"))
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
