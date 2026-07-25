#!/usr/bin/env python3
from __future__ import annotations

import os
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "4565")


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)
    for i in range(90):
        _, o, _ = c.exec_command(
            "if pgrep -f '_tmp_base_bus56_lab.py --prove' >/dev/null; then echo RUN; else echo DONE; fi; "
            "tail -50 /tmp/prove360.log"
        )
        out = o.read().decode(errors="replace")
        print(f"--- poll {i} ---")
        print(out)
        if out.startswith("DONE"):
            break
        time.sleep(8)
    else:
        print("TIMEOUT waiting")
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
