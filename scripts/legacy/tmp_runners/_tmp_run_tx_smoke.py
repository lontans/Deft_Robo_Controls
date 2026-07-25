#!/usr/bin/env python3
"""Kill stale CDC owners, deploy lab, run --tx-smoke on Jetson."""
from __future__ import annotations

import os
import time
from pathlib import Path

import paramiko

LOCAL = Path(__file__).resolve().parent
HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb/scripts"


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)

    # Free CDC: hung interactive lab was holding /dev/ttyACM0 with blank+DIAG.
    _, o, _ = c.exec_command(
        "pkill -9 -f '_tmp_base_bus56_lab.py' || true; "
        "pkill -9 -f 'yam_continuous_all.py' || true; "
        "sleep 0.4; "
        "fuser /dev/ttyACM0 2>/dev/null || echo 'ACM0 free'; "
        f"pgrep -f 'python3 -u pdb_uart_sim' >/dev/null || "
        f"(cd {REMOTE} && nohup python3 -u pdb_uart_sim.py "
        f"--port /dev/ttyTHS1 --hz 20 --force-kill-state 0 --estop-sense 1 "
        f"--wander --pack-v 4800 4800 4800 4800 --rail-v 4800 1900 1200 500 "
        f"--pack-i 180 140 160 120 --rail-i 90 70 40 25 "
        f">/tmp/pdb_uart_sim.log 2>&1 </dev/null &)"
    )
    print(o.read().decode(errors="replace"))
    time.sleep(0.6)

    sftp = c.open_sftp()
    sftp.put(str(LOCAL / "_tmp_base_bus56_lab.py"), f"{REMOTE}/_tmp_base_bus56_lab.py")
    sftp.close()

    _, o, e = c.exec_command(
        f"cd {REMOTE} && python3 -m py_compile _tmp_base_bus56_lab.py && "
        f"python3 -u _tmp_base_bus56_lab.py --tx-smoke --hold-s 10",
        timeout=120,
    )
    print(o.read().decode(errors="replace"))
    err = e.read().decode(errors="replace")
    if err.strip():
        print("STDERR", err[-4000:])
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
