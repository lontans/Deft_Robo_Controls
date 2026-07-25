#!/usr/bin/env python3
"""Deploy lab + run --prove-360 on Jetson; write log locally as UTF-8."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import paramiko

LOCAL = Path(__file__).resolve().parent
HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb/scripts"
OUT = LOCAL / "_tmp_prove360_out.txt"


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)

    _, o, _ = c.exec_command(
        "pkill -9 -f '_tmp_base_bus56_lab.py' || true; "
        "pkill -9 -f 'yam_continuous_all.py' || true; "
        "sleep 0.4; "
        f"pgrep -f 'python3 -u pdb_uart_sim' >/dev/null || "
        f"(cd {REMOTE} && nohup python3 -u pdb_uart_sim.py "
        f"--port /dev/ttyTHS1 --hz 20 --force-kill-state 0 --estop-sense 1 "
        f"--wander --pack-v 4800 4800 4800 4800 --rail-v 4800 1900 1200 500 "
        f"--pack-i 180 140 160 120 --rail-i 90 70 40 25 "
        f">/tmp/pdb_uart_sim.log 2>&1 </dev/null &)"
    )
    o.read()
    time.sleep(0.5)

    sftp = c.open_sftp()
    sftp.put(str(LOCAL / "_tmp_base_bus56_lab.py"), f"{REMOTE}/_tmp_base_bus56_lab.py")
    sftp.close()

    _, stdout, stderr = c.exec_command(
        f"cd {REMOTE} && python3 -u _tmp_base_bus56_lab.py --prove-360 --rate 0.5 "
        f">/tmp/prove360.log 2>&1; echo EXIT:$?; tail -n 200 /tmp/prove360.log",
        timeout=600,
    )
    text = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    OUT.write_text(text + ("\nSTDERR\n" + err if err.strip() else ""), encoding="utf-8")
    print(text)
    if err.strip():
        print("STDERR", err[-2000:])
    c.close()
    print(f"\n(full log: {OUT})", flush=True)
    return 0 if "OVERALL: PASS" in text else 1


if __name__ == "__main__":
    raise SystemExit(main())
