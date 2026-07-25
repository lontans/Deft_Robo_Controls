#!/usr/bin/env python3
"""Deploy arm/enable fix + run --fix-74 (cali + 360) on Jetson."""
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
OUT = LOCAL / "_tmp_fix74_out.txt"


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)

    _, o, _ = c.exec_command(
        "pkill -9 -f '_tmp_base_bus56_lab.py' || true; "
        "pkill -9 -f 'yam_continuous_all.py' || true; "
        "sleep 0.4"
    )
    o.read()
    time.sleep(0.4)

    sftp = c.open_sftp()
    sftp.put(str(LOCAL / "_tmp_base_bus56_lab.py"), f"{REMOTE}/_tmp_base_bus56_lab.py")
    sftp.put(
        str(LOCAL / "deft_controls_sdk" / "bench" / "robstride.py"),
        f"{REMOTE}/deft_controls_sdk/bench/robstride.py",
    )
    sftp.close()

    # cali ~30s + arm + 360 @ 0.5 rad/s ~13s
    _, stdout, stderr = c.exec_command(
        f"cd {REMOTE} && python3 -u _tmp_base_bus56_lab.py --fix-74 --rate 0.5 "
        f">/tmp/fix74.log 2>&1; echo EXIT:$?; tail -n 120 /tmp/fix74.log",
        timeout=300,
    )
    text = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    OUT.write_text(text + (("\nSTDERR\n" + err) if err.strip() else ""), encoding="utf-8")
    print(text)
    if err.strip():
        print("STDERR", err[-2000:])
    c.close()
    print(f"\n(full log: {OUT})", flush=True)
    return 0 if "FIX 0x74 RESULT: PASS" in text else 1


if __name__ == "__main__":
    raise SystemExit(main())
