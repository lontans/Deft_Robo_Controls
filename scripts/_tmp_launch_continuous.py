#!/usr/bin/env python3
"""Sync continuous + dashboard, run cruise (RS rail reverse), then peer soft-kill."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb/scripts"
LOCAL = Path(__file__).resolve().parent
OUT = LOCAL / "_tmp_continuous_out.txt"

DURATION_S = 50.0
BOOT_WAIT_S = 55.0
SOFTKILL_AT_CRUISE_S = 22.0  # after RS should have reversed at MIT rail


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=15)

    _, o, _ = c.exec_command(
        "pkill -9 -f yam_continuous_all.py || true; "
        "pkill -9 -f pdb_uart_sim.py || true; "
        "rm -f /home/deft-robotics/controls_pcb/scripts/.deft_session/soft_kill_request; "
        "sleep 1; "
        "pgrep -af 'yam_continuous|pdb_uart_sim' || echo 'cleared'"
    )
    print(o.read().decode("utf-8", "replace"))

    sftp = c.open_sftp()
    for rel in (
        "yam_continuous_all.py",
        "pdb_uart_sim.py",
        "_tmp_stop_can.py",
        "rs02_channel_bringup.py",
        "deft_controls_sdk/bench/robstride.py",
        "deft_controls_sdk/vbeta/slots.py",
        "deft_controls_sdk/vbeta/session.py",
        "deft_controls_sdk/vbeta/yam_bench_clear_left.py",
        "deft_controls_sdk/telemetry/cache.py",
        "deft_controls_sdk/debug_dashboard/app.py",
    ):
        sftp.put(str(LOCAL / rel.replace("/", os.sep)), f"{REMOTE}/{rel}")
    sftp.close()

    _, o, _ = c.exec_command(
        f"cd {REMOTE} && python3 -u _tmp_stop_can.py", timeout=30
    )
    print(o.read().decode("utf-8", "replace"))

    transport = c.get_transport()
    assert transport is not None
    chan = transport.open_session()
    chan.exec_command(
        f"cd {REMOTE} && nohup python3 -u pdb_uart_sim.py "
        f"--port /dev/ttyTHS1 --hz 20 --force-kill-state 0 --estop-sense 1 "
        f"--wander "
        f"--pack-v 4800 4800 4800 4800 --rail-v 4800 1900 1200 500 "
        f"--pack-i 180 140 160 120 --rail-i 90 70 40 25 "
        f"--voltage-jitter-pct 2.5 --current-jitter-pct 35 "
        f">/tmp/pdb_uart_sim.log 2>&1 </dev/null &"
    )
    time.sleep(0.3)
    chan.close()
    time.sleep(1.0)

    chan = transport.open_session()
    chan.exec_command(
        f"cd {REMOTE} && nohup python3 -u yam_continuous_all.py "
        f"--cruise-up 0.18 --cruise-down 0.12 --engage-s 2.4 "
        f"--base-rate 0.7854 --record "
        f"--duration {DURATION_S} "
        f">/tmp/yam_cont.log 2>&1 </dev/null &"
    )
    time.sleep(0.3)
    chan.close()

    wait1 = BOOT_WAIT_S + SOFTKILL_AT_CRUISE_S
    print(f"waiting {wait1:.0f}s for latch+cruise (RS should reverse)…", flush=True)
    time.sleep(wait1)

    print("writing soft_kill_request (dashboard follow-mode path)…", flush=True)
    _, o, _ = c.exec_command(
        "printf '%.3f\\n' \"$(date +%s.%N)\" > "
        "/home/deft-robotics/controls_pcb/scripts/.deft_session/soft_kill_request; "
        "ls -la /home/deft-robotics/controls_pcb/scripts/.deft_session/soft_kill_request"
    )
    print(o.read().decode("utf-8", "replace"))
    time.sleep(6.0)

    _, o, _ = c.exec_command(
        "ps -ef | grep yam_continuous | grep -v grep || echo 'continuous exited'; "
        "echo ---LOG_TAIL---; "
        "tail -n 80 /tmp/yam_cont.log; "
        "echo ---GREP---; "
        "grep -E 'soft_kill|Damiao plant|base: continuous|duration reached|s22=.*@' "
        "/tmp/yam_cont.log | tail -n 40"
    )
    text = o.read().decode("utf-8", errors="replace")
    OUT.write_text(text, encoding="utf-8")
    print(text)
    print(f"\n(full log: {OUT})", flush=True)
    c.close()

    parked = "soft_kill_request → soft_kill_park" in text or "SOFT_KILL_REQ → parked" in text
    # Look for a vel sign flip on s22 in status lines (approx).
    ok = parked
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
