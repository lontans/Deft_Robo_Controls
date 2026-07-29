"""Sync Mission Impossible to Jetson and run ``all`` (or a single mission)."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb"
LOCAL = Path(__file__).resolve().parents[1]
MISSION = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "all --continue-on-fail"

SYNC = [
    "scripts/mission_impossible.py",
    "scripts/bench_load_matrix.py",
    "scripts/yam_continuous_all.py",
    "scripts/pdb_uart_sim.py",
    "scripts/stop_can.py",
    "scripts/soft_dfu_flash.py",
    "scripts/rs02_channel_bringup.py",
    "docs/legacy/bench/mission_impossible_findings.md",
    "scripts/deft_controls_sdk/bench/metrics.py",
    "scripts/deft_controls_sdk/bench/rs02_motion.py",
    "scripts/deft_controls_sdk/bench/servo_fb.py",
    "scripts/deft_controls_sdk/bench/soft_dfu.py",
    "scripts/deft_controls_sdk/vbeta/cfg.py",
    "scripts/deft_controls_sdk/vbeta/slots.py",
    "scripts/deft_controls_sdk/vbeta/session.py",
    "scripts/deft_controls_sdk/vbeta/yam_bench_clear_left.py",
    "scripts/deft_controls_sdk/pdb/limits.py",
]


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=20)

    # Free CDC owners except dashboard
    _, o, _ = c.exec_command(
        "pkill -9 -f yam_continuous_all.py || true; "
        "pkill -9 -f mission_impossible.py || true; "
        "pkill -9 -f pdb_uart_sim.py || true; sleep 1; "
        "fuser /dev/ttyACM0 2>/dev/null || echo CDC_FREE"
    )
    print(o.read().decode(errors="replace"), flush=True)

    sftp = c.open_sftp()
    for rel in SYNC:
        local = LOCAL / rel.replace("/", os.sep)
        remote = f"{REMOTE}/{rel}"
        if not local.is_file():
            print("skip missing", rel, flush=True)
            continue
        # ensure remote dir
        rdir = "/".join(remote.split("/")[:-1])
        try:
            sftp.stat(rdir)
        except OSError:
            c.exec_command(f"mkdir -p {rdir}")
            time.sleep(0.1)
        sftp.put(str(local), remote)
        print("synced", rel, flush=True)
    sftp.close()

    cmd = (
        f"cd {REMOTE}/scripts && PYTHONPATH=. python3 -u mission_impossible.py "
        f"{MISSION} --port /dev/ttyACM0 2>&1"
    )
    print(">>>", cmd, flush=True)
    chan = c.get_transport().open_session()
    chan.settimeout(900)
    chan.exec_command(cmd)
    chunks: list[str] = []
    while True:
        if chan.recv_ready():
            data = chan.recv(8192).decode(errors="replace")
            chunks.append(data)
            sys.stdout.write(data)
            sys.stdout.flush()
        if chan.exit_status_ready() and not chan.recv_ready():
            break
        time.sleep(0.15)
    while chan.recv_ready():
        data = chan.recv(8192).decode(errors="replace")
        chunks.append(data)
        sys.stdout.write(data)
    rc = chan.recv_exit_status()

    # Pull findings back
    sftp = c.open_sftp()
    try:
        sftp.get(
            f"{REMOTE}/docs/legacy/bench/mission_impossible_findings.md",
            str(LOCAL / "docs" / "mission_impossible_findings.md"),
        )
        print("\npulled findings.md", flush=True)
    except Exception as exc:
        print("findings pull failed", exc, flush=True)
    sftp.close()

    _, o2, _ = c.exec_command(
        "fuser /dev/ttyACM0 2>/dev/null || echo CDC_FREE; "
        "pgrep -af 'pdb_uart_sim|yam_continuous|mission_impossible' || echo NO_OWNERS"
    )
    print("POST", o2.read().decode(errors="replace"), flush=True)
    c.close()
    print(f"EXIT={rc}", flush=True)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
