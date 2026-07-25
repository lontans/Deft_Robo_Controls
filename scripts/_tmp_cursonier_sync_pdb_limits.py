"""Sync host pdb.limits + hub/dashboard so Jetson imports match local."""
from __future__ import annotations

import os
import time

import paramiko

HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FILES = [
    "scripts/deft_controls_sdk/pdb/limits.py",
    "scripts/deft_controls_sdk/pdb/__init__.py",
    "scripts/deft_controls_sdk/controls_pcb_hub.py",
    "scripts/deft_controls_sdk/debug_dashboard/app.py",
    "scripts/deft_controls_sdk/debug_dashboard/__main__.py",
]


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    for rel in FILES:
        local = os.path.join(ROOT, *rel.split("/"))
        remote = f"/home/deft-robotics/controls_pcb/{rel}"
        print("SFTP", rel)
        sftp.put(local, remote)
    sftp.close()

    transport = c.get_transport()
    assert transport is not None
    chan = transport.open_session()
    chan.exec_command("pkill -f pdb_uart_sim.py || true")
    time.sleep(0.4)
    chan.close()

    # Prove import
    _, o, e = c.exec_command(
        "cd ~/controls_pcb/scripts && PYTHONPATH=. python3 -c "
        "'from deft_controls_sdk import ControlsPcbHub; from deft_controls_sdk.pdb import limits; print(\"IMPORT_OK\", limits.PACK_V_MIN_COUNTS)'",
        timeout=20,
    )
    print(o.read().decode(errors="replace"))
    print(e.read().decode(errors="replace"))

    start = (
        "cd /home/deft-robotics/controls_pcb/scripts && "
        "nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        "--force-kill-state 0 --estop-sense 1 "
        "--pack-v 4800 4800 0 0 --rail-v 4800 1900 1200 500 "
        "--pack-i 180 140 0 0 --rail-i 90 70 40 25 --contactor-state 15 "
        "--control-port 8767 >/tmp/pdb_uart_sim.log 2>&1 </dev/null & echo STARTED"
    )
    chan = transport.open_session()
    chan.settimeout(10)
    chan.exec_command(start)
    print(chan.recv(256).decode(errors="replace"))
    chan.close()
    time.sleep(1.0)
    _, o, _ = c.exec_command(
        "ps -ef | grep pdb_uart_sim | grep -v grep; ss -ltn | grep 8767 || true; "
        "cd ~/controls_pcb && PYTHONPATH=scripts python3 scripts/soft_dfu_flash.py scan",
        timeout=25,
    )
    print(o.read().decode(errors="replace"))
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
