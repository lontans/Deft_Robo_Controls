"""Move Jetson pdb_uart_sim control panel off 8765 (dashboard default clash)."""
from __future__ import annotations

import os
import time

import paramiko

HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)

    # Sync dashboard fixes (Claudius/Claudacious trees untouched).
    sftp = c.open_sftp()
    for rel in [
        "scripts/deft_controls_sdk/debug_dashboard/app.py",
        "scripts/deft_controls_sdk/debug_dashboard/__main__.py",
        "scripts/deft_controls_sdk/controls_pcb_hub.py",
    ]:
        local = os.path.join(ROOT, *rel.split("/"))
        remote = f"/home/deft-robotics/controls_pcb/{rel}"
        print("SFTP", rel)
        sftp.put(local, remote)
    sftp.close()

    transport = c.get_transport()
    assert transport is not None
    for cmd in (
        "pkill -f pdb_uart_sim.py || true",
        "pkill -f 'python -m deft_controls_sdk.debug_dashboard' || true",
    ):
        chan = transport.open_session()
        chan.exec_command(cmd)
        time.sleep(0.3)
        chan.close()

    time.sleep(0.5)
    start = (
        "cd /home/deft-robotics/controls_pcb/scripts && "
        "nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        "--force-kill-state 0 --estop-sense 1 "
        "--pack-v 4800 4800 0 0 --rail-v 4800 1900 1200 500 "
        "--pack-i 180 140 0 0 --rail-i 90 70 40 25 --contactor-state 15 "
        "--control-port 8767 "
        ">/tmp/pdb_uart_sim.log 2>&1 </dev/null & echo STARTED"
    )
    chan = transport.open_session()
    chan.settimeout(10)
    chan.exec_command(start)
    out = chan.recv(4096).decode(errors="replace")
    print(out)
    chan.close()
    time.sleep(1.0)

    _, stdout, _ = c.exec_command(
        "ps -ef | grep pdb_uart_sim | grep -v grep; "
        "ss -ltn | grep -E ':8765|:8766|:8767' || true",
        timeout=15,
    )
    print(stdout.read().decode(errors="replace"))
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
