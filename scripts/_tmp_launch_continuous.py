#!/usr/bin/env python3
"""Blank CAN, sync, start J2 CLEAR + bus5/6 continuous (persist telemetry)."""
from __future__ import annotations

import os
import time

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb/scripts"
LOCAL = os.path.dirname(__file__)


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=10)

    _, o, _ = c.exec_command(
        "pkill -9 -f yam_continuous_all.py; pkill -9 -f pdb_uart_sim.py; sleep 1; true"
    )
    o.channel.recv_exit_status()

    sftp = c.open_sftp()
    for rel in (
        "yam_continuous_all.py",
        "_tmp_stop_can.py",
        "deft_controls_sdk/vbeta/slots.py",
        "deft_controls_sdk/vbeta/session.py",
        "deft_controls_sdk/vbeta/yam_bench_clear_left.py",
        "deft_controls_sdk/debug_dashboard/app.py",
        "deft_controls_sdk/debug_dashboard/__main__.py",
        "deft_controls_sdk/telemetry/cache.py",
    ):
        sftp.put(os.path.join(LOCAL, rel.replace("/", os.sep)), f"{REMOTE}/{rel}")
    sftp.close()

    _, o, _ = c.exec_command(
        f"cd {REMOTE} && python3 -u _tmp_stop_can.py", timeout=30
    )
    print(o.read().decode("utf-8", "replace"))

    transport = c.get_transport()
    chan = transport.open_session()
    chan.exec_command(
        f"cd {REMOTE} && nohup python3 -u pdb_uart_sim.py "
        f"--port /dev/ttyTHS1 --hz 20 --force-kill-state 0 --estop-sense 1 "
        f">/tmp/pdb_uart_sim.log 2>&1 </dev/null &"
    )
    time.sleep(0.3)
    chan.close()
    time.sleep(1.0)

    chan = transport.open_session()
    chan.exec_command(
        f"cd {REMOTE} && nohup python3 -u yam_continuous_all.py "
        f"--cruise-up 0.18 --cruise-down 0.12 --engage-s 2.4 "
        f"--base-amp 0.60 --base-rate 0.7854 "
        f">/tmp/yam_cont.log 2>&1 </dev/null &"
    )
    time.sleep(0.3)
    chan.close()
    time.sleep(22)
    _, o, _ = c.exec_command(
        "ps -ef | grep yam_continuous | grep -v grep; echo ---; "
        "tail -n 80 /tmp/yam_cont.log; echo ---STATE---; "
        "python3 - <<'PY'\n"
        "import json, pathlib\n"
        "p=pathlib.Path('/home/deft-robotics/controls_pcb/scripts/.deft_session/state.json')\n"
        "print('exists', p.is_file(), p)\n"
        "if p.is_file():\n"
        "  d=json.loads(p.read_text())\n"
        "  acts=d.get('actuators') or []\n"
        "  for i in [1,22,23,24,25]:\n"
        "    a=acts[i] if i < len(acts) else None\n"
        "    if not a: print(f'  slot{i}: None'); continue\n"
        "    print(f\"  slot{i}: pos={a.get('position')} tau={a.get('torque')} fault={a.get('fault')}\")\n"
        "  print('fb_hz', d.get('fb_hz'), 'plant_block', d.get('plant_block'))\n"
        "PY"
    )
    print(o.read().decode("utf-8", "replace").encode("ascii", "replace").decode())
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
