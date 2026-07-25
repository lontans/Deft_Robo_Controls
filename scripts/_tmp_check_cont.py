#!/usr/bin/env python3
import os

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PASS, timeout=10)
_, o, _ = c.exec_command(
    "ps -ef | grep yam_continuous | grep -v grep | wc -l; echo ---; "
    "tail -n 30 /tmp/yam_cont.log; echo ---STATE---; "
    "python3 - <<'PY'\n"
    "import json, pathlib\n"
    "p=pathlib.Path('/home/deft-robotics/controls_pcb/scripts/.deft_session/state.json')\n"
    "d=json.loads(p.read_text())\n"
    "acts=d.get('actuators') or []\n"
    "for i in range(7):\n"
    "  a=acts[i]\n"
    "  print(f\"arm{i}: pos={a.get('position'):+.4f} tau={a.get('torque'):+.3f} "
    "fault={a.get('fault')}\")\n"
    "for i in [22,23,24,25]:\n"
    "  a=acts[i]\n"
    "  print(f\"slot{i}: pos={a.get('position'):+.4f} tau={a.get('torque'):+.3f} "
    "fault={a.get('fault')}\")\n"
    "sv=d.get('servos') or []\n"
    "for i,s in enumerate(sv[:2]):\n"
    "  print(f\"dxl{i}: {s}\")\n"
    "print('fb_hz', d.get('fb_hz'))\n"
    "PY"
)
print(o.read().decode("utf-8", "replace").encode("ascii", "replace").decode())
c.close()
