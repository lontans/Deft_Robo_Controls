#!/usr/bin/env python3
"""Start dashboard on Jetson (no COM), curl /api/state follow path, stop."""
from __future__ import annotations

import os
import time

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb/scripts"

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PASS, timeout=10)

sftp = c.open_sftp()
for rel in (
    "deft_controls_sdk/telemetry/cache.py",
    "deft_controls_sdk/debug_dashboard/app.py",
    "deft_controls_sdk/debug_dashboard/__main__.py",
):
    sftp.put(
        os.path.join(os.path.dirname(__file__), rel.replace("/", os.sep)),
        f"{REMOTE}/{rel}",
    )
sftp.close()

_, o, _ = c.exec_command("pkill -f 'debug_dashboard' || true; sleep 0.5; true")
o.channel.recv_exit_status()

chan = c.get_transport().open_session()
chan.exec_command(
    f"cd {REMOTE} && nohup python3 -m deft_controls_sdk.debug_dashboard "
    f"--http-port 8765 --no-browser >/tmp/dash_follow.log 2>&1 </dev/null &"
)
time.sleep(0.3)
chan.close()
time.sleep(2.5)

_, o, e = c.exec_command(
    "python3 - <<'PY'\n"
    "import json, urllib.request\n"
    "raw=urllib.request.urlopen('http://127.0.0.1:8765/api/state', timeout=3).read()\n"
    "d=json.loads(raw)\n"
    "print('following', d.get('following_state_file'))\n"
    "print('connected', d.get('connected'), 'peer', d.get('peer_connected'))\n"
    "print('summary', d.get('summary'))\n"
    "print('grade', d.get('grade'), 'fb_hz', d.get('fb_hz'))\n"
    "print('state_path', d.get('state_path'))\n"
    "acts=d.get('actuators') or []\n"
    "print('n_acts', len(acts))\n"
    "if len(acts)>1: print('j2', acts[1].get('position'), 'fault', acts[1].get('fault'))\n"
    "PY\n"
    "echo ---DASHLOG---; tail -n 15 /tmp/dash_follow.log"
)
print(o.read().decode("utf-8", "replace").encode("ascii", "replace").decode())
err = e.read().decode("utf-8", "replace")
if err:
    print("ERR", err.encode("ascii", "replace").decode()[:1000])

_, o, _ = c.exec_command("pkill -f 'debug_dashboard' || true")
o.channel.recv_exit_status()
c.close()
