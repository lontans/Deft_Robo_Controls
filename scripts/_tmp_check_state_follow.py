#!/usr/bin/env python3
"""Verify continuous state.json path + dashboard follow would see FB."""
from __future__ import annotations

import json
import os
import time

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")

CMD = r"""
cd /home/deft-robotics/controls_pcb/scripts && python3 - <<'PY'
import json, time
from pathlib import Path
from deft_controls_sdk.telemetry import default_session_dir

d = default_session_dir()
p = d / "state.json"
print("default_session_dir", d)
print("state_path", p, "exists", p.is_file())
cwd_alt = Path.cwd() / ".deft_session" / "state.json"
print("cwd_alt", cwd_alt, "exists", cwd_alt.is_file())
if p.is_file():
    raw = json.loads(p.read_text())
    age = time.time() - float(raw.get("updated_at") or 0)
    print("updated_age_s", round(age, 2), "fb_hz", raw.get("fb_hz"),
          "connected", raw.get("connected"), "grade", raw.get("grade"))
    acts = raw.get("actuators") or []
    for i in [1, 5, 6, 22, 24]:
        a = acts[i] if i < len(acts) else None
        if a:
            print(f"  slot{i} pos={a.get('position'):+.3f} fault={a.get('fault')}")
# Simulate dashboard follow overlay
if p.is_file():
    payload = json.loads(p.read_text())
    payload["following_state_file"] = True
    payload["connected"] = False
    print("follow_ui_summary_would_show fb_hz", payload.get("fb_hz"),
          "n_acts", len(payload.get("actuators") or []))
PY
ps -ef | grep yam_continuous | grep -v grep | head -2
"""

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PASS, timeout=10)
_, o, e = c.exec_command(CMD, timeout=30)
print(o.read().decode("utf-8", "replace").encode("ascii", "replace").decode())
err = e.read().decode("utf-8", "replace")
if err:
    print("ERR", err.encode("ascii", "replace").decode()[:1500])
c.close()
