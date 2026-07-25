#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys

import paramiko

PW = os.environ.get("JETSON_PASS", "4565")


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)
    _, o, _ = c.exec_command(
        "pgrep -af pdb_uart_sim || echo no_pdb; "
        "ls -lt /home/deft-robotics/controls_pcb/scripts/.deft_session/recordings/ | head -5"
    )
    print(o.read().decode("utf-8", "replace"))
    _, o, _ = c.exec_command(
        "python3 - <<'PY'\n"
        "import json, pathlib\n"
        "recs=sorted(pathlib.Path('/home/deft-robotics/controls_pcb/scripts/.deft_session/recordings').glob('record_*.ndjson'), key=lambda p: p.stat().st_mtime, reverse=True)\n"
        "print('n_recs', len(recs))\n"
        "if not recs: raise SystemExit(0)\n"
        "p=recs[0]\n"
        "print('path', p, 'bytes', p.stat().st_size)\n"
        "mcus={}; blocks={}; n=0\n"
        "for line in open(p, encoding='utf-8', errors='replace'):\n"
        "  n+=1\n"
        "  try: d=json.loads(line)\n"
        "  except Exception: continue\n"
        "  m=d.get('mcu_state'); b=d.get('plant_block')\n"
        "  if m is not None: mcus[m]=mcus.get(m,0)+1\n"
        "  if b is not None: blocks[b]=blocks.get(b,0)+1\n"
        "print('lines', n, 'mcu_hist', mcus, 'plant_block_hist', blocks)\n"
        "for ln in open(p, encoding='utf-8', errors='replace').readlines()[-2:]:\n"
        "  d=json.loads(ln)\n"
        "  print({k:d.get(k) for k in ('t_mono','mcu_state','plant_block','seq','fb_hz')})\n"
        "PY"
    )
    print(o.read().decode("utf-8", "replace"))
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
