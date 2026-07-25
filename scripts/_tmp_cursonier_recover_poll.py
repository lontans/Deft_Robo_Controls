"""Poll Jetson for board return; optional USB authorized bounce."""
from __future__ import annotations

import os
import time

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "4565")


def run(c, cmd, timeout=40.0) -> str:
    print("JETSON>>>", cmd[:180])
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    if out:
        print(out, end="" if out.endswith("\n") else "\n")
    if err.strip():
        print("STDERR:", err[:800])
    return out


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    run(c, f"echo {PW!r} | sudo -S dmesg | tail -60")
    run(c, "lsusb")

    # Soft bounce USB devices that look like STM32 if still present under sysfs.
    bounce = r"""
python3 - <<'PY'
import glob, os, time
paths = []
for uevent in glob.glob('/sys/bus/usb/devices/*/uevent'):
    try:
        txt = open(uevent).read()
    except OSError:
        continue
    if 'PRODUCT=483/' in txt or 'PRODUCT=0483/' in txt:
        paths.append(os.path.dirname(uevent))
print('stm_paths', paths)
for p in paths:
    auth = os.path.join(p, 'authorized')
    if os.path.exists(auth):
        print('bounce', p)
        open(auth, 'w').write('0')
        time.sleep(0.5)
        open(auth, 'w').write('1')
PY
"""
    run(c, f"echo {PW!r} | sudo -S bash -lc {bounce!r}")

    for i in range(10):
        time.sleep(2)
        out = run(c, "cd ~/controls_pcb && python3 scripts/soft_dfu_flash.py scan")
        if "[CDC]" in out or "DFU 0483:DF11: yes" in out or "sn=" in out:
            print("BOARD_VISIBLE")
            c.close()
            return 0
        print(f"poll {i}: still gone")

    print("BOARD_STILL_GONE — need physical USB re-plug or ST-Link recovery")
    c.close()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
