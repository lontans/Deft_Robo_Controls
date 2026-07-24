#!/usr/bin/env python3
"""Upload + sudo-run jetson-io GPIO+uarta config on the Jetson."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "")
LOCAL = Path(__file__).resolve().parent / "_tmp_jetson_io_gpio_config.py"
REMOTE = "/home/deft-robotics/controls_pcb/scripts/_tmp_jetson_io_gpio_config.py"


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    text = LOCAL.read_text(encoding="utf-8").replace("\r\n", "\n")

    c = None
    for attempt in range(1, 30):
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(
                HOST,
                username=USER,
                password=PW,
                timeout=12,
                banner_timeout=12,
                auth_timeout=12,
            )
            print(f"SSH ok attempt {attempt}")
            break
        except Exception as ex:
            print(f"attempt {attempt}: {type(ex).__name__}: {ex}")
            c = None
            time.sleep(3)
    if c is None:
        return 3

    try:
        sftp = c.open_sftp()
        with sftp.file(REMOTE, "w") as f:
            f.write(text)
        sftp.chmod(REMOTE, 0o755)
        sftp.close()

        cmd = f'echo {PW} | sudo -S -p "" python3 {REMOTE}'
        print(">>>", cmd)
        chan = c.get_transport().open_session()
        chan.get_pty()
        chan.exec_command(cmd)
        t0 = time.time()
        while True:
            if chan.recv_ready():
                sys.stdout.write(chan.recv(8192).decode("utf-8", "replace"))
                sys.stdout.flush()
            if chan.exit_status_ready() and not chan.recv_ready():
                break
            if time.time() - t0 > 90:
                print("TIMEOUT")
                break
            time.sleep(0.05)
        while chan.recv_ready():
            sys.stdout.write(chan.recv(8192).decode("utf-8", "replace"))
        code = chan.recv_exit_status()
        print("exit", code)

        # verify overlay strings + ACM
        chan = c.get_transport().open_session()
        chan.get_pty()
        chan.exec_command(
            f'echo {PW} | sudo -S -p "" strings /boot/jetson-io-hdr40-user-custom.dtbo '
            "| grep -E 'hdr40-pin8|hdr40-pin10|uarta|hdr40-pin18' ; "
            "ls -l /dev/ttyACM* 2>/dev/null || echo no_ACM; "
            "grep -E 'DEFAULT|OVERLAYS' /boot/extlinux/extlinux.conf"
        )
        time.sleep(2)
        while chan.recv_ready():
            sys.stdout.write(chan.recv(8192).decode("utf-8", "replace"))
        return code
    finally:
        c.close()


if __name__ == "__main__":
    raise SystemExit(main())
