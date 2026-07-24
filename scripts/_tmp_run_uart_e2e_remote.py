#!/usr/bin/env python3
"""Upload UART e2e prove script to Jetson and run it."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "4565")
REPO = "/home/deft-robotics/controls_pcb"
LOCAL = Path(__file__).resolve().parent


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=20)
    sftp = c.open_sftp()
    for name in ("_tmp_uart_e2e_prove.py", "pdb_uart_sim.py"):
        local = LOCAL / name
        remote = f"{REPO}/scripts/{name}"
        print(f"upload {local.name} -> {remote}")
        sftp.put(str(local), remote)
    listen = LOCAL / "jetson_uart_listen.py"
    if listen.exists():
        sftp.put(str(listen), f"{REPO}/scripts/{listen.name}")
    sftp.close()

    cmd = (
        f"bash -lc 'cd {REPO}/scripts && "
        f"JETSON_PASS={PW!r} PYTHONPATH={REPO}/scripts "
        f"python3 -u _tmp_uart_e2e_prove.py'"
    )
    print(">>>", cmd)
    transport = c.get_transport()
    assert transport is not None
    chan = transport.open_session()
    chan.settimeout(180)
    chan.exec_command(cmd)
    while True:
        if chan.recv_ready():
            sys.stdout.write(chan.recv(4096).decode("utf-8", "replace"))
            sys.stdout.flush()
        if chan.recv_stderr_ready():
            sys.stderr.write(chan.recv_stderr(4096).decode("utf-8", "replace"))
            sys.stderr.flush()
        if chan.exit_status_ready() and not chan.recv_ready() and not chan.recv_stderr_ready():
            break
    code = chan.recv_exit_status()
    # drain
    while chan.recv_ready():
        sys.stdout.write(chan.recv(4096).decode("utf-8", "replace"))
    while chan.recv_stderr_ready():
        sys.stderr.write(chan.recv_stderr(4096).decode("utf-8", "replace"))
    print(f"\nREMOTE_EXIT={code}")
    c.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
