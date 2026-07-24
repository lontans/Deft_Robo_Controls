#!/usr/bin/env python3
"""One-shot Jetson SSH probe for PDB UART prove (local temp helper)."""
from __future__ import annotations

import os
import sys

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "")


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    def run(cmd: str) -> tuple[int, str, str]:
        print(">>>", cmd)
        _, stdout, stderr = c.exec_command(cmd, timeout=45)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        code = stdout.channel.recv_exit_status()
        if out:
            print(out, end="" if out.endswith("\n") else "\n")
        if err:
            print("STDERR:", err, end="" if err.endswith("\n") else "\n")
        print("exit", code)
        return code, out, err

    run("uname -a; whoami; pwd")
    run("ls -la /dev/ttyTHS* /dev/ttyUSB* 2>/dev/null; ls /dev/serial/by-id 2>/dev/null; ls /dev/serial/by-path 2>/dev/null")
    run(
        "find $HOME -maxdepth 5 -name pdb_uart_sim.py 2>/dev/null | head -20; "
        "ls -la $HOME/DeftRoboticsControlsPCB/scripts/pdb_uart_sim.py 2>/dev/null; "
        "ls -la $HOME/Documents/DeftRoboticsControlsPCB/scripts/pdb_uart_sim.py 2>/dev/null"
    )
    run("python3 -c 'import Jetson.GPIO as G; print(\"Jetson.GPIO\", getattr(G, \"VERSION\", \"?\"))'")
    run("python3 -c 'import serial; print(\"pyserial\", serial.__version__)'")
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
