#!/usr/bin/env python3
"""Sniff Jetson UART for Controls PDBC; restart pdb_uart_sim on THS1."""
from __future__ import annotations

import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)

    def run(cmd: str, timeout: float = 30.0) -> str:
        print(">>>", cmd[:120])
        _, o, e = c.exec_command(cmd, timeout=timeout)
        text = o.read().decode("utf-8", "replace") + e.read().decode("utf-8", "replace")
        print(text, end="" if text.endswith("\n") else "\n")
        return text

    run("python3 /home/deft-robotics/controls_pcb/scripts/jetson_estop_sense.py --once")
    run("pkill -f pdb_uart_sim.py || true; sleep 0.4")
    run(
        r"""python3 - <<'PY'
import serial, time
for port in ("/dev/ttyTHS1", "/dev/ttyTHS2"):
    try:
        s = serial.Serial(port, 115200, timeout=0.2)
    except Exception as ex:
        print(port, "open_fail", ex)
        continue
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 1.5:
        b = s.read(256)
        if b:
            buf.extend(b)
    s.close()
    print(port, "n", len(buf), "nz", sum(1 for x in buf if x),
          "pdbc", buf.find(b"PDBC"), "head", bytes(buf[:16]).hex() if buf else None)
PY""",
        timeout=20.0,
    )
    run(
        "cd /home/deft-robotics/controls_pcb/scripts && "
        "setsid nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        "--gpio-estop 16 --seed 1 </dev/null >/tmp/pdb_uart_sim.log 2>&1 & "
        "sleep 1.2; tail -n 8 /tmp/pdb_uart_sim.log"
    )
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
