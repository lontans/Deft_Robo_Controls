#!/usr/bin/env python3
"""Hunt for UART4_PROBE across bauds on ttyTHS1 (MCU streaming 50 Hz)."""
from __future__ import annotations

import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")
REMOTE = "/tmp/uart_baud_hunt.py"
BODY = r"""
import serial, time
bauds = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
port = "/dev/ttyTHS1"
for baud in bauds:
    try:
        s = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(baud, "OPEN_FAIL", e)
        continue
    s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 1.5:
        b = s.read(4096)
        if b:
            buf.extend(b)
    s.close()
    asc = "".join(chr(c) if 32 <= c < 127 else "." for c in buf[:80])
    print(
        f"baud={baud:7d} n={len(buf):6d} nz={sum(1 for x in buf if x):6d} "
        f"probe={b'UART4_PROBE' in buf} x55={sum(1 for x in buf if x==0x55)} "
        f"asc={asc!r}"
    )
"""


def main() -> int:
    if not PW:
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    with sftp.file(REMOTE, "w") as f:
        f.write(BODY)
    sftp.close()
    t = c.get_transport()
    assert t is not None
    ch = t.open_session()
    ch.set_combine_stderr(True)
    ch.exec_command("python3 " + REMOTE)
    buf = b""
    t0 = time.time()
    while True:
        if ch.recv_ready():
            buf += ch.recv(8192)
        if ch.exit_status_ready() and not ch.recv_ready():
            break
        if time.time() - t0 > 40:
            break
        time.sleep(0.05)
    sys.stdout.write(buf.decode("utf-8", "replace"))
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
