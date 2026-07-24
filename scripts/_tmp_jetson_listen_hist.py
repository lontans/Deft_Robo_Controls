#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")
HOST = "192.168.50.48"
REMOTE_PY = "/tmp/uart_hist_listen.py"
LOCAL_LISTENER = """
import serial
import time

for port in ("/dev/ttyTHS1", "/dev/ttyTHS2"):
    try:
        s = serial.Serial(port, 115200, timeout=0.05)
    except Exception as e:
        print(port, "OPEN_FAIL", e)
        continue
    s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        b = s.read(4096)
        if b:
            buf.extend(b)
    s.close()
    hist = {}
    for x in buf:
        hist[x] = hist.get(x, 0) + 1
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:12]
    asc = "".join(chr(b) if 32 <= b < 127 else "." for b in buf[:96])
    print(port, "n", len(buf), "nz", sum(1 for x in buf if x))
    print("  top", top)
    print("  asc", repr(asc))
    print(
        "  probe",
        b"UART4_PROBE" in buf,
        "x55_count",
        sum(1 for x in buf if x == 0x55),
    )
"""


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    with sftp.file(REMOTE_PY, "w") as f:
        f.write(LOCAL_LISTENER)
    sftp.close()

    t = c.get_transport()
    assert t is not None
    ch = t.open_session()
    ch.set_combine_stderr(True)
    ch.exec_command("python3 " + REMOTE_PY)
    buf = b""
    t0 = time.time()
    while True:
        if ch.recv_ready():
            buf += ch.recv(8192)
        if ch.exit_status_ready() and not ch.recv_ready():
            break
        if time.time() - t0 > 25:
            print("TIMEOUT", file=sys.stderr)
            break
        time.sleep(0.05)
    sys.stdout.write(buf.decode("utf-8", "replace"))
    code = ch.recv_exit_status() if ch.exit_status_ready() else -1
    print("exit", code)
    c.close()
    return 0 if code == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
