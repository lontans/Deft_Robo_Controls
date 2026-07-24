#!/usr/bin/env python3
"""Diagnose Jetson UART1 TX/RX + ESTOP GPIO08 pin16 mapping."""
from __future__ import annotations

import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")


def run(c: paramiko.SSHClient, cmd: str, timeout: float = 40.0) -> str:
    print(">>>", cmd[:120] + ("…" if len(cmd) > 120 else ""))
    t = c.get_transport()
    assert t is not None
    ch = t.open_session()
    ch.settimeout(timeout)
    ch.exec_command(cmd)
    out = b""
    while True:
        if ch.recv_ready():
            out += ch.recv(8192)
        if ch.recv_stderr_ready():
            out += ch.recv_stderr(8192)
        if ch.exit_status_ready() and not ch.recv_ready():
            break
        time.sleep(0.05)
    try:
        code = ch.recv_exit_status()
    except Exception:
        code = -1
    text = out.decode("utf-8", "replace")
    print(text, end="" if text.endswith("\n") else "\n")
    print("exit", code)
    return text


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)

    run(c, "pkill -f pdb_uart_sim.py || true; sleep 0.3")

    # Pinmux / jetson-io state for 40-pin UART
    run(
        c,
        r"""python3 - <<'PY'
import os, glob
print('JETSON_TYPE from gpio:')
try:
    import Jetson.GPIO as G
    print(G.JETSON_INFO)
except Exception as e:
    print('gpio err', e)
print('--- jetson-io overlays ---')
for p in ('/boot/extlinux/extlinux.conf',):
    if os.path.exists(p):
        for line in open(p):
            if 'OVERLAY' in line or 'JetsonIO' in line or 'FDT' in line or 'LABEL' in line:
                print(line.rstrip())
print('--- hdr40 pin8/10 in live DT (if present) ---')
for root, dirs, files in os.walk('/proc/device-tree'):
    for name in files:
        if 'hdr40' in root or 'pin8' in root or 'uart1' in name.lower():
            path = os.path.join(root, name)
            if any(k in path for k in ('pin8','pin10','uart1_tx','uart1_rx','hdr40-pin8','hdr40-pin10')):
                try:
                    raw = open(path,'rb').read().replace(b'\0', b' ')
                    print(path, raw[:80])
                except Exception as e:
                    print(path, e)
PY""",
    )

    # Software TX activity test on THS1 (no loopback wire needed for "does open work")
    # Hardware loopback: user would need pins 8-10 shorted. We measure GPIO levels via sysfs if available.
    run(
        c,
        r"""python3 - <<'PY'
import serial, time, statistics
# TX burn test: open THS1, write patterned bytes; also sample pin16 ESTOP as input
import Jetson.GPIO as G
G.setmode(G.BOARD)
G.setup(16, G.IN)
print('pin16 ESTOP sense sample before UART:', [G.input(16) for _ in range(5)])

for port in ('/dev/ttyTHS1','/dev/ttyTHS2'):
    try:
        s = serial.Serial(port, 115200, timeout=0.2, write_timeout=1.0)
    except Exception as e:
        print(port, 'open ERR', e)
        continue
    payload = b'PDBC' + bytes(range(60))
    t0 = time.time()
    n = 0
    while time.time() - t0 < 1.0:
        s.write(payload)
        n += 1
    s.flush()
    # brief RX while writing (only sees data if loopback or peer)
    time.sleep(0.05)
    rx = s.read(256)
    s.close()
    print(port, 'tx_frames', n, 'rx_while', len(rx), 'rx_head', rx[:16].hex() if rx else None)
    print('  pin16 during/after', [G.input(16) for _ in range(5)])
G.cleanup()
print('NOTE: without pin8-pin10 short, rx_while should be 0; TX pad should idle ~3.3V on pin8')
PY""",
    )

    # Read pin16 for a few seconds while we ask PC side to toggle later — just baseline here
    run(
        c,
        r"""python3 - <<'PY'
import Jetson.GPIO as G, time
G.setmode(G.BOARD)
G.setup(16, G.IN)
print('pin16 BOARD=16 (GPIO08) levels over 2s:')
for i in range(20):
    print(i, G.input(16))
    time.sleep(0.1)
G.cleanup()
PY""",
    )

    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
