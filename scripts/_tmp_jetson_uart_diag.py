#!/usr/bin/env python3
import os
import sys

import paramiko

PW = os.environ.get("JETSON_PASS", "")
CMDS = [
    "fuser -v /dev/ttyTHS1 /dev/ttyTHS2 2>&1 || true",
    "ps aux | grep -iE 'ttyTHS|getty|nvgetty|pdb_uart' | grep -v grep || true",
    "dmesg | grep -iE 'serial@3100000|ttyTHS1|3100000' | tail -n 20 || true",
    # Raw: does RX pin toggle when we don't open UART? use busybox or python gpio - skip
    # Compare: open THS1 with exclusive, count bytes vs open after stty -brkint
    """python3 - <<'PY'
import serial, time, termios, fcntl
port='/dev/ttyTHS1'
# try to clear break
s=serial.Serial(port, 115200, timeout=0.1)
print('opened', port, 'is_open', s.is_open)
# dump linux counters if any
t0=time.time(); n=0; nz=0; ones=0; hist={}
while time.time()-t0<1.0:
    b=s.read(4096)
    n+=len(b)
    for x in b:
        nz += 1 if x else 0
        ones += bin(x).count('1')
        hist[x]=hist.get(x,0)+1
s.close()
top=sorted(hist.items(), key=lambda kv: -kv[1])[:8]
print(f'1s n={n} nz={nz} ones={ones} top={top}')
print('probe_in_buf', any(k==ord(c) for k in hist for c in 'UART4'))
PY""",
]


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)
    for cmd in CMDS:
        print(">>>", cmd[:100].replace("\n", " "))
        _, o, e = c.exec_command(cmd, timeout=30)
        sys.stdout.write(o.read().decode("utf-8", "replace"))
        err = e.read().decode("utf-8", "replace")
        if err:
            sys.stdout.write(err)
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
