#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")
LOCAL = os.path.join(os.path.dirname(__file__), "jetson_estop_sense.py")
REMOTE = "/home/deft-robotics/controls_pcb/scripts/jetson_estop_sense.py"


def run(c: paramiko.SSHClient, cmd: str, timeout: float = 30.0) -> str:
    print(">>>", cmd)
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
    sftp = c.open_sftp()
    sftp.put(LOCAL, REMOTE)
    sftp.chmod(REMOTE, 0o755)
    sftp.close()
    print("uploaded", REMOTE)

    run(c, f"python3 {REMOTE} --once")
    run(c, f"python3 {REMOTE} --seconds 3 --hz 10")

    run(c, "pkill -f pdb_uart_sim.py || true; sleep 0.3")
    start = (
        "cd /home/deft-robotics/controls_pcb/scripts && "
        "rm -f /tmp/pdb_uart_sim.log && "
        "setsid nohup python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 "
        "--gpio-estop 16 --seed 1 </dev/null >/tmp/pdb_uart_sim.log 2>&1 & "
        "echo PID=$!"
    )
    run(c, start, timeout=8.0)
    time.sleep(2.0)
    run(c, "ps aux | grep -v grep | grep pdb_uart_sim || echo NO_PROC")
    run(c, "tail -n 20 /tmp/pdb_uart_sim.log")

    # Sniff for PDBC from Controls while sim is running occupies THS1 —
    # instead watch sim log for PDBC. Also brief: stop sim and sniff.
    run(c, "pkill -f pdb_uart_sim.py || true; sleep 0.4")
    run(
        c,
        """python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.2)
t0 = time.time()
buf = bytearray()
while time.time() - t0 < 2.5:
    b = s.read(256)
    if b:
        buf.extend(b)
s.close()
print('sniff_n', len(buf))
print('head', bytes(buf[:32]).hex() if buf else None)
print('find_PDBC', buf.find(b'PDBC'))
nz = sum(1 for x in buf if x)
print('nonzero', nz)
# restart sim for user
import subprocess
subprocess.Popen(
    ['python3','-u','pdb_uart_sim.py','--port','/dev/ttyTHS1','--hz','20','--gpio-estop','16','--seed','1'],
    cwd='/home/deft-robotics/controls_pcb/scripts',
    stdout=open('/tmp/pdb_uart_sim.log','w'),
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
print('sim_restarted')
time.sleep(1.0)
print(open('/tmp/pdb_uart_sim.log').read()[-500:])
PY""",
        timeout=25.0,
    )
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
