#!/usr/bin/env python3
import os
import sys
import time

import paramiko

PW = os.environ.get("JETSON_PASS", "")
PORT = os.environ.get("JETSON_UART", "/dev/ttyTHS1")


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("192.168.50.48", username="deft-robotics", password=PW, timeout=15)

    # Use transport channel so background process isn't waited on.
    transport = c.get_transport()
    assert transport is not None

    def run(cmd: str, timeout: float = 20.0) -> str:
        print(">>>", cmd)
        chan = transport.open_session()
        chan.settimeout(timeout)
        chan.exec_command(cmd)
        out = b""
        while True:
            if chan.recv_ready():
                out += chan.recv(4096)
            if chan.recv_stderr_ready():
                out += chan.recv_stderr(4096)
            if chan.exit_status_ready() and not chan.recv_ready():
                break
            time.sleep(0.05)
        try:
            code = chan.recv_exit_status()
        except Exception:
            code = -1
        text = out.decode("utf-8", "replace")
        print(text)
        print("exit", code)
        return text

    run("pkill -f pdb_uart_sim.py || true; sleep 0.4; fuser -v /dev/ttyTHS1 /dev/ttyTHS2 2>&1 || true")
    # Start detached via setsid; do not wait on python.
    start_cmd = (
        "cd /home/deft-robotics/controls_pcb/scripts && "
        f"rm -f /tmp/pdb_uart_sim.log && "
        f"setsid nohup python3 -u pdb_uart_sim.py --port {PORT} --hz 20 "
        f"--gpio-estop 16 --seed 1 </dev/null >/tmp/pdb_uart_sim.log 2>&1 & "
        "echo PID=$!"
    )
    run(start_cmd, timeout=5.0)
    time.sleep(2.0)
    run("ps aux | grep -v grep | grep pdb_uart_sim || echo NO_PROC")
    run("tail -n 25 /tmp/pdb_uart_sim.log || echo NO_LOG")
    # Peek RX on the *other* port while sim holds THS1 — and sniff THS1 if free.
    # Also dump first bytes Controls might be sending if we briefly stop sim.
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
