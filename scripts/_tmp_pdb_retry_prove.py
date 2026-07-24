#!/usr/bin/env python3
"""Retry PDB live prove: restart Jetson sim on THS1 then THS2; check COM5 mirror."""
from __future__ import annotations

import os
import struct
import sys
import time

import paramiko

from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.link.exchange.wire_layout import PDB_OFF, SYSTEM_FB_OFF
from deft_controls_sdk.pdb.frame import MAGIC_FB, KILL_STATE_NAMES, parse_feedback as parse_pdbf

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "")
REPO = "/home/deft-robotics/controls_pcb/scripts"


def ssh_run(c: paramiko.SSHClient, cmd: str, timeout: float = 30.0) -> str:
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


def start_sim(c: paramiko.SSHClient, port: str) -> None:
    ssh_run(c, "pkill -f pdb_uart_sim.py || true; sleep 0.4")
    cmd = (
        f"cd {REPO} && rm -f /tmp/pdb_uart_sim.log && "
        f"setsid nohup python3 -u pdb_uart_sim.py --port {port} --hz 20 "
        f"--gpio-estop 16 --seed 1 </dev/null >/tmp/pdb_uart_sim.log 2>&1 & echo PID=$!"
    )
    ssh_run(c, cmd, timeout=8.0)
    time.sleep(2.0)
    ssh_run(c, "ps aux | grep -v grep | grep pdb_uart_sim || echo NO_PROC")
    ssh_run(c, "tail -n 12 /tmp/pdb_uart_sim.log || echo NO_LOG")


def check_com5(label: str) -> dict:
    print(f"\n=== COM5 check ({label}) ===")
    with ControlsPcbHub.connect("COM5", persist_telemetry=False) as hub:
        c = hub._connection
        last = c.read_feedback(timeout_s=3.0).raw
        t0 = time.time()
        while time.time() - t0 < 2.0:
            c.send_once()
            try:
                last = c.read_feedback(timeout_s=0.15).raw
            except Exception:
                pass
            time.sleep(0.03)
    pdb = last[PDB_OFF : PDB_OFF + 64]
    magic = struct.unpack_from("<I", pdb, 0)[0]
    ks, kr, es = struct.unpack_from("<BBB", last, SYSTEM_FB_OFF + 14)
    parsed = parse_pdbf(pdb)
    out = {
        "magic": magic,
        "pdbf": magic == MAGIC_FB,
        "kill": ks,
        "kill_name": KILL_STATE_NAMES.get(ks, "?"),
        "reason": kr,
        "estop": es,
        "parsed": parsed,
        "head": pdb[:8].hex(),
    }
    print(
        f"magic=0x{magic:08X} PDBF={out['pdbf']} head={out['head']} "
        f"sys.kill={ks}({out['kill_name']}) reason={kr} estop={es}"
    )
    if parsed:
        print(
            f"  pack_v={parsed['pack_v']} rail_v={parsed['rail_v']} "
            f"pdb_kill={parsed['kill_state']} pdb_estop={parsed['estop_sense']}"
        )
    return out


def dt_uart_info(c: paramiko.SSHClient) -> None:
    ssh_run(
        c,
        r"""python3 - <<'PY'
import os
aliases = '/proc/device-tree/aliases'
if os.path.isdir(aliases):
    for name in sorted(os.listdir(aliases)):
        path = os.path.join(aliases, name)
        if not os.path.isfile(path):
            continue
        if 'uart' in name.lower() or 'serial' in name.lower():
            raw = open(path,'rb').read().split(b'\0')[0]
            print(name, '->', raw.decode('ascii','replace'))
for ths in ('ttyTHS1','ttyTHS2'):
    base = f'/sys/class/tty/{ths}/device/of_node'
    print('===', ths)
    for f in ('name','status','compatible'):
        p = os.path.join(base, f)
        if os.path.exists(p):
            print(f, open(p,'rb').read().replace(b'\0', b' ').decode('ascii','replace'))
    # reg address
    reg = os.path.join(base, 'reg')
    if os.path.exists(reg):
        print('reg', open(reg,'rb').read().hex())
PY""",
    )


def sniff_other(c: paramiko.SSHClient, listen_port: str, seconds: float = 2.5) -> None:
    """Stop sim briefly and sniff listen_port for PDBC from Controls."""
    ssh_run(
        c,
        f"""python3 - <<'PY'
import serial, time, subprocess
subprocess.call(['pkill','-f','pdb_uart_sim.py'])
time.sleep(0.5)
s = serial.Serial('{listen_port}', 115200, timeout=0.2)
t0 = time.time()
buf = bytearray()
while time.time() - t0 < {seconds}:
    b = s.read(256)
    if b:
        buf.extend(b)
s.close()
print('sniff_port', '{listen_port}', 'n', len(buf))
print('head', bytes(buf[:32]).hex() if buf else None)
print('find_PDBC', buf.find(b'PDBC'), 'find_PDBF', buf.find(b'PDBF'))
nonzero = sum(1 for x in buf if x)
print('nonzero_bytes', nonzero, 'of', len(buf))
PY""",
        timeout=20.0,
    )


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2

    print("COM5: Cursor")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    dt_uart_info(c)

    # Baseline: no sim
    ssh_run(c, "pkill -f pdb_uart_sim.py || true")
    time.sleep(0.5)
    base = check_com5("no_sim")

    results = {"baseline": base}
    for port in ("/dev/ttyTHS1", "/dev/ttyTHS2"):
        start_sim(c, port)
        time.sleep(1.0)
        results[port] = check_com5(port)
        ssh_run(c, "tail -n 6 /tmp/pdb_uart_sim.log")
        if results[port]["pdbf"]:
            print(f"SUCCESS on {port}")
            break

    # If still dead: sniff both UARTs for Controls PDBC (sim off)
    if not any(results.get(p, {}).get("pdbf") for p in ("/dev/ttyTHS1", "/dev/ttyTHS2")):
        print("\n=== sniff Controls→Jetson (sim off) ===")
        for port in ("/dev/ttyTHS1", "/dev/ttyTHS2"):
            sniff_other(c, port, 2.5)

        # Leave sim on THS1 for user swap experiment
        start_sim(c, "/dev/ttyTHS1")

    c.close()
    print("\n=== summary ===")
    for k, v in results.items():
        if isinstance(v, dict):
            print(f"{k}: PDBF={v.get('pdbf')} kill={v.get('kill_name')} estop={v.get('estop')}")
    print("COM5: free")
    return 0 if any(isinstance(v, dict) and v.get("pdbf") for v in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
