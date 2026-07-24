#!/usr/bin/env python3
"""Re-prove Jetson↔Controls UART/ESTOP after USB-powered common ground.

Assumes Controls CDC is plugged into the Jetson (not Windows COM5). Syncs
scripts, then on the Jetson: THS1 listen, CDC pdb/estop poll, pdb_uart_sim,
ESTOP pin16 hold vs estop_sense.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "")
REMOTE_ROOT = "/home/deft-robotics/controls_pcb"
REMOTE_SCRIPTS = f"{REMOTE_ROOT}/scripts"
LOCAL_SCRIPTS = Path(__file__).resolve().parent

SYNC_FILES = [
    "jetson_uart_listen.py",
    "jetson_estop_drive.py",
    "pdb_uart_sim.py",
    "deft_controls_sdk/__init__.py",
    "deft_controls_sdk/controls_pcb_hub.py",
    "deft_controls_sdk/pdb/__init__.py",
    "deft_controls_sdk/pdb/frame.py",
    "deft_controls_sdk/pdb/framing.py",
    "deft_controls_sdk/link/__init__.py",
    "deft_controls_sdk/link/api_types.py",
    "deft_controls_sdk/link/connection.py",
    "deft_controls_sdk/link/exceptions.py",
    "deft_controls_sdk/link/exchange/__init__.py",
    "deft_controls_sdk/link/exchange/wire_layout.py",
    "deft_controls_sdk/link/exchange/pack.py",
    "deft_controls_sdk/link/exchange/parse.py",
    "deft_controls_sdk/link/exchange/transport.py",
    "deft_controls_sdk/link/exchange/bench.py",
    "deft_controls_sdk/bench/__init__.py",
    "deft_controls_sdk/bench/soft_dfu.py",
    "deft_controls_sdk/telemetry/__init__.py",
    "deft_controls_sdk/telemetry/cache.py",
]


REMOTE_PROVE = r'''
import glob, os, struct, subprocess, sys, time
sys.path.insert(0, ".")

print("=== identity ===")
print("ttyACM", sorted(glob.glob("/dev/ttyACM*")))
print("ttyTHS", sorted(glob.glob("/dev/ttyTHS*")))
subprocess.call(["lsusb"])
subprocess.call(
    "pkill -f pdb_uart_sim.py 2>/dev/null; "
    "pkill -f jetson_estop_drive.py 2>/dev/null; "
    "pkill -f jetson_uart_listen.py 2>/dev/null; sleep 0.3",
    shell=True,
)

print("\n=== A: THS1 listen 3s ===")
rc = subprocess.call(
    [sys.executable, "jetson_uart_listen.py", "--ports", "/dev/ttyTHS1", "/dev/ttyTHS2", "--seconds", "3"]
)
print("listen_rc", rc)

acms = sorted(glob.glob("/dev/ttyACM*"))
if not acms:
    print("FAIL: no /dev/ttyACM* — Controls USB not on Jetson?")
    sys.exit(3)
cdc = acms[0]
print("\n=== CDC", cdc, "===")

from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.link.exchange.wire_layout import PDB_OFF, SYSTEM_FB_OFF

def sample(hub, seconds=1.5):
    last = None
    magics = set()
    estops = set()
    kills = set()
    n = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            raw = fb.raw
            last = raw
            n += 1
            pdb = raw[PDB_OFF:PDB_OFF+64]
            magics.add(struct.unpack_from("<I", pdb, 0)[0])
            ks, kr, es = struct.unpack_from("<BBB", raw, SYSTEM_FB_OFF + 14)
            kills.add((ks, kr))
            estops.add(es)
        time.sleep(0.02)
    return last, n, magics, kills, estops

def summarize(label, last, n, magics, kills, estops):
    if last is None:
        print(label, "NO_FB")
        return
    pdb = last[PDB_OFF:PDB_OFF+64]
    clk = last[SYSTEM_FB_OFF + 4]  # reserved0 in some builds; print nearby bytes
    # system block dump around kill/estop/reserved
    sys_slice = last[SYSTEM_FB_OFF:SYSTEM_FB_OFF+30]
    print(label, f"n={n} magics={[hex(m) for m in sorted(magics)]} "
          f"kills={sorted(kills)} estops={sorted(estops)} "
          f"pdb_head={pdb[:8].hex()} sys30={sys_slice.hex()}")

with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
    hub.recover()
    last, n, magics, kills, estops = sample(hub, 2.0)
    summarize("baseline_cdc", last, n, magics, kills, estops)

print("\n=== D: start pdb_uart_sim on THS1 ===")
open("/tmp/pdb_uart_sim.log", "w").close()
subprocess.Popen(
    [sys.executable, "-u", "pdb_uart_sim.py", "--port", "/dev/ttyTHS1", "--hz", "20"],
    cwd=".",
    stdout=open("/tmp/pdb_uart_sim.log", "a"),
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
time.sleep(2.5)
print("--- sim log ---")
print(open("/tmp/pdb_uart_sim.log").read()[-1200:])

with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
    hub.recover()
    last, n, magics, kills, estops = sample(hub, 4.0)
    summarize("with_sim", last, n, magics, kills, estops)

print("--- sim log after poll ---")
print(open("/tmp/pdb_uart_sim.log").read()[-1200:])
subprocess.call("pkill -f pdb_uart_sim.py 2>/dev/null || true", shell=True)
time.sleep(0.4)

print("\n=== F: ESTOP pin16 hold vs estop_sense ===")
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT, initial=GPIO.HIGH)

def hold_sample(level, label, hold_s=2.0):
    GPIO.output(16, GPIO.HIGH if level else GPIO.LOW)
    print(f"drive pin16={level} ({label})", flush=True)
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        last, n, magics, kills, estops = sample(hub, hold_s)
        summarize(f"estop_{label}", last, n, magics, kills, estops)
        return estops

try:
    e_hi = hold_sample(1, "RELEASE_HI")
    e_lo = hold_sample(0, "ASSERT_LO")
    e_hi2 = hold_sample(1, "RELEASE_HI_2")
    moved = (e_hi | e_lo | e_hi2)
    print("estop_sense_values_seen", sorted(moved))
    if len(moved) > 1:
        print("ESTOP_SENSE: IMPROVED (toggled)")
    else:
        print("ESTOP_SENSE: STILL STUCK", sorted(moved))
finally:
    GPIO.output(16, GPIO.HIGH)
    GPIO.cleanup()

print("\nDONE")
'''


def ssh_run(c: paramiko.SSHClient, cmd: str, timeout: float = 120.0) -> tuple[int, str]:
    print(">>>", cmd[:200] + ("…" if len(cmd) > 200 else ""))
    _, o, e = c.exec_command(cmd, timeout=timeout)
    out = o.read().decode("utf-8", "replace")
    err = e.read().decode("utf-8", "replace")
    code = o.channel.recv_exit_status()
    text = out + (("\n" + err) if err else "")
    sys.stdout.write(text if text.endswith("\n") else text + "\n")
    print("exit", code)
    return code, text


def sync(c: paramiko.SSHClient) -> None:
    sftp = c.open_sftp()
    for rel in SYNC_FILES:
        local = LOCAL_SCRIPTS / rel
        remote = f"{REMOTE_SCRIPTS}/{rel}"
        if not local.is_file():
            print("skip missing", rel)
            continue
        # ensure remote dir
        parts = rel.split("/")
        if len(parts) > 1:
            d = REMOTE_SCRIPTS
            for p in parts[:-1]:
                d = f"{d}/{p}"
                try:
                    sftp.stat(d)
                except OSError:
                    sftp.mkdir(d)
        print("put", rel)
        sftp.put(str(local), remote)
        sftp.chmod(remote, 0o755 if rel.endswith(".py") else 0o644)
    sftp.close()


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    print("NOTE: Controls CDC expected on Jetson USB (/dev/ttyACM*), not Windows COM5")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=20)
    try:
        sync(c)
        # write prove script remotely to avoid quoting hell
        sftp = c.open_sftp()
        with sftp.file(f"{REMOTE_SCRIPTS}/_tmp_gnd_reprove_remote.py", "w") as f:
            f.write(REMOTE_PROVE)
        sftp.chmod(f"{REMOTE_SCRIPTS}/_tmp_gnd_reprove_remote.py", 0o755)
        sftp.close()
        code, _ = ssh_run(
            c,
            f"cd {REMOTE_SCRIPTS} && python3 -u _tmp_gnd_reprove_remote.py",
            timeout=180.0,
        )
        return code
    finally:
        c.close()


if __name__ == "__main__":
    raise SystemExit(main())
