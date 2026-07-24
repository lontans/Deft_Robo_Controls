#!/usr/bin/env python3
"""Sweep Jetson BOARD GPIOs; watch Controls PB7 via CDC estop_sense.

Hunts pin-mapping: which (if any) Jetson header pin reaches MCU PB7.
Runs entirely on the Jetson (Controls expected on /dev/ttyACM0).
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
REMOTE = "/home/deft-robotics/controls_pcb/scripts/_tmp_gpio_sweep_remote.py"
LOCAL_SCRIPTS = Path(__file__).resolve().parent

REMOTE_PY = r'''
import glob, struct, sys, time
sys.path.insert(0, ".")

import Jetson.GPIO as GPIO
from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.link.exchange.wire_layout import SYSTEM_FB_OFF

acms = sorted(glob.glob("/dev/ttyACM*"))
if not acms:
    print("FAIL: no /dev/ttyACM*")
    sys.exit(3)
cdc = acms[0]
print("CDC", cdc)
print("model", open("/proc/device-tree/model", "rb").read().split(b"\0")[0])

# Valid GPIO-capable BOARD pins on this AGX Orin (probed earlier).
# Pins 8/10 are UART-only here (Jetson.GPIO rejects them as OUT).
CANDIDATES = [7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40]

def read_estop(hub, samples=8, dt=0.03):
    vals = []
    for _ in range(samples):
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            es = fb.raw[SYSTEM_FB_OFF + 16]
            vals.append(es)
        time.sleep(dt)
    return vals

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

hits = []
print("pin  hi_vals  lo_vals  note")
with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
    hub.recover()
    base = read_estop(hub, samples=12)
    print(f"idle {sorted(set(base))}  (n={len(base)})")

    for pin in CANDIDATES:
        try:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        except Exception as ex:
            print(f"{pin:3d}  SETUP_FAIL {ex}")
            continue
        try:
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.08)
            hi = read_estop(hub)
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.08)
            lo = read_estop(hub)
            note = ""
            if 1 in hi:
                note = "HIT_HIGH"
                hits.append((pin, "HIGH", hi, lo))
            if 1 in lo and 1 not in hi:
                note = (note + "+").strip("+") + "HIT_LOW_ODD"
                hits.append((pin, "LOW_ODD", hi, lo))
            if max(hi + lo) != min(hi + lo):
                note = (note + " TOGGLE").strip()
                if pin not in [h[0] for h in hits]:
                    hits.append((pin, "TOGGLE", hi, lo))
            print(f"{pin:3d}  {sorted(set(hi))}  {sorted(set(lo))}  {note}")
        finally:
            try:
                GPIO.output(pin, GPIO.LOW)
            except Exception:
                pass
            try:
                GPIO.cleanup(pin)
            except Exception:
                pass

print("---")
if hits:
    print("HITS", hits)
else:
    print("HITS: none — no candidate Jetson GPIO produced estop_sense==1 on MCU PB7")
print("DONE")
'''


def main() -> int:
    if not PW:
        print("set JETSON_PASS", file=sys.stderr)
        return 2
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=20)
    try:
        # ensure hub code present (from prior sync); refresh drive helper only
        sftp = c.open_sftp()
        with sftp.file(REMOTE, "w") as f:
            f.write(REMOTE_PY)
        sftp.chmod(REMOTE, 0o755)
        sftp.close()
        cmd = (
            "pkill -f pdb_uart_sim.py 2>/dev/null || true; "
            "pkill -f jetson_estop_drive.py 2>/dev/null || true; "
            "sleep 0.3; "
            f"cd /home/deft-robotics/controls_pcb/scripts && python3 -u _tmp_gpio_sweep_remote.py"
        )
        print(">>>", cmd)
        _, o, e = c.exec_command(cmd, timeout=300)
        # stream
        while True:
            if o.channel.recv_ready():
                sys.stdout.write(o.channel.recv(8192).decode("utf-8", "replace"))
                sys.stdout.flush()
            if o.channel.recv_stderr_ready():
                sys.stderr.write(o.channel.recv_stderr(8192).decode("utf-8", "replace"))
            if o.channel.exit_status_ready() and not o.channel.recv_ready():
                break
            time.sleep(0.05)
        # drain
        sys.stdout.write(o.read().decode("utf-8", "replace"))
        err = e.read().decode("utf-8", "replace")
        if err:
            sys.stderr.write(err)
        return o.channel.recv_exit_status()
    finally:
        c.close()


if __name__ == "__main__":
    raise SystemExit(main())
