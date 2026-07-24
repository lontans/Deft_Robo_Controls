#!/usr/bin/env python3
"""Post jetson-io reboot: pin18 GPIO, ESTOP sense, UART/PDB prove."""
from __future__ import annotations

import glob
import os
import struct
import subprocess
import sys
import time

sys.path.insert(0, "/home/deft-robotics/controls_pcb/scripts")

from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.link.exchange.wire_layout import PDB_OFF, SYSTEM_FB_OFF


def sample(hub, seconds: float = 1.0):
    est, mag, n = set(), set(), 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            n += 1
            est.add(fb.raw[SYSTEM_FB_OFF + 16])
            mag.add(struct.unpack_from("<I", fb.raw, PDB_OFF)[0])
        time.sleep(0.02)
    return n, sorted(est), [hex(m) for m in sorted(mag)]


def main() -> int:
    print("=== identity ===")
    print(open("/proc/device-tree/model", "rb").read().split(b"\0")[0])
    subprocess.call("uptime; hostname -I", shell=True)
    print("extlinux:")
    subprocess.call(
        "grep -E 'DEFAULT|OVERLAYS|LABEL JetsonIO|MENU LABEL Custom' "
        "/boot/extlinux/extlinux.conf || true",
        shell=True,
    )

    print("\n=== pinmux / labels ===")
    subprocess.call("gpioinfo | grep -E 'PH.00|PQ.06|PBB.01' || true", shell=True)
    pw = os.environ.get("JETSON_PASS", "4565")
    subprocess.call(
        f'echo {pw} | sudo -S -p "" grep -n SOC_GPIO21_PH0 '
        "/sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins || true",
        shell=True,
    )
    subprocess.call(
        "python3 /opt/nvidia/jetson-io/config-by-pin.py -p 18; "
        "python3 /opt/nvidia/jetson-io/config-by-pin.py -p 16 || true",
        shell=True,
    )

    print("\n=== A: THS1 listen 3s ===")
    subprocess.call(
        [
            sys.executable,
            "jetson_uart_listen.py",
            "--ports",
            "/dev/ttyTHS1",
            "/dev/ttyTHS2",
            "--seconds",
            "3",
        ],
        cwd="/home/deft-robotics/controls_pcb/scripts",
    )

    acms = sorted(glob.glob("/dev/ttyACM*"))
    print("\nCDC", acms)
    if not acms:
        print("FAIL: no Controls CDC on Jetson USB")
        return 3
    cdc = acms[0]

    import Jetson.GPIO as GPIO

    print("\n=== F: BOARD pin18 vs estop_sense ===")
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        for level, label in ((1, "HIGH"), (0, "LOW"), (1, "HIGH2"), (0, "LOW2")):
            GPIO.output(18, GPIO.HIGH if level else GPIO.LOW)
            time.sleep(0.2)
            n, est, mag = sample(hub, 1.0)
            print(f"pin18={level} {label}: n={n} estop={est} pdb_magic={mag}")
            if level == 1:
                subprocess.call("gpioinfo | grep PH.00 || true", shell=True)

    try:
        GPIO.cleanup(18)
    except Exception:
        pass

    print("\n=== gpioset chip0 line43=1 ===")
    p = subprocess.Popen(["gpioset", "-m", "time", "-s", "4", "gpiochip0", "43=1"])
    time.sleep(0.5)
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        n, est, mag = sample(hub, 2.5)
        print(f"gpioset43=1: n={n} estop={est} pdb_magic={mag}")
        subprocess.call("gpioinfo | grep PH.00 || true", shell=True)
    p.wait()

    print("\n=== D: pdb_uart_sim ===")
    subprocess.call("pkill -f pdb_uart_sim.py 2>/dev/null || true", shell=True)
    time.sleep(0.3)
    open("/tmp/pdb_uart_sim.log", "w").close()
    subprocess.Popen(
        [sys.executable, "-u", "pdb_uart_sim.py", "--port", "/dev/ttyTHS1", "--hz", "20"],
        cwd="/home/deft-robotics/controls_pcb/scripts",
        stdout=open("/tmp/pdb_uart_sim.log", "a"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(2.5)
    print(open("/tmp/pdb_uart_sim.log").read()[-900:])
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        n, est, mag = sample(hub, 3.5)
        print(f"with_sim: n={n} estop={est} pdb_magic={mag}")
    print(open("/tmp/pdb_uart_sim.log").read()[-900:])
    subprocess.call("pkill -f pdb_uart_sim.py 2>/dev/null || true", shell=True)

    print("DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
