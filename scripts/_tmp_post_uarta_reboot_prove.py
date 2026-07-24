#!/usr/bin/env python3
"""Post-reboot prove: UART1 pinmux hog, ESTOP pin18, PDB sim ↔ CDC."""
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
    est, mag, n, last = set(), set(), 0, None
    t0 = time.time()
    while time.time() - t0 < seconds:
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            n += 1
            last = fb.raw
            est.add(fb.raw[SYSTEM_FB_OFF + 16])
            mag.add(struct.unpack_from("<I", fb.raw, PDB_OFF)[0])
        time.sleep(0.02)
    return n, sorted(est), [hex(m) for m in sorted(mag)], last


def main() -> int:
    pw = os.environ.get("JETSON_PASS", "4565")
    print("=== boot / overlays ===")
    subprocess.call("uptime; hostname -I", shell=True)
    subprocess.call(
        "grep -E 'DEFAULT|OVERLAYS|LABEL JetsonIO' /boot/extlinux/extlinux.conf",
        shell=True,
    )

    print("\n=== pinmux UART1 + PH.00 (need sudo) ===")
    subprocess.call(
        f'echo {pw} | sudo -S -p "" grep -nE '
        '"UART1_TX_PR2|UART1_RX_PR3|SOC_GPIO21_PH0" '
        "/sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins",
        shell=True,
    )
    subprocess.call(
        f'echo {pw} | sudo -S -p "" python3 '
        "/opt/nvidia/jetson-io/config-by-pin.py -p 8; "
        f'echo {pw} | sudo -S -p "" python3 '
        "/opt/nvidia/jetson-io/config-by-pin.py -p 10; "
        f'echo {pw} | sudo -S -p "" python3 '
        "/opt/nvidia/jetson-io/config-by-pin.py -p 18",
        shell=True,
    )

    print("\n=== A: THS1 listen 3s ===")
    rc = subprocess.call(
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
    print("listen_rc", rc)

    acms = sorted(glob.glob("/dev/ttyACM*"))
    print("\nCDC", acms)
    if not acms:
        print("FAIL: no CDC")
        return 3
    cdc = acms[0]

    import Jetson.GPIO as GPIO

    print("\n=== F: pin18 ESTOP ===")
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(18, GPIO.OUT, initial=GPIO.HIGH)
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        for level, label in ((1, "HIGH"), (0, "LOW"), (1, "HIGH2")):
            GPIO.output(18, GPIO.HIGH if level else GPIO.LOW)
            time.sleep(0.15)
            n, est, mag, last = sample(hub, 0.8)
            clk = brr = None
            if last is not None:
                clk = last[SYSTEM_FB_OFF + 17]
                brr = last[SYSTEM_FB_OFF + 30] | (last[SYSTEM_FB_OFF + 31] << 8)
            print(
                f"pin18={level} {label}: estop={est} pdb={mag} "
                f"clk={clk} BRR={None if brr is None else hex(brr)} n={n}"
            )
    try:
        GPIO.cleanup(18)
    except Exception:
        pass

    print("\n=== D: pdb_uart_sim on THS1 ===")
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
    print("--- sim early ---")
    print(open("/tmp/pdb_uart_sim.log").read()[-1000:])
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        n, est, mag, last = sample(hub, 4.0)
        head = None if last is None else last[PDB_OFF : PDB_OFF + 8].hex()
        print(f"with_sim: n={n} estop={est} pdb_magic={mag} pdb_head={head}")
    print("--- sim late ---")
    print(open("/tmp/pdb_uart_sim.log").read()[-1200:])
    subprocess.call("pkill -f pdb_uart_sim.py 2>/dev/null || true", shell=True)

    # verdict helpers
    print("\n=== VERDICT ===")
    pinmux = subprocess.check_output(
        f'echo {pw} | sudo -S -p "" grep -E '
        '"UART1_TX_PR2|UART1_RX_PR3|SOC_GPIO21_PH0" '
        "/sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins || true",
        shell=True,
        text=True,
        errors="replace",
    )
    print(pinmux)
    print("DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
