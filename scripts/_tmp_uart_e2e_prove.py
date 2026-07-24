#!/usr/bin/env python3
"""End-to-end UART4 prove: Jetson termios + MCU RX counters + both directions."""
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

# system feedback offsets within system block (SYSTEM_FB_OFF base)
# usb_rx_drop @ +10, can_rx_drop @ +12, kill @ +14, reserved0 @ +17, reserved @ +30


def decode_sys(raw: bytes) -> dict:
    base = SYSTEM_FB_OFF
    usb_drop, can_drop = struct.unpack_from("<HH", raw, base + 10)
    ks, kr, es, r0 = struct.unpack_from("<BBBB", raw, base + 14)
    last_rx, applied = struct.unpack_from("<II", raw, base + 18)
    brr = raw[base + 30] | (raw[base + 31] << 8)
    return {
        "rx_bytes": usb_drop,  # overlaid
        "rx_events": can_drop & 0xFF,
        "rx_valid": (can_drop >> 8) & 0xFF,
        "kill": ks,
        "reason": kr,
        "estop": es,
        "clk": r0 & 0x0F,
        "err": (r0 >> 4) & 0x0F,
        "brr": brr,
        "last_rx": f"0x{last_rx:08X}",
        "last_rx_asc": bytes(
            [(last_rx >> (8 * i)) & 0xFF for i in range(4)]
        ).decode("ascii", "replace"),
        "crc_fail": applied & 0xFFFF,
        "tx_cplt": (applied >> 16) & 0xFFFF,
        "pdb_magic": struct.unpack_from("<I", raw, PDB_OFF)[0],
        "pdb_head": raw[PDB_OFF : PDB_OFF + 8].hex(),
    }


def sample(hub, seconds: float = 1.5):
    last = None
    n = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            last = fb.raw
            n += 1
        time.sleep(0.02)
    return n, None if last is None else decode_sys(last)


def main() -> int:
    pw = os.environ.get("JETSON_PASS", "4565")
    print("=== Jetson UART1 node ===")
    subprocess.call(
        "ls -l /dev/ttyTHS1; "
        f'echo {pw} | sudo -S -p "" grep -nE '
        '"UART1_TX_PR2|UART1_RX_PR3" '
        "/sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins",
        shell=True,
    )
    print("--- termios before open ---")
    subprocess.call("stty -F /dev/ttyTHS1 -a 2>&1 | head -5 || true", shell=True)

    print("\n=== Jetson TX blast (known pattern) while MCU listens ===")
    # 200 x 0x55 then PDBF-like cadence via sim
    py_tx = r"""
import serial, time
s = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.05)
s.reset_input_buffer(); s.reset_output_buffer()
print('termios after open', flush=True)
import subprocess; subprocess.call(['stty','-F','/dev/ttyTHS1','-a'])
blob = bytes([0x55])*64
for i in range(40):
    s.write(blob)
    time.sleep(0.02)
print('sent', 40*64, 'bytes of 0x55', flush=True)
s.close()
"""
    open("/tmp/jetson_tx55.py", "w").write(py_tx)

    acms = sorted(glob.glob("/dev/ttyACM*"))
    if not acms:
        print("FAIL no CDC")
        return 3
    cdc = acms[0]
    print("CDC", cdc)

    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        n0, d0 = sample(hub, 1.0)
        print("baseline", d0, "n", n0)

    subprocess.call([sys.executable, "-u", "/tmp/jetson_tx55.py"])
    time.sleep(0.3)

    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        n1, d1 = sample(hub, 1.2)
        print("after_0x55", d1, "n", n1)

    print("\n=== A: MCU→Jetson listen 3s ===")
    subprocess.call(
        [
            sys.executable,
            "jetson_uart_listen.py",
            "--ports",
            "/dev/ttyTHS1",
            "--seconds",
            "3",
        ],
        cwd="/home/deft-robotics/controls_pcb/scripts",
    )

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
    time.sleep(3.0)
    with ControlsPcbHub.connect(cdc, persist_telemetry=False) as hub:
        hub.recover()
        n2, d2 = sample(hub, 3.0)
        print("with_sim", d2, "n", n2)
    print(open("/tmp/pdb_uart_sim.log").read()[-1000:])
    subprocess.call("pkill -f pdb_uart_sim.py 2>/dev/null || true", shell=True)

    print("\n=== INTERPRET ===")
    if d0 and d1:
        db = d1["rx_bytes"] - d0["rx_bytes"]
        de = d1["rx_events"] - d0["rx_events"]
        print(f"delta after 0x55 blast: rx_bytes={db} rx_events={de} err={d1['err']}")
        if db > 0:
            print("MCU_RX: HEARING Jetson TX (raw bytes rising)")
        else:
            print("MCU_RX: DEAF to Jetson TX (rx_bytes flat)")
    if d2:
        print(
            f"sim window: rx_bytes={d2['rx_bytes']} events={d2['rx_events']} "
            f"valid={d2['rx_valid']} pdb_magic=0x{d2['pdb_magic']:08X} err={d2['err']}"
        )
    print("DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
