#!/usr/bin/env python3
"""Check whether jetson-io GPIO overlay left UART/SPI pinmux intact."""
from __future__ import annotations

import os
import subprocess
import sys


def sh(cmd: str) -> None:
    print(">>>", cmd)
    subprocess.call(cmd, shell=True)


def sudo(cmd: str) -> None:
    pw = os.environ.get("JETSON_PASS", "4565")
    sh(f'echo {pw} | sudo -S -p "" {cmd}')


def main() -> int:
    print("=== enabled functions ===")
    sudo("python3 /opt/nvidia/jetson-io/config-by-function.py -l enabled")

    print("\n=== pin labels (header) ===")
    for p in (7, 8, 10, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 36):
        sudo(f"python3 /opt/nvidia/jetson-io/config-by-pin.py -p {p}")

    print("\n=== pinmux UART1 + SPI1 + PH.00 ===")
    sudo(
        "grep -nE 'UART1_|SPI1_|SOC_GPIO21_PH0|PR\\.2|PR\\.3|PR\\.4|PR\\.5|"
        "PZ\\.3|PZ\\.4|PZ\\.5|PZ\\.6|PZ\\.7' "
        "/sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins"
    )

    print("\n=== dtbo strings (overlay we wrote) ===")
    sudo("strings /boot/jetson-io-hdr40-user-custom.dtbo | head -100")

    print("\n=== Jetson.GPIO BOARD 8/10 ===")
    import Jetson.GPIO as GPIO

    board = GPIO.gpio_pin_data.get_data()[2]["BOARD"]
    for pin in (8, 10, 18):
        info = board.get(pin)
        if info is None:
            print(f"BOARD {pin}: not a Jetson.GPIO channel (expected for UART pins)")
        else:
            print(
                f"BOARD {pin}: {info.gpio_name} chip={info.gpio_chip} "
                f"line={info.line_offset}"
            )

    print("\n=== serial nodes ===")
    sh("ls -l /dev/ttyTHS1 /dev/ttyTHS2")
    sh("cat /proc/device-tree/bus@0/serial@3100000/status; echo")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
