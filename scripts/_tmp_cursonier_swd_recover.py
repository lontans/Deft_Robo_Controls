"""ST-Link recovery: restore nBOOT0=1, flash Debug ELF, confirm Jetson CDC."""
from __future__ import annotations

import os
import subprocess
import time

import paramiko

CUBE = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
ELF = os.path.join(
    os.path.dirname(__file__),
    "..",
    "Debug",
    "DeftRoboticsControlsPCB.elf",
)
ELF = os.path.abspath(ELF)
HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")


def cube(args: list[str]) -> int:
    cmd = [CUBE, *args]
    print("LOCAL>>>", " ".join(cmd))
    p = subprocess.run(cmd, capture_output=True, text=True)
    out = (p.stdout or "") + (p.stderr or "")
    print(out[-2500:])
    return p.returncode


def jetson_scan() -> str:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    _, stdout, _ = c.exec_command(
        "cd ~/controls_pcb && python3 scripts/soft_dfu_flash.py scan", timeout=30
    )
    out = stdout.read().decode(errors="replace")
    c.close()
    print("JETSON SCAN:\n", out)
    return out


def main() -> int:
    # Restore flash boot if stuck with nBOOT0=0 and no DF11.
    rc = cube(["-c", "port=SWD", "mode=UR", "-ob", "nBOOT0=1", "-rst"])
    if rc != 0:
        print("WARN: nBOOT0 restore rc", rc)

    time.sleep(1.0)
    rc = cube(
        [
            "-c",
            "port=SWD",
            "mode=UR",
            "-w",
            ELF,
            "-v",
            "-rst",
        ]
    )
    if rc != 0:
        print("FAIL SWD flash", rc)
        return rc or 1

    print("waiting for Jetson CDC…")
    for i in range(15):
        time.sleep(2)
        out = jetson_scan()
        if "[CDC]" in out:
            print("RECOVERED_CDC")
            return 0
        print(f"poll {i}: no CDC yet")

    print("FAIL: Jetson CDC never returned after SWD flash")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
