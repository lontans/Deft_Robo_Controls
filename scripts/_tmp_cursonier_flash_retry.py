"""After SWD recovery attempt: scan Jetson, Soft-DFU USB-only if CDC up."""
from __future__ import annotations

import os
import re
import time

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = "deft-robotics"
PW = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb"


def run(c, cmd, timeout=180.0):
    print("JETSON>>>", cmd[:220])
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    code = stdout.channel.recv_exit_status()
    if out:
        print(out, end="" if out.endswith("\n") else "\n")
    if err.strip():
        print("STDERR:", err[:600])
    return code, out


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    for i in range(8):
        _, out = run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan", timeout=30)
        sns = re.findall(r"sn=([0-9A-Fa-f]+)", out)
        if sns:
            serial = sns[0]
            print(f"using serial={serial}")
            break
        time.sleep(2)
    else:
        print("no CDC on Jetson")
        c.close()
        return 2

    # Ensure ELF still present; flash Debug then Release USB-only.
    for image in (
        "Debug/DeftRoboticsControlsPCB.elf",
        "Release/DeftRoboticsControlsPCB.elf",
    ):
        code, out = run(
            c,
            f"cd {REMOTE} && echo {PW!r} | sudo -S -E python3 scripts/soft_dfu_flash.py "
            f"--image {image} --serial {serial} --require-usb-dfu",
            timeout=180,
        )
        if code != 0:
            print(f"FAIL {image}")
            # wait for CDC recovery
            for _ in range(10):
                time.sleep(2)
                _, scan = run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan", 30)
                if "CDC" in scan:
                    break
            c.close()
            return code or 1
        time.sleep(2)
        run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan", 30)

    print(f"FLASH_OK serial={serial}")
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
