"""One Soft-DFU USB-only cycle; SWD rescue if DF11 missing."""
from __future__ import annotations

import os
import subprocess
import time

import paramiko

HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb"
SERIAL = "3167376F3435"
CUBE = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def run(c, cmd, timeout=180.0):
    print("JETSON>>>", cmd[:220])
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    code = stdout.channel.recv_exit_status()
    print(out, end="" if not out or out.endswith("\n") else "\n")
    if err.strip():
        print("STDERR:", err[:500])
    return code, out


def swd_rescue(image_rel: str) -> None:
    elf = os.path.join(ROOT, *image_rel.split("/"))
    print("SWD rescue", elf)
    subprocess.run(
        [CUBE, "-c", "port=SWD", "mode=UR", "-ob", "nBOOT0=1", "-rst"],
        check=False,
    )
    time.sleep(0.5)
    subprocess.run(
        [CUBE, "-c", "port=SWD", "mode=UR", "-w", elf, "-v", "-rst"],
        check=True,
    )


def main() -> int:
    # Ensure Release ELF on Jetson
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    sftp.put(
        os.path.join(ROOT, "Release", "DeftRoboticsControlsPCB.elf"),
        f"{REMOTE}/Release/DeftRoboticsControlsPCB.elf",
    )
    sftp.close()

    code, out = run(
        c,
        f"cd {REMOTE} && echo {PW!r} | sudo -S -E python3 scripts/soft_dfu_flash.py "
        f"--image Release/DeftRoboticsControlsPCB.elf --serial {SERIAL} --require-usb-dfu",
    )
    if code == 0:
        print("SOFT_DFU_USB_OK")
        run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan")
        c.close()
        return 0

    print("Soft-DFU USB failed — SWD rescue with Release ELF")
    c.close()
    swd_rescue("Release/DeftRoboticsControlsPCB.elf")
    time.sleep(2)
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    for _ in range(10):
        _, out = run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan", 30)
        if "[CDC]" in out:
            print("RESCUED_CDC_AFTER_SWD")
            c.close()
            return 2  # FW on board via SWD; Soft-DFU USB not proven
        time.sleep(2)
    c.close()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
