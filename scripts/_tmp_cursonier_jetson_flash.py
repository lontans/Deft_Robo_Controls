"""Cursonier: sync V/I soft-kill FW to Jetson and Soft-DFU flash."""
from __future__ import annotations

import os
import sys
import time

import paramiko

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PW = os.environ.get("JETSON_PASS", "4565")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REMOTE = "/home/deft-robotics/controls_pcb"
# Plan pinned Soft-DFU target was 3167375E3435; fall back to sole CDC if absent.
PREFERRED_SERIAL = "3167375E3435"

FILES = [
    "App/Inc/host/pdb_vi_limits.h",
    "App/Inc/host/pdb_link.h",
    "App/Src/host/pdb_link.c",
    "Debug/DeftRoboticsControlsPCB.elf",
    "Release/DeftRoboticsControlsPCB.elf",
    "scripts/tests/test_pdb_link_frames.py",
]


def run(c: paramiko.SSHClient, cmd: str, timeout: float = 120.0) -> tuple[int, str, str]:
    print("JETSON>>>", cmd[:200])
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    code = stdout.channel.recv_exit_status()
    if out:
        print(out, end="" if out.endswith("\n") else "\n")
    if err:
        print("STDERR:", err, end="" if err.endswith("\n") else "\n")
    return code, out, err


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PW, timeout=15)

    sftp = c.open_sftp()
    for rel in FILES:
        local = os.path.join(ROOT, *rel.split("/"))
        remote = f"{REMOTE}/{rel}"
        remote_dir = os.path.dirname(remote)
        run(c, f"mkdir -p {remote_dir}", timeout=10)
        print(f"SFTP {rel} -> {remote}")
        sftp.put(local, remote)
    sftp.close()

    run(c, "ps -ef | grep -E 'yam_continuous|vbeta|soft_dfu|pdb_uart_sim|debug_dashboard' | grep -v grep || true")
    # Dashboard follow-mode is fine; kill anything holding CDC enter path.
    run(
        c,
        "pkill -f 'yam_continuous_all|soft_dfu_flash' || true; "
        "fuser -k /dev/ttyACM0 2>/dev/null || true",
        timeout=15,
    )
    time.sleep(1.0)
    code, out, _ = run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan")
    import re

    sns = re.findall(r"sn=([0-9A-Fa-f]+)", out)
    if PREFERRED_SERIAL in sns:
        serial = PREFERRED_SERIAL
    elif len(sns) == 1:
        serial = sns[0]
        print(f"NOTE: preferred {PREFERRED_SERIAL} absent; using sole CDC sn={serial}")
    else:
        print(f"FAIL: cannot pin serial from scan: {sns}", file=sys.stderr)
        c.close()
        return 2

    # Alternating Soft-DFU prove: Debug then Release, USB-only, pinned serial.
    for image in (
        "Debug/DeftRoboticsControlsPCB.elf",
        "Release/DeftRoboticsControlsPCB.elf",
    ):
        # Non-interactive sudo for dfu-util (udev may already allow user).
        flash = (
            f"cd {REMOTE} && echo {PW!r} | sudo -S -E python3 scripts/soft_dfu_flash.py "
            f"--image {image} --serial {serial} --require-usb-dfu"
        )
        code, out, err = run(c, flash, timeout=180)
        if code != 0:
            print(f"FAIL flash {image} exit={code}", file=sys.stderr)
            c.close()
            return code or 1
        time.sleep(2.0)
        run(c, f"cd {REMOTE} && python3 scripts/soft_dfu_flash.py scan")

    print(f"FLASH_OK serial={serial}")
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
