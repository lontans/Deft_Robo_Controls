"""Host-side launcher: deploy + run _tmp_cursonier_vi_prove.py on Jetson."""
from __future__ import annotations

import os

import paramiko

HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    sftp.put(
        os.path.join(ROOT, "scripts", "_tmp_cursonier_vi_prove.py"),
        "/home/deft-robotics/controls_pcb/scripts/_tmp_cursonier_vi_prove.py",
    )
    sftp.close()

    cmds = [
        "pkill -f 'python -m deft_controls_sdk.debug_dashboard' || true",
        "fuser -k /dev/ttyACM0 2>/dev/null || true",
        "cd /home/deft-robotics/controls_pcb && python3 scripts/soft_dfu_flash.py scan",
        "cd /home/deft-robotics/controls_pcb/scripts && PYTHONPATH=. python3 _tmp_cursonier_vi_prove.py",
    ]
    rc = 0
    for cmd in cmds:
        print("JETSON>>>", cmd)
        _, stdout, stderr = c.exec_command(cmd, timeout=120)
        out = stdout.read().decode(errors="replace")
        err = stderr.read().decode(errors="replace")
        code = stdout.channel.recv_exit_status()
        sys_out = (out or "").encode("ascii", "replace").decode("ascii")
        sys_err = (err or "").encode("ascii", "replace").decode("ascii")
        if sys_out:
            print(sys_out, end="" if sys_out.endswith("\n") else "\n")
        if sys_err.strip():
            print("STDERR:", sys_err[-2500:])
        print("exit", code)
        if "vi_prove" in cmd or "_tmp_cursonier_vi_prove" in cmd:
            rc = code
    c.close()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
