"""Prove dashboard observe connect does not latch ESTOP on Jetson CDC."""
from __future__ import annotations

import os
import time

import paramiko

HOST = "192.168.50.48"
PW = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb"


PROOF = r'''
import sys, time
sys.path.insert(0, "scripts")
from deft_controls_sdk.debug_dashboard.app import AppState
from deft_controls_sdk.link import McuState

state = AppState(persist_telemetry=False, stream_hz=40.0, telemetry_hz=10.0)
state.connect("/dev/ttyACM0", mode="observe")
time.sleep(0.8)
assert state.control_mode == "observe"
hub = state.hub
assert hub is not None
# MCU desire should be DIAG_ONLY (2)
assert int(hub._connection.mcu_state) == int(McuState.DIAG_ONLY), hub._connection.mcu_state
st = hub.pdb_status()
print("observe_ok", "mcu_cmd", int(hub._connection.mcu_state),
      "usb_kill", None if st is None else (st.kill_state_name, st.kill_reason_name),
      "fb_mcu", None if st is None else st)
# soft-kill hooks off: even if PDU soft_kill_req, we should NOT have parked to ESTOP
assert int(hub._connection.mcu_state) != int(McuState.ESTOP)
state.disconnect()
print("DISCONNECT_OK")
'''


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="deft-robotics", password=PW, timeout=15)
    sftp = c.open_sftp()
    remote_py = f"{REMOTE}/scripts/_tmp_cursonier_dash_observe_prove_remote.py"
    with sftp.file(remote_py, "w") as f:
        f.write(PROOF)
    sftp.close()
    _, stdout, stderr = c.exec_command(
        f"cd {REMOTE} && PYTHONPATH=scripts python3 scripts/_tmp_cursonier_dash_observe_prove_remote.py",
        timeout=40,
    )
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    code = stdout.channel.recv_exit_status()
    print(out)
    if err.strip():
        print("STDERR:", err[-2000:])
    print("exit", code)
    c.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
