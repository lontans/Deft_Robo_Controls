#!/usr/bin/env python3
"""Stop continuous, blank CAN, calibrate bus5/6 RobStrides (shafts must be free)."""
from __future__ import annotations

import os
import time

import paramiko

HOST = "192.168.50.48"
USER = "deft-robotics"
PASS = os.environ.get("JETSON_PASS", "4565")
REMOTE = "/home/deft-robotics/controls_pcb/scripts"
LOCAL = os.path.dirname(__file__)

# (bus, motor_id, label)
RS_MOTORS = (
    (5, 0x70, "CH5 RS02"),
    (5, 0x74, "CH5 RS01"),
    (6, 0x75, "CH6 RS01"),
)


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=10)

    _, o, _ = c.exec_command(
        "pkill -9 -f yam_continuous_all.py; sleep 1; true"
    )
    o.channel.recv_exit_status()

    sftp = c.open_sftp()
    sftp.put(os.path.join(LOCAL, "_tmp_stop_can.py"), f"{REMOTE}/_tmp_stop_can.py")
    sftp.close()

    _, o, _ = c.exec_command(
        f"cd {REMOTE} && python3 -u _tmp_stop_can.py", timeout=30
    )
    print(o.read().decode("utf-8", "replace").encode("ascii", "replace").decode())

    # Keep PDU sim if present; don't kill dashboard (follow mode OK during cali gap).
    cali_py = r'''
import time
from deft_controls_sdk import ControlsPcbHub, McuState, ActuatorDesire
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta.cfg import pause_plant_stream

RS = [(5, 0x70, "CH5 RS02"), (5, 0x74, "CH5 RS01"), (6, 0x75, "CH6 RS01")]

port = find_cdc_port()
print("calibrate port", port, flush=True)
results = []
with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
    hub.recover()
    hub.set_rx_sim_mask(0)
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
    hub._connection.set_actuators(blank, send=True)
    time.sleep(0.2)

    # CFG enable the three RS slots so probes/cali resolve IDs.
    with pause_plant_stream(hub):
        for slot, bus, mid in ((22, 5, 0x70), (23, 5, 0x74), (24, 6, 0x75)):
            hub.debug.cfg_set_slot(
                slot=slot, bus=bus, protocol=1, motor_id=mid, master_id=0,
                enabled=True, persist=False,
            )
        # leave arm/Damiao off during cali
        for i in range(7):
            hub.debug.cfg_set_slot(
                slot=i, bus=1, protocol=3, motor_id=0x01+i, master_id=0x11+i,
                enabled=False, persist=False,
            )
        hub.debug.cfg_set_slot(
            slot=25, bus=6, protocol=3, motor_id=0x06, master_id=0x16,
            enabled=False, persist=False,
        )

    for bus, mid, label in RS:
        print(f"\n== CALI {label} bus={bus} id=0x{mid:02X} ==", flush=True)
        print("  shaft must spin freely", flush=True)
        hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
        try:
            ok = hub.debug.calibrate_robstride(
                bus=bus, motor_id=mid, cal_listen_s=28.0
            )
        except Exception as exc:
            print(f"  EXCEPTION {exc}", flush=True)
            ok = False
        print(f"  cali={'OK' if ok else 'FAIL'}", flush=True)
        # Re-enable after cali (drive left disabled by cali/reset).
        try:
            en = hub.debug.probe_robstride(bus=bus, motor_id=mid)
            print(
                f"  post-enable found={en.get('found') if en else None} "
                f"pos={en.get('position') if en else None}",
                flush=True,
            )
        except Exception as exc:
            print(f"  post-enable {exc}", flush=True)
        results.append((label, ok))
        time.sleep(0.4)

    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    hub._connection.set_actuators(blank, send=True)

print("\n== SUMMARY ==", flush=True)
for label, ok in results:
    print(f"  {label}: {'OK' if ok else 'FAIL'}", flush=True)
raise SystemExit(0 if all(ok for _, ok in results) else 2)
'''
    # Upload and run (cali can take ~2 min for 3 motors)
    sftp = c.open_sftp()
    with sftp.file(f"{REMOTE}/_tmp_run_base_cali.py", "w") as f:
        f.write(cali_py)
    sftp.close()

    print("Running RS cali on Jetson (shafts free, ~2 min)...", flush=True)
    _, o, e = c.exec_command(
        f"cd {REMOTE} && python3 -u _tmp_run_base_cali.py",
        timeout=240,
    )
    print(o.read().decode("utf-8", "replace").encode("ascii", "replace").decode())
    err = e.read().decode("utf-8", "replace")
    if err:
        print("STDERR", err.encode("ascii", "replace").decode()[:2000])
    c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
