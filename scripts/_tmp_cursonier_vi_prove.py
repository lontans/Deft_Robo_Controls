"""Prove FW V/I overlay: bad pack_v → USB SOFT_KILL_REQ/UV; stop sim → COMMS_LOSS.

Runs on Jetson (CDC owner). Restart paced pdb_uart_sim with controllable values.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(__file__))

from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.pdb import (
    KILL_HARD_ESTOP,
    KILL_NORMAL,
    KILL_REASON_COMMS_LOSS,
    KILL_REASON_OVERCURRENT,
    KILL_REASON_UNDERVOLTAGE,
    KILL_SOFT_REQ,
)

PORT = os.environ.get("CONTROLS_CDC", "/dev/ttyACM0")
SIM_PORT = "/dev/ttyTHS1"
PANEL = "http://127.0.0.1:8765"


def _http_json(method: str, path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        PANEL + path,
        data=data,
        method=method,
        headers={"Content-Type": "application/json"} if body is not None else {},
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode())


def _restart_sim(pack_v, pack_i, rail_v, rail_i) -> None:
    subprocess.run(["pkill", "-f", "pdb_uart_sim.py"], check=False)
    time.sleep(0.5)
    cmd = [
        "python3",
        "-u",
        "pdb_uart_sim.py",
        "--port",
        SIM_PORT,
        "--hz",
        "20",
        "--force-kill-state",
        "0",
        "--estop-sense",
        "1",
        "--pack-v",
        *[str(v) for v in pack_v],
        "--rail-v",
        *[str(v) for v in rail_v],
        "--pack-i",
        *[str(v) for v in pack_i],
        "--rail-i",
        *[str(v) for v in rail_i],
        "--contactor-state",
        "15",
        "--control-port",
        "8765",
    ]
    log = open("/tmp/pdb_uart_sim_vi.log", "w")
    subprocess.Popen(
        cmd,
        cwd=os.path.dirname(__file__),
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(1.5)


def _wait_kill(hub: ControlsPcbHub, want_state: int, want_reason: int | None, label: str, t=8.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < t:
        st = hub.pdb_status()
        last = (
            None if st is None else
            (st.kill_state, st.kill_reason, st.kill_state_name, st.kill_reason_name, st.pack_v_V)
        )
        if st is not None and st.kill_state == want_state and (
            want_reason is None or st.kill_reason == want_reason
        ):
            print(f"PASS {label}: {last}")
            return st
        time.sleep(0.05)
    print(f"FAIL {label}: last={last}")
    raise SystemExit(1)


def main() -> int:
    good_pack = (4800, 4800, 0, 0)
    good_rail = (4800, 1900, 1200, 500)
    good_pi = (180, 140, 0, 0)
    good_ri = (90, 70, 40, 25)

    print("=== restart sim NORMAL good V/I ===")
    _restart_sim(good_pack, good_pi, good_rail, good_ri)
    # Confirm sim is alive before opening CDC.
    time.sleep(0.5)
    alive = subprocess.run(
        ["pgrep", "-f", "pdb_uart_sim.py"],
        check=False,
        capture_output=True,
    )
    if alive.returncode != 0:
        print("FAIL: pdb_uart_sim did not stay up — check /tmp/pdb_uart_sim_vi.log")
        try:
            print(open("/tmp/pdb_uart_sim_vi.log").read()[-1500:])
        except OSError:
            pass
        return 2

    with ControlsPcbHub.connect(PORT, persist_telemetry=False) as hub:
        hub.start_streaming(hz=40.0)
        time.sleep(0.4)
        # Peer reason may be stale/other; only system kill_state must be NORMAL.
        _wait_kill(hub, KILL_NORMAL, None, "fresh good -> NORMAL")

        print("=== panel: pack_v undervoltage 39 V ===")
        try:
            _http_json(
                "POST",
                "/api/set",
                {
                    "pack_v": [3900, 4800, 0, 0],
                    "kill_state": 0,
                },
            )
        except Exception as exc:
            print("panel POST failed, restarting sim with bad V:", exc)
            _restart_sim((3900, 4800, 0, 0), good_pi, good_rail, good_ri)

        _wait_kill(
            hub,
            KILL_SOFT_REQ,
            KILL_REASON_UNDERVOLTAGE,
            "bad pack_v -> SOFT_KILL_REQ/UV",
        )

        print("=== panel: restore good V, inject OC ===")
        try:
            _http_json(
                "POST",
                "/api/set",
                {
                    "pack_v": [4800, 4800, 0, 0],
                    "pack_i": [3100, 0, 0, 0],
                    "kill_state": 0,
                },
            )
        except Exception:
            _restart_sim((4800, 4800, 0, 0), (3100, 0, 0, 0), good_rail, good_ri)

        _wait_kill(
            hub,
            KILL_SOFT_REQ,
            KILL_REASON_OVERCURRENT,
            "overcurrent -> SOFT_KILL_REQ/OC",
        )

        print("=== stop sim -> COMMS_LOSS ===")
        subprocess.run(["pkill", "-f", "pdb_uart_sim.py"], check=False)
        _wait_kill(
            hub,
            KILL_HARD_ESTOP,
            KILL_REASON_COMMS_LOSS,
            "stale -> HARD/COMMS_LOSS",
            t=3.0,
        )

    # Leave a healthy sim for Claudacious/Claudius
    _restart_sim(good_pack, good_pi, good_rail, good_ri)
    print("ALL_VI_PROVES_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
