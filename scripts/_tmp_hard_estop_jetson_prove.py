#!/usr/bin/env python3
"""Prove Jetson-driven hard ESTOP: hold pin16, watch Controls system.estop_sense on COM5.

Hard wire only — PDB UART soft-kill is irrelevant. Pass/fail = estop_sense tracks
Jetson drive. GPIO must stay held open on the Jetson for the whole sample window
(process exit releases the line and the net can float low again).
"""
from __future__ import annotations

import os
import struct
import sys
import threading
import time

import paramiko

from deft_controls_sdk import ControlsPcbHub, McuState
from deft_controls_sdk.link.exchange.wire_layout import SYSTEM_FB_OFF
from deft_controls_sdk.vbeta.leds import led_fault, led_solid_green

JETSON = "192.168.50.48"
USER = "deft-robotics"
REMOTE = "/home/deft-robotics/controls_pcb/scripts/jetson_estop_drive.py"
LOCAL = os.path.join(os.path.dirname(__file__), "jetson_estop_drive.py")


def _sys(raw: bytes) -> tuple[int, int, int]:
    return struct.unpack_from("<BBB", raw, SYSTEM_FB_OFF + 14)


def _sample(hub: ControlsPcbHub, seconds: float = 0.6) -> tuple[int, int, int]:
    last = (3, 5, 0)
    t0 = time.time()
    while time.time() - t0 < seconds:
        hub._connection.send_once()
        fb = hub._connection.poll_feedback()
        if fb is not None:
            last = _sys(fb.raw)
        time.sleep(0.02)
    return last


def main() -> int:
    pw = os.environ.get("JETSON_PASS", "")
    if not pw:
        print("set JETSON_PASS", file=sys.stderr)
        return 2

    print("COM5: Cursor")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(JETSON, username=USER, password=pw, timeout=15)
    sftp = c.open_sftp()
    sftp.put(LOCAL, REMOTE)
    sftp.chmod(REMOTE, 0o755)
    sftp.close()

    # Free anyone holding pin 16
    _, o, e = c.exec_command(
        "pkill -f pdb_uart_sim.py 2>/dev/null || true; "
        "pkill -f jetson_estop_sense.py 2>/dev/null || true; "
        "pkill -f jetson_estop_drive.py 2>/dev/null || true; sleep 0.3",
        timeout=10,
    )
    o.channel.recv_exit_status()

    def hold_drive(level: int, hold_s: float) -> paramiko.Channel:
        """Start a remote hold that keeps the GPIO line claimed for hold_s seconds."""
        # Inline so we never rely on a fire-and-forget that exits immediately.
        py = f"""
import time
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT, initial={level})
GPIO.output(16, {level})
print('holding pin16={level}', flush=True)
time.sleep({hold_s:.2f})
print('done', flush=True)
# leave driven until process exit (then OS releases line)
"""
        cmd = "python3 - <<'PY'\n" + py + "PY"
        print(f"JETSON>>> hold pin16={level} for {hold_s:.1f}s")
        transport = c.get_transport()
        assert transport is not None
        ch = transport.open_session()
        ch.exec_command(cmd)
        return ch

    def wait_hold_ready(ch: paramiko.Channel, timeout: float = 5.0) -> None:
        t0 = time.time()
        buf = b""
        while time.time() - t0 < timeout:
            if ch.recv_ready():
                buf += ch.recv(4096)
                if b"holding" in buf:
                    print(buf.decode("utf-8", "replace").rstrip())
                    return
            time.sleep(0.05)
        print("warn: no holding ack yet", buf.decode("utf-8", "replace"))

    results: list[str] = []
    with ControlsPcbHub.connect("COM5", persist_telemetry=False) as hub:
        hub.set_mcu_state(McuState.NORMAL, send=True)
        led_solid_green(hub)

        # --- RELEASE (HIGH) held ---
        ch = hold_drive(1, 4.0)
        wait_hold_ready(ch)
        time.sleep(0.2)
        ks, kr, es = _sample(hub, 1.0)
        print(f"while RELEASE held: kill={ks} reason={kr} estop_sense={es} (want 1)")
        results.append(f"release_sense={'PASS' if es == 1 else 'FAIL'}")
        # drain remote
        while not ch.exit_status_ready():
            time.sleep(0.1)
        ch.recv_exit_status()

        # --- ASSERT (LOW) held ---
        ch = hold_drive(0, 4.0)
        wait_hold_ready(ch)
        time.sleep(0.2)
        ks, kr, es = _sample(hub, 1.0)
        print(f"while ASSERT held:  kill={ks} reason={kr} estop_sense={es} (want 0)")
        if es == 0:
            led_fault(hub)
            for _ in range(15):
                hub.send_once()
                time.sleep(0.02)
        results.append(f"assert_sense={'PASS' if es == 0 else 'FAIL'}")
        while not ch.exit_status_ready():
            time.sleep(0.1)
        ch.recv_exit_status()

        # --- RELEASE again ---
        ch = hold_drive(1, 4.0)
        wait_hold_ready(ch)
        time.sleep(0.2)
        ks, kr, es = _sample(hub, 1.0)
        print(f"while RELEASE held: kill={ks} reason={kr} estop_sense={es} (want 1)")
        led_solid_green(hub)
        for _ in range(15):
            hub.send_once()
            time.sleep(0.02)
        results.append(f"rerelease_sense={'PASS' if es == 1 else 'FAIL'}")
        while not ch.exit_status_ready():
            time.sleep(0.1)
        ch.recv_exit_status()

        # Leave released briefly then exit (net may float after)
        ch = hold_drive(1, 1.0)
        wait_hold_ready(ch)
        while not ch.exit_status_ready():
            time.sleep(0.05)
        ch.recv_exit_status()

    c.close()
    print("--- hard ESTOP (Jetson GPIO drive → Controls PB7 / system.estop_sense) ---")
    for r in results:
        print(r)
    ok = all("PASS" in r for r in results)
    print("COM5: free")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
