"""Regression test for Connection's _state_lock/_write_lock (no hardware).

Before the controller dashboard, only one thread ever touched a Connection's
held desires. Now an HTTP handler thread (one per request) can call
set_actuator()/set_mcu_state() while the background streaming thread is
concurrently calling build_command()/send_once() — a real race that didn't
exist until this feature. Without the lock, set_actuators() mutating
_desires while another thread's build_command() does `dict(self._desires)`
can raise "dictionary changed size during iteration".
"""
from __future__ import annotations

import os
import sys
import threading

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.link import ActuatorDesire, Connection, McuState


def test_concurrent_actuator_writes_and_build_command_do_not_raise() -> None:
    conn = Connection("TESTPORT")
    errors: list[BaseException] = []
    stop = threading.Event()

    def writer(slot: int) -> None:
        try:
            for i in range(500):
                conn.set_actuator(slot, ActuatorDesire(position=float(i), kp=8.0, kd=0.4), send=False)
        except BaseException as exc:  # noqa: BLE001 — capture anything, including RuntimeError
            errors.append(exc)

    def builder() -> None:
        try:
            while not stop.is_set():
                conn.build_command()
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)

    build_thread = threading.Thread(target=builder)
    build_thread.start()
    writers = [threading.Thread(target=writer, args=(slot,)) for slot in range(14)]
    for t in writers:
        t.start()
    for t in writers:
        t.join()
    stop.set()
    build_thread.join(timeout=2.0)

    assert errors == []


def test_concurrent_mcu_state_transitions_do_not_raise() -> None:
    conn = Connection("TESTPORT")
    conn.set_actuator(0, ActuatorDesire(kp=8.0), send=False)
    errors: list[BaseException] = []
    stop = threading.Event()

    def flipper() -> None:
        try:
            for _ in range(300):
                conn.set_mcu_state(McuState.RECOVERY, send=False)
                conn.set_mcu_state(McuState.NORMAL, send=False)
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)

    def builder() -> None:
        try:
            while not stop.is_set():
                conn.build_command()
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)

    build_thread = threading.Thread(target=builder)
    build_thread.start()
    flip_thread = threading.Thread(target=flipper)
    flip_thread.start()
    flip_thread.join()
    stop.set()
    build_thread.join(timeout=2.0)

    assert errors == []
