"""Dynamixel present-position sampling helpers (host plant servo slots)."""
from __future__ import annotations

import time
from typing import Optional

from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.link import ActuatorDesire, ServoDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, parse_feedback_header, parse_servo_feedback


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _drain_all(hub: ControlsPcbHub):
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        yield frame


def sample_servo_fb(
    hub: ControlsPcbHub,
    slot: int,
    *,
    servo_id: int,
    hold_pos: Optional[int] = None,
    timeout_s: float = 3.0,
    hz: float = 40.0,
) -> Optional[int]:
    """Arm session and return present_position.

    Until FB is known, torque stays OFF so we never slew to a guessed mid.
    Once FB arrives, hold at that pose (or hold_pos if given) with torque ON.
    """
    dt = 1.0 / hz
    deadline = time.perf_counter() + timeout_s
    last: Optional[int] = None
    next_t = time.perf_counter()
    frames = 0
    while time.perf_counter() < deadline:
        if last is None:
            desire = ServoDesire(
                servo_id=servo_id,
                native_step_position=0,
                torque_enable=False,
                operating_mode=3,
            )
        else:
            goal = int(hold_pos) if hold_pos is not None else int(last)
            desire = ServoDesire(
                servo_id=servo_id,
                native_step_position=goal,
                torque_enable=True,
                operating_mode=3,
            )
        hub.set_servo(slot, desire, send=False)
        hub.set_servo(1 - slot, ServoDesire(servo_id=0), send=False)
        _conn(hub).set_actuators(
            {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}, send=False
        )
        _conn(hub).send_once()

        for raw in _drain_all(hub):
            hdr = parse_feedback_header(raw)
            if hdr is None or hdr.get("is_debug"):
                continue
            frames += 1
            sv = parse_servo_feedback(raw, slot)
            if sv is None:
                continue
            pos = int(sv["present_position"])
            mid = int(sv.get("motor_source_id", 0) or 0)
            if mid in (0, servo_id) or pos != 0:
                last = pos

        if last is not None and hold_pos is None and frames >= 8:
            hold_pos = last

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

        if last is not None and hold_pos is not None and frames >= 12:
            break

    if last is None:
        print(f"  arm: plant_fb_frames={frames} (no servo present_position yet)")
    return last
