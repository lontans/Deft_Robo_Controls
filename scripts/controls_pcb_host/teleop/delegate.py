"""Teleop and calibrate — canonical implementations in control_hub."""
from __future__ import annotations

import sys

from ..actuator_config import slot_config
from .._bootstrap import ensure_scripts_path


def run_plant_teleop_for_slot(port: str, slot: int, *, skip_home: bool = False) -> None:
    ensure_scripts_path()
    from control_hub.teleop.plant import run_for_slot  # noqa: WPS433

    cfg = slot_config(slot)
    run_for_slot(port, slot, skip_home=skip_home, damiao=(cfg.protocol_name == "damiao"))


def run_servo_teleop(port: str) -> None:
    ensure_scripts_path()
    import dynamixel_teleop  # noqa: WPS433

    sys.argv = [sys.argv[0], "--port", port]
    dynamixel_teleop.main()


def run_calibrate(
    port: str,
    bus: int,
    motor_id: int,
    *,
    cal_timeout: float | None = None,
    strict_cali: bool = False,
) -> int:
    ensure_scripts_path()
    from control_hub.protocol.rs02 import DEFAULT_CAL_LISTEN_S  # noqa: WPS433
    from control_hub.rs02.calibrate import run_encoder_cal  # noqa: WPS433

    listen_s = cal_timeout if cal_timeout is not None else DEFAULT_CAL_LISTEN_S
    return run_encoder_cal(
        port,
        bus,
        motor_id,
        cal_listen_s=listen_s,
        strict_cali=strict_cali,
    )
