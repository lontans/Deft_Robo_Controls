"""Delegate interactive teleop to proven legacy runners."""
from __future__ import annotations

import sys

from .._bootstrap import ensure_scripts_path
from ..actuator_config import slot_config


def run_plant_teleop_for_slot(port: str, slot: int, *, skip_home: bool = False) -> None:
    cfg = slot_config(slot)
    ensure_scripts_path()
    import host_teleop_laptop_usb as teleop  # noqa: WPS433

    if cfg.protocol_name == "damiao":
        argv = [sys.argv[0], "--port", port, "--damiao-teleop"]
    else:
        argv = [
            sys.argv[0],
            "--port",
            port,
            "--plant-teleop",
            "--plant-slots",
            str(slot),
        ]
    if skip_home:
        argv.append("--plant-skip-home")
    sys.argv = argv
    teleop.main()


def run_servo_teleop(port: str) -> None:
    ensure_scripts_path()
    import dynamixel_teleop  # noqa: WPS433

    sys.argv = [sys.argv[0], "--port", port]
    dynamixel_teleop.main()


def run_calibrate(port: str, slot: int) -> None:
    cfg = slot_config(slot)
    ensure_scripts_path()
    import host_teleop_laptop_usb as teleop  # noqa: WPS433

    sys.argv = [
        sys.argv[0],
        "--port",
        port,
        "--calibrate",
        "--bus",
        str(cfg.bus),
        "--motor-id",
        hex(cfg.motor_id),
    ]
    teleop.main()
