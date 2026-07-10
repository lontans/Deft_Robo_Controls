"""Teleop and calibrate — canonical implementations in control_hub."""
from __future__ import annotations

import sys

from ..actuator_config import Protocol, slot_config
from .._bootstrap import ensure_scripts_path


def parse_slot_list(text: str) -> list[int]:
    """Comma-separated slot indices, e.g. ``0,1`` or ``2``."""
    out: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part, 0))
    return out


def run_plant_extremity_teleop_for_slot(
    port: str,
    slot: int,
    *,
    skip_home: bool = False,
    hz: float | None = None,
    kd: float | None = None,
    slew_rate: float | None = None,
    kp: float | None = None,
    home_kp: float | None = None,
    home_slew: float | None = None,
) -> None:
    ensure_scripts_path()
    from control_hub.teleop import defaults as D  # noqa: WPS433
    from control_hub.teleop.plant import run_extremity_for_slot  # noqa: WPS433

    run_extremity_for_slot(
        port,
        slot,
        skip_home=skip_home,
        hz=hz if hz is not None else D.EXTREMITY_HZ,
        kd=kd if kd is not None else D.KD,
        slew_rate=slew_rate if slew_rate is not None else D.EXTREMITY_SLEW_RAD_S,
        kp=kp,
        home_kp=home_kp if home_kp is not None else D.HOME_KP,
        home_slew=home_slew if home_slew is not None else D.HOME_SLEW_RAD_S,
    )


def run_plant_teleop_for_slot(
    port: str,
    slot: int,
    *,
    skip_home: bool = False,
    hz: float | None = None,
    kd: float | None = None,
    arrow_vel: float | None = None,
    ramp_up_s: float | None = None,
    ramp_down_s: float | None = None,
    kp: float | None = None,
    home_kp: float | None = None,
    home_slew: float | None = None,
    debug_trace: str | None = None,
) -> None:
    ensure_scripts_path()
    from control_hub.teleop import defaults as D  # noqa: WPS433
    from control_hub.teleop.plant import run_for_slot  # noqa: WPS433

    cfg = slot_config(slot)
    run_for_slot(
        port,
        slot,
        skip_home=skip_home,
        damiao=(cfg.protocol_name == "damiao"),
        hz=hz if hz is not None else D.HZ,
        kd=kd if kd is not None else D.KD,
        arrow_vel=arrow_vel if arrow_vel is not None else D.ARROW_VEL,
        ramp_up_s=ramp_up_s if ramp_up_s is not None else D.RAMP_UP_S,
        ramp_down_s=ramp_down_s if ramp_down_s is not None else D.RAMP_DOWN_S,
        kp=kp,
        home_kp=home_kp if home_kp is not None else D.HOME_KP,
        home_slew=home_slew if home_slew is not None else D.HOME_SLEW_RAD_S,
        debug_trace=debug_trace,
    )


def run_plant_teleop_for_slots(
    port: str,
    slots: list[int],
    *,
    skip_home: bool = False,
    hz: float | None = None,
    kd: float | None = None,
    arrow_vel: float | None = None,
    ramp_up_s: float | None = None,
    ramp_down_s: float | None = None,
    kp: float | None = None,
    home_kp: float | None = None,
    home_slew: float | None = None,
    debug_trace: str | None = None,
    damiao_only: bool = False,
) -> None:
    if not slots:
        raise ValueError("plant teleop requires at least one slot index")
    if len(slots) == 1 and not damiao_only:
        run_plant_teleop_for_slot(
            port,
            slots[0],
            skip_home=skip_home,
            hz=hz,
            kd=kd,
            arrow_vel=arrow_vel,
            ramp_up_s=ramp_up_s,
            ramp_down_s=ramp_down_s,
            kp=kp,
            home_kp=home_kp,
            home_slew=home_slew,
            debug_trace=debug_trace,
        )
        return

    ensure_scripts_path()
    from control_hub.teleop import defaults as D  # noqa: WPS433
    from control_hub.teleop.plant import run_plant_teleop  # noqa: WPS433

    if damiao_only or (
        len(slots) == 1 and slot_config(slots[0]).protocol == Protocol.DAMIAO
    ):
        dm_slot = slots[0]
        dm_kp = kp if kp is not None else (
            D.SLOT_KP[dm_slot] if dm_slot < len(D.SLOT_KP) else D.DM_KP
        )
        # slot_kp is indexed by slot number in _make_slots
        dm_kp_table = [dm_kp] * (dm_slot + 1)
        run_plant_teleop(
            port,
            [dm_slot],
            hz=hz if hz is not None else D.HZ,
            kd=kd if kd is not None else D.DM_KD,
            arrow_vel=arrow_vel if arrow_vel is not None else D.DM_ARROW_VEL,
            ramp_up_s=ramp_up_s if ramp_up_s is not None else D.RAMP_UP_S,
            ramp_down_s=ramp_down_s if ramp_down_s is not None else D.RAMP_DOWN_S,
            slot_kp=tuple(dm_kp_table),
            skip_home=skip_home,
            home_kp=home_kp if home_kp is not None else D.DM_HOME_KP,
            home_slew=home_slew if home_slew is not None else D.DM_HOME_SLEW,
            recovery_on_exit=True,
            teleop_title="Damiao plant teleop",
            show_dm_fault=True,
            home_on_fb=True,
            idle_kp=D.DM_IDLE_KP,
        )
        return

    kp_table = list(D.SLOT_KP)
    if kp is not None:
        for slot in slots:
            while len(kp_table) <= slot:
                kp_table.append(kp_table[-1] if kp_table else 8.0)
            kp_table[slot] = kp
    run_plant_teleop(
        port,
        slots,
        skip_home=skip_home,
        hz=hz if hz is not None else D.HZ,
        kd=kd if kd is not None else D.KD,
        arrow_vel=arrow_vel if arrow_vel is not None else D.ARROW_VEL,
        ramp_up_s=ramp_up_s if ramp_up_s is not None else D.RAMP_UP_S,
        ramp_down_s=ramp_down_s if ramp_down_s is not None else D.RAMP_DOWN_S,
        slot_kp=tuple(kp_table),
        home_kp=home_kp if home_kp is not None else D.HOME_KP,
        home_slew=home_slew if home_slew is not None else D.HOME_SLEW_RAD_S,
        recovery_on_exit=True,
        debug_trace=debug_trace,
    )


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
