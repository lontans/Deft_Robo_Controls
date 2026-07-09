"""500 Hz plant teleop — actuator_commands[] on 562 B image (no RS2/DM PDU)."""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from controls_pcb_host import commands as cmd
from controls_pcb_host.actuator_config import ActuatorSlotConfig, slot_config
from controls_pcb_host.feedback import parse_actuator_feedback, parse_feedback_header
from controls_pcb_host.protocol import PLANT_MCU_STATE_NORMAL, PLANT_MCU_STATE_RECOVERY
from controls_pcb_host.protocol.can_bus import can_bus_label, is_mcp_bus
from controls_pcb_host.protocol.schema import MAX_CAN_BUS
from controls_pcb_host.session import PcbSession

from ..link import (
    PlantRuntimeError,
    assert_plant_teleop_slot,
    ensure_plant_runtime,
    heal_usb,
    release_stuck,
)
from . import defaults as D
from .input import poll_arrow_direction, poll_key_nonblocking

_live_len = 0
_link_ack: Optional[int] = None
_link_block: str = "?"
_link_tx_seq: Optional[int] = None
_dir_hold_s = 0.06
_latched_dir = 0
_dir_hold_until = 0.0


def _poll_motion_dir() -> int:
    """Arrow direction with short hold — avoids decay glitches on Windows."""
    global _latched_dir, _dir_hold_until
    raw = poll_arrow_direction()
    now = time.monotonic()
    if raw != 0:
        _latched_dir = raw
        _dir_hold_until = now + _dir_hold_s
        return raw
    if _latched_dir != 0 and now < _dir_hold_until:
        return _latched_dir
    _latched_dir = 0
    return 0


def _live(text: str) -> None:
    global _live_len
    line = text.replace("\n", " ")[:150]
    pad = max(0, _live_len - len(line))
    sys.stdout.write("\r" + line + " " * pad + "\x1b[K")
    sys.stdout.flush()
    _live_len = len(line)


def _notice(text: str) -> None:
    global _live_len
    sys.stdout.write("\n" + text + "\n")
    sys.stdout.flush()
    _live_len = 0


@dataclass
class SlotState:
    slot: int
    bus: int
    motor_id: int
    max_kp: float
    cmd_position: float = 0.0
    cmd_velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    fb_position: float = 0.0
    fb_velocity: float = 0.0
    fb_fault: int = 0
    feedback_synced: bool = False
    last_drive_dir: int = 0

    def label(self) -> str:
        return f"slot{self.slot} {can_bus_label(self.bus)} 0x{self.motor_id:02X} kp<={self.max_kp:.0f}"


def _sanitize_pos(pos: float, fallback: float) -> float:
    if abs(pos) <= D.SYNC_POS_MAX:
        return pos
    if abs(fallback) <= D.SYNC_POS_MAX:
        return fallback
    return 0.0


def _apply_fb(st: SlotState, fb: dict, *, sync_cmd: bool) -> None:
    pos = _sanitize_pos(float(fb["position"]), st.fb_position)
    st.fb_position = pos
    st.fb_velocity = fb["velocity"]
    st.fb_fault = int(fb.get("fault", 0))
    if sync_cmd or not st.feedback_synced:
        st.cmd_position = pos
        st.feedback_synced = True


def _poll_fb(session: PcbSession, slots: List[SlotState]) -> None:
    global _link_ack, _link_block
    latest: Optional[bytes] = None
    frame = session.reader.pop()
    while frame is not None:
        latest = frame
        frame = session.reader.pop()
    if latest is None:
        return
    hdr = parse_feedback_header(latest)
    if hdr is not None:
        _link_ack = int(hdr["last_cmd_seq"])
        _link_block = str(hdr.get("plant_block_name", "?"))
    for st in slots:
        fb = parse_actuator_feedback(latest, slot=st.slot)
        if fb is not None:
            _apply_fb(st, fb, sync_cmd=not st.feedback_synced)


def _send_slots(session: PcbSession, slots: List[SlotState], kd: float) -> None:
    global _link_tx_seq
    slot_commands = {}
    for st in slots:
        eff_kd = st.kd if (st.kd != 0.0 or st.kp == 0.0) else kd
        slot_commands[st.slot] = (st.cmd_position, st.cmd_velocity, st.kp, eff_kd, 0.0)
    _link_tx_seq = session.seq
    session.send_plant(slot_commands)


def _bus_label(active: int) -> str:
    if active == 0:
        return "ACTIVE_BUS=ALL"
    return f"ACTIVE_BUS={active} ({can_bus_label(active)})"


def _targets(slots: List[SlotState], active_bus: int) -> List[SlotState]:
    if active_bus == 0:
        return slots
    return [st for st in slots if st.bus == active_bus]


def _make_slots(slot_indices: List[int], slot_kp: Tuple[float, ...]) -> List[SlotState]:
    out: List[SlotState] = []
    for i in slot_indices:
        cfg = slot_config(i)
        kp = slot_kp[i] if i < len(slot_kp) else slot_kp[-1]
        out.append(SlotState(slot=i, bus=cfg.bus, motor_id=cfg.motor_id, max_kp=kp))
    return out


def _sync_feedback(
    session: PcbSession,
    slots: List[SlotState],
    *,
    hz: float,
    seconds: float,
) -> bool:
    """Stream plant commands until every slot has actuator_feedback."""
    dt = 1.0 / hz
    rounds = max(12, int(seconds * hz))
    for _ in range(rounds):
        session.send_plant()
        time.sleep(dt)
        _poll_fb(session, slots)
        if all(st.feedback_synced for st in slots):
            return True
    return any(st.feedback_synced for st in slots)


def _wake(session: PcbSession, slots: List[SlotState], kd: float, hz: float) -> None:
    dt = 1.0 / hz
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        for st in slots:
            st.cmd_velocity = 0.0
            st.kp = st.kd = 0.0
            if st.feedback_synced:
                st.cmd_position = st.fb_position
        _send_slots(session, slots, kd)
        _poll_fb(session, slots)
        time.sleep(dt)
    for st in slots:
        st.cmd_velocity = st.kp = st.kd = 0.0
        if st.feedback_synced:
            st.cmd_position = st.fb_position


def _anchor_cmd_from_fb(st: SlotState) -> None:
    if st.feedback_synced:
        st.cmd_position = st.fb_position


def _clamp_cmd_lead(st: SlotState) -> None:
    if not st.feedback_synced:
        return
    lead = st.cmd_position - st.fb_position
    if abs(lead) > D.MAX_CMD_LEAD:
        st.cmd_position = st.fb_position + math.copysign(D.MAX_CMD_LEAD, lead)


def _home(
    session: PcbSession,
    slots: List[SlotState],
    kd: float,
    hz: float,
    *,
    home_kp: float,
    home_slew: float,
    home_on_fb: bool,
    idle_kp: float,
) -> bool:
    dt = 1.0 / hz
    active = [st for st in slots if st.feedback_synced]
    if not active:
        print("Homing skipped: no feedback.")
        return False

    for st in active:
        _anchor_cmd_from_fb(st)
        st.cmd_velocity = 0.0
        st.last_drive_dir = 0

    if all(abs(st.fb_position - D.HOME_TARGET) <= D.HOME_POS_TOL for st in active):
        pos_s = ", ".join(f"{st.fb_position:+.4f}" for st in active)
        print(f"Already at home ({pos_s} rad) — short dwell, then teleop.")
        for st in active:
            st.cmd_position = D.HOME_TARGET
            st.kp = st.kd = 0.0
        for _ in range(max(4, int(hz * 0.5))):
            _send_slots(session, slots, kd)
            _poll_fb(session, slots)
            time.sleep(dt)
        print("Homing complete — arrow keys enabled.\n")
        return False

    start_pos = active[0].fb_position if len(active) == 1 else None
    if len(active) == 1:
        print(
            f"Homing from {start_pos:+.4f} rad → {D.HOME_TARGET:+.2f} rad "
            f"(slew {home_slew:.2f} rad/s, kp={home_kp:.1f}) — q aborts"
        )
    else:
        print(
            f"Homing to {D.HOME_TARGET:+.2f} rad (slew {home_slew:.2f} rad/s, kp={home_kp:.1f}) — q aborts"
        )
    deadline = time.monotonic() + D.HOME_TIMEOUT_S
    dwell = 0.0
    while time.monotonic() < deadline:
        if poll_key_nonblocking() == "q":
            print("Homing aborted.")
            return True
        slew_done = True
        for st in active:
            delta = D.HOME_TARGET - st.cmd_position
            step = home_slew * dt
            if abs(delta) <= step:
                st.cmd_position = D.HOME_TARGET
            else:
                st.cmd_position += math.copysign(step, delta)
                slew_done = False
            st.cmd_position = max(D.P_MIN, min(D.P_MAX, st.cmd_position))
            st.cmd_velocity = 0.0
            at_fb = abs(st.fb_position - D.HOME_TARGET) <= D.HOME_POS_TOL
            at_cmd = abs(st.cmd_position - D.HOME_TARGET) <= D.HOME_POS_TOL * 0.1
            at_target = at_fb if home_on_fb else at_cmd
            near = abs(st.fb_position - D.HOME_TARGET) < 0.12
            eff_kp = min(home_kp, 4.0) if near else home_kp
            st.kp = 0.0 if at_target else eff_kp
            st.kd = 0.0 if at_target else kd
            if not at_cmd or (home_on_fb and not at_fb):
                slew_done = False
        _send_slots(session, slots, kd)
        _poll_fb(session, slots)
        dwell = dwell + dt if slew_done else 0.0
        if slew_done and dwell >= D.HOME_DWELL_S:
            break
        time.sleep(dt)

    for st in slots:
        if st.feedback_synced:
            _anchor_cmd_from_fb(st)
        elif not home_on_fb and abs(st.fb_position) <= D.HOME_POS_TOL:
            st.cmd_position = D.HOME_TARGET
        st.cmd_velocity = 0.0
        st.last_drive_dir = 0
        st.kd = 0.0 if (idle_kp if home_on_fb else 0.0) == 0.0 else kd
        st.kp = idle_kp if home_on_fb else 0.0
    print("Homing complete — arrow keys enabled.\n")
    return False


def _shutdown(
    session: PcbSession,
    slots: List[SlotState],
    kd: float,
    hz: float,
    ramp_down_s: float,
    *,
    recovery: bool,
) -> None:
    dt = 1.0 / hz
    for st in slots:
        st.cmd_velocity = st.kp = st.kd = 0.0
        if st.feedback_synced:
            st.cmd_position = st.fb_position
    for _ in range(max(10, int(hz * ramp_down_s))):
        _send_slots(session, slots, kd)
        time.sleep(dt)
    if recovery:
        session.set_mcu_state(PLANT_MCU_STATE_RECOVERY)
        time.sleep(0.12)
    session.set_mcu_state(PLANT_MCU_STATE_NORMAL)
    heal_usb(session)
    session.reader.drain()
    time.sleep(0.15)


def _approach_velocity(
    cmd_vel: float,
    target: float,
    ramp_s: float,
    dt: float,
    *,
    cruise_speed: float,
) -> float:
    """Linear ramp to target, then lock — hold does not keep accelerating."""
    if abs(target - cmd_vel) < 1e-5:
        return target
    max_step = abs(cruise_speed) / max(ramp_s, 0.02) * dt
    diff = target - cmd_vel
    if abs(diff) <= max_step:
        return target
    return cmd_vel + math.copysign(max_step, diff)


def _decay_velocity(
    cmd_vel: float,
    ramp_down_s: float,
    dt: float,
    vel_stop: float,
    *,
    cruise_speed: float,
) -> float:
    """Linear coast to zero — symmetric with ramp-up."""
    if abs(cmd_vel) < vel_stop:
        return 0.0
    max_step = abs(cruise_speed) / max(ramp_down_s, 0.02) * dt
    if abs(cmd_vel) <= max_step:
        return 0.0
    return cmd_vel - math.copysign(max_step, cmd_vel)


def _update_slot_motion(
    st: SlotState,
    *,
    active: bool,
    motion_dir: int,
    arrow_vel: float,
    ramp_up_s: float,
    ramp_down_s: float,
    dt: float,
    vel_stop: float,
    kd: float,
    idle_kp: float,
) -> None:
    cruise = abs(arrow_vel)
    drive_dir = motion_dir if (active and motion_dir != 0) else 0

    if drive_dir != 0 and drive_dir != st.last_drive_dir:
        _anchor_cmd_from_fb(st)
        st.cmd_velocity = 0.0

    if active and motion_dir != 0:
        target = motion_dir * cruise
        st.cmd_velocity = _approach_velocity(
            st.cmd_velocity, target, ramp_up_s, dt, cruise_speed=cruise
        )
    else:
        st.cmd_velocity = _decay_velocity(
            st.cmd_velocity, ramp_down_s, dt, vel_stop, cruise_speed=cruise
        )

    if drive_dir != 0:
        st.last_drive_dir = drive_dir
    elif abs(st.cmd_velocity) < vel_stop:
        st.last_drive_dir = 0

    if not st.feedback_synced:
        st.kp = st.kd = 0.0
        return

    arrow_active = active and motion_dir != 0
    moving = abs(st.cmd_velocity) >= vel_stop
    if arrow_active or moving:
        st.kp = st.max_kp
        st.kd = kd
    elif idle_kp > 0.0:
        st.kp = idle_kp
        st.kd = kd
        _anchor_cmd_from_fb(st)
    else:
        st.kp = 0.0
        st.kd = 0.0
        _anchor_cmd_from_fb(st)


def _integrate_slot_position(
    st: SlotState,
    *,
    dt: float,
    vel_stop: float,
) -> None:
    if abs(st.cmd_velocity) < vel_stop:
        return
    st.cmd_position = max(
        D.P_MIN,
        min(D.P_MAX, st.cmd_position + st.cmd_velocity * dt),
    )


def run_plant_teleop(
    port: str,
    slots: List[int],
    *,
    hz: float = D.HZ,
    kd: float = D.KD,
    arrow_vel: float = D.ARROW_VEL,
    ramp_up_s: float = D.RAMP_UP_S,
    ramp_down_s: float = D.RAMP_DOWN_S,
    slot_kp: Tuple[float, ...] = D.SLOT_KP,
    skip_home: bool = False,
    home_kp: float = D.HOME_KP,
    home_slew: float = D.HOME_SLEW_RAD_S,
    recovery_on_exit: bool = False,
    teleop_title: str = "Plant teleop",
    show_dm_fault: bool = False,
    home_on_fb: bool = False,
    idle_kp: float = 0.0,
) -> None:
    slot_states = _make_slots(slots, slot_kp)
    active_bus = 0
    dt = 1.0 / hz

    with PcbSession(port) as session:
        with session.rx_pump():
            print("Preparing plant runtime...")
            try:
                ensure_plant_runtime(
                    session,
                    label="plant runtime",
                    bus=slot_states[0].bus if slot_states else None,
                )
            except PlantRuntimeError as exc:
                print(f"ERROR: {exc}")
                return

            for st in slot_states:
                if is_mcp_bus(st.bus):
                    print(f"  MCP {st.label()} — same plant 500 Hz path as FDCAN (reflash for SPI fixes)")

            print(f"{teleop_title} on {port} @ {hz:.0f} Hz")
            for st in slot_states:
                print(f"  {st.label()}")
            print(f"  Idle: kp=0 kd=0 (backdrivable).  Arrows: ±{arrow_vel:.1f} rad/s  0–{MAX_CAN_BUS}: bus  q: quit")
            print(
                f"  Hold arrow = cruise at ±{arrow_vel:.1f} rad/s "
                f"(ramp in {ramp_up_s:.2f}s on press, coast {ramp_down_s:.2f}s on release)"
            )
            print(
                f"  Tuning: kd={kd:.2f}  vel_stop={D.VEL_STOP:.2f}  "
                f"(CLI: --arrow-vel --ramp-up --ramp-down --kd --kp)"
            )
            print()

            sync_s = 3.0 if any(is_mcp_bus(st.bus) for st in slot_states) else 1.5
            if not _sync_feedback(session, slot_states, hz=hz, seconds=sync_s):
                print(
                    "  WARNING: no actuator_feedback yet — 500 Hz apply needs USB stream + reflash."
                )
                print("  Motion stays kp=0 until fb syncs (no wake jolt).")
            else:
                for st in slot_states:
                    if st.feedback_synced:
                        print(f"  synced {st.label()}  pos={st.cmd_position:+.4f} rad")
            print()

            _wake(session, slot_states, kd, hz)
            print("Wake done — idle kp=0 kd=0 (backdrivable).\n")

            if not skip_home:
                if _home(
                    session,
                    slot_states,
                    kd,
                    hz,
                    home_kp=home_kp,
                    home_slew=home_slew,
                    home_on_fb=home_on_fb,
                    idle_kp=idle_kp,
                ):
                    return

            fb_line = 0
            vel_stop = D.VEL_STOP
            try:
                while True:
                    quit_req = False
                    motion_dir = _poll_motion_dir()
                    while True:
                        key = poll_key_nonblocking(extra=D.BUS_KEYS)
                        if key is None:
                            break
                        if key == "q":
                            quit_req = True
                            break
                        if key == "0":
                            active_bus = 0
                            _notice(_bus_label(active_bus))
                        elif key in D.BUS_KEYS[1:]:
                            pick = int(key)
                            if any(st.bus == pick for st in slot_states):
                                active_bus = pick
                                _notice(_bus_label(active_bus))
                        elif key == "r":
                            session.reader.drain()
                            for st in slot_states:
                                st.cmd_velocity = 0.0
                                st.last_drive_dir = 0
                                _anchor_cmd_from_fb(st)
                            for _ in range(int(hz * 0.5)):
                                session.send_plant()
                                time.sleep(dt)
                                _poll_fb(session, slot_states)
                            _notice("re-synced from feedback")
                        elif key in ("left", "l"):
                            motion_dir = -1
                        elif key in ("right", "o"):
                            motion_dir = 1
                    if quit_req:
                        break

                    motion_dir = _poll_motion_dir()
                    motion_targets = _targets(slot_states, active_bus)
                    target_ids = {id(st) for st in motion_targets}
                    for st in slot_states:
                        _update_slot_motion(
                            st,
                            active=id(st) in target_ids,
                            motion_dir=motion_dir,
                            arrow_vel=arrow_vel,
                            ramp_up_s=ramp_up_s,
                            ramp_down_s=ramp_down_s,
                            dt=dt,
                            vel_stop=vel_stop,
                            kd=kd,
                            idle_kp=idle_kp,
                        )
                    for st in slot_states:
                        _integrate_slot_position(st, dt=dt, vel_stop=vel_stop)
                        _clamp_cmd_lead(st)

                    _send_slots(session, slot_states, kd)
                    _poll_fb(session, slot_states)

                    fb_line += 1
                    if fb_line % max(1, int(hz / 4)) == 0:
                        parts = [
                            _bus_label(active_bus),
                            f"tx={_link_tx_seq if _link_tx_seq is not None else '?'}",
                            f"ack={_link_ack if _link_ack is not None else '?'}",
                            f"block={_link_block}",
                        ]
                        for st in slot_states:
                            mark = "*" if id(st) in target_ids else " "
                            lead = st.cmd_position - st.fb_position
                            parts.append(
                                f"{mark}{st.slot}:v={st.cmd_velocity:+.1f} kp={st.kp:.0f} "
                                f"cmd={st.cmd_position:+.3f} fb={st.fb_position:+.3f} "
                                f"lead={lead:+.2f}"
                            )
                        _live("  ".join(parts))
                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\nStopping...")
            finally:
                _shutdown(
                    session,
                    slot_states,
                    kd,
                    hz,
                    ramp_down_s,
                    recovery=recovery_on_exit,
                )
    print("Done.")


def run_for_slot(
    port: str,
    slot: int,
    *,
    skip_home: bool = False,
    damiao: bool = False,
    hz: float = D.HZ,
    kd: float = D.KD,
    arrow_vel: float = D.ARROW_VEL,
    ramp_up_s: float = D.RAMP_UP_S,
    ramp_down_s: float = D.RAMP_DOWN_S,
    slot_kp: Optional[Tuple[float, ...]] = None,
    kp: Optional[float] = None,
    home_kp: float = D.HOME_KP,
    home_slew: float = D.HOME_SLEW_RAD_S,
) -> None:
    cfg = slot_config(slot)
    if not damiao and cfg.protocol_name == "robstride":
        assert_plant_teleop_slot(slot, cfg.bus, cfg.protocol_name)
    kp_table = list(slot_kp if slot_kp is not None else D.SLOT_KP)
    if kp is not None:
        while len(kp_table) <= slot:
            kp_table.append(kp_table[-1] if kp_table else 8.0)
        kp_table[slot] = kp
    kp_tuple = tuple(kp_table)
    if damiao or cfg.protocol_name == "damiao":
        run_plant_teleop(
            port,
            [D.DM_SLOT],
            kd=D.DM_KD,
            arrow_vel=D.DM_ARROW_VEL,
            slot_kp=(D.DM_KP,),
            skip_home=skip_home,
            home_kp=D.DM_HOME_KP,
            home_slew=D.DM_HOME_SLEW,
            recovery_on_exit=True,
            teleop_title="Damiao plant teleop",
            show_dm_fault=True,
            home_on_fb=True,
            idle_kp=D.DM_IDLE_KP,
            hz=hz,
            ramp_up_s=ramp_up_s,
            ramp_down_s=ramp_down_s,
        )
    else:
        run_plant_teleop(
            port,
            [slot],
            skip_home=skip_home,
            hz=hz,
            kd=kd,
            arrow_vel=arrow_vel,
            ramp_up_s=ramp_up_s,
            ramp_down_s=ramp_down_s,
            slot_kp=kp_tuple,
            home_kp=home_kp,
            home_slew=home_slew,
        )
