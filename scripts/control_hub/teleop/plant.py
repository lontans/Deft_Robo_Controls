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
from controls_pcb_host.protocol.can_bus import can_bus_label
from controls_pcb_host.protocol.schema import MAX_CAN_BUS
from controls_pcb_host.session import PcbSession

from ..link import heal_usb, release_stuck
from . import defaults as D
from .input import poll_arrow_direction, poll_key_nonblocking

_live_len = 0
_link_ack: Optional[int] = None
_link_block: str = "?"
_link_tx_seq: Optional[int] = None


def _live(text: str) -> None:
    global _live_len
    line = text.replace("\n", " ")[:118]
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


def _wake(session: PcbSession, slots: List[SlotState], kd: float, hz: float) -> None:
    dt = 1.0 / hz
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        for st in slots:
            if st.feedback_synced:
                st.cmd_velocity = 0.0
                st.kp = 4.0
                st.kd = min(kd, 0.3)
                st.cmd_position = st.fb_position
            else:
                st.kp = st.kd = 0.0
        _send_slots(session, slots, kd)
        _poll_fb(session, slots)
        time.sleep(dt)
    for st in slots:
        st.cmd_velocity = st.kp = st.kd = 0.0
        if st.feedback_synced:
            st.cmd_position = st.fb_position


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
            st.kp = 0.0 if at_target else home_kp
            st.kd = 0.0 if at_target else kd
            if not at_cmd or (home_on_fb and not at_fb):
                slew_done = False
        _send_slots(session, slots, kd)
        _poll_fb(session, slots)
        dwell = dwell + dt if slew_done else 0.0
        if slew_done and dwell >= D.HOME_DWELL_S:
            break
        time.sleep(dt)

    handoff = idle_kp if home_on_fb else 0.0
    for st in slots:
        st.cmd_position = st.fb_position if home_on_fb or abs(st.fb_position) > D.HOME_POS_TOL else D.HOME_TARGET
        st.cmd_velocity = st.kp = 0.0
        st.kd = 0.0 if handoff == 0.0 else kd
        st.kp = handoff
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
            print("Healing USB link...")
            if not heal_usb(session):
                release_stuck(session)
                heal_usb(session)

            print(f"{teleop_title} on {port} @ {hz:.0f} Hz")
            for st in slot_states:
                print(f"  {st.label()}")
            print(f"  Idle: kp=0 kd=0 (backdrivable).  Arrows: ±{arrow_vel:.1f} rad/s  0–{MAX_CAN_BUS}: bus  q: quit")
            print()

            for _ in range(max(4, int(hz * 0.5))):
                session.send_plant()
                time.sleep(dt)
                _poll_fb(session, slot_states)
            for st in slot_states:
                if st.feedback_synced:
                    print(f"  synced {st.label()}  pos={st.cmd_position:+.4f} rad")
            print()

            _wake(session, slot_states, kd, hz)
            print("Wake hold done — idle is kp=0 kd=0.\n")

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
            vel_stop = 0.12
            try:
                while True:
                    quit_req = False
                    motion_dir = poll_arrow_direction()
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

                    motion_targets = _targets(slot_states, active_bus)
                    target_ids = {id(st) for st in motion_targets}
                    for st in slot_states:
                        if id(st) not in target_ids:
                            st.cmd_velocity *= math.exp(-dt / max(ramp_down_s, 0.05))
                            if abs(st.cmd_velocity) < vel_stop:
                                st.cmd_velocity = 0.0
                            st.kp = st.kd = 0.0
                            if st.feedback_synced:
                                st.cmd_position = st.fb_position
                            continue
                        if motion_dir != 0:
                            tv = motion_dir * abs(arrow_vel)
                            alpha = 1.0 - math.exp(-dt / max(ramp_up_s, 0.05))
                            st.cmd_velocity += (tv - st.cmd_velocity) * alpha
                        else:
                            st.cmd_velocity *= math.exp(-dt / max(ramp_down_s, 0.05))
                        if abs(st.cmd_velocity) < vel_stop:
                            st.cmd_velocity = 0.0
                        if not st.feedback_synced:
                            st.kp = st.kd = 0.0
                        elif abs(st.cmd_velocity) < vel_stop:
                            st.kp = idle_kp if idle_kp > 0.0 else 0.0
                            st.kd = kd if idle_kp > 0.0 else 0.0
                            st.cmd_position = st.fb_position
                        else:
                            st.kp = st.max_kp
                            st.kd = kd

                    for st in slot_states:
                        if id(st) in target_ids and motion_dir != 0:
                            if abs(st.cmd_velocity) >= vel_stop:
                                st.cmd_position = max(
                                    D.P_MIN,
                                    min(D.P_MAX, st.cmd_position + st.cmd_velocity * dt),
                                )
                            else:
                                st.cmd_position = max(
                                    D.P_MIN,
                                    min(
                                        D.P_MAX,
                                        st.cmd_position + motion_dir * D.POS_STEP,
                                    ),
                                )
                            if st.feedback_synced:
                                lead = st.cmd_position - st.fb_position
                                if abs(lead) > D.MAX_CMD_LEAD:
                                    st.cmd_position = st.fb_position + math.copysign(
                                        D.MAX_CMD_LEAD, lead
                                    )

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
                            parts.append(
                                f"{mark}{st.slot}:v={st.cmd_velocity:+.1f} kp={st.kp:.0f} "
                                f"cmd={st.cmd_position:+.3f} fb={st.fb_position:+.3f}"
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
) -> None:
    cfg = slot_config(slot)
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
        )
    else:
        run_plant_teleop(port, [slot], skip_home=skip_home)
