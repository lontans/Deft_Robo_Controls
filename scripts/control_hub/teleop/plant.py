"""500 Hz plant teleop — actuator_commands[] on 562 B image (no RS2/DM PDU)."""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from controls_pcb_host import commands as cmd
from controls_pcb_host.actuator_config import ActuatorSlotConfig, slot_config
from controls_pcb_host.feedback import parse_actuator_feedback, parse_feedback_header
from controls_pcb_host.protocol import PLANT_MCU_STATE_NORMAL, PLANT_MCU_STATE_RECOVERY
from controls_pcb_host.protocol.can_bus import can_bus_label, is_mcp_bus, is_rs02_plant_bus
from controls_pcb_host.protocol.schema import MAX_CAN_BUS
from controls_pcb_host.session import PcbSession

from ..link import (
    PlantRuntimeError,
    assert_plant_teleop_slot,
    ensure_plant_runtime,
    heal_usb,
    release_stuck,
    warmup_plant_actuators,
)
from . import defaults as D
from .input import (
    poll_arrow_direction,
    poll_arrow_keys_raw,
    poll_key_nonblocking,
    poll_vertical_arrow_pressed,
)

_link_ack: Optional[int] = None
_link_block: str = "?"
_link_tx_seq: Optional[int] = None
_link_lap_ms: Optional[int] = None
_link_lap_max_ms: Optional[int] = None
_link_ticks_svc: Optional[int] = None
_link_ticks_pending: Optional[int] = None
_link_fb_seq: Optional[int] = None
_link_mcu_tick: Optional[int] = None
_link_ack_at: Optional[float] = None
_link_fb_at: Optional[float] = None
_link_last_ack: Optional[int] = None
_link_last_fb_pos: Optional[float] = None
_latched_dir = 0
_dir_release_at = 0.0


def _poll_motion_dir() -> int:
    """L/R cruise direction — latched until arrow released for RELEASE_CONFIRM_S."""
    global _latched_dir, _dir_release_at
    raw = poll_arrow_direction()
    now = time.monotonic()
    if raw != 0:
        _latched_dir = raw
        _dir_release_at = 0.0
        return raw
    if _latched_dir == 0:
        return 0
    if _dir_release_at == 0.0:
        _dir_release_at = now + D.RELEASE_CONFIRM_S
    if now < _dir_release_at:
        return _latched_dir
    _latched_dir = 0
    _dir_release_at = 0.0
    return 0


def _debug_log(*lines: str) -> None:
    for line in lines:
        print(line, flush=True)


def _notice(text: str) -> None:
    print(text, flush=True)


def _link_debug_line() -> str:
    parts = [
        f"tx={_link_tx_seq if _link_tx_seq is not None else '?'}",
        f"ack={_link_ack if _link_ack is not None else '?'}",
        f"block={_link_block}",
    ]
    if _link_lap_ms is not None:
        parts.append(f"lap={_link_lap_ms}ms")
    if _link_lap_max_ms is not None:
        parts.append(f"lap_max={_link_lap_max_ms}ms")
    if _link_ticks_svc is not None:
        parts.append(f"ptick={_link_ticks_svc}")
    if _link_ticks_pending is not None:
        parts.append(f"pend={_link_ticks_pending}")
    if _link_fb_seq is not None:
        parts.append(f"fb_seq={_link_fb_seq}")
    if _link_mcu_tick is not None:
        parts.append(f"mcu_tick={_link_mcu_tick}")
    if _link_ack_at is not None:
        parts.append(f"ack_age={int((time.monotonic() - _link_ack_at) * 1000)}ms")
    if _link_fb_at is not None:
        parts.append(f"fb_age={int((time.monotonic() - _link_fb_at) * 1000)}ms")
    return "link: " + "  ".join(parts)


def _slot_debug_line(
    st: SlotState,
    *,
    active: bool,
    extremity: bool,
    motion_dir: int = 0,
) -> str:
    mark = "*" if active else " "
    bus = can_bus_label(st.bus)
    if extremity:
        tgt_s = f"{st.goto_target:+.1f}" if st.goto_target is not None else "idle"
        return (
            f"  {mark}slot{st.slot} {bus} 0x{st.motor_id:02X}  "
            f"tgt={tgt_s}  v={st.cmd_velocity:+.2f}  kp={st.kp:.0f}  "
            f"cmd={st.cmd_position:+.4f}  fb={st.fb_position:+.4f}  "
            f"v_fb={st.fb_velocity:+.3f}  fault={st.fb_fault}"
        )
    lead = st.cmd_position - st.fb_position
    return (
        f"  {mark}slot{st.slot} {bus} 0x{st.motor_id:02X}  "
        f"dir={motion_dir:+d}  rate={st.slew_rate:+.2f}  kp={st.kp:.0f}  "
        f"cmd={st.cmd_position:+.4f}  fb={st.fb_position:+.4f}  "
        f"lead={lead:+.3f}  v_fb={st.fb_velocity:+.3f}  fault={st.fb_fault}"
    )


def _emit_teleop_debug(
    *,
    active_bus: int,
    slots: List[SlotState],
    target_ids: set,
    extremity: bool,
    motion_dir: int = 0,
) -> None:
    ts = time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
    _debug_log(
        f"--- teleop {ts}  {_bus_label(active_bus)} ---",
        _link_debug_line(),
        *[
            _slot_debug_line(st, active=id(st) in target_ids, extremity=extremity, motion_dir=motion_dir)
            for st in slots
        ],
        "",
    )


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
    fb_live: bool = False  # MCP: set once non-trivial pos/vel seen (zeros = no CAN RX)
    last_drive_dir: int = 0
    goto_target: Optional[float] = None
    slew_rate: float = 0.0  # internal ramp (rad/s); MIT wire uses v=0 like homing/extremity

    def label(self) -> str:
        return f"slot{self.slot} {can_bus_label(self.bus)} 0x{self.motor_id:02X} kp<={self.max_kp:.0f}"


def _sanitize_pos(pos: float, fallback: float) -> float:
    if abs(pos) <= D.SYNC_POS_MAX:
        return pos
    if abs(fallback) <= D.SYNC_POS_MAX:
        return fallback
    return 0.0


def _mcp_sample_live(pos: float, vel: float) -> bool:
    return abs(pos) >= 1e-4 or abs(vel) >= 1e-4


def _apply_fb(st: SlotState, fb: dict, *, sync_cmd: bool) -> None:
    pos = _sanitize_pos(float(fb["position"]), st.fb_position)
    vel = float(fb["velocity"])
    # Ignore glitch samples (e.g. v≈±25 after a hard MIT jolt) so teleop does not
    # keep commanding against a frozen/faulted readback.
    if abs(vel) > D.FB_VEL_ABS_MAX and st.feedback_synced:
        st.fb_fault = int(fb.get("fault", 0))
        return
    st.fb_position = pos
    st.fb_velocity = vel
    st.fb_fault = int(fb.get("fault", 0))
    if is_mcp_bus(st.bus):
        if _mcp_sample_live(pos, vel):
            st.fb_live = True
        elif not st.fb_live:
            # All-zero MCP slot = firmware got no CAN RX for this actuator_table ID.
            return
    if sync_cmd or not st.feedback_synced:
        st.cmd_position = pos
        st.feedback_synced = True


def _poll_fb(session: PcbSession, slots: List[SlotState]) -> None:
    global _link_ack, _link_block, _link_lap_ms, _link_lap_max_ms
    global _link_ticks_svc, _link_ticks_pending, _link_fb_seq, _link_mcu_tick
    global _link_ack_at, _link_fb_at, _link_last_ack, _link_last_fb_pos
    latest: Optional[bytes] = None
    frame = session.reader.pop()
    while frame is not None:
        latest = frame
        frame = session.reader.pop()
    if latest is None:
        return
    hdr = parse_feedback_header(latest)
    if hdr is not None:
        now = time.monotonic()
        ack = int(hdr["last_cmd_seq"])
        if _link_last_ack != ack:
            _link_last_ack = ack
            _link_ack_at = now
        _link_ack = ack
        _link_block = str(hdr.get("plant_block_name", "?"))
        lap = hdr.get("lap_ms")
        _link_lap_ms = int(lap) if lap is not None else None
        lap_max = hdr.get("lap_max_ms")
        _link_lap_max_ms = int(lap_max) if lap_max is not None else None
        ticks = hdr.get("ticks_svc")
        _link_ticks_svc = int(ticks) if ticks is not None else None
        pend = hdr.get("ticks_pending")
        _link_ticks_pending = int(pend) if pend is not None else None
        _link_fb_seq = int(hdr["fb_seq"])
        _link_mcu_tick = int(hdr["tick"])
    for st in slots:
        fb = parse_actuator_feedback(latest, slot=st.slot)
        if fb is not None:
            # Any fresh sample for this slot resets fb_age — not only when
            # position moves (flat rest / lagged MCP otherwise looks "stale").
            _link_last_fb_pos = float(fb["position"])
            _link_fb_at = time.monotonic()
            _apply_fb(st, fb, sync_cmd=not st.feedback_synced)


def _send_slots(session: PcbSession, slots: List[SlotState], kd: float) -> None:
    global _link_tx_seq
    slot_commands = {}
    for st in slots:
        eff_kd = st.kd if (st.kd != 0.0 or st.kp == 0.0) else kd
        # RS02 plant teleop: position slew + kp only (matches homing/extremity; MCU interpolates p).
        mit_v = 0.0
        pos = st.cmd_position
        # At exact home, keep the active slot non-blank so MCP firmware still RX-polls
        # (blank MCP slots are skipped entirely — no SPI TX/RX).
        if abs(pos) < D.HOME_POS_EPS:
            pos = D.HOME_POS_EPS
        slot_commands[st.slot] = (pos, mit_v, st.kp, eff_kd, 0.0)
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
        kp = slot_kp[i] if i < len(slot_kp) else (slot_kp[-1] if slot_kp else 6.0)
        out.append(SlotState(slot=i, bus=cfg.bus, motor_id=cfg.motor_id, max_kp=kp))
    return out


def _slots_for_teleop(
    session: PcbSession,
    slot_indices: List[int],
    slot_kp: Tuple[float, ...],
) -> List[SlotState]:
    from controls_pcb_host.actuator_config import refresh_host_table_from_mcu

    refresh_host_table_from_mcu(session)
    return _make_slots(slot_indices, slot_kp)


def _resync_from_feedback(
    session: PcbSession,
    slots: List[SlotState],
    kd: float,
    hz: float,
) -> None:
    """Re-anchor cmd from fb while keeping teleop slots non-blank (MCP needs HOME_POS_EPS)."""
    dt = 1.0 / hz
    for st in slots:
        st.slew_rate = 0.0
        st.cmd_velocity = 0.0
        st.last_drive_dir = 0
        _anchor_cmd_from_fb(st)
    for _ in range(int(hz * 0.5)):
        _send_slots(session, slots, kd)
        _poll_fb(session, slots)
        time.sleep(dt)


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
        # Keep MCP slots non-blank so firmware RX-polls SPI-CAN during sync.
        slot_commands = {}
        for st in slots:
            pos = st.cmd_position if abs(st.cmd_position) >= D.HOME_POS_EPS else D.HOME_POS_EPS
            slot_commands[st.slot] = (pos, 0.0, 0.0, 0.0, 0.0)
        session.send_plant(slot_commands)
        time.sleep(dt)
        _poll_fb(session, slots)
        if all(st.feedback_synced for st in slots):
            return True
    return all(st.feedback_synced for st in slots)


def _dead_mcp_slots(slots: List[SlotState]) -> List[SlotState]:
    """MCP slots with no live CAN feedback — usually motor_id mismatch vs MCU table."""
    return [
        st
        for st in slots
        if is_mcp_bus(st.bus) and not st.fb_live
    ]


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
    progress_at = time.monotonic()
    while time.monotonic() < deadline:
        if poll_key_nonblocking() == "q":
            for st in slots:
                st.cmd_velocity = st.kp = st.kd = 0.0
                if st.feedback_synced:
                    st.cmd_position = st.fb_position
            for _ in range(max(4, int(hz * 0.25))):
                _send_slots(session, slots, kd)
                time.sleep(dt)
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
        if time.monotonic() - progress_at >= 1.0:
            progress_at = time.monotonic()
            parts = []
            for st in active:
                parts.append(
                    f"{st.label()} cmd={st.cmd_position:+.3f} fb={st.fb_position:+.3f} kp={st.kp:.1f}"
                )
            print("  homing… " + "  ".join(parts))
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
    if home_on_fb and not all(
        abs(st.fb_position - D.HOME_TARGET) <= D.HOME_POS_TOL
        for st in active
    ):
        pos_s = ", ".join(f"{st.fb_position:+.4f}" for st in active)
        print(
            f"WARNING: homing timed out with fb still at {pos_s} rad "
            "(motor did not track cmd — check bus TX / recover / reflash)."
        )
        return True
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


def _exit_teleop(
    session: PcbSession,
    slots: List[SlotState],
    kd: float,
    hz: float,
    ramp_down_s: float,
    *,
    recovery: bool,
    reason: str = "",
) -> None:
    if reason:
        print(reason)
    print("Stopping plant stream (kp=0) and sending MCU recovery...")
    _shutdown(session, slots, kd, hz, ramp_down_s, recovery=recovery)
    print(
        "If PC2+PC3 blink together after this, power-cycle the board "
        "(MCU fault loop — not CAN activity on PC6/PC7)."
    )


def _advance_cmd_slew(st: SlotState, *, dt: float, max_lead: float) -> None:
    """Integrate cmd from previous cmd; freeze if lead vs fb would widen past max_lead.

    Do NOT rebase cmd onto fb each tick — MCP (and any lagged plant) can deliver
    stale/backward fb samples; rebasing causes visible position snaps.
    """
    if not st.feedback_synced or abs(st.slew_rate) < 1e-6:
        return
    proposed = st.cmd_position + st.slew_rate * dt
    proposed = max(D.P_MIN, min(D.P_MAX, proposed))
    lead = proposed - st.fb_position
    if abs(lead) > max_lead:
        cur_lead = st.cmd_position - st.fb_position
        # Only block advances that widen the gap; never teleport onto fb.
        if abs(lead) > abs(cur_lead) + 1e-9:
            return
    st.cmd_position = proposed


class _TeleopTrace:
    def __init__(self, path: str) -> None:
        self._path = path
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._file)
        self._w.writerow(
            [
                "t_s",
                "dir",
                "raw_l",
                "raw_r",
                "rate",
                "kp",
                "cmd",
                "fb",
                "lead",
                "d_fb",
                "block",
                "tx",
                "ack",
            ]
        )
        self._last_fb: Optional[float] = None
        self._t0 = time.monotonic()

    def row(
        self,
        *,
        motion_dir: int,
        raw_left: bool,
        raw_right: bool,
        st: SlotState,
        block: str,
        tx: Optional[int],
        ack: Optional[int],
    ) -> None:
        fb = st.fb_position
        d_fb = 0.0 if self._last_fb is None else fb - self._last_fb
        self._last_fb = fb
        self._w.writerow(
            [
                f"{time.monotonic() - self._t0:.4f}",
                motion_dir,
                int(raw_left),
                int(raw_right),
                f"{st.slew_rate:.3f}",
                f"{st.kp:.0f}",
                f"{st.cmd_position:.4f}",
                f"{fb:.4f}",
                f"{st.cmd_position - fb:.4f}",
                f"{d_fb:.4f}",
                block,
                tx if tx is not None else "",
                ack if ack is not None else "",
            ]
        )

    def close(self) -> None:
        self._file.close()


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
    """Hold-to-cruise via position slew (v=0 on wire) — same MIT path as homing/extremity."""
    cruise = abs(arrow_vel)
    drive_dir = motion_dir if (active and motion_dir != 0) else 0

    if drive_dir != 0 and drive_dir != st.last_drive_dir:
        # Re-anchor only when starting from rest. Mid-cruise direction flips keep
        # cmd continuous so a lagged fb sample cannot yank the target.
        if st.last_drive_dir == 0:
            _anchor_cmd_from_fb(st)
        st.slew_rate = 0.0

    if active and motion_dir != 0:
        target_rate = motion_dir * cruise
        st.slew_rate = _approach_velocity(
            st.slew_rate, target_rate, ramp_up_s, dt, cruise_speed=cruise
        )
    else:
        st.slew_rate = _decay_velocity(
            st.slew_rate, ramp_down_s, dt, vel_stop, cruise_speed=cruise
        )

    if drive_dir != 0:
        st.last_drive_dir = drive_dir
    elif abs(st.slew_rate) < vel_stop:
        st.last_drive_dir = 0

    st.cmd_velocity = st.slew_rate

    if not st.feedback_synced:
        st.kp = st.kd = 0.0
        return

    # Gate on MCU ack freshness only. Position-flat fb_age is normal at rest and
    # also happens when MCP synced on zeros before the first MIT reply.
    ack_stale = (
        _link_ack_at is None or (time.monotonic() - _link_ack_at) > D.ACK_STALE_S
    )
    if ack_stale:
        st.slew_rate = 0.0
        st.cmd_velocity = 0.0
        st.kp = 0.0
        st.kd = 0.0
        return

    arrow_active = active and motion_dir != 0
    moving = abs(st.slew_rate) >= vel_stop
    if moving:
        _advance_cmd_slew(st, dt=dt, max_lead=D.MAX_CMD_LEAD)

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
        # Soft catch-up only when nearly stopped and close — avoid hard snap onto
        # a lagged MCP fb sample at the end of a coast.
        if abs(st.cmd_position - st.fb_position) <= D.MAX_CMD_LEAD:
            _anchor_cmd_from_fb(st)


def _arm_goto_extreme(slots: List[SlotState], target: float) -> None:
    for st in slots:
        st.goto_target = target


def _update_slot_goto_slew(
    st: SlotState,
    *,
    slew_rate: float,
    dt: float,
    kd: float,
    pos_tol: float,
) -> None:
    """Slew toward latched goto_target — same MIT style as homing (v=0, kp tracks position)."""
    if not st.feedback_synced:
        st.kp = st.kd = 0.0
        st.cmd_velocity = 0.0
        return

    target = st.goto_target
    if target is None:
        st.cmd_velocity = 0.0
        st.kp = 0.0
        st.kd = 0.0
        return

    delta = target - st.cmd_position
    step = slew_rate * dt
    if abs(delta) <= step:
        st.cmd_position = target
        st.cmd_velocity = 0.0
        st.goto_target = None
        st.kp = 0.0
        st.kd = 0.0
        return

    st.cmd_position += math.copysign(step, delta)
    st.cmd_position = max(D.P_MIN, min(D.P_MAX, st.cmd_position))
    st.cmd_velocity = 0.0

    near = abs(st.fb_position - target) < max(0.12, pos_tol * 2.4)
    eff_kp = min(st.max_kp, 4.0) if near else st.max_kp
    st.kp = eff_kp
    st.kd = kd



def run_plant_extremity_teleop(
    port: str,
    slots: List[int],
    *,
    hz: float = D.EXTREMITY_HZ,
    kd: float = D.KD,
    slew_rate: float = D.EXTREMITY_SLEW_RAD_S,
    slot_kp: Tuple[float, ...] = D.SLOT_KP,
    skip_home: bool = False,
    home_kp: float = D.HOME_KP,
    home_slew: float = D.HOME_SLEW_RAD_S,
    pos_tol: float = D.EXTREMITY_POS_TOL,
) -> None:
    """Press up/down once: smooth move to +P_MAX / -P_MIN at slew_rate rad/s (RS02)."""
    active_bus = 0
    dt = 1.0 / hz

    with PcbSession(port) as session:
        with session.rx_pump():
            print("Preparing plant runtime...")
            try:
                ensure_plant_runtime(
                    session,
                    label="plant runtime",
                    bus=None,
                )
            except PlantRuntimeError as exc:
                print(f"ERROR: {exc}")
                return

            slot_states = _slots_for_teleop(session, slots, slot_kp)
            rs02 = [st for st in slot_states if is_rs02_plant_bus(st.bus)]
            if not rs02:
                print("ERROR: no RS02 plant slots in selection.")
                return

            print(f"RS02 extremity teleop on {port} @ {hz:.0f} Hz")
            for st in rs02:
                print(f"  {st.label()}")
            print(
                f"  Press Up once → go to +{D.P_MAX:.2f} rad   "
                f"Press Down once → go to {D.P_MIN:.2f} rad   "
                f"({slew_rate:.2f} rad/s homing-style slew)"
            )
            print("  Motion runs to limit after key release.  MCU interpolates p @ 500 Hz (reflash).")
            print(f"  0–{MAX_CAN_BUS}: bus filter   q: quit   r: re-sync fb")
            print()

            sync_s = 3.0 if any(is_mcp_bus(st.bus) for st in rs02) else 1.5
            if not _sync_feedback(session, slot_states, hz=hz, seconds=sync_s):
                print("  WARNING: no actuator_feedback yet — motion stays kp=0 until fb syncs.")
            else:
                for st in rs02:
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
                    home_on_fb=False,
                    idle_kp=0.0,
                ):
                    _exit_teleop(
                        session,
                        slot_states,
                        kd,
                        hz,
                        D.RAMP_DOWN_S,
                        recovery=True,
                        reason="",
                    )
                    return

            fb_line = 0
            print("Debug: scrolling status every ~250 ms (copy/paste friendly).\n")
            try:
                while True:
                    quit_req = False
                    press = poll_vertical_arrow_pressed()
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
                            if any(st.bus == pick for st in rs02):
                                active_bus = pick
                                _notice(_bus_label(active_bus))
                        elif key == "r":
                            session.reader.drain()
                            for st in slot_states:
                                st.cmd_velocity = 0.0
                                st.goto_target = None
                                _anchor_cmd_from_fb(st)
                            for _ in range(int(hz * 0.5)):
                                _send_slots(session, slot_states, kd)
                                time.sleep(dt)
                                _poll_fb(session, slot_states)
                            _notice("re-synced from feedback")
                        elif key == "up":
                            press = 1
                        elif key == "down":
                            press = -1
                    if quit_req:
                        break

                    motion_targets = _targets(rs02, active_bus)
                    if press > 0:
                        _arm_goto_extreme(motion_targets, D.P_MAX)
                        _notice(f"go → +{D.P_MAX:.2f} rad")
                    elif press < 0:
                        _arm_goto_extreme(motion_targets, D.P_MIN)
                        _notice(f"go → {D.P_MIN:.2f} rad")

                    for st in slot_states:
                        _update_slot_goto_slew(
                            st,
                            slew_rate=slew_rate,
                            dt=dt,
                            kd=kd,
                            pos_tol=pos_tol,
                        )

                    _send_slots(session, slot_states, kd)
                    _poll_fb(session, slot_states)

                    fb_line += 1
                    if fb_line % max(1, int(hz / 4)) == 0:
                        target_ids = {id(st) for st in motion_targets}
                        _emit_teleop_debug(
                            active_bus=active_bus,
                            slots=rs02,
                            target_ids=target_ids,
                            extremity=True,
                        )
                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\nStopping...")
            finally:
                _shutdown(
                    session,
                    slot_states,
                    kd,
                    hz,
                    D.RAMP_DOWN_S,
                    recovery=False,
                )
    print("Done.")


def run_extremity_for_slot(
    port: str,
    slot: int,
    *,
    skip_home: bool = False,
    hz: float = D.EXTREMITY_HZ,
    kd: float = D.KD,
    slew_rate: float = D.EXTREMITY_SLEW_RAD_S,
    slot_kp: Optional[Tuple[float, ...]] = None,
    kp: Optional[float] = None,
    home_kp: float = D.HOME_KP,
    home_slew: float = D.HOME_SLEW_RAD_S,
) -> None:
    cfg = slot_config(slot)
    if cfg.protocol_name != "robstride":
        raise PlantRuntimeError(
            f"slot {slot}: extremity teleop is RS02-only ({cfg.protocol_name} not supported yet)"
        )
    assert_plant_teleop_slot(slot, cfg.bus, cfg.protocol_name)
    kp_table = list(slot_kp if slot_kp is not None else D.SLOT_KP)
    if kp is not None:
        while len(kp_table) <= slot:
            kp_table.append(kp_table[-1] if kp_table else 8.0)
        kp_table[slot] = kp
    run_plant_extremity_teleop(
        port,
        [slot],
        skip_home=skip_home,
        hz=hz,
        kd=kd,
        slew_rate=slew_rate,
        slot_kp=tuple(kp_table),
        home_kp=home_kp,
        home_slew=home_slew,
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
    recovery_on_exit: bool = True,
    teleop_title: str = "Plant teleop",
    show_dm_fault: bool = False,
    home_on_fb: Optional[bool] = None,
    idle_kp: float = 0.0,
    debug_trace: Optional[str] = None,
    skip_warmup: bool = False,
) -> None:
    active_bus = 0
    dt = 1.0 / hz

    with PcbSession(port) as session:
        with session.rx_pump():
            print("Preparing plant runtime...")
            try:
                ensure_plant_runtime(
                    session,
                    label="plant runtime",
                    bus=None,
                )
            except PlantRuntimeError as exc:
                print(f"ERROR: {exc}")
                return

            slot_states = _slots_for_teleop(session, slots, slot_kp)
            for slot in slots:
                cfg = slot_config(slot)
                if cfg.protocol_name == "robstride":
                    assert_plant_teleop_slot(slot, cfg.bus, cfg.protocol_name)
            if home_on_fb is None:
                home_on_fb = any(is_mcp_bus(st.bus) for st in slot_states)

            if not skip_warmup:
                warmup_plant_actuators(session, slots)
                print()

            for st in slot_states:
                if is_mcp_bus(st.bus):
                    print(f"  MCP {st.label()} — same plant 500 Hz path as FDCAN (reflash for SPI fixes)")

            print(f"{teleop_title} on {port} @ {hz:.0f} Hz")
            for st in slot_states:
                print(f"  {st.label()}")
            print(f"  Idle: kp=0 kd=0 (backdrivable).  Arrows: ±{arrow_vel:.1f} rad/s  0–{MAX_CAN_BUS}: bus  q: quit")
            print(
                f"  Hold arrow = cruise at ±{arrow_vel:.1f} rad/s "
                f"(position slew, v=0 MIT — ramp in {ramp_up_s:.2f}s, coast {ramp_down_s:.2f}s)"
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
            dead = _dead_mcp_slots(slot_states)
            if dead:
                print()
                print("  WARNING: MCP slot(s) have fb=0 (no CAN RX on plant path):")
                for st in dead:
                    print(f"    {st.label()} — discover OK but teleop dead?")
                print(
                    "  Check `config show` motor_id matches discover per bus."
                )
                print(
                    "  Host: python scripts/control_hub.py config show --port COM5  "
                    "(CFG PDU; use config set --persist to save motor IDs in flash)"
                )
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
                    _exit_teleop(
                        session,
                        slot_states,
                        kd,
                        hz,
                        ramp_down_s,
                        recovery=recovery_on_exit,
                        reason="",
                    )
                    return

            fb_line = 0
            vel_stop = D.VEL_STOP
            trace = _TeleopTrace(debug_trace) if debug_trace else None
            trace_slot = slot_states[0].slot if len(slot_states) == 1 else None
            print("Debug: scrolling status every ~250 ms (copy/paste friendly).\n")
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
                            _resync_from_feedback(session, slot_states, kd, hz)
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

                    _send_slots(session, slot_states, kd)
                    _poll_fb(session, slot_states)

                    if trace is not None and trace_slot is not None:
                        raw_left, raw_right = poll_arrow_keys_raw()
                        for st in slot_states:
                            if st.slot == trace_slot:
                                trace.row(
                                    motion_dir=motion_dir,
                                    raw_left=raw_left,
                                    raw_right=raw_right,
                                    st=st,
                                    block=_link_block,
                                    tx=_link_tx_seq,
                                    ack=_link_ack,
                                )
                                break

                    fb_line += 1
                    if fb_line % max(1, int(hz / 4)) == 0:
                        _emit_teleop_debug(
                            active_bus=active_bus,
                            slots=slot_states,
                            target_ids=target_ids,
                            extremity=False,
                            motion_dir=motion_dir,
                        )
                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\nStopping...")
            finally:
                if trace is not None:
                    trace.close()
                    print(f"Debug trace written to {debug_trace}")
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
    debug_trace: Optional[str] = None,
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
            recovery_on_exit=True,
            home_on_fb=True,
            debug_trace=debug_trace,
        )
