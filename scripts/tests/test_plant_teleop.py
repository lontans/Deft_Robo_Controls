"""Plant teleop helpers — multi-slot / MCP control invariants."""
from __future__ import annotations

import math

from control_hub.teleop import defaults as D
from control_hub.teleop.plant import SlotState, _make_slots, _slot_home_on_fb, _update_slot_motion
from controls_pcb_host import commands as cmd
from controls_pcb_host.actuator_config import Protocol, default_table, host_table
from controls_pcb_host.actuator_config import _HOST_TABLE  # noqa: PLC2701


def test_slot_home_on_fb_fdcan_rs_uses_cmd() -> None:
    st = SlotState(slot=1, bus=2, motor_id=0x70, max_kp=8.0)
    _HOST_TABLE[1] = default_table()[1]
    _HOST_TABLE[1].bus = 2
    _HOST_TABLE[1].protocol = Protocol.ROBSTRIDE
    assert _slot_home_on_fb(st) is False


def test_slot_home_on_fb_damiao_uses_fb() -> None:
    st = SlotState(slot=0, bus=1, motor_id=0x06, max_kp=10.0)
    _HOST_TABLE[0] = default_table()[0]
    _HOST_TABLE[0].protocol = Protocol.DAMIAO
    assert _slot_home_on_fb(st) is True


def test_slot_home_on_fb_mcp_rs_uses_fb() -> None:
    st = SlotState(slot=3, bus=4, motor_id=0x73, max_kp=6.0)
    _HOST_TABLE[3] = default_table()[3]
    assert _slot_home_on_fb(st) is True


def test_make_slots_mcp_kp_uses_slot_index() -> None:
    slots = _make_slots([3, 4, 5], D.SLOT_KP)
    assert [s.max_kp for s in slots] == [6.0, 6.0, 6.0]
    assert slots[0].motor_id == 0x73  # CH4 after bus 4/5 swap
    assert slots[1].motor_id == 0x73  # CH5


def test_send_slots_keeps_mcp_non_blank_at_home() -> None:
    frame = cmd.build_plant_command(
        1,
        {3: (0.0, 0.0, 0.0, 0.0, 0.0)},
    )
    # Blank zero position — bad for MCP skip path.
    assert frame is not None
    patched = cmd.build_plant_command(
        1,
        {3: (D.HOME_POS_EPS, 0.0, 0.0, 0.0, 0.0)},
    )
    assert patched != frame


def test_inactive_slot_does_not_track_arrow() -> None:
    st = SlotState(slot=3, bus=4, motor_id=0x70, max_kp=6.0)
    st.feedback_synced = True
    st.cmd_position = st.fb_position = 0.5
    dt = 1.0 / 40.0
    for _ in range(20):
        _update_slot_motion(
            st,
            active=False,
            motion_dir=1,
            arrow_vel=D.ARROW_VEL,
            ramp_up_s=D.RAMP_UP_S,
            ramp_down_s=D.RAMP_DOWN_S,
            dt=dt,
            vel_stop=D.VEL_STOP,
            kd=D.KD,
            idle_kp=0.0,
        )
    assert st.kp == 0.0
    assert abs(st.slew_rate) < D.VEL_STOP
    assert math.isclose(st.cmd_position, 0.5, abs_tol=1e-4)
