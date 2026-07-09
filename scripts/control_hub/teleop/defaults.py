"""Plant teleop defaults — gentle gains for bench RS02 slots."""
from __future__ import annotations

from typing import Tuple

from controls_pcb_host.protocol.can_bus import MAX_CAN_BUS

HZ = 40.0
KD = 0.45
ARROW_VEL = 3.5
RAMP_UP_S = 0.12
RAMP_DOWN_S = 0.35
VEL_STOP = 0.05
SLOT_KP: Tuple[float, ...] = (10.0, 8.0, 8.0, 8.0, 8.0, 8.0)

HOME_TARGET = 0.0
HOME_SLEW_RAD_S = 0.18
HOME_KP = 6.0
HOME_POS_TOL = 0.05
HOME_VEL_TOL = 0.15
HOME_DWELL_S = 0.6
HOME_TIMEOUT_S = 120.0

P_MIN, P_MAX = -12.57, 12.57
SYNC_POS_MAX = 3.0
MAX_CMD_LEAD = 0.35

BUS_KEYS: Tuple[str, ...] = tuple(str(i) for i in range(MAX_CAN_BUS + 1))

# Damiao slot 2
DM_SLOT = 2
DM_KP = 25.0
DM_KD = 0.5
DM_ARROW_VEL = 3.0
DM_HOME_KP = 8.0
DM_HOME_SLEW = 0.2
DM_IDLE_KP = 6.0
