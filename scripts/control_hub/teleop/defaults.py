"""Plant teleop defaults — gentle gains for bench RS02 slots."""
from __future__ import annotations

from typing import Tuple

from controls_pcb_host.protocol.can_bus import MAX_CAN_BUS

HZ = 40.0
KD = 0.5
ARROW_VEL = 5.0
RAMP_UP_S = 0.4
RAMP_DOWN_S = 1.2
SLOT_KP: Tuple[float, ...] = (12.0, 8.0, 8.0, 8.0, 8.0, 8.0)

HOME_TARGET = 0.0
HOME_SLEW_RAD_S = 0.20
HOME_KP = 6.0
POS_STEP = 0.02
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
