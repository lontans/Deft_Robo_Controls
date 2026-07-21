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
# Per-slot max kp (indexed by slot). Dual identical-arm bench, Jul 2026: arm1 = slots
# 0-6 (CH1, joints 1-7), arm2 = slots 7-13 (CH2, joints 8-14) — arm2 mirrors arm1's
# tuning exactly (same hardware), not yet independently bench-verified.
#
# J1-J4 (and mirrored J8-J11) are the load-bearing base/shoulder/elbow/forearm joints
# (DM-J4340, real headroom well above these values) — the old flat 12.0 for every slot
# was tuned for wrist joints and was not enough stiffness to move J2/J3/J4 against
# gravity/static load (bench-confirmed Jul 2026: J2/J4 needed ~40, J3 needed ~80 to
# track a jog at all). J5-J7 (and mirrored J12-J14, wrist/EE) were left at the original
# gentle 12.0 and turned out too soft once other joints stopped going fully limp when
# idle/braced (Jul 2026: braced/held joints now hold at full max_kp always, not a weak
# idle_kp — 12.0 wasn't firm enough for that duty). Bumped everything up with margin.
_ARM_KP = (40.0, 60.0, 90.0, 60.0, 25.0, 25.0, 20.0)
SLOT_KP: Tuple[float, ...] = _ARM_KP + _ARM_KP

HOME_TARGET = 0.0
HOME_SLEW_RAD_S = 0.18
HOME_KP = 6.0
HOME_POS_TOL = 0.05
HOME_VEL_TOL = 0.15
HOME_DWELL_S = 0.6
HOME_TIMEOUT_S = 120.0

P_MIN, P_MAX = -12.57, 12.57
SYNC_POS_MAX = 3.0
MAX_CMD_LEAD = 0.18  # was 0.35 — cut so cmd can't run far ahead of fb before capping;
# at higher SLOT_KP a big lead is a lot of stored torque, which read as "whipping"
# when it released (Jul 2026). Tighter cap makes cmd reassert onto fb sooner.
# Soft-stop only when MCU stops acking (USB/link dead) — not when position is flat.
# Flat fb at rest (or MCP synced on zeros) must not block arrow motion.
ACK_STALE_S = 0.50
# Reject absurd velocity samples (decode glitch / motor fault dump).
FB_VEL_ABS_MAX = 20.0
# Keep active teleop slot non-blank at home so MCP idle path still RX-polls SPI-CAN.
HOME_POS_EPS = 1e-6
# Brief GetAsyncKeyState dropouts while a key is held — keep cruise until release is confirmed.
CRUISE_DIR_HOLD_S = 0.15  # legacy alias; prefer RELEASE_CONFIRM_S
RELEASE_CONFIRM_S = 0.30

# Up/down: press once to slew toward ±P_MAX (mirrors homing; MCU interpolates @ 500 Hz).
EXTREMITY_HZ = 40.0
EXTREMITY_SLEW_RAD_S = 0.35
EXTREMITY_POS_TOL = 0.05

BUS_KEYS: Tuple[str, ...] = tuple(str(i) for i in range(MAX_CAN_BUS + 1))

# Legacy factory Damiao slot (CH3 0x06). Prefer the slot passed to teleop/--plant-slots.
DM_SLOT = 2
DM_KP = 12.0  # fallback only — per-slot gain now comes from SLOT_KP (see run_for_slot)
DM_KD = 1.0   # bumped from 0.5 -> 0.8 -> 1.0 to damp the higher SLOT_KP values
DM_ARROW_VEL = 0.12  # slow cruise on purpose: SLOT_KP now supplies the holding/torque
# authority, so velocity is the only lever left to give a human time to react before
# MAX_CMD_LEAD worth of error builds up. Went 3.0 -> 0.6 -> 0.25 -> 0.12 (Jul 2026,
# bench feedback: arms were still "whipping" at 0.25 combined with the wider old
# MAX_CMD_LEAD=0.35 — cut both together).
DM_HOME_KP = 8.0
DM_HOME_SLEW = 0.2
DM_IDLE_KP = 6.0
