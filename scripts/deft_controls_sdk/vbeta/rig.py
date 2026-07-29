"""Composable rig components for the single-Damiao-arm bench (product session).

Optional pieces that can sit *beside* one `PcbArmDriver` on the same
`PcbRobotSession` — RobStride soft-hold, neck DXL hold-at-present, LED idle,
PDU status. Each is a thin, independently-callable helper (no COM open, no
`send=True` — every write here is a held-desire update; the caller's existing
`send_once()` / streaming loop does the actual TX, same pattern as the rest
of `deft_controls_sdk.vbeta`).

`RigComponents` batches whichever pieces are attached behind one `.tick()`
so a smoke/session loop has one call to make per cycle instead of wiring
each helper by hand. Nothing here is required — the single Damiao arm works
standalone via `PcbArmDriver` alone; this module is additive.

Bring-up order (see docs/vbeta-pcb-adapter.md "Rig integration"): hold
baseline (arm only) -> add RobStride soft-hold -> neck DXL -> LED idle -> PDU
strip. One `PcbRobotSession` stays the sole COM owner throughout.

No HW proven from this module — offline scaffold only until the rig gate
opens (see docs/single-damiao-arm-jetson-rig.md).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.link.exchange import parse_servo_feedback
from deft_controls_sdk.pdb import PdbStatus
from deft_controls_sdk.vbeta.leds import led_idle
from deft_controls_sdk.vbeta.neck import PcbNeckDriver, steps_to_deg
from deft_controls_sdk.vbeta.session import PcbRobotSession
from deft_controls_sdk.vbeta.slots import DEFAULT_STEER_KD, DEFAULT_STEER_KP

# RS02 canonical bench slot for bus 6 under layout v3 — Track B bench notes
# (docs/legacy/bench/bench-pdb-plant-integ-2026-07-23.md: "RS slot=24 id=0x70"; was 23
# under v2). This slot is CFG-disabled/spare in yam_product_rows() — driving
# it requires a bench/rig CFG override, not the YAM product CFG.
RIG_RS02_BUS6_SLOT = 24
RIG_RS02_BUS6_MOTOR_ID = 0x70

# Conservative hold gains — reuse the base-steer defaults rather than invent
# new numbers; same "hold position, don't fight the operator" intent.
DEFAULT_RS02_HOLD_KP = DEFAULT_STEER_KP
DEFAULT_RS02_HOLD_KD = DEFAULT_STEER_KD


def robstride_soft_hold(
    session: PcbRobotSession,
    *,
    slot: int = RIG_RS02_BUS6_SLOT,
    position: Optional[float] = None,
    kp: float = DEFAULT_RS02_HOLD_KP,
    kd: float = DEFAULT_RS02_HOLD_KD,
    send: bool = False,
) -> ActuatorDesire:
    """Hold the rig's RobStride at ``position`` (present FB if omitted, else 0.0).

    Read-modify-write against the *current* FB every call — safe to call every
    tick even before any FB has arrived (holds 0.0 until real feedback shows
    up, matching the "hold present, don't snap" convention `PcbArmDriver`
    already uses via ``skip_home_on_connect``).
    """
    if position is None:
        fb = session.latest_feedback()
        state = fb.actuator(slot) if fb is not None else None
        position = float(state.position) if state is not None else 0.0
    desire = ActuatorDesire(position=float(position), velocity=0.0, kp=float(kp), kd=float(kd), torque=0.0)
    session.set_actuator(slot, desire, send=send)
    return desire


def neck_hold_present(
    session: PcbRobotSession,
    neck: PcbNeckDriver,
    *,
    send: bool = False,
) -> Optional[Tuple[float, float]]:
    """Hold the neck at its present pitch/yaw — no-op until the first FB frame.

    Reads native DXL steps via ``parse_servo_feedback`` (present_position),
    converts to degrees, and re-issues the same physical pose as the goal so
    the neck doesn't drift/relax rather than actively moving it anywhere.

    Present FB is the true physical angle (no offset baked in), but
    ``PcbNeckDriver.go_to`` adds its own ``pitch_offset_deg`` before commanding
    — same convention as the VR teleop path, where the offset is applied once
    at the call site (see the `PcbPlatformClient` neck_cmd parity note in
    docs/vbeta-pcb-adapter.md). Subtract it here so `go_to`'s internal +offset
    nets out to the same physical position, not present+offset.
    """
    fb = session.latest_feedback()
    if fb is None:
        return None
    pitch = parse_servo_feedback(fb.raw, 0)
    yaw = parse_servo_feedback(fb.raw, 1)
    if pitch is None or yaw is None:
        return None
    pitch_deg = steps_to_deg(pitch["present_position"])
    yaw_deg = steps_to_deg(yaw["present_position"])
    neck.go_to(pitch_deg - neck.pitch_offset_deg, yaw_deg)
    if send:
        session.send_once()
    return pitch_deg, yaw_deg


def pdb_poll(session: PcbRobotSession) -> Optional[PdbStatus]:
    """Typed PDU status from the shared session's hub (Track B API)."""
    return session.hub.pdb_status()


@dataclass
class RigTickResult:
    """What one `RigComponents.tick()` did — for smoke logging, not control flow."""

    soft_kill_parked: bool = False
    robstride_position: Optional[float] = None
    neck_pose_deg: Optional[Tuple[float, float]] = None
    pdb_status: Optional[PdbStatus] = None


@dataclass
class RigComponents:
    """Optional pieces attached beside one `PcbArmDriver` on a shared session.

    Every field defaults "off" — construct with only what the bench actually
    has wired. `tick()` is idempotent and side-effect-free on held desires
    that aren't attached (never touches slots/servos this instance doesn't own).
    """

    session: PcbRobotSession
    use_robstride: bool = False
    robstride_slot: int = RIG_RS02_BUS6_SLOT
    neck: Optional[PcbNeckDriver] = None
    use_led_idle: bool = False
    led_brightness: int = 12
    poll_pdb: bool = False
    _robstride_gains: Tuple[float, float] = field(
        default=(DEFAULT_RS02_HOLD_KP, DEFAULT_RS02_HOLD_KD), repr=False
    )

    def tick(self) -> RigTickResult:
        """Service soft-kill first — every other component skips this tick if
        parked, matching `PcbRobotSession.send_once()`'s own park-first order."""
        result = RigTickResult(soft_kill_parked=self.session.service_soft_kill())
        if result.soft_kill_parked:
            return result

        if self.use_robstride:
            kp, kd = self._robstride_gains
            desire = robstride_soft_hold(
                self.session, slot=self.robstride_slot, kp=kp, kd=kd
            )
            result.robstride_position = desire.position

        if self.neck is not None:
            result.neck_pose_deg = neck_hold_present(self.session, self.neck)

        if self.use_led_idle:
            led_idle(self.session, brightness=self.led_brightness, send=False)

        if self.poll_pdb:
            result.pdb_status = pdb_poll(self.session)

        return result
