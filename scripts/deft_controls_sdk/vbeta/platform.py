"""FeatherPlatformClient-compatible base / neck / stub lift over PCB."""
from __future__ import annotations

import math
import time
from typing import Optional

from deft_controls_sdk.link import ActuatorDesire, ServoDesire
from deft_controls_sdk.vbeta.neck import deg_to_steps, steps_to_deg
from deft_controls_sdk.vbeta.session import PcbRobotSession
from deft_controls_sdk.vbeta.slots import (
    BASE_DRIVE_SLOTS,
    BASE_STEER_SLOTS,
    DEFAULT_DRIVE_KD,
    DEFAULT_STEER_KD,
    DEFAULT_STEER_KP,
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
)


class PcbPlatformClient:
    """In-process Feather-shaped platform client (no multiprocessing)."""

    def __init__(
        self,
        session: PcbRobotSession,
        *,
        use_neck: bool = False,
        watchdog_s: float = 1.0,
        steer_kp: float = DEFAULT_STEER_KP,
        steer_kd: float = DEFAULT_STEER_KD,
        drive_kd: float = DEFAULT_DRIVE_KD,
    ) -> None:
        self._session = session
        self._use_neck = bool(use_neck)
        self._watchdog_s = float(watchdog_s)
        self._steer_kp = float(steer_kp)
        self._steer_kd = float(steer_kd)
        self._drive_kd = float(drive_kd)
        self._connected = False
        self._last_cmd_t = 0.0
        self._drive_enabled = True
        self._targets = {
            "BwC": 0.0,
            "BwR": 0.0,
            "BwL": 0.0,
            "BpC": 0.0,
            "BpR": 0.0,
            "BpL": 0.0,
        }
        self._lift_vel_cmd = 0.0

    def connect(self, timeout_s: float = 150.0) -> None:
        del timeout_s  # no subprocess
        self._connected = True
        self._last_cmd_t = time.monotonic()
        self.heartbeat()

    def disconnect(self, timeout_s: float = 10.0) -> None:
        del timeout_s
        if not self._connected:
            return
        self.send_command(("shutdown",))
        self._connected = False

    def is_alive(self) -> bool:
        return self._connected

    def heartbeat(self) -> None:
        self.send_command(("heartbeat",))

    def send_target_state(
        self,
        turning_angles: dict[str, float],
        spinning_velocities: dict[str, float],
    ) -> None:
        self.send_command(
            (
                "base_target_cmd",
                turning_angles["BwC"],
                turning_angles["BwR"],
                turning_angles["BwL"],
                spinning_velocities["BpC"],
                spinning_velocities["BpR"],
                spinning_velocities["BpL"],
            )
        )

    def send_command(self, cmd: tuple) -> None:
        if not self._connected and cmd and cmd[0] != "shutdown":
            return
        self._last_cmd_t = time.monotonic()
        if not cmd:
            return
        kind = cmd[0]
        if kind == "heartbeat":
            self._service_watchdog(force=False)
            return
        if kind == "shutdown":
            self._stop_base()
            self._lift_vel_cmd = 0.0
            return
        if kind == "base_target_cmd":
            _, bwc, bwr, bwl, bpc, bpr, bpl = cmd
            self._apply_base_targets(bwc, bwr, bwl, bpc, bpr, bpl)
            return
        if kind == "base_cmd":
            # Planner path not ported — stop or approximate with zero-turn creep.
            _, turn_angle, linear, angular = cmd
            if abs(float(angular)) < 1e-9 and abs(float(linear)) < 1e-9:
                self._stop_base()
            else:
                # Hold steer at turn_angle (deg→rad) on all modules; drive ~0.
                # Full nav_planner stays host-side for a later pass.
                steer = math.radians(float(turn_angle))
                self._apply_base_targets(steer, steer, steer, 0.0, 0.0, 0.0)
            return
        if kind == "lift_cmd":
            # Stub: accept mm/s, do not enqueue plant.
            self._lift_vel_cmd = float(cmd[1]) if len(cmd) > 1 else 0.0
            return
        if kind == "neck_cmd" and self._use_neck:
            pitch_deg, yaw_deg = float(cmd[1]), float(cmd[2])
            self._apply_neck(pitch_deg, yaw_deg)
            return
        if kind == "enable_drive_current":
            self._drive_enabled = True
            self._reapply_targets()
            return
        if kind == "disable_drive_current":
            self._drive_enabled = False
            self._stop_drive_only()
            return

    def _apply_base_targets(
        self,
        bwc: float,
        bwr: float,
        bwl: float,
        bpc: float,
        bpr: float,
        bpl: float,
    ) -> None:
        self._targets.update(
            {
                "BwC": float(bwc),
                "BwR": float(bwr),
                "BwL": float(bwl),
                "BpC": float(bpc),
                "BpR": float(bpr),
                "BpL": float(bpl),
            }
        )
        self._reapply_targets()

    def _reapply_targets(self) -> None:
        desires: dict[int, ActuatorDesire] = {}
        for name, slot in BASE_STEER_SLOTS.items():
            desires[slot] = ActuatorDesire(
                position=self._targets[name],
                velocity=0.0,
                kp=self._steer_kp,
                kd=self._steer_kd,
            )
        for name, slot in BASE_DRIVE_SLOTS.items():
            if not self._drive_enabled:
                desires[slot] = ActuatorDesire()
            else:
                desires[slot] = ActuatorDesire(
                    position=0.0,
                    velocity=self._targets[name],
                    kp=0.0,
                    kd=self._drive_kd,
                )
        self._session.set_actuators(desires, send=False)

    def _stop_drive_only(self) -> None:
        desires = {
            slot: ActuatorDesire() for slot in BASE_DRIVE_SLOTS.values()
        }
        self._session.set_actuators(desires, send=False)

    def _stop_base(self) -> None:
        self._targets.update({"BpC": 0.0, "BpR": 0.0, "BpL": 0.0})
        desires = {slot: ActuatorDesire() for slot in BASE_STEER_SLOTS.values()}
        desires.update({slot: ActuatorDesire() for slot in BASE_DRIVE_SLOTS.values()})
        # Keep last steer pose with light kp so modules do not go limp blank.
        for name, slot in BASE_STEER_SLOTS.items():
            desires[slot] = ActuatorDesire(
                position=self._targets[name],
                kp=self._steer_kp,
                kd=self._steer_kd,
            )
        self._session.set_actuators(desires, send=False)

    def _apply_neck(self, pitch_deg: float, yaw_deg: float) -> None:
        # pitch_deg/yaw_deg arrive pre-offset: YAMAIMobile.convert_vr_head_angles
        # already bakes neck_pitch_offset_deg in before enqueueing "neck_cmd"
        # (ref: yam_ai_mobile.py teleop_neck/convert_vr_head_angles), and the
        # reference FeatherPlatformClient control loop forwards them raw
        # (neck.go_to(pitch, yaw), no second offset). Match that here.
        self._session.set_servo(
            NECK_PITCH_SERVO_SLOT,
            ServoDesire(
                servo_id=NECK_PITCH_DXL_ID,
                native_step_position=deg_to_steps(pitch_deg),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )
        self._session.set_servo(
            NECK_YAW_SERVO_SLOT,
            ServoDesire(
                servo_id=NECK_YAW_DXL_ID,
                native_step_position=deg_to_steps(yaw_deg),
                torque_enable=True,
                operating_mode=3,
            ),
            send=False,
        )

    def _service_watchdog(self, *, force: bool) -> None:
        if not self._connected:
            return
        if force or (time.monotonic() - self._last_cmd_t) >= self._watchdog_s:
            self._stop_base()
            self._lift_vel_cmd = 0.0

    def get_state(self) -> dict[str, float]:
        self._service_watchdog(force=False)
        fb = self._session.latest_feedback()
        state = {
            "bwc_angle": 0.0,
            "bwr_angle": 0.0,
            "bwl_angle": 0.0,
            "bpc_velocity": 0.0,
            "bpr_velocity": 0.0,
            "bpl_velocity": 0.0,
            "lift_velocity": 0.0,
            "lift_height": 0.0,
            "lift_unimplemented": 1.0,
            "bpc_temp": math.nan,
            "target_bwc_angle": self._targets["BwC"],
            "target_bwr_angle": self._targets["BwR"],
            "target_bwl_angle": self._targets["BwL"],
            "target_bpc_velocity": self._targets["BpC"],
            "target_bpr_velocity": self._targets["BpR"],
            "target_bpl_velocity": self._targets["BpL"],
            "battery_percent": 0.0,
            "estop_pressed": 0.0,
        }
        if fb is None:
            return state
        for slot, sk in (
            (BASE_STEER_SLOTS["BwC"], "bwc_angle"),
            (BASE_STEER_SLOTS["BwR"], "bwr_angle"),
            (BASE_STEER_SLOTS["BwL"], "bwl_angle"),
        ):
            st = fb.actuator(slot)
            if st is not None:
                state[sk] = float(st.position)
        for slot, sk, is_bpc in (
            (BASE_DRIVE_SLOTS["BpC"], "bpc_velocity", True),
            (BASE_DRIVE_SLOTS["BpR"], "bpr_velocity", False),
            (BASE_DRIVE_SLOTS["BpL"], "bpl_velocity", False),
        ):
            st = fb.actuator(slot)
            if st is not None:
                state[sk] = float(st.velocity)
                if is_bpc:
                    state["bpc_temp"] = float(st.temperature)
        return state
