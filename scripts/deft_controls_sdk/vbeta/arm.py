"""I2RTArmDriver-compatible Damiao arm over Controls PCB slots."""
from __future__ import annotations

import time
from typing import Optional, Sequence, Union

import numpy as np

from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.vbeta.session import PcbRobotSession
from deft_controls_sdk.vbeta.slots import (
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    arm_slots,
)
from deft_controls_sdk.vbeta.yam_limits import SOFT_MARGIN, clamp_q7


class RobotDeviceAlreadyConnectedError(RuntimeError):
    pass


class RobotDeviceNotConnectedError(RuntimeError):
    pass


class PcbArmDriver:
    """Mirror of deft_vbeta I2RTArmDriver for left/right YAM arms."""

    GRIPPER_MOTOR_NAME = "joint_6"

    def __init__(
        self,
        session: PcbRobotSession,
        *,
        side: str = "left",
        kp: Sequence[float] = DEFAULT_ARM_KP,
        kd: float = DEFAULT_ARM_KD,
        home_pose: Optional[np.ndarray] = None,
        sleep_pose: Optional[np.ndarray] = None,
        skip_home_on_connect: bool = True,
        clamp_goals: bool = True,
        soft_margin: float = SOFT_MARGIN,
    ) -> None:
        self._session = session
        self.side = side
        self.slots = arm_slots(side)
        self.kp = tuple(float(x) for x in kp)
        if len(self.kp) != 7:
            raise ValueError("kp must have length 7")
        self.kd = float(kd)
        self.is_connected = False
        self.clamp_goals = bool(clamp_goals)
        self.soft_margin = float(soft_margin)
        self.home_pose = (
            np.asarray(home_pose, dtype=np.float32)
            if home_pose is not None
            else np.array([0, np.pi / 4, np.pi / 4, 0, 0, 0, 0], dtype=np.float32)
        )
        self.sleep_pose = (
            np.asarray(sleep_pose, dtype=np.float32)
            if sleep_pose is not None
            else np.zeros(7, dtype=np.float32)
        )
        self.calibration_pose = np.array(
            [0, 3 * np.pi / 4, 3 * np.pi / 4, 0, 0, 0, 0], dtype=np.float32
        )
        self._skip_home = bool(skip_home_on_connect)
        self._zero_torque = False
        self._setpoint = np.zeros(7, dtype=np.float32)
        self.motors = {
            "joint_0": [1, "4340"],
            "joint_1": [2, "4340"],
            "joint_2": [3, "4340"],
            "joint_3": [4, "4310"],
            "joint_4": [5, "4310"],
            "joint_5": [6, "4310"],
            "joint_6": [7, "4310"],
        }

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def joint_motor_names(self) -> list[str]:
        return [n for n in self.motors if n != self.GRIPPER_MOTOR_NAME]

    @property
    def gripper_motor_names(self) -> list[str]:
        return [self.GRIPPER_MOTOR_NAME]

    @property
    def num_joints(self) -> int:
        return len(self.joint_motor_names)

    def connect(self) -> None:
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "PcbArmDriver already connected — do not call connect() twice."
            )
        self.is_connected = True
        self._zero_torque = False
        if not self._skip_home:
            self.go_to(self.home_pose, 2.0)
        else:
            # Hold current FB as setpoint so we do not snap to zero.
            q = self.read("Position_Rad")
            self._command_joint_pos(q, send=True)

    def reconnect(self) -> None:
        pass

    def _require(self) -> None:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "PcbArmDriver is not connected. Call connect() first."
            )

    def _fb_vectors(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        fb = self._session.latest_feedback()
        pos = np.zeros(7, dtype=np.float32)
        vel = np.zeros(7, dtype=np.float32)
        tau = np.zeros(7, dtype=np.float32)
        temp = np.zeros(7, dtype=np.float32)
        if fb is None:
            return pos, vel, tau, temp
        for i, slot in enumerate(self.slots):
            st = fb.actuator(slot)
            if st is None:
                continue
            pos[i] = float(st.position)
            vel[i] = float(st.velocity)
            tau[i] = float(st.torque)
            temp[i] = float(st.temperature)
        return pos, vel, tau, temp

    def read(self, data_name: str, motor_names: Union[str, list[str], None] = None):
        self._require()
        pos, vel, tau, temp = self._fb_vectors()
        if data_name == "Position_Rad":
            return pos
        if data_name == "velocity":
            return vel
        if data_name == "torque":
            return tau
        if data_name == "Position_Setpoint":
            return self._setpoint.copy()
        if data_name == "temperature":
            return temp
        if data_name == "gripper_index":
            return 6
        print(f"Data name: {data_name} is not supported for reading.")
        return None

    def read_all(self) -> dict[str, np.ndarray]:
        self._require()
        pos, vel, tau, _temp = self._fb_vectors()
        return {
            "joint_pos": pos[:6].astype(np.float32),
            "gripper_pos": np.asarray([pos[6]], dtype=np.float32),
            "joint_vel": vel[:6].astype(np.float32),
            "joint_torque": tau[:6].astype(np.float32),
        }

    def _desires_for(self, q: np.ndarray) -> dict[int, ActuatorDesire]:
        out: dict[int, ActuatorDesire] = {}
        if self._zero_torque:
            for slot in self.slots:
                out[slot] = ActuatorDesire()
            return out
        for i, slot in enumerate(self.slots):
            out[slot] = ActuatorDesire(
                position=float(q[i]),
                velocity=0.0,
                kp=float(self.kp[i]),
                kd=self.kd,
                torque=0.0,
            )
        return out

    def _command_joint_pos(self, values: np.ndarray, *, send: bool) -> None:
        q = np.asarray(values, dtype=np.float32).reshape(7)
        if self.clamp_goals:
            q = clamp_q7(q, self.side, margin=self.soft_margin)
        self._setpoint = q.copy()
        self._session.set_actuators(self._desires_for(q), send=send)

    def write(
        self,
        data_name: str,
        values: Union[bool, int, float, np.ndarray],
        motor_names: Union[str, list[str], None] = None,
    ) -> None:
        self._require()
        if data_name == "Goal_Position":
            self._command_joint_pos(np.asarray(values, dtype=np.float32), send=False)
            return
        if data_name == "Zero_Torque":
            self._zero_torque = bool(values)
            self._command_joint_pos(self._setpoint, send=False)
            return
        print(f"Data name: {data_name} value: {values} is not supported for writing.")

    def go_to(self, position: np.ndarray, time_interval: float = 2.0) -> None:
        self._require()
        target = np.asarray(position, dtype=np.float32).reshape(7)
        start = self.read("Position_Rad")
        dt = max(float(time_interval), 1e-3)
        t0 = time.perf_counter()
        while True:
            u = (time.perf_counter() - t0) / dt
            if u >= 1.0:
                break
            q = start + (target - start) * u
            self._command_joint_pos(q, send=False)
            time.sleep(0.01)
        self._command_joint_pos(target, send=False)

    def disconnect(self) -> None:
        self._require()
        self._zero_torque = False
        self.go_to(self.sleep_pose, 2.0)
        for slot in self.slots:
            self._session.set_actuator(slot, ActuatorDesire(), send=False)
        self.is_connected = False

    def __del__(self) -> None:
        if getattr(self, "is_connected", False):
            try:
                self.disconnect()
            except Exception:
                pass
