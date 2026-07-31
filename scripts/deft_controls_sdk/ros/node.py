"""ControlsPcbHostNode — ROS 2 node adapter wrapping a single HostProxy.

    ROS topics/services -> ControlsPcbHostNode -> HostProxy -> Hub -> USB CDC

One process, one COM: this node constructs (or wraps, for tests) exactly one
``HostProxy`` and never a per-peripheral one. Commands go through the same
``actions.ActuatorAction`` / ``LedAction`` / ``ServoAction`` every other host
app uses — no Damiao/RobStride motion drivers live here.

Debug RPC (CFG, discover, cal) is intentionally not exposed: this node
defaults to ``mode="bandwidth"`` and refuses ``mode="soft_dfu"``. Use
``mode="debug"`` via ``pcb_lab`` / ``hub.debug`` for CFG work.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from deft_controls_sdk.config import (
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
    bench_continuous_profile,
    yam_product_profile,
)
from deft_controls_sdk.host_proxy import HostProxy

from . import bridges, topics

_DEFAULT_COMPONENTS: Sequence[str] = ("left_arm", "right_arm", "base", "lift")
_PROFILE_BUILDERS = {
    "product": yam_product_profile,
    "bench": bench_continuous_profile,
}


class ControlsPcbHostNode(Node):
    """Teleop node: per-component JointState feedback + position commands."""

    def __init__(
        self,
        *,
        proxy: Optional[HostProxy] = None,
        node_name: str = "controls_pcb_host",
        port: Optional[str] = None,
        profile: Optional[str] = None,
        stream_hz: Optional[float] = None,
        listen_pdu: Optional[bool] = None,
        mode: Optional[str] = None,
        components: Optional[Sequence[str]] = None,
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter("port", port or "")
        self.declare_parameter("profile", profile or "product")
        self.declare_parameter(
            "stream_hz", float(stream_hz) if stream_hz is not None else 200.0
        )
        self.declare_parameter(
            "listen_pdu", bool(listen_pdu) if listen_pdu is not None else False
        )
        self.declare_parameter("mode", mode or "bandwidth")
        # Comma-joined string, not a string-array parameter: rclpy can't infer
        # an array parameter's type from an empty default list, and an empty
        # allow-list (all actuator topics disabled) is a legitimate value.
        self.declare_parameter(
            "components",
            ",".join(components) if components is not None else ",".join(_DEFAULT_COMPONENTS),
        )

        resolved_mode = str(self.get_parameter("mode").value)
        if resolved_mode == "soft_dfu":
            raise ValueError(
                "mode=soft_dfu is not valid on the teleop node; use "
                "pcb_lab flash / soft_dfu_flash.py instead"
            )

        self._owns_proxy = proxy is None
        if proxy is not None:
            self._proxy = proxy
        else:
            profile_name = str(self.get_parameter("profile").value)
            try:
                profile_obj = _PROFILE_BUILDERS[profile_name]()
            except KeyError:
                known = ", ".join(sorted(_PROFILE_BUILDERS))
                raise ValueError(f"unknown profile {profile_name!r}; known: {known}") from None
            resolved_port = str(self.get_parameter("port").value) or None
            self._proxy = HostProxy.connect(
                resolved_port,
                profile=profile_obj,
                stream_hz=float(self.get_parameter("stream_hz").value),
                listen_pdu=bool(self.get_parameter("listen_pdu").value),
                mode=resolved_mode,
            )

        requested = [
            c.strip() for c in str(self.get_parameter("components").value).split(",") if c.strip()
        ]
        available = set(self._proxy.profile.components)
        self._components: List[str] = [c for c in requested if c in available]
        for name in requested:
            if name not in available:
                self.get_logger().warning(
                    f"component {name!r} not in profile {self._proxy.profile.name!r}; skipping"
                )

        self._state_pubs: Dict[str, object] = {}
        self._command_subs: List[object] = []
        for name in self._components:
            self._state_pubs[name] = self.create_publisher(
                JointState, topics.state_topic(name), topics.state_qos()
            )
            self._command_subs.append(
                self.create_subscription(
                    Float64MultiArray,
                    topics.command_topic(name),
                    self._make_actuator_command_cb(name),
                    topics.command_qos(),
                )
            )

        self._led_sub = self.create_subscription(
            String, topics.LED_COMMAND_TOPIC, self._on_led_command, topics.command_qos()
        )
        self._servo_sub = self.create_subscription(
            JointState,
            topics.SERVO_NECK_COMMAND_TOPIC,
            self._on_servo_command,
            topics.command_qos(),
        )

        stream_hz = float(self.get_parameter("stream_hz").value)
        timer_hz = min(stream_hz, 100.0) if stream_hz > 0 else 50.0
        self._timer = self.create_timer(1.0 / timer_hz, self._on_timer)

    @property
    def proxy(self) -> HostProxy:
        return self._proxy

    def _make_actuator_command_cb(self, name: str):
        def _cb(msg) -> None:
            try:
                positions = bridges.positions_from_command(msg)
            except TypeError as exc:
                self.get_logger().error(f"{name}: {exc}")
                return
            try:
                self._proxy.actuators(name).hold(positions, send=False)
            except ValueError as exc:
                self.get_logger().error(f"{name}: {exc}")

        return _cb

    def _on_led_command(self, msg: String) -> None:
        preset = bridges.led_command_from_msg(msg)
        try:
            self._proxy.led().apply_preset(preset, send=False)
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def _on_servo_command(self, msg: JointState) -> None:
        try:
            pitch, yaw = bridges.neck_positions_from_joint_state(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        servo = self._proxy.servo()
        servo.set(NECK_PITCH_SERVO_SLOT, servo_id=NECK_PITCH_DXL_ID, position=pitch, send=False)
        servo.set(NECK_YAW_SERVO_SLOT, servo_id=NECK_YAW_DXL_ID, position=yaw, send=False)

    def _on_timer(self) -> None:
        # HostProxy already owns a background plant-TX thread at stream_hz
        # (started inside HostProxy.connect / hub.start_streaming) — desires
        # set with send=False above ride that stream. Calling send_once()
        # here too would double-send outside that cadence, so this timer
        # only publishes feedback.
        stamp = self.get_clock().now().to_msg()
        for name, pub in self._state_pubs.items():
            positions = self._proxy.actuators(name).positions()
            if positions is None:
                continue
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = bridges.joint_names(len(positions))
            msg.position = [float(p) for p in positions]
            pub.publish(msg)

    def destroy_node(self) -> None:
        if self._owns_proxy:
            self._proxy.close()
        super().destroy_node()


__all__ = ["ControlsPcbHostNode"]
