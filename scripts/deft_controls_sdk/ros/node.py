"""ControlsPcbHostNode — ROS 2 node adapter wrapping a single HostProxy.

    ROS topics/services -> ControlsPcbHostNode -> HostProxy.set_section -> Hub

One process, one COM: this node constructs (or wraps, for tests) exactly one
``HostProxy`` and never a per-peripheral one. Product commands demux via
``set_section`` — publishers author full MIT fields (p/v/kp/kd/τ); this node
does not invent gains.

Command wire per actuator section: ``Float64MultiArray`` length ``5 * n``
interleaved ``[p, v, kp, kd, τ] * n`` (see ``bridges.desires_from_mit_command``).

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
    NECK_YAW_DXL_ID,
    PRODUCT_ACTUATOR_SECTIONS,
    bench_continuous_assembly,
    yam_product_assembly,
)
from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link import ServoDesire

from . import bridges, topics

_DEFAULT_COMPONENTS: Sequence[str] = PRODUCT_ACTUATOR_SECTIONS
_ASSEMBLY_BUILDERS = {
    "product": yam_product_assembly,
    "bench": bench_continuous_assembly,
}


class ControlsPcbHostNode(Node):
    """Teleop node: per-section JointState feedback + MIT section commands."""

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
        profile_default = str(profile or "product")
        # Product teleop listens for PDU soft-kill by default; bench does not.
        default_listen = True if profile_default == "product" else False
        self.declare_parameter(
            "listen_pdu",
            bool(listen_pdu) if listen_pdu is not None else default_listen,
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
                assembly = _ASSEMBLY_BUILDERS[profile_name]()
            except KeyError:
                known = ", ".join(sorted(_ASSEMBLY_BUILDERS))
                raise ValueError(f"unknown profile {profile_name!r}; known: {known}") from None
            resolved_port = str(self.get_parameter("port").value) or None
            self._proxy = HostProxy.connect(
                resolved_port,
                assembly=assembly,
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
        n = len(self._proxy.section_slots(name))

        def _cb(msg) -> None:
            try:
                desires = bridges.desires_from_mit_command(msg, n=n)
            except (TypeError, ValueError) as exc:
                self.get_logger().error(f"{name}: {exc}")
                return
            try:
                self._proxy.set_section(name, desires, send=False)
            except (KeyError, ValueError) as exc:
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
        self._proxy.set_servo_section(
            "neck",
            [
                ServoDesire(
                    servo_id=NECK_PITCH_DXL_ID,
                    native_step_position=int(pitch),
                    torque_enable=True,
                    operating_mode=3,
                ),
                ServoDesire(
                    servo_id=NECK_YAW_DXL_ID,
                    native_step_position=int(yaw),
                    torque_enable=True,
                    operating_mode=3,
                ),
            ],
            send=False,
        )

    def _on_timer(self) -> None:
        # HostProxy already owns a background plant-TX thread at stream_hz
        # (started inside HostProxy.connect / hub.start_streaming) — desires
        # set with send=False above ride that stream. Calling send_once()
        # here too would double-send outside that cadence, so this timer
        # only publishes feedback.
        stamp = self.get_clock().now().to_msg()
        for name, pub in self._state_pubs.items():
            states = self._proxy.section_feedback(name)
            if all(st is None for st in states):
                continue
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = bridges.joint_names(len(states))
            msg.position = [
                float(st.position) if st is not None else 0.0 for st in states
            ]
            msg.velocity = [
                float(st.velocity) if st is not None else 0.0 for st in states
            ]
            msg.effort = [
                float(st.torque) if st is not None else 0.0 for st in states
            ]
            pub.publish(msg)

    def destroy_node(self) -> None:
        if self._owns_proxy:
            self._proxy.close()
        super().destroy_node()


__all__ = ["ControlsPcbHostNode"]
