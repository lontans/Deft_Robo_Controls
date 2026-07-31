"""python -m deft_controls_sdk.ros [--port COM5] [--profile product|bench] ...

Thin launcher for ``ControlsPcbHostNode`` — teleop only. CFG / discover / cal
stay on ``pcb_lab`` / ``hub.debug`` (``mode="debug"``), not this node.
"""
from __future__ import annotations

import argparse
import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m deft_controls_sdk.ros",
        description=(
            "ROS 2 teleop node wrapping one HostProxy — section MIT commands "
            "via set_section (Float64MultiArray 5*n interleaved p/v/kp/kd/τ)."
        ),
    )
    p.add_argument("--port", default=None, help="CDC COM port (auto if omitted)")
    p.add_argument(
        "--profile",
        default="product",
        choices=("product", "bench"),
        help="HostProxy demux assembly (default: product)",
    )
    p.add_argument(
        "--stream-hz",
        type=float,
        default=200.0,
        help="plant TX rate for HostProxy's background stream (default: 200)",
    )
    p.add_argument(
        "--listen-pdu",
        action="store_true",
        help=(
            "force PDU soft-kill listen on (product already defaults on; "
            "use for bench when a PDU peer is present)"
        ),
    )
    p.add_argument(
        "--mode",
        default="bandwidth",
        choices=("bandwidth", "debug"),
        help="link mode (default: bandwidth — timing-safe teleop, no debug RPC)",
    )
    p.add_argument(
        "--components",
        default=None,
        help=(
            "comma-separated section allow-list (default product: "
            "left_arm,right_arm,base_wheel_1,base_wheel_2,base_wheel_3,torso)"
        ),
    )
    p.add_argument("--node-name", default="controls_pcb_host")
    return p


def main(argv: "list[str] | None" = None) -> int:
    args = _build_parser().parse_args(argv)

    import rclpy

    from .node import ControlsPcbHostNode

    rclpy.init(args=None)
    node = ControlsPcbHostNode(
        node_name=args.node_name,
        port=args.port,
        profile=args.profile,
        stream_hz=args.stream_hz,
        # None → node defaults (product: True, bench: False)
        listen_pdu=True if args.listen_pdu else None,
        mode=args.mode,
        components=args.components.split(",") if args.components else None,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
