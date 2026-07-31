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
            "ROS 2 teleop node wrapping one HostProxy — actuators/led/servo "
            "topics over ActuatorAction/LedAction/ServoAction."
        ),
    )
    p.add_argument("--port", default=None, help="CDC COM port (auto if omitted)")
    p.add_argument(
        "--profile",
        default="product",
        choices=("product", "bench"),
        help="HostProxy demux profile (default: product)",
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
        help="honor PDB kill bytes for soft-kill + LED (default off, bench-safe)",
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
            "comma-separated profile-component allow-list "
            "(default: left_arm,right_arm,base,lift when present in the profile)"
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
        listen_pdu=bool(args.listen_pdu),
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
