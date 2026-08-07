#!/usr/bin/env python3
"""Demo 04 — bench path: set_section on spare-slot base (slots 22–25).

Talk track:
  This Jetson harness often has live CH5/CH6 motors on spare slots, not product
  base_wheel_* IDs. Same demux API (set_section), different Assembly.
  Default is IDLE hold only. Pass --nudge for a small position step (motors on).
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (  # noqa: E402
    add_port_args,
    connect_kwargs,
    pause,
    require_one_owner,
    say_done,
    step,
    summarize_section_fb,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    add_port_args(p)
    p.add_argument(
        "--hold-s",
        type=float,
        default=1.5,
        help="seconds armed (default 1.5)",
    )
    p.add_argument(
        "--nudge",
        action="store_true",
        help="after sampling FB, command +delta rad on first base slot (MOTION)",
    )
    p.add_argument(
        "--delta",
        type=float,
        default=0.15,
        help="nudge size in rad when --nudge (default 0.15)",
    )
    args = p.parse_args(argv)

    from deft_controls_sdk import HostProxy
    from deft_controls_sdk.config import bench_continuous_assembly
    from deft_controls_sdk.link import IDLE, ActuatorDesire

    require_one_owner()
    section = "base"

    step(
        1,
        "Connect bandwidth + bench_continuous_assembly",
        "section 'base' → spare slots (typically 22–25)",
    )
    with HostProxy.connect(
        **connect_kwargs(args),
        mode="bandwidth",
        armed=False,
        listen_pdu=False,
        assembly=bench_continuous_assembly(),
    ) as proxy:
        slots = proxy.section_slots(section)
        step(2, f'set_section("{section}", IDLE)', f"slots={slots}")
        proxy.set_section(section, [IDLE for _ in slots], send=True)
        proxy.arm_plant()
        pause(0.4, why="sample FB")

        fb = proxy.section_feedback(section)
        summarize_section_fb(section, fb, slots)

        if args.nudge:
            step(
                3,
                f"MOTION nudge first slot +{args.delta} rad",
                "requires clear / powered actuators — Ctrl+C to abort before arm",
            )
            desires = []
            for i, st in enumerate(fb):
                if st is None:
                    desires.append(IDLE)
                else:
                    pos = float(st.position)
                    if i == 0:
                        pos += float(args.delta)
                    desires.append(
                        ActuatorDesire(position=pos, kp=20.0, kd=1.0, torque=0.0)
                    )
            proxy.set_section(section, desires, send=True)
            pause(float(args.hold_s), why="nudge hold")
            # return toward sampled positions
            back = []
            for st in fb:
                if st is None:
                    back.append(IDLE)
                else:
                    back.append(
                        ActuatorDesire(
                            position=float(st.position), kp=20.0, kd=1.0, torque=0.0
                        )
                    )
            proxy.set_section(section, back, send=True)
            pause(1.0, why="return")
        else:
            step(3, "IDLE hold (no --nudge)", "pass --nudge for a small move")
            pause(float(args.hold_s), why="idle hold")

        step(4, "disarm + disconnect")
        proxy.disarm_plant()

    say_done("04_bench_base_hold")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
