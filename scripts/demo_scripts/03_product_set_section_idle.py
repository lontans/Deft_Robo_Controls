#!/usr/bin/env python3
"""Demo 03 — product path: set_section idle hold on left_arm.

Talk track:
  Parent deft_vbeta (and this script) author ActuatorDesire rows, then call
  HostProxy.set_section — NOT proxy.actions. Section demux packs the 694 B CMDH.
  IDLE = zero MIT (no torque). Arm briefly, read FB, disarm, release CDC.
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
        "--section",
        default="left_arm",
        help="product section name (default left_arm)",
    )
    p.add_argument(
        "--hold-s",
        type=float,
        default=1.0,
        help="seconds armed while holding IDLE (default 1)",
    )
    args = p.parse_args(argv)

    from deft_controls_sdk import HostProxy
    from deft_controls_sdk.link import IDLE

    require_one_owner()
    section = str(args.section)

    step(
        1,
        "Connect bandwidth (product teleop mode)",
        "CFG should already be on the board (run 02 first if unsure)",
    )
    with HostProxy.connect(
        **connect_kwargs(args),
        mode="bandwidth",
        armed=False,
        listen_pdu=False,
    ) as proxy:
        slots = proxy.section_slots(section)
        step(
            2,
            f'set_section("{section}", IDLE × {len(slots)})',
            f"slots={slots} — zero MIT desires (no torque)",
        )
        proxy.set_section(section, [IDLE for _ in slots], send=True)

        step(3, "arm_plant()", "plant_apply ON — motors track held desires")
        proxy.arm_plant()
        pause(float(args.hold_s), why="hold window")

        step(4, "Read section feedback", "proves USB duplex + demux slots")
        fb = proxy.section_feedback(section)
        summarize_section_fb(section, fb, slots)

        step(5, "disarm_plant()", "plant_apply OFF")
        proxy.disarm_plant()
        step(6, "Disconnect", "with-block → close() blanks + releases CDC")

    say_done("03_product_set_section_idle")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
