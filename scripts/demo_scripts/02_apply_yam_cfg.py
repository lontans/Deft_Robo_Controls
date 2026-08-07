#!/usr/bin/env python3
"""Demo 02 — apply YAM product CFG (debug mode).

Talk track:
  Product bring-up writes the YAM actuator map into RAM via CFG.
  Needs mode=debug. Teleop demos reconnect in bandwidth afterward.
  Does not torque motors (armed=False).
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
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    add_port_args(p)
    args = p.parse_args(argv)

    from deft_controls_sdk import HostProxy

    require_one_owner()
    step(
        1,
        "Connect HostProxy in debug + apply_yam_cfg",
        "Writes product CH1/CH2 arms + CH4–6 base wheel CFG into live RAM",
    )
    with HostProxy.connect(
        **connect_kwargs(args),
        mode="debug",
        apply_yam_cfg=True,
        armed=False,
        listen_pdu=False,
    ) as proxy:
        pause(0.5, why="let CFG settle / first feedback")
        snap = proxy.cfg_snapshot() if hasattr(proxy, "cfg_snapshot") else None
        if isinstance(snap, dict):
            print(f"    enabled_count={snap.get('enabled_count')}")
        print(f"    armed={proxy.armed}  mode={proxy.mode}")
        step(2, "Disconnect", "with-block exits → proxy.close() releases CDC")

    say_done("02_apply_yam_cfg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
