"""LabRobot + CLI — hold / step / blank / doctor via HostProxy."""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import List, Optional, Sequence

from deft_controls_sdk.host_proxy import HostProxy, Profile, yam_product_profile
from deft_controls_sdk.link import ActuatorDesire


class LabRobot:
    """Thin lab façade over HostProxy (board prove, not YAMAIMobile)."""

    def __init__(self, proxy: HostProxy) -> None:
        self.proxy = proxy

    @classmethod
    def connect(
        cls,
        port: Optional[str] = None,
        *,
        profile: Optional[Profile] = None,
        stream_hz: float = 40.0,
        apply_yam_cfg: bool = False,
    ) -> "LabRobot":
        proxy = HostProxy.connect(
            port,
            stream_hz=stream_hz,
            profile=profile or yam_product_profile(),
            apply_yam_cfg=apply_yam_cfg,
        )
        return cls(proxy)

    def close(self) -> None:
        self.proxy.close()

    def __enter__(self) -> "LabRobot":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def doctor(self) -> dict:
        return self.proxy.doctor()

    def hold(
        self,
        component: str,
        *,
        positions: Optional[Sequence[float]] = None,
        kp: float = 8.0,
        kd: float = 0.5,
        hold_s: float = 2.0,
    ) -> None:
        view = self.proxy.component(component)
        if positions is None:
            fb = view.positions()
            if fb is None:
                positions = [0.0] * len(view.slots)
            else:
                positions = fb
        view.hold(positions, kp=kp, kd=kd, send=False)
        self.proxy.send_once()
        time.sleep(max(0.0, float(hold_s)))

    def step(
        self,
        component: str,
        *,
        joint: int = 0,
        delta: float = 0.05,
        kp: float = 8.0,
        kd: float = 0.5,
        hold_s: float = 1.0,
    ) -> None:
        view = self.proxy.component(component)
        pos = view.positions() or [0.0] * len(view.slots)
        if not (0 <= joint < len(pos)):
            raise ValueError(f"joint {joint} out of range 0..{len(pos) - 1}")
        pos[joint] = float(pos[joint]) + float(delta)
        view.hold(pos, kp=kp, kd=kd, send=False)
        self.proxy.send_once()
        time.sleep(max(0.0, float(hold_s)))

    def blank(self, component: str) -> None:
        self.proxy.component(component).blank(send=False)
        self.proxy.send_once()


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="pcb_lab", description=__doc__)
    p.add_argument("--port", default=None, help="CDC COM port (auto if omitted)")
    p.add_argument("--cfg", action="store_true", help="apply YAM product CFG before plant")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("doctor", help="print HostProxy / CFG / FB snapshot")
    d.set_defaults(_cmd="doctor")

    h = sub.add_parser("hold", help="hold component at FB (or zeros)")
    h.add_argument("--component", default="left_arm")
    h.add_argument("--kp", type=float, default=8.0)
    h.add_argument("--kd", type=float, default=0.5)
    h.add_argument("--hold-s", type=float, default=2.0)
    h.set_defaults(_cmd="hold")

    s = sub.add_parser("step", help="step one joint by delta rad")
    s.add_argument("--component", default="left_arm")
    s.add_argument("--joint", type=int, default=0)
    s.add_argument("--delta", type=float, default=0.05)
    s.add_argument("--kp", type=float, default=8.0)
    s.add_argument("--kd", type=float, default=0.5)
    s.add_argument("--hold-s", type=float, default=1.0)
    s.set_defaults(_cmd="step")

    b = sub.add_parser("blank", help="blank component desires (idle)")
    b.add_argument("--component", default="left_arm")
    b.set_defaults(_cmd="blank")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    with LabRobot.connect(args.port, apply_yam_cfg=bool(args.cfg)) as lab:
        if args._cmd == "doctor":
            print(json.dumps(lab.doctor(), indent=2))
            return 0
        if args._cmd == "hold":
            lab.hold(
                args.component,
                kp=args.kp,
                kd=args.kd,
                hold_s=args.hold_s,
            )
            return 0
        if args._cmd == "step":
            lab.step(
                args.component,
                joint=args.joint,
                delta=args.delta,
                kp=args.kp,
                kd=args.kd,
                hold_s=args.hold_s,
            )
            return 0
        if args._cmd == "blank":
            lab.blank(args.component)
            return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
