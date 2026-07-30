"""LabRobot + CLI — hold / step / blank / doctor / demux via HostProxy."""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import List, Optional, Sequence

from deft_controls_sdk.host_proxy import (
    HostProxy,
    Profile,
    bench_continuous_profile,
    yam_product_profile,
)
from deft_controls_sdk.link import ActuatorDesire


def _resolve_profile(name: str) -> Profile:
    key = (name or "product").strip().lower()
    if key in ("product", "yam", "yam_product"):
        return yam_product_profile()
    if key in ("bench", "continuous", "yam_bench_continuous"):
        return bench_continuous_profile()
    raise SystemExit(f"unknown --profile {name!r}; use product|bench")


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
        stream_hz: float = 200.0,
        apply_yam_cfg: bool = False,
        listen_pdu: bool = False,
    ) -> "LabRobot":
        proxy = HostProxy.connect(
            port,
            stream_hz=stream_hz,
            profile=profile or yam_product_profile(),
            apply_yam_cfg=apply_yam_cfg,
            listen_pdu=listen_pdu,
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
    p.add_argument(
        "--profile",
        default="product",
        help="HostProxy demux profile: product (default) | bench",
    )
    p.add_argument(
        "--listen-pdu",
        action="store_true",
        help="honor PDB kill bytes (soft-kill + LED); default off for bench",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("doctor", help="print HostProxy / CFG / FB snapshot")
    d.set_defaults(_cmd="doctor")

    x = sub.add_parser(
        "demux",
        help="show Profile→slots joined with CFG (when DEBUG allows)",
    )
    x.set_defaults(_cmd="demux")

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

    c = sub.add_parser(
        "continuous",
        help="HostProxy bench cruise (forwards to legacy yam_continuous_all)",
    )
    c.add_argument(
        "cont_args",
        nargs=argparse.REMAINDER,
        help="args after -- forwarded to continuous (e.g. -- --duration 20)",
    )
    c.set_defaults(_cmd="continuous")

    dbg = sub.add_parser(
        "debug",
        help="CFG/NVM show|set (forwards to python -m pcb_lab.debug)",
    )
    dbg.add_argument(
        "debug_args",
        nargs=argparse.REMAINDER,
        help="args after -- e.g. -- show --cfg --status",
    )
    dbg.set_defaults(_cmd="debug")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    if args._cmd == "continuous":
        from pcb_lab.continuous import main as cont_main

        # Drop a leading "--" that argparse.REMAINDER often leaves.
        extra = list(args.cont_args or [])
        if extra and extra[0] == "--":
            extra = extra[1:]
        if args.port is not None:
            extra = ["--port", args.port, *extra]
        return cont_main(extra)

    if args._cmd == "debug":
        from pcb_lab.debug.cli import main as debug_main

        extra = list(args.debug_args or [])
        if extra and extra[0] == "--":
            extra = extra[1:]
        # Global flags that apply before the debug subcommand.
        prefix: List[str] = []
        if args.port is not None:
            prefix.extend(["--port", args.port])
        if args.listen_pdu:
            prefix.append("--listen-pdu")
        return debug_main([*prefix, *extra])

    profile = _resolve_profile(args.profile)
    with LabRobot.connect(
        args.port,
        profile=profile,
        apply_yam_cfg=bool(args.cfg),
        listen_pdu=bool(args.listen_pdu),
    ) as lab:
        if args._cmd == "doctor":
            print(json.dumps(lab.doctor(), indent=2))
            return 0
        if args._cmd == "demux":
            print(json.dumps(lab.proxy.demux_report(), indent=2))
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
