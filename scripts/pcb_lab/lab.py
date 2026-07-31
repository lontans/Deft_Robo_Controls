"""pcb_lab — board-only toolkit (USB / Soft-DFU / bandwidth health).

Bare invocation opens an interactive menu:

    python -m pcb_lab
    python -m pcb_lab -h

Board commands (no peripherals / no CFG demux):

    python -m pcb_lab scan
    python -m pcb_lab status
    python -m pcb_lab leave
    python -m pcb_lab flash [--image PATH]
    python -m pcb_lab images
    python -m pcb_lab build [--config Debug]
    python -m pcb_lab show defaults|health

Peripheral / CFG / motion proves live under ``pcb_lab.debug`` only:

    python -m pcb_lab.debug --port COM5 show --pcb
    python -m pcb_lab.debug --port COM5 set --cfg
    python -m pcb_lab.debug --port COM5 test
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional, Sequence

from deft_controls_sdk.actions import ActuatorAction, ServoAction
from deft_controls_sdk.config import (
    Assembly,
    Profile,
    assembly_from_name,
)
from deft_controls_sdk.host_proxy import HostProxy


class LabRobot:
    """Optional thin script façade over ``HostProxy`` + assembly demux.

    Prefer the SDK in notebooks::

        from deft_controls_sdk import HostProxy
        from deft_controls_sdk.actions import ActuatorAction
        from deft_controls_sdk.config import assembly_from_name

        with HostProxy.connect(
            "COM5", mode="debug", armed=False, assembly=assembly_from_name("bench")
        ) as proxy:
            proxy.arm_plant()
            view = proxy.actuators("base")  # or ActuatorAction.from_slots(...)
            view.hold(send=True)  # sample FB → stay put
            proxy.disarm_plant()

    ``LabRobot`` only wraps that pattern. Not a controls / teleop stack.
    """

    def __init__(
        self,
        proxy: HostProxy,
        *,
        assembly: Optional[Assembly] = None,
    ) -> None:
        self.proxy = proxy
        self.assembly = assembly

    @classmethod
    def connect(
        cls,
        port: Optional[str] = None,
        *,
        profile: Optional[Profile] = None,
        assembly: Optional[Assembly] = None,
        stream_hz: float = 200.0,
        listen_pdu: bool = False,
        mode: str = "bandwidth",
        assembly_name: str = "bench",
    ) -> "LabRobot":
        if assembly is None and profile is None:
            assembly = assembly_from_name(assembly_name)
        proxy = HostProxy.connect(
            port,
            stream_hz=stream_hz,
            assembly=assembly,
            profile=profile,
            armed=False,
            listen_pdu=listen_pdu,
            mode=mode,
        )
        return cls(proxy, assembly=assembly or proxy.assembly)

    def close(self) -> None:
        self.proxy.close()

    def __enter__(self) -> "LabRobot":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def hub(self):
        """ControlsPcbHub escape hatch (raw slots / debug)."""
        return self.proxy.hub

    def actuator_profile(self, name: str):
        """Typed ``ActuatorProfile`` from the bound assembly."""
        if self.assembly is None:
            raise RuntimeError("LabRobot has no Assembly; pass assembly= to connect()")
        return self.assembly.actuator(name)

    def actuators(self, name: str) -> ActuatorAction:
        """``ActuatorAction`` from assembly section when present, else demux."""
        if self.assembly is not None and name in self.assembly.actuators:
            return ActuatorAction.from_actuator_profile(
                self.proxy, self.assembly.actuator(name)
            )
        return self.proxy.actuators(name)

    def component(self, name: str) -> ActuatorAction:
        """Alias of :meth:`actuators` (section / demux name)."""
        return self.actuators(name)

    def led(self):
        return self.proxy.led()

    def servo(self, name: str = "neck") -> ServoAction:
        if self.assembly is not None and name in self.assembly.servos:
            return ServoAction.from_servo_profile(
                self.proxy, self.assembly.servo(name)
            )
        return self.proxy.servo()

    def pdu_link(self):
        return self.proxy.pdu_link()

    def doctor(self) -> dict:
        """HostProxy health snapshot (scripts / notebooks)."""
        return self.proxy.doctor()

    def hold(
        self,
        component: str,
        *,
        positions: Optional[Sequence[float]] = None,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        hold_s: float = 2.0,
    ) -> None:
        """Thin wrapper: ``actuators(component).hold`` + sleep. Prefer ``proxy.actions``."""
        view = self.actuators(component)
        view.hold(positions, kp=kp, kd=kd, send=False)
        self.proxy.send_once()
        time.sleep(max(0.0, float(hold_s)))

    def step(
        self,
        component: str,
        *,
        joint: int = 0,
        delta: float = 0.05,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        hold_s: float = 1.0,
    ) -> None:
        """Thin wrapper around ``ActuatorAction.nudge``."""
        view = self.actuators(component)
        view.nudge(index=joint, delta=delta, kp=kp, kd=kd, send=False)
        self.proxy.send_once()
        time.sleep(max(0.0, float(hold_s)))

    def blank(self, component: str) -> None:
        self.component(component).blank(send=False)
        self.proxy.send_once()


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="pcb_lab",
        description=(
            "Controls PCB board toolkit (USB / Soft-DFU / bandwidth). "
            "Peripherals + CFG: python -m pcb_lab.debug {show|set|test}."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python -m pcb_lab\n"
            "  python -m pcb_lab scan\n"
            "  python -m pcb_lab status --port COM5\n"
            "  python -m pcb_lab flash\n"
            "  python -m pcb_lab.debug --port COM5 show --pcb\n"
            "  python -m pcb_lab.debug --port COM5 test\n"
        ),
    )
    p.add_argument("--port", default=None, help="CDC COM port (auto if omitted)")
    p.add_argument(
        "--serial",
        default=None,
        help="USB serial when multiple boards are present",
    )
    p.add_argument(
        "--listen-pdu",
        action="store_true",
        help="honor PDB kill bytes during status (default off for bench)",
    )
    sub = p.add_subparsers(dest="cmd", required=False)

    sc = sub.add_parser("scan", help="list STM32 CDC / DFU presence")
    sc.set_defaults(_cmd="scan")

    st = sub.add_parser(
        "status",
        help="bandwidth link health (fb_hz / ack_lag / stm32_mode)",
    )
    st.add_argument("--seconds", type=float, default=2.0)
    st.add_argument(
        "--hz",
        type=float,
        default=200.0,
        help="host TX rate during status hold (default 200; fb floor ~80%% of this)",
    )
    st.set_defaults(_cmd="status")

    lv = sub.add_parser("leave", help="Soft-DFU Leave -> recover app CDC")
    lv.set_defaults(_cmd="leave")

    fl = sub.add_parser("flash", help="flash firmware (Soft-DFU, SWD fallback)")
    fl.add_argument(
        "--image",
        default=None,
        help="path to .elf/.bin (default: newest Release/Debug ELF)",
    )
    fl.add_argument(
        "--require-usb-dfu",
        action="store_true",
        help="fail instead of ST-Link SWD fallback",
    )
    fl.set_defaults(_cmd="flash")

    im = sub.add_parser("images", help="list Release/Debug build images")
    im.set_defaults(_cmd="images")

    bd = sub.add_parser("build", help="rebuild ELF via make, or print CubeIDE hint")
    bd.add_argument(
        "--config",
        default="Debug",
        help="Debug (default) or Release",
    )
    bd.set_defaults(_cmd="build")

    sh = sub.add_parser("show", help="show defaults or health")
    sh_sub = sh.add_subparsers(dest="show_what", required=True)
    sh_sub.add_parser("defaults", help="factory CFG scaffold from firmware")
    sh_health = sh_sub.add_parser(
        "health",
        help="alias of status (bandwidth stream health)",
    )
    sh_health.add_argument("--seconds", type=float, default=2.0)
    sh_health.add_argument(
        "--hz",
        type=float,
        default=200.0,
        help="host TX rate during status hold (default 200; fb floor ~80%% of this)",
    )
    sh.set_defaults(_cmd="show")

    dbg = sub.add_parser(
        "debug",
        help="peripheral/CFG suite (forwards to pcb_lab.debug show|set|test)",
    )
    dbg.add_argument(
        "debug_args",
        nargs=argparse.REMAINDER,
        help="args after -- e.g. -- show --pcb / -- test --bandwidth",
    )
    dbg.set_defaults(_cmd="debug")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    argv_list = list(sys.argv[1:] if argv is None else argv)
    if not argv_list:
        from pcb_lab.board import run_menu

        return run_menu()

    args = _build_parser().parse_args(argv_list)
    if args.cmd is None:
        from pcb_lab.board import run_menu

        return run_menu(port=args.port)

    cmd = args._cmd

    if cmd == "debug":
        from deft_controls_sdk.debug.suite import main as debug_main

        extra = list(args.debug_args or [])
        if extra and extra[0] == "--":
            extra = extra[1:]
        prefix: List[str] = []
        if args.port is not None:
            prefix.extend(["--port", args.port])
        if args.listen_pdu:
            prefix.append("--listen-pdu")
        return debug_main([*prefix, *extra])

    from pcb_lab import board

    if cmd == "scan":
        return board.cmd_scan(port=args.port, serial=args.serial)
    if cmd == "status":
        return board.cmd_status(
            port=args.port,
            serial=args.serial,
            seconds=float(args.seconds),
            hz=float(args.hz),
            listen_pdu=bool(args.listen_pdu),
        )
    if cmd == "leave":
        return board.cmd_leave(port=args.port, serial=args.serial)
    if cmd == "flash":
        return board.cmd_flash(
            port=args.port,
            serial=args.serial,
            image=args.image,
            require_usb_dfu=bool(args.require_usb_dfu),
        )
    if cmd == "images":
        return board.print_images()
    if cmd == "build":
        return board.try_rebuild(config=str(args.config))
    if cmd == "show":
        if args.show_what == "defaults":
            return board.print_factory_defaults()
        return board.cmd_status(
            port=args.port,
            serial=args.serial,
            seconds=float(getattr(args, "seconds", 2.0)),
            hz=float(getattr(args, "hz", 200.0)),
            listen_pdu=bool(args.listen_pdu),
        )

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
