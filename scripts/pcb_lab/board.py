"""Board-level helpers for ``pcb_lab`` (USB / Soft-DFU / bandwidth — no peripherals).

Scan / Soft-DFU / bandwidth health / build images / factory defaults.
CFG / discover / inventory / motion: ``python -m pcb_lab.debug {show|set|test}``.
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional


def repo_root() -> Path:
    # scripts/pcb_lab/board.py → repo
    return Path(__file__).resolve().parents[2]


def list_firmware_images(root: Optional[Path] = None) -> List[dict]:
    """Locate Release/Debug ELF (+ sibling .bin) under the repo."""
    root = root or repo_root()
    rows: List[dict] = []
    for cfg in ("Release", "Debug"):
        elf = root / cfg / "DeftRoboticsControlsPCB.elf"
        bin_path = root / cfg / "DeftRoboticsControlsPCB.bin"
        row: dict = {
            "config": cfg,
            "elf": str(elf) if elf.is_file() else None,
            "bin": str(bin_path) if bin_path.is_file() else None,
        }
        if elf.is_file():
            st = elf.stat()
            row["elf_mtime"] = time.strftime(
                "%Y-%m-%d %H:%M:%S", time.localtime(st.st_mtime)
            )
            row["elf_bytes"] = st.st_size
        rows.append(row)
    return rows


def print_images(root: Optional[Path] = None) -> int:
    from deft_controls_sdk.debug.soft_dfu import default_firmware_elf

    root = root or repo_root()
    print(f"repo: {root}")
    rows = list_firmware_images(root)
    for row in rows:
        elf = row.get("elf")
        if elf:
            print(
                f"  [{row['config']}] {elf}  "
                f"{row.get('elf_bytes', 0)} B  mtime={row.get('elf_mtime')}"
            )
        else:
            print(f"  [{row['config']}] (no DeftRoboticsControlsPCB.elf)")
        if row.get("bin"):
            print(f"           bin: {row['bin']}")
    try:
        pick = default_firmware_elf()
        print(f"soft-dfu default pick: {pick}")
    except FileNotFoundError as exc:
        print(f"soft-dfu default pick: ({exc})")
    return 0


def try_rebuild(*, config: str = "Debug", root: Optional[Path] = None) -> int:
    """Best-effort external rebuild (make). CubeIDE is the canonical path."""
    root = root or repo_root()
    cfg = (config or "Debug").strip()
    makefile = root / cfg / "makefile"
    if not makefile.is_file():
        makefile = root / cfg / "Makefile"
    make = shutil.which("make") or shutil.which("mingw32-make")
    if makefile.is_file() and make:
        print(f"running: {make} -C {makefile.parent}", flush=True)
        proc = subprocess.run(
            [make, "-C", str(makefile.parent)],
            cwd=str(root),
        )
        return int(proc.returncode)

    print(
        "No usable makefile + make found for an automated rebuild.\n"
        "Build in STM32CubeIDE (Debug or Release), then:\n"
        f"  python -m pcb_lab flash\n"
        f"Expected ELF: {root / cfg / 'DeftRoboticsControlsPCB.elf'}"
    )
    return 1


def print_factory_defaults() -> int:
    """Documented factory CFG scaffold (matches plant_config_load_factory_defaults)."""
    print(
        """Factory / default CFG scaffold (firmware plant_config_load_factory_defaults):

  Actuator layout (product-shaped, all RobStride enabled):
    CH1 x 8  motor_id 0x01..0x08
    CH2 x 8  motor_id 0x01..0x08
    CH3 x 4  motor_id 0x01..0x04
    CH4 x 2  motor_id 0x00 (placeholder RS-06 IDs)
    CH5 x 2  motor_id 0x00
    CH6 x 2  motor_id 0x00
    remaining slots: PROTO_NONE / disabled

  Neck DXL (bench defaults):
    servo0 XL430 id=1  pos 1024..3072  profile_vel=180
    servo1 XL430 id=2  pos 700..2500   profile_vel=180

  LED: strip max, mode idle_cornflower (8), brightness 8
  listen_pdu: 0 (bench - missing PDU is not live kill policy)

Live NVM on the board may differ. Use:
  python -m pcb_lab.debug --port COMx show --cfg
to read what is actually programmed."""
    )
    return 0


def cmd_scan(*, port: Optional[str] = None, serial: Optional[str] = None) -> int:
    from deft_controls_sdk.debug.soft_dfu import main as soft_main

    argv: List[str] = ["scan"]
    if port:
        argv.extend(["--port", port])
    if serial:
        argv.extend(["--serial", serial])
    return soft_main(argv)


def cmd_leave(*, port: Optional[str] = None, serial: Optional[str] = None) -> int:
    from deft_controls_sdk.debug.soft_dfu import main as soft_main

    argv: List[str] = ["leave"]
    if port:
        argv.extend(["--port", port])
    if serial:
        argv.extend(["--serial", serial])
    return soft_main(argv)


def cmd_flash(
    *,
    port: Optional[str] = None,
    serial: Optional[str] = None,
    image: Optional[str] = None,
    require_usb_dfu: bool = False,
) -> int:
    from deft_controls_sdk.debug.soft_dfu import main as soft_main

    argv: List[str] = ["flash"]
    if port:
        argv.extend(["--port", port])
    if serial:
        argv.extend(["--serial", serial])
    if image:
        argv.extend(["--image", image])
    if require_usb_dfu:
        argv.append("--require-usb-dfu")
    return soft_main(argv)


def cmd_status(
    *,
    port: Optional[str] = None,
    serial: Optional[str] = None,
    seconds: float = 2.0,
    hz: float = 200.0,
    listen_pdu: bool = False,
) -> int:
    """Bandwidth link health: short plant stream at product-ish host rate."""
    from deft_controls_sdk import ControlsPcbHub
    from deft_controls_sdk.debug.metrics import measure_hold
    from deft_controls_sdk.debug.soft_dfu import (
        cdc_info_for_port,
        find_cdc_port,
        resolve_flash_identity,
    )
    from deft_controls_sdk.link import ActuatorDesire

    port, serial = resolve_flash_identity(port=port, serial=serial)
    device = find_cdc_port(port=port, serial=serial)
    info = cdc_info_for_port(device)
    print(f"CDC {device}" + (f"  sn={info.serial}" if info and info.serial else ""))
    print(f"link mode=bandwidth  hold {seconds:.1f}s @ {hz:.0f} Hz")

    with ControlsPcbHub.connect(device, mode="bandwidth") as hub:
        hub.listen_pdu = bool(listen_pdu)
        # Idle-anchor every slot (p=1e-6, kp=0): proves USB duplex and keeps
        # CH4-6 in the plant apply path the same as CH1-3.
        from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

        desires = {
            s: ActuatorDesire(position=1e-6) for s in range(ACTUATOR_COUNT)
        }
        report = measure_hold(
            hub,
            "pcb_lab status",
            desires,
            seconds=float(seconds),
            hz=float(hz),
            expected_stm32_mode="bandwidth",
            print_report=True,
        )
    print(
        json.dumps(
            {
                "ok": report.get("ok"),
                "raw_fb_hz": report.get("raw_fb_hz"),
                "stm32_mode": report.get("fb_stm32_mode_name"),
                "host_stm32_mode": report.get("host_stm32_mode_name"),
            },
            indent=2,
        )
    )
    return 0 if report.get("ok") else 1


def run_menu(*, port: Optional[str] = None) -> int:
    """Interactive menu when ``python -m pcb_lab`` is run with no subcommand."""
    items = [
        ("1", "Scan USB (CDC / DFU)", "scan"),
        ("2", "Status / health (bandwidth stream)", "status"),
        ("3", "Leave Soft-DFU (recover CDC)", "leave"),
        ("4", "Flash firmware", "flash"),
        ("5", "List build images", "images"),
        ("6", "Rebuild ELF (make / CubeIDE hint)", "build"),
        ("7", "Show factory / default CFG scaffold", "defaults"),
        ("8", "Open debug suite help (pcb_lab.debug)", "debug_help"),
        ("q", "Quit", "quit"),
    ]

    while True:
        print()
        print("pcb_lab - board toolkit (USB / Soft-DFU / bandwidth)")
        if port:
            print(f"  default --port {port}")
        print("  peripherals / CFG: python -m pcb_lab.debug {show|set|test}")
        print()
        for key, label, _ in items:
            print(f"  [{key}] {label}")
        try:
            choice = input("\nSelect: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0

        action = next((a for k, _, a in items if k == choice), None)
        if action is None:
            print(f"unknown choice {choice!r}")
            continue
        if action == "quit":
            return 0
        if action == "debug_help":
            print(
                "Peripheral / CFG suite (HostProxy mode=debug):\n"
                "  python -m pcb_lab.debug -h\n"
                "  python -m pcb_lab.debug --port COMx show --pcb\n"
                "  python -m pcb_lab.debug --port COMx show --cfg\n"
                "  python -m pcb_lab.debug --port COMx set --cfg\n"
                "  python -m pcb_lab.debug --port COMx test\n"
                "  python -m pcb_lab.debug test --inventory --preset bench --buses 5,6\n"
                "  python -m pcb_lab.debug test --bandwidth\n"
                "  python -m pcb_lab.debug test --actuators"
            )
            continue

        try:
            if action == "scan":
                rc = cmd_scan(port=port)
            elif action == "status":
                rc = cmd_status(port=port)
            elif action == "leave":
                rc = cmd_leave(port=port)
            elif action == "flash":
                confirm = input("Flash default ELF now? [y/N] ").strip().lower()
                if confirm not in ("y", "yes"):
                    print("cancelled")
                    continue
                rc = cmd_flash(port=port)
            elif action == "images":
                rc = print_images()
            elif action == "build":
                cfg = (
                    input("Build config [Debug/Release] (default Debug): ").strip()
                    or "Debug"
                )
                rc = try_rebuild(config=cfg)
            elif action == "defaults":
                rc = print_factory_defaults()
            else:
                rc = 1
        except Exception as exc:  # noqa: BLE001 — keep menu alive
            print(f"error: {exc}", file=sys.stderr)
            rc = 1
        if rc != 0:
            print(f"(exit {rc})")
