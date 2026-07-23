#!/usr/bin/env python3
"""Lift (torso) CANopen discovery -- OFFLINE SCAFFOLD, not a working tool yet.

See docs/feathersdk-lift-teardown.md section 5-6 for the full derivation. This
script exists so the discovery plan (candidate node IDs, baud, which CiA-301
objects to read first) is written down and importable *before* the lift
drive's CAN wire is landed on Controls PCB CH3 -- not because it can run a
real scan today.

What's missing (same shape as `hub.debug.discover_zeroerr()` in
scripts/deft_controls_sdk/bench/__init__.py, which raises NotImplementedError
for the identical reason): the Controls-side CANopen SDO helpers
(`canopen_sdo_read_u32` etc., App/Src/plant/can/canopen.c) exist in firmware
but have no host-visible DEBUG PDU bridge yet. Nothing over USB CDC can
trigger an SDO transaction on CH3 remotely until that bridge is built. This
script's `discover()` therefore raises NotImplementedError with the same
"here's what would run" message `--dry-run` already prints, so filling in
one bridge call later is what turns this from a scaffold into a real tool.

**COM5 clarification (do not misread this script as a raw CAN sniffer):**
`--port` here is the Controls PCB's own USB-CDC link (COM5 by the repo's
convention) -- the *same* link every other bench script in this repo already
uses (`rs02_channel_bringup.py`, `hub.debug.*`, etc). It does not speak
CANopen directly; it speaks the 672 B host_exchange / DEBUG PDU protocol to
the STM32, which is the only thing that ever touches CH3's physical CAN
wires. If you are looking for a way to point a USB-CAN dongle (candump,
slcan0, PCAN, etc.) directly at the lift drive, that is a different tool
entirely and not what this script does -- it refuses obviously-mistaken
`--port` values (e.g. `can0`, `vcan0`, `slcan0`) below rather than silently
misinterpreting them as a serial device.

Usage (today -- plan only, no hardware needed):
    python lift_canopen_discover.py --dry-run

Usage (once the DEBUG PDU bridge exists):
    python lift_canopen_discover.py --port COM5 --bus 3
"""
from __future__ import annotations

import argparse
import sys
from typing import List, Optional

sys.path.insert(0, ".")

from deft_controls_sdk import ControlsPcbHub, find_cdc_port

# CH3 is FDCAN2 per docs/bringup.md section 2 schematic mapping -- prefer it for the
# lift the same way docs/zeroerr-firmware-bringup.md section 1 prefers FDCAN CH1-3
# over MCP CH4-6 for a new CANopen bring-up (cheap enqueue, no SPI cost).
DEFAULT_BUS = 3

# CANopen node IDs are 1..127 (0 is reserved for NMT broadcast).
NODE_ID_MIN = 1
NODE_ID_MAX = 127

# EDS-documented baud for ZeroErr's eDriver was 1 Mbps only (no other rate
# advertised) -- treat as the first guess for the lift too, since every other
# bus on this board already runs 1 Mbps, but do not assume it matches
# without a scope trace (see the teardown doc's "what's unknown" table).
CANDIDATE_BAUDS_BPS: List[int] = [1_000_000, 500_000, 250_000, 125_000]

# Generic CiA-301 objects, not vendor-specific -- safe first reads on *any*
# CANopen node before anything more specific is known (mirrors
# zeroerr_read_identity()'s use of the same two objects).
IDENTITY_OBJECTS = {
    "device_type": (0x1000, 0),
    "vendor_id": (0x1018, 1),
    "product_code": (0x1018, 2),
    "revision_number": (0x1018, 3),
}

_CAN_IFACE_LOOKALIKES = ("can0", "can1", "vcan0", "slcan0", "slcan1")


def _refuse_if_can_iface_name(port: str) -> None:
    lowered = port.strip().lower()
    if lowered in _CAN_IFACE_LOOKALIKES or lowered.startswith(("can", "vcan", "slcan")):
        raise SystemExit(
            f"--port {port!r} looks like a raw CAN interface name (SocketCAN/"
            "slcan), not a Controls PCB USB-CDC serial port. This script talks "
            "to the Controls PCB over USB CDC (COM5 by this repo's convention) "
            "and lets its firmware bridge to CH3 -- it does not open a CAN "
            "interface directly. See this file's module docstring."
        )


def print_plan(bus: int) -> None:
    print(f"Lift CANopen discovery plan (CH{bus}) -- see docs/feathersdk-lift-teardown.md")
    print(f"  Node ID sweep: {NODE_ID_MIN}..{NODE_ID_MAX}")
    print(f"  Baud candidates (try in order): {CANDIDATE_BAUDS_BPS}")
    print("  First reads per responding node (generic CiA-301, not vendor-specific):")
    for name, (index, sub) in IDENTITY_OBJECTS.items():
        print(f"    {name:16s} 0x{index:04X}:{sub}")
    print(
        "  Then: compare vendor_id/product_code against a known EDS if one can "
        "be found; if not, probe standard CiA-402 objects blind (0x6060 mode-of-"
        "operation, 0x6040/0x6041 controlword/statusword, 0x6064/0x606C actual "
        "position/velocity, 0x607A/0x60FF target position/velocity) -- see the "
        "teardown doc section 6 for why these are a reasonable blind guess and what "
        "still can't be known this way (counts<->mm scale, travel limits, what "
        "recalibrate() actually does on the wire)."
    )


def discover(port: str, bus: int) -> None:
    """Not implemented -- see module docstring. Connects to the board (proves
    the USB-CDC link is alive) then raises, same shape as
    hub.debug.discover_zeroerr()."""
    with ControlsPcbHub.connect(port) as hub:
        del hub  # link liveness only; no CANopen DEBUG PDU bridge exists yet
    raise NotImplementedError(
        "lift CANopen discovery has no DEBUG PDU bridge yet (see module "
        "docstring). Firmware has canopen_sdo_read_u32() etc. "
        "(App/Src/plant/can/canopen.c) but nothing exposes them over USB CDC "
        "for a specific slot/node the way discover_damiao()/discover_robstride() "
        "do. Run with --dry-run to see the planned sweep without a bridge."
    )


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", default=None, help="Controls PCB USB CDC port (default: autodetect)")
    ap.add_argument("--bus", type=int, default=DEFAULT_BUS, help=f"schematic CH (default {DEFAULT_BUS})")
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="print the discovery plan only -- no hub connection, no hardware needed",
    )
    args = ap.parse_args(argv)

    if args.dry_run:
        print_plan(args.bus)
        return 0

    if args.port is not None:
        _refuse_if_can_iface_name(args.port)
    port = args.port or find_cdc_port()
    _refuse_if_can_iface_name(port)

    print_plan(args.bus)
    print()
    try:
        discover(port, args.bus)
    except NotImplementedError as exc:
        print(f"\n{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
