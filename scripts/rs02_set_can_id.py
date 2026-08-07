#!/usr/bin/env python3
"""One-off test: reassign a RobStride actuator's own CAN ID (comm=0x07).

Bench-only tool for the new ``RS02_PROBE_SET_CAN_ID`` firmware path — NOT
wired into pcb_lab / the debug dashboard yet. Talks straight to
``deft_controls_sdk.debug.robstride.set_can_id``.

    python scripts/rs02_set_can_id.py --port COM5 --bus 5 --old-id 0x7F --new-id 0x80
    python scripts/rs02_set_can_id.py --port COM5 --bus 6 --old-id 0x7F --new-id 0x81

    # dry run: SET_CAN_ID + verify only, skip DATA_SAVE (id reverts on power-cycle)
    python scripts/rs02_set_can_id.py --port COM5 --bus 5 --old-id 0x7F --new-id 0x80 --no-save

Sequence: RESET -> comm=0x07 set-ID frame (addressed to --old-id, new id in
payload) -> verify by probing --new-id -> DATA_SAVE addressed to --new-id
(only if verify succeeded). Only one process may own the CDC port at a time.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.debug.robstride import set_can_id  # noqa: E402
from deft_controls_sdk.host_proxy import HostProxy  # noqa: E402


def _parse_id(text: str) -> int:
    """0x.. / decimal, 1..255 — deliberately NOT clamped to 0x7F like the
    RobStride discover presets; the wire format's dest byte is a full byte
    (see docs/vendor.md), only the discover-range convention assumes 7-bit."""
    v = int(text, 0)
    if not (1 <= v <= 0xFF):
        raise argparse.ArgumentTypeError(f"id out of range 1..255: {text!r}")
    return v


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="CDC port (COM5 / /dev/ttyACM0)")
    p.add_argument("--bus", type=int, required=True, help="CAN bus 1..6")
    p.add_argument("--old-id", required=True, help="current CAN id, e.g. 0x7F")
    p.add_argument("--new-id", required=True, help="target CAN id, e.g. 0x80")
    p.add_argument("--no-verify", action="store_true", help="skip post-change probe")
    p.add_argument("--no-save", action="store_true", help="skip DATA_SAVE (RAM-only change)")
    args = p.parse_args(argv)

    old_id = _parse_id(args.old_id)
    new_id = _parse_id(args.new_id)

    print(
        f"rs02_set_can_id  bus={args.bus}  old=0x{old_id:02X}  new=0x{new_id:02X}  "
        f"verify={not args.no_verify}  save={not args.no_save}"
    )
    print("Note: only one process may own the CDC port — close dashboard/pcb_lab.debug first.")

    with HostProxy.connect(args.port, stream_hz=50.0, telemetry_hz=50.0, armed=False, mode="debug") as proxy:
        result = set_can_id(
            proxy.hub._connection,  # noqa: SLF001
            proxy.hub.telemetry,
            bus=args.bus,
            old_id=old_id,
            new_id=new_id,
            verify=not args.no_verify,
            save=not args.no_save,
        )

    print()
    print(f"result: set_ok={result.get('set_ok')}  verify_ok={result.get('verify_ok')}  save_ok={result.get('save_ok')}")
    ok = bool(result.get("set_ok")) and (args.no_verify or bool(result.get("verify_ok")))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
