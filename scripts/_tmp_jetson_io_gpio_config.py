#!/usr/bin/env python3
"""Configure AGX Orin 40-pin: GPIO for ESTOP + forced SFIO hog for uarta.

Prior pass left uarta "enabled" in jetson-io labels but UART1_TX/RX stayed
MUX UNCLAIMED — create_dtbo dropped default/already-enabled SFIO pins.
Force uarta through UNUSED -> SFIO so the overlay retains pin8/10 nodes.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, "/opt/nvidia/jetson-io")

from Jetson import board, io  # noqa: E402

GPIO_GROUPS = (
    "pwm5",            # 18 — ESTOP / CVM GPIO35
    "dmic3",           # 16,32
    "extperiph4_clk",  # 7
    "pwm1",            # 15
    "can0",            # 29,31
    "can1",            # 33,37
)

# Must be forced non-default so create_dtbo keeps the SFIO nodes.
FORCE_SFIO = (
    "uarta",           # 8,10 — UART1 / ttyTHS1 (PDB)
)


def _force_sfio(j: board.Board, group: str) -> None:
    """Cycle UNUSED -> SFIO so pinmux hog lands in the user-custom dtbo."""
    # UI cycle uses pingroup_set_state; UNUSED then enable as SFIO.
    j.header.pingroup_set_state(group, io.PinMode.UNUSED)
    j.header.pingroup_set_state(group, io.PinMode.SFIO)
    # pingroup_enable also marks the group selected for function list
    j.header.pingroup_enable(group)
    st = j.header.pingroup_get_state(group)
    print(
        "SFIO force:",
        group,
        "pins",
        j.header.pingroup_get_pins(group),
        "state",
        st,
    )


def main() -> int:
    if os.geteuid() != 0:
        print("must run as root (sudo)", file=sys.stderr)
        return 2

    j = board.Board()
    headers = j.get_board_headers()
    if not headers:
        raise RuntimeError("no jetson-io headers found")
    hdr = headers[0]
    print("active header:", hdr)
    j.set_active_header(hdr)

    available = set(j.header.pingroups_available())
    print("available groups:", sorted(available))

    for g in FORCE_SFIO:
        if g not in available:
            print("WARN: SFIO group missing:", g)
            continue
        _force_sfio(j, g)

    for g in GPIO_GROUPS:
        if g not in available:
            print("WARN: GPIO group missing:", g)
            continue
        j.header.pingroup_set_state(g, io.PinMode.GPIO)
        st = j.header.pingroup_get_state(g)
        print("GPIO set:", g, "pins", j.header.pingroup_get_pins(g), "state", st)

    for pin in (8, 10, 16, 18):
        print("pin", pin, "label=", j.header.pin_get_label(pin))

    if j.header.pins_are_default():
        print("WARNING: header still default — dtbo may not generate")

    dtbo = j.create_dtbo_for_header()
    print("created dtbo:", dtbo)

    # Prove uarta nodes are in the overlay this time
    import subprocess

    strings = subprocess.check_output(["strings", dtbo], text=True, errors="replace")
    for needle in ("hdr40-pin8", "hdr40-pin10", "uarta", "uart1", "hdr40-pin18"):
        hit = needle in strings
        print(f"dtbo contains {needle!r}: {hit}")

    messages = j.configure_dt_for_next_boot([dtbo])
    for m in messages:
        print(m)

    print()
    print("Done. Reboot / power-cycle the Jetson for pinmux to take effect.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
