"""ZeroErr (eDriver) bench notes — CFG protocol value only for now.

Firmware helpers exist (`zeroerr_read_identity`, `zeroerr_boot_blocking`) but
DEBUG PDU discover/probe is not wired yet. Configure a slot:

    hub.debug.cfg_set_slot(
        slot=0, bus=1, protocol=PROTO_ZEROERR, motor_id=<node_id>, persist=False
    )

Bus must be 1 Mbps (EDS BaudRate_1000 only). Prefer FDCAN CH1–3 for bringup.
See docs/zeroerr-firmware-bringup.md.
"""

from __future__ import annotations

PROTO_ZEROERR = 4

__all__ = ["PROTO_ZEROERR"]
