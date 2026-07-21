"""DebugAPI — DEBUG-mode bench ops (discover / config), under a lease on the
same Connection `hub.plant` already uses. No second serial port.

    with ControlsPcbHub.connect("COM5") as hub:
        with hub.debug.lease(bus=2):
            hit = hub.debug.discover_robstride(bus=2)
        table = hub.debug.cfg_get_table()

See docs/architecture.md#host-api-modes for the PLANT vs DEBUG mode split and
docs/plan-host-api-streamline.md for the rollout this implements (P0 lease,
P1 config, P2 discover). calibrate_robstride is intentionally NOT implemented
yet — see its docstring for why.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence

from . import config as _config
from . import damiao as _damiao
from . import robstride as _robstride
from . import soft_dfu as _soft_dfu
from .lease import lease

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

__all__ = ["DebugAPI", "lease"]


class DebugAPI:
    def __init__(self, connection: "Connection", telemetry: Optional["TelemetryCache"]) -> None:
        self._connection = connection
        self._telemetry = telemetry

    def lease(self, *, bus: int = 1):
        """RS2 session_begin/end bracket — plant apply may be gated
        (plant_block=BENCH_SESSION) while held. See bench/lease.py."""
        return lease(self._connection, self._telemetry, bus=bus)

    # -- RobStride (RS2) -----------------------------------------------------------

    def discover_robstride(self, *, bus: int, start: int = 0x40, end: int = 0x80) -> Optional[int]:
        """Sweep start..end on bus for a responding RS02 motor. Manages its own
        lease — do not also wrap this call in `with hub.debug.lease():`."""
        return _robstride.discover(self._connection, self._telemetry, bus=bus, start=start, end=end)

    def probe_robstride(self, *, bus: int, motor_id: int, timeout_s: float = 0.55) -> Optional[dict]:
        return _robstride.probe(self._connection, self._telemetry, bus=bus, motor_id=motor_id, timeout_s=timeout_s)

    def calibrate_robstride(self, *, bus: int, motor_id: int, **kwargs: object) -> None:
        """NOT PORTED — see scripts/legacy/control_hub/rs02/calibrate.py.

        RS02 encoder cal (comm 0x04 reset -> 0x702D iq_test -> 0x05 cali (shaft
        spins freely) -> 0x06 zero -> 0x16 save -> pararead verify) depends on
        four more legacy modules (calibrate.py, probe.py, display.py, link.py)
        with timing-sensitive listen windows, VBUS range checks, and mms-state
        tracking around a physically spinning shaft. That is meaningfully
        deeper and more hardware-sensitive than discover/probe/config, and
        deserves its own dedicated port + HIL verification pass rather than a
        rushed one bundled into this change. Use the legacy CLI today:

            python scripts/legacy/control_hub.py calibrate --port COM5 --bus {bus} --id 0x{motor_id:02X}
        """
        raise NotImplementedError(
            "calibrate_robstride is not ported yet — deliberately deferred (see docstring). "
            f"Use: python scripts/legacy/control_hub.py calibrate --port <COM> --bus {bus} --id 0x{motor_id:02X}"
        )

    # -- Damiao (DM0) ----------------------------------------------------------------

    def discover_damiao(
        self,
        *,
        bus: int = 1,
        start: int = 1,
        end: int = 16,
        listen_ms: int = 40,
        known_ids: Sequence[int] = (),
    ) -> Optional[int]:
        """ID_SWEEP then per-ID REG_SCAN fallback. Pass known_ids (e.g. this
        bus's configured slot IDs) first — scanning unconfigured IDs first can
        flood the bus and stop the drive from replying (docs/lessons.md)."""
        return _damiao.discover(
            self._connection,
            self._telemetry,
            bus=bus,
            start=start,
            end=end,
            listen_ms=listen_ms,
            known_ids=known_ids,
        )

    # -- Config (CFG PDU — actuator table get/set/save) -------------------------------

    def cfg_get_table(self, *, timeout_s: float = 1.5) -> List[dict]:
        """Read the MCU's actuator_table[] (paged; dual-arm ACTUATOR_COUNT=14)."""
        return _config.fetch_table(self._connection, timeout_s=timeout_s)

    def cfg_set_slot(
        self,
        *,
        slot: int,
        bus: int,
        protocol: int,
        motor_id: int,
        master_id: int = 0,
        enabled: bool = True,
        persist: bool = False,
        timeout_s: float = 1.5,
    ) -> dict:
        """RAM apply (always) + flash persist (if persist=True). Flash SAVE is
        unreliable in practice — a raised exception after persist=True means
        RAM apply already took effect but the change will not survive reboot."""
        return _config.set_slot(
            self._connection,
            self._telemetry,
            slot=slot,
            bus=bus,
            protocol=protocol,
            motor_id=motor_id,
            master_id=master_id,
            enabled=enabled,
            persist=persist,
            timeout_s=timeout_s,
        )

    # -- Soft-DFU (reboot into ROM bootloader, no ST-Link needed) --------------------

    def enter_bootloader(self, *, confirm: bool = False) -> None:
        """Resets the board straight into the ROM bootloader over USB CDC —
        see bench/soft_dfu.py and App/Inc/host/soft_dfu.h. UNVERIFIED WITHOUT
        HARDWARE as of this writing; keep the ST-Link handy the first time."""
        _soft_dfu.enter_bootloader(self._connection, confirm=confirm)
