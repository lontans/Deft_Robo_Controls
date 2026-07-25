"""DebugAPI — DEBUG-mode bench ops (discover / config / calibrate), under a lease
on the same Connection plant streaming already uses. No second serial port.

    with ControlsPcbHub.connect("COM5") as hub:
        hit = hub.debug.discover_robstride(bus=2)
        table = hub.debug.cfg_get_table()
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence

from . import config as _config
from . import damiao as _damiao
from . import robstride as _robstride
from . import robstride_calibrate as _robstride_calibrate
from . import soft_dfu as _soft_dfu
from . import zeroerr as _zeroerr
from .lease import lease

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

__all__ = [
    "DebugAPI",
    "lease",
    "PROTO_ZEROERR",
    "find_cdc_port",
    "list_cdc_ports",
    "enter_bootloader",
    "leave_bootloader",
    "wait_for_cdc",
    "wait_for_dfu",
    "flash_firmware",
]

PROTO_ZEROERR = _zeroerr.PROTO_ZEROERR

find_cdc_port = _soft_dfu.find_cdc_port
list_cdc_ports = _soft_dfu.list_cdc_ports
enter_bootloader = _soft_dfu.enter_bootloader
leave_bootloader = _soft_dfu.leave_bootloader
wait_for_cdc = _soft_dfu.wait_for_cdc
wait_for_dfu = _soft_dfu.wait_for_dfu
flash_firmware = _soft_dfu.flash_firmware


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
        """Sweep start..end; return the first responding RS02/RS01 id.

        Full range is always scanned (see :meth:`discover_robstride_all`). Manages
        its own lease — do not also wrap this call in `with hub.debug.lease():`.
        """
        return _robstride.discover(self._connection, self._telemetry, bus=bus, start=start, end=end)

    def discover_robstride_all(
        self, *, bus: int, start: int = 0x40, end: int = 0x80
    ) -> List[int]:
        """Sweep start..end; return every unique responding RobStride id in
        discovery order. Light enable+promisc only — no deep wake spam."""
        return _robstride.discover_all(
            self._connection, self._telemetry, bus=bus, start=start, end=end
        )

    def probe_robstride(self, *, bus: int, motor_id: int, timeout_s: float = 0.55) -> Optional[dict]:
        return _robstride.probe(self._connection, self._telemetry, bus=bus, motor_id=motor_id, timeout_s=timeout_s)

    def calibrate_robstride(
        self,
        *,
        bus: int,
        motor_id: int,
        cal_listen_s: float = 28.0,
        skip_iq_test: bool = False,
        strict_cali: bool = False,
    ) -> bool:
        """RS02 encoder cal (reset → iq_test → cali → zero → save → verify).

        Shaft must spin freely; supply 24–60 V. Manages its own lease.
        Returns True when mechPos verify is near zero.
        """
        return _robstride_calibrate.calibrate(
            self._connection,
            self._telemetry,
            bus=bus,
            motor_id=motor_id,
            cal_listen_s=cal_listen_s,
            skip_iq_test=skip_iq_test,
            strict_cali=strict_cali,
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
        """ID_SWEEP then REG_SCAN fallback. Leave known_ids empty unless you
        already know ESC IDs on this bus (wrong hints just waste REG_SCAN)."""
        return _damiao.discover(
            self._connection,
            self._telemetry,
            bus=bus,
            start=start,
            end=end,
            listen_ms=listen_ms,
            known_ids=known_ids,
        )

    def discover_damiao_all(
        self,
        *,
        bus: int = 1,
        start: int = 1,
        end: int = 16,
        listen_ms: int = 40,
        known_ids: Sequence[int] = (),
    ) -> List[int]:
        """Like discover_damiao but returns every unique hit (discovery order)."""
        return _damiao.discover_all(
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
        """Read the MCU's actuator_table[] (paged)."""
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
        """RAM apply (always) + flash persist (if persist=True).

        Needs firmware with the G4 BKER NVM erase fix. A raised exception after
        persist=True means RAM applied but flash did not — reboot would revert."""
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

    def enter_bootloader(
        self,
        *,
        confirm: bool = False,
        port: Optional[str] = None,
        serial: Optional[str] = None,
    ) -> str:
        """Reset into ROM USB DFU (CDC drops). Uses this hub's port by default."""
        if port is not None or serial is not None:
            return _soft_dfu.enter_bootloader(
                None, confirm=confirm, port=port, serial=serial
            )
        return _soft_dfu.enter_bootloader(self._connection, confirm=confirm)

    def leave_bootloader(
        self,
        *,
        serial: Optional[str] = None,
        address: int = 0x0803F800,
        timeout_s: float = 8.0,
    ) -> bool:
        """Leave ROM DFU over USB; default jumps to the reset trampoline."""
        return _soft_dfu.leave_bootloader(
            serial=serial, address=address, timeout_s=timeout_s
        )

    # -- ZeroErr (CiA 402 PP) --------------------------------------------------------

    def discover_zeroerr(self, *, bus: int = 1, start: int = 1, end: int = 127) -> Optional[int]:
        """NOT WIRED — firmware SDO identity helpers exist; DEBUG PDU TBD."""
        raise NotImplementedError(
            "discover_zeroerr DEBUG PDU not wired yet. "
            f"Use cfg_set_slot(..., bus={bus}, protocol={PROTO_ZEROERR}, motor_id=<node_id>). "
            f"(scan range would be {start}..{end})"
        )
