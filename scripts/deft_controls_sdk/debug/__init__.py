"""DebugAPI — DEBUG-mode bench ops (discover / config / calibrate), under a lease
on the same Connection plant streaming already uses. No second serial port.

    with ControlsPcbHub.connect("COM5", mode="debug") as hub:
        hit = hub.debug.discover_robstride(bus=2)
        table = hub.debug.cfg_get_table()

Notebook helpers (same data the suite TUI uses)::

    from deft_controls_sdk import HostProxy
    from deft_controls_sdk.debug import (
        as_hex, collect_cfg, pause_plant_stream, run_inventory,
    )

Plant CFG / LED CLI suite (HostProxy)::

    python -m deft_controls_sdk.debug.suite --port COM5 show --pcb
    python -m pcb_lab.debug --port COM5 show --pcb   # lab CLI alias
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence, Tuple

from . import config as _config
from . import cubemars as _cubemars
from . import damiao as _damiao
from . import discover as _discover
from . import inventory as _inventory
from . import robstride as _robstride
from . import robstride_calibrate as _robstride_calibrate
from . import soft_dfu as _soft_dfu
from . import zeroerr as _zeroerr
from .discover import as_hex, hex_id
from .inventory import run_inventory
from .lease import lease
from .snapshot import collect_bandwidth, collect_cfg
from .stream_pause import pause_plant_stream

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

__all__ = [
    "DebugAPI",
    "as_hex",
    "collect_bandwidth",
    "collect_cfg",
    "hex_id",
    "lease",
    "pause_plant_stream",
    "run_inventory",
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

    def _require_debug(self, op: str) -> None:
        self._connection.require_debug_mode(op)

    def lease(self, *, bus: int = 1):
        """RS2 session_begin/end bracket — plant apply may be gated
        (plant_block=BENCH_SESSION) while held. See bench/lease.py."""
        self._require_debug("hub.debug.lease")
        return lease(self._connection, self._telemetry, bus=bus)

    # -- RobStride (RS2) -----------------------------------------------------------

    def discover_robstride(self, *, bus: int, start: int = 0x40, end: int = 0x80) -> Optional[int]:
        """Sweep start..end; return the first responding RS02/RS01 id.

        Full range is always scanned (see :meth:`discover_robstride_all`). Manages
        its own lease — do not also wrap this call in `with hub.debug.lease():`.
        """
        self._require_debug("hub.debug.discover_robstride")
        return _robstride.discover(self._connection, self._telemetry, bus=bus, start=start, end=end)

    def discover_robstride_all(
        self,
        *,
        bus: int = 1,
        buses: Optional[Sequence[int]] = None,
        start: int = 0x40,
        end: int = 0x80,
    ) -> List[int]:
        """Sweep start..end; return every unique responding RobStride id.

        Pass ``buses=[1,5,6]`` for overlapping multi-bus discover (one lease,
        FW round-robin listen). Light enable+promisc only.
        """
        self._require_debug("hub.debug.discover_robstride_all")
        return _robstride.discover_all(
            self._connection,
            self._telemetry,
            bus=bus,
            buses=buses,
            start=start,
            end=end,
        )

    def discover_robstride_by_bus(
        self,
        *,
        buses: Sequence[int],
        start: int = 0x40,
        end: int = 0x80,
    ) -> dict:
        """Multi-bus RobStride discover → ``{bus: [ids…]}``."""
        self._require_debug("hub.debug.discover_robstride_by_bus")
        return _robstride.discover_all_by_bus(
            self._connection,
            self._telemetry,
            buses=buses,
            start=start,
            end=end,
        )

    def probe_robstride(
        self,
        *,
        bus: int,
        motor_id: int,
        timeout_s: float = 0.55,
        reset: bool = True,
    ) -> Optional[dict]:
        self._require_debug("hub.debug.probe_robstride")
        return _robstride.probe(
            self._connection,
            self._telemetry,
            bus=bus,
            motor_id=motor_id,
            timeout_s=timeout_s,
            reset=reset,
        )

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
        self._require_debug("hub.debug.calibrate_robstride")
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
        self._require_debug("hub.debug.discover_damiao")
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
        self._require_debug("hub.debug.discover_damiao_all")
        return _damiao.discover_all(
            self._connection,
            self._telemetry,
            bus=bus,
            start=start,
            end=end,
            listen_ms=listen_ms,
            known_ids=known_ids,
        )

    # -- CubeMars (CM0) --------------------------------------------------------------

    def discover_cubemars(
        self,
        *,
        bus: int = 1,
        start: int = 1,
        end: int = 16,
        listen_ms: int = 40,
    ) -> Optional[int]:
        """MIT-frame ID_SWEEP (no register-read scheme — see docs/vendor.md)."""
        self._require_debug("hub.debug.discover_cubemars")
        return _cubemars.discover(
            self._connection, self._telemetry, bus=bus, start=start, end=end, listen_ms=listen_ms
        )

    def discover_cubemars_all(
        self,
        *,
        bus: int = 1,
        start: int = 1,
        end: int = 16,
        listen_ms: int = 40,
    ) -> List[int]:
        """Like discover_cubemars but returns every unique hit (discovery order)."""
        self._require_debug("hub.debug.discover_cubemars_all")
        return _cubemars.discover_all(
            self._connection, self._telemetry, bus=bus, start=start, end=end, listen_ms=listen_ms
        )

    def discover_queued(
        self,
        *,
        buses: Sequence[int],
        protocols: Sequence[str] = ("robstride", "damiao"),
        ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
        listen_ms: int = 40,
    ) -> List[dict]:
        """Queue single-bus discover across buses × protocols (sequential).

        Not parallel — FW one session bus at a time. Same DEBUG DBGC/DBGF
        path as :meth:`discover_robstride_all` / :meth:`discover_damiao_all`.
        """
        self._require_debug("hub.debug.discover_queued")
        return _discover.discover_queued(
            self._connection,
            self._telemetry,
            buses=buses,
            protocols=protocols,
            ranges=ranges,
            listen_ms=listen_ms,
        )

    def inventory(
        self,
        *,
        buses: Sequence[int] = (1, 2, 3, 4, 5, 6),
        protocols: Sequence[str] = ("robstride", "damiao"),
        ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
        preset: Optional[str] = None,
        listen_ms: int = 40,
        print_report: bool = True,
    ) -> dict:
        """Actuators-only smoke (DEBUG lanes discover).

        **Requires** ``ranges=`` or ``preset='bench'|'product'|'full'`` — no
        silent wide ID sweep. For servos + PDU as well, use
        ``run_inventory(proxy, …)`` / ``python -m pcb_lab.debug test --inventory``.
        """
        self._require_debug("hub.debug.inventory")
        return _inventory.inventory_smoke(
            self._connection,
            self._telemetry,
            buses=buses,
            protocols=protocols,
            ranges=ranges,
            preset=preset,
            listen_ms=listen_ms,
            print_report=print_report,
        )

    # alias for discoverers / CLI copy
    smoke = inventory

    # -- Config (CFG PDU — actuator table get/set/save) -------------------------------

    def cfg_get_table(self, *, timeout_s: float = 1.5) -> List[dict]:
        """Read live RAM actuator_table[] (paged). Not a flash/persist indicator."""
        self._require_debug("hub.debug.cfg_get_table")
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
        self._require_debug("hub.debug.cfg_set_slot")
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

    def cfg_apply_table(
        self,
        rows: Sequence[Optional[Mapping[str, Any]]],
        *,
        disable_missing: bool = True,
        timeout_s: float = 1.5,
    ) -> List[dict]:
        """RAM-apply a full table (blank missing slots when ``disable_missing``)."""
        self._require_debug("hub.debug.cfg_apply_table")
        return _config.apply_table(
            self._connection,
            rows,
            disable_missing=disable_missing,
            timeout_s=timeout_s,
        )

    def cfg_fetch_bundle(self, *, timeout_s: float = 1.5) -> Dict[str, Any]:
        """Live RAM snapshot: ``{actuators, periph}`` (host memory; not flash)."""
        self._require_debug("hub.debug.cfg_fetch_bundle")
        return _config.fetch_bundle(self._connection, timeout_s=timeout_s)

    def cfg_apply_bundle(
        self,
        bundle: Mapping[str, Any],
        *,
        disable_missing: bool = True,
        timeout_s: float = 1.5,
    ) -> Dict[str, Any]:
        """RAM-apply actuators+periph bundle (no flash SAVE)."""
        self._require_debug("hub.debug.cfg_apply_bundle")
        return _config.apply_bundle(
            self._connection,
            bundle,
            telemetry=self._telemetry,
            disable_missing=disable_missing,
            timeout_s=timeout_s,
        )

    def cfg_save_nvm(self, *, timeout_s: float = 8.0) -> dict:
        """Flash-persist the full NVM v2 image (actuators + periph + flags)."""
        self._require_debug("hub.debug.cfg_save_nvm")
        return _config.save_nvm(self._connection, timeout_s=timeout_s)

    def cfg_load_nvm(self, *, timeout_s: float = 1.5) -> dict:
        """Reload flash → RAM (wipes unsaved RAM edits)."""
        self._require_debug("hub.debug.cfg_load_nvm")
        return _config.load_nvm(self._connection, timeout_s=timeout_s)

    def cfg_get_periph(self, *, timeout_s: float = 1.5) -> dict:
        """Read neck servos + LED defaults + listen_pdu flag (RAM)."""
        self._require_debug("hub.debug.cfg_get_periph")
        return _config.get_periph(self._connection, timeout_s=timeout_s)

    def cfg_set_periph(
        self,
        periph: dict,
        *,
        persist: bool = False,
        timeout_s: float = 1.5,
    ) -> dict:
        """Write peripheral block to RAM; ``persist=True`` runs CFG SAVE (NVM v2)."""
        self._require_debug("hub.debug.cfg_set_periph")
        return _config.set_periph(
            self._connection,
            self._telemetry,
            periph,
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

    def discover_zeroerr(
        self,
        *,
        bus: int = 1,
        start: int = 1,
        end: int = 16,
        sdo_timeout_ms: int = 30,
    ) -> Optional[int]:
        """CANopen node sweep via SDO-read 0x1018 (Identity Object)."""
        self._require_debug("hub.debug.discover_zeroerr")
        return _zeroerr.discover(
            self._connection, self._telemetry, bus=bus, start=start, end=end,
            sdo_timeout_ms=sdo_timeout_ms,
        )

    def discover_zeroerr_all(
        self,
        *,
        bus: int = 1,
        start: int = 1,
        end: int = 16,
        sdo_timeout_ms: int = 30,
    ) -> List[int]:
        """Like discover_zeroerr but returns every unique hit (discovery order)."""
        self._require_debug("hub.debug.discover_zeroerr_all")
        return _zeroerr.discover_all(
            self._connection, self._telemetry, bus=bus, start=start, end=end,
            sdo_timeout_ms=sdo_timeout_ms,
        )

    def probe_zeroerr(
        self,
        *,
        bus: int = 1,
        node_id: int,
        sdo_timeout_ms: int = 30,
    ) -> Optional[dict]:
        """Single-node identity probe (0x1018) — no session/sweep bookkeeping."""
        self._require_debug("hub.debug.probe_zeroerr")
        return _zeroerr.probe_node(
            self._connection, self._telemetry, bus=bus, node_id=node_id,
            sdo_timeout_ms=sdo_timeout_ms,
        )

    def boot_zeroerr(
        self,
        *,
        bus: int = 1,
        node_id: int,
        sdo_timeout_ms: int = 30,
    ) -> bool:
        """Stage-1 bring-up: NMT + PDO1 remap only, NO controlword — brake
        stays engaged, cannot move the shaft. See zeroerr.py's ``boot``."""
        self._require_debug("hub.debug.boot_zeroerr")
        return _zeroerr.boot(
            self._connection, self._telemetry, bus=bus, node_id=node_id,
            sdo_timeout_ms=sdo_timeout_ms,
        )

    def read_position_zeroerr(
        self,
        *,
        bus: int = 1,
        node_id: int,
        sdo_timeout_ms: int = 30,
    ) -> Optional[float]:
        """SDO read of 0x6064 (radians) — no enable, no PDO/NMT-Operational
        required. Safe before boot_zeroerr, before any CFG slot exists."""
        self._require_debug("hub.debug.read_position_zeroerr")
        return _zeroerr.read_position(
            self._connection, self._telemetry, bus=bus, node_id=node_id,
            sdo_timeout_ms=sdo_timeout_ms,
        )
