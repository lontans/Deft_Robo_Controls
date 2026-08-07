"""System panel — thin wrappers over existing debug/inventory/CFG primitives.

No new business logic: every function here just adapts an already-working
SDK call to the shared panel calling convention (``fn(proxy, **params) ->
dict``) so an HTTP dashboard layer can call it directly. See each function's
docstring for the primitive it wraps.

Exception: :func:`run_bandwidth_panel` does not take a ``proxy`` — bandwidth
timing needs its own exclusive ``mode="bandwidth"`` connection (Windows CDC
is exclusive-open, and bandwidth timing must not share the USB pipe with
debug-lane RPCs), so it opens and closes its own ``HostProxy`` around a bare
COM port string.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, Optional, Sequence, Tuple

from deft_controls_sdk.debug import collect_cfg, run_inventory
from deft_controls_sdk.debug.inventory import _collect_pdu
from deft_controls_sdk.debug.metrics import measure_hold
from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.link.exchange import DEFAULT_BAUD

if TYPE_CHECKING:
    from deft_controls_sdk.host_proxy import HostProxy as HostProxyType

__all__ = [
    "run_inventory_panel",
    "run_bandwidth_panel",
    "show_cfg_panel",
    "set_cfg_slot_panel",
    "set_cfg_periph_panel",
    "apply_cfg_table_panel",
    "save_cfg_nvm_panel",
    "host_link_eval_panel",
]

# Representative "hold in place" slots for the default bandwidth panel probe
# (mirrors the light single-hold path in debug/suite/test_bandwidth.py).
_DEFAULT_BANDWIDTH_SLOTS: Tuple[int, ...] = (0, 1)


def run_inventory_panel(proxy: "HostProxyType", **kwargs: Any) -> dict:
    """Wrap :func:`deft_controls_sdk.debug.run_inventory`.

    Requires the proxy's connection to already be ``mode="debug"`` — this
    wrapper does not enforce that itself, it just passes through and lets
    the underlying call raise. ``preset="full"`` can take minutes; pass
    ``preset``/``ranges``/``buses``/``protocols`` to pick something faster
    (e.g. ``preset="bench"``).
    """
    return run_inventory(proxy, **kwargs)


def run_bandwidth_panel(
    port: str,
    *,
    baud: int = DEFAULT_BAUD,
    seconds: float = 3.0,
    hz: float = 40.0,
    slots: Optional[Sequence[int]] = None,
    listen_pdu: bool = False,
    rx_sim: bool = False,
    label: str = "system panel bandwidth hold",
    print_report: bool = True,
) -> dict:
    """Bandwidth hold-in-place measurement over its own ``mode="bandwidth"`` link.

    Opens a dedicated ``HostProxy.connect(port, mode="bandwidth")`` session
    (bandwidth timing cannot share a USB pipe/session with a debug-mode
    dashboard connection), holds ``slots`` (default: two representative
    slots) at a tiny non-zero position via :func:`measure_hold`, then closes
    the connection. Mirrors the connect-and-measure pattern in
    ``debug/suite/test_bandwidth.py``'s private ``_hold_once`` (not called
    directly — that helper is private/argparse-adjacent).
    """
    held_slots: Tuple[int, ...] = (
        _DEFAULT_BANDWIDTH_SLOTS if slots is None else tuple(int(s) for s in slots)
    )
    stream_hz = float(hz)
    with HostProxy.connect(
        port,
        baud=baud,
        stream_hz=stream_hz,
        telemetry_hz=stream_hz,
        armed=False,
        listen_pdu=listen_pdu,
        mode="bandwidth",
    ) as proxy:
        hub = proxy.hub
        hub.set_plant_apply(True, send=True)
        if rx_sim:
            hub.set_rx_sim(True)
        desires: Dict[int, ActuatorDesire] = {
            int(s): ActuatorDesire(position=1e-6) for s in held_slots
        }
        report = measure_hold(
            hub,
            label,
            desires,
            seconds=float(seconds),
            hz=float(hz),
            expected_stm32_mode="bandwidth",
            print_report=print_report,
        )
        if rx_sim:
            try:
                hub.set_rx_sim(False)
            except Exception:  # noqa: BLE001
                pass

    report = dict(report)
    report["hz"] = float(hz)
    report["seconds"] = float(seconds)
    report["rx_sim"] = bool(rx_sim)
    report["slots"] = list(held_slots)
    report["port"] = port
    return report


def show_cfg_panel(proxy: "HostProxyType") -> dict:
    """Wrap :func:`deft_controls_sdk.debug.snapshot.collect_cfg`. Requires ``mode="debug"``."""
    return collect_cfg(proxy)


def set_cfg_slot_panel(
    proxy: "HostProxyType",
    *,
    slot: int,
    bus: int,
    protocol: int,
    motor_id: int,
    master_id: int = 0,
    enabled: bool = True,
    persist: bool = False,
) -> dict:
    """Wrap ``proxy.hub.debug.cfg_set_slot``. RAM apply always; flash persist if ``persist``."""
    return proxy.hub.debug.cfg_set_slot(
        slot=slot,
        bus=bus,
        protocol=protocol,
        motor_id=motor_id,
        master_id=master_id,
        enabled=enabled,
        persist=persist,
    )


def set_cfg_periph_panel(
    proxy: "HostProxyType",
    periph: dict,
    *,
    persist: bool = False,
) -> dict:
    """Wrap ``proxy.hub.debug.cfg_set_periph``. RAM apply always; NVM SAVE if ``persist``."""
    return proxy.hub.debug.cfg_set_periph(periph, persist=persist)


def apply_cfg_table_panel(
    proxy: "HostProxyType",
    *,
    rows: Sequence[Optional[Dict[str, Any]]],
    disable_missing: bool = True,
    persist: bool = False,
) -> dict:
    """Wrap ``proxy.hub.debug.cfg_apply_table`` — RAM-replace the whole actuator
    table in one call (e.g. a pasted/preset table from the dashboard's CFG
    panel), rather than N individual ``set_cfg_slot`` round trips.
    ``disable_missing=True`` (default): any slot with no row in ``rows`` is
    SET disabled/PROTO_NONE — this is a real *replace*, not a merge. Pass
    ``persist=True`` to also flash-save afterward (equivalent to calling
    :func:`save_cfg_nvm_panel` next)."""
    responses = proxy.hub.debug.cfg_apply_table(rows, disable_missing=disable_missing)
    result: Dict[str, Any] = {"responses": responses, "slot_count": len(responses)}
    if persist:
        result["save"] = proxy.hub.debug.cfg_save_nvm()
    return result


def save_cfg_nvm_panel(proxy: "HostProxyType") -> dict:
    """Wrap ``proxy.hub.debug.cfg_save_nvm`` — flash-persist the full NVM v2 image."""
    return proxy.hub.debug.cfg_save_nvm()


def host_link_eval_panel(
    proxy: "HostProxyType",
    *,
    require_peer: bool = False,
) -> dict:
    """Host-link (PDU) evaluation: NVM ``listen_pdu`` + wire kill-byte mirror.

    Reuses ``deft_controls_sdk.debug.inventory._collect_pdu`` (the same logic
    the inventory panel's ``pdu`` section uses) rather than re-deriving it.
    """
    return _collect_pdu(proxy.hub, require_peer=require_peer)
