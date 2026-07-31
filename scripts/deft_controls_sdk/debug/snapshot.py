"""Programmatic CFG / bandwidth snapshots (same data the suite ``show`` TUI uses).

Requires ``mode=\"debug\"`` for ``collect_cfg`` (CFG RPC). Notebook::

    with HostProxy.connect("COM5", mode="debug", armed=False) as proxy:
        cfg = proxy.cfg_snapshot()
        print(cfg["enabled_count"], cfg.get("listen_pdu"))
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from deft_controls_sdk.config import (
    PROTO_CUBEMARS,
    PROTO_DAMIAO,
    PROTO_NONE,
    PROTO_ROBSTRIDE,
    PROTO_ZEROERR,
)
from deft_controls_sdk.debug.stream_pause import pause_plant_stream

_PROTO_NAMES = {
    PROTO_NONE: "none",
    PROTO_ROBSTRIDE: "robstride",
    PROTO_CUBEMARS: "cubemars",
    PROTO_DAMIAO: "damiao",
    PROTO_ZEROERR: "zeroerr",
}


def protocol_name(protocol: int) -> str:
    return _PROTO_NAMES.get(int(protocol), f"proto({protocol})")


def _normalize_table(table: List[dict]) -> List[dict]:
    out: List[dict] = []
    for i, row in enumerate(table):
        if row is None:
            continue
        slot = int(row.get("slot", i))
        proto = int(row.get("protocol", 0))
        out.append(
            {
                "slot": slot,
                "enabled": bool(row.get("enabled", False)),
                "bus": int(row.get("bus", 0)),
                "protocol": proto,
                "protocol_name": protocol_name(proto),
                "motor_id": int(row.get("motor_id", 0)),
                "motor_id_hex": f"0x{int(row.get('motor_id', 0)) & 0xFF:02X}",
            }
        )
    return out


def collect_cfg(proxy: Any) -> Dict[str, Any]:
    """Live RAM CFG: actuator slots + periph (neck/LED/listen_pdu).

    ``proxy`` is a ``HostProxy`` (needs ``.hub``).
    """
    with pause_plant_stream(proxy.hub):
        table = proxy.hub.debug.cfg_get_table()
        try:
            periph = dict(proxy.hub.debug.cfg_get_periph())
            periph_ok = True
            periph_error = None
        except Exception as exc:  # noqa: BLE001
            periph = {}
            periph_ok = False
            periph_error = str(exc)
    rows = _normalize_table(table)
    enabled = [r for r in rows if r["enabled"]]
    out: Dict[str, Any] = {
        "slots": rows,
        "enabled_count": len(enabled),
        "total": len(rows),
        "periph_ok": periph_ok,
    }
    if periph_ok:
        out["listen_pdu"] = bool(periph.get("listen_pdu", False))
        out["flags"] = int(periph.get("flags", 0))
        out["servos"] = list(periph.get("servos") or [])
        out["led"] = dict(periph.get("led") or {})
    else:
        out["periph_error"] = periph_error
    return out


def collect_bandwidth(proxy: Any) -> Dict[str, Any]:
    """Plant / periph lap timing from latest FB + host stream rates."""
    hub = proxy.hub
    conn = hub._connection  # noqa: SLF001
    hot = getattr(conn, "_hot_stats", None) or {}
    telem = getattr(hub, "telemetry", None)
    snap = None
    if telem is not None:
        try:
            snap = telem.snapshot()
        except Exception:  # noqa: BLE001
            snap = None

    def _from_snap(name: str) -> Optional[Any]:
        if snap is None:
            return None
        return getattr(snap, name, None)

    return {
        "stream_hz": hub.stream_hz,
        "telemetry_hz": hub.telemetry_hz,
        "act_lap_ms": _from_snap("lap_ms") if _from_snap("lap_ms") is not None else hot.get("lap_ms"),
        "act_lap_peak_ms": (
            _from_snap("lap_max_ms")
            if _from_snap("lap_max_ms") is not None
            else hot.get("lap_max_ms")
        ),
        "periph_lap_ms": (
            _from_snap("periph_lap_ms")
            if _from_snap("periph_lap_ms") is not None
            else hot.get("periph_lap_ms")
        ),
        "periph_lap_peak_ms": (
            _from_snap("periph_lap_max_ms")
            if _from_snap("periph_lap_max_ms") is not None
            else hot.get("periph_lap_max_ms")
        ),
        "ticks_pending": (
            _from_snap("ticks_pending")
            if _from_snap("ticks_pending") is not None
            else hot.get("ticks_pending")
        ),
        "note": "lap_* from plant FB / telemetry; stream_hz is host TX rate",
    }


__all__ = [
    "collect_bandwidth",
    "collect_cfg",
    "protocol_name",
]
