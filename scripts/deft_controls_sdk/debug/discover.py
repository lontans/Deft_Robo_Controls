"""Queued multi-bus / multi-protocol discover.

RobStride with multiple buses uses one multi-bus SESSION_BEGIN + overlapping
listen (FW ``robstride_probe_id_buses``). Damiao / CubeMars stay per-bus
sequential for now. CubeMars is MIT-frame discover (no register-read scheme,
unlike Damiao); ZeroErr is a CANopen SDO node sweep (0x1018), not MIT-shaped.
Protocol queue is still outer (no cross-protocol overlap on the same session).
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence, Tuple

from . import cubemars as _cubemars
from . import damiao as _damiao
from . import robstride as _robstride
from . import zeroerr as _zeroerr

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

# Discoverable protocols in default ``all`` order.
DISCOVER_PROTOCOLS_DEFAULT: Tuple[str, ...] = ("robstride", "damiao", "cubemars", "zeroerr")
DISCOVER_PROTOCOL_ALIASES = {
    "robstride": "robstride",
    "rs": "robstride",
    "rs02": "robstride",
    "rs2": "robstride",
    "1": "robstride",
    "damiao": "damiao",
    "dm": "damiao",
    "3": "damiao",
    "cubemars": "cubemars",
    "cm": "cubemars",
    "ak": "cubemars",
    "2": "cubemars",
    "zeroerr": "zeroerr",
    "ze": "zeroerr",
    "erob": "zeroerr",
    "4": "zeroerr",
}
DEFAULT_ID_RANGE: Dict[str, Tuple[int, int]] = {
    "robstride": (0x40, 0x80),
    "damiao": (1, 16),
    "cubemars": (1, 16),
    "zeroerr": (1, 16),
}


def hex_id(n: int) -> str:
    """One CAN/motor id as ``0xNN`` (low 8 bits)."""
    return f"0x{int(n) & 0xFF:02X}"


def as_hex(obj: Any) -> Any:
    """Recursively format discover ids for display.

    ``int`` → ``\"0xNN\"``; ``list``/``tuple`` → list of hex; ``dict`` (e.g.
    ``discover_robstride_by_bus`` ``{bus: [ids…]}``) → same keys with hex values.
    Non-int leaves are returned unchanged.

    Notebook::

        from deft_controls_sdk.debug import as_hex
        print(as_hex(hub.debug.discover_robstride_by_bus(buses=[5, 6])))
        # {5: ['0x70', '0x74'], 6: ['0x75']}
    """
    if isinstance(obj, Mapping):
        return {k: as_hex(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [as_hex(x) for x in obj]
    if isinstance(obj, int) and not isinstance(obj, bool):
        return hex_id(obj)
    return obj


_hex_id = hex_id  # internal alias (queued discover JSON)


def parse_bus_list(text: str) -> List[int]:
    """Parse ``1,5`` / ``1 2 3`` / ``all`` into unique buses 1..6 (order preserved)."""
    raw = (text or "").strip().lower()
    if not raw:
        raise ValueError("empty bus list")
    if raw in ("all", "*"):
        return list(range(1, 7))
    tokens = raw.replace(",", " ").split()
    buses: List[int] = []
    for tok in tokens:
        bus = int(tok, 0)
        if not (1 <= bus <= 6):
            raise ValueError(f"bus {bus} out of range 1..6")
        if bus not in buses:
            buses.append(bus)
    if not buses:
        raise ValueError("empty bus list")
    return buses


def parse_protocol_queue(text: str) -> List[str]:
    """Parse ``robstride,damiao`` / ``all`` into ordered unique protocol names.

    ``all`` expands to default discover order (robstride, damiao, …).
    First token runs first when queued.
    """
    raw = (text or "").strip().lower()
    if not raw:
        raise ValueError("empty protocol list")
    if raw in ("all", "*"):
        return list(DISCOVER_PROTOCOLS_DEFAULT)
    tokens = [t for t in raw.replace(",", " ").split() if t]
    out: List[str] = []
    for tok in tokens:
        if tok in ("all", "*"):
            for name in DISCOVER_PROTOCOLS_DEFAULT:
                if name not in out:
                    out.append(name)
            continue
        name = DISCOVER_PROTOCOL_ALIASES.get(tok)
        if name is None:
            known = ", ".join(DISCOVER_PROTOCOLS_DEFAULT) + ", all"
            raise ValueError(f"unknown protocol {tok!r}; known: {known}")
        if name not in out:
            out.append(name)
    if not out:
        raise ValueError("empty protocol list")
    return out


def _discover_one(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    protocol: str,
    bus: int,
    start: int,
    end: int,
    listen_ms: int,
) -> List[int]:
    if protocol == "robstride":
        return list(
            _robstride.discover_all(
                connection, telemetry, bus=bus, start=start, end=end
            )
        )
    if protocol == "damiao":
        return list(
            _damiao.discover_all(
                connection,
                telemetry,
                bus=bus,
                start=start,
                end=end,
                listen_ms=listen_ms,
            )
        )
    if protocol == "cubemars":
        return list(
            _cubemars.discover_all(
                connection,
                telemetry,
                bus=bus,
                start=start,
                end=end,
                listen_ms=listen_ms,
            )
        )
    if protocol == "zeroerr":
        return list(
            _zeroerr.discover_all(
                connection,
                telemetry,
                bus=bus,
                start=start,
                end=end,
                sdo_timeout_ms=listen_ms,
            )
        )
    raise ValueError(f"unsupported discover protocol {protocol!r}")


def discover_queued(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    buses: Sequence[int],
    protocols: Sequence[str],
    ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
    listen_ms: int = 40,
) -> List[dict]:
    """Run single-bus discover for each ``(protocol, bus)`` in queue order.

    Order: protocol queue first, then each bus. Not parallel — FW allows one
    bench session bus at a time. Returns one result dict per cell::

        {"bus", "protocol", "id_start", "id_end", "ids", "ok", "error"?}

    ``ids`` are hex strings (``0xNN``). ``ranges`` defaults per protocol from
    :data:`DEFAULT_ID_RANGE`.
    """
    if not buses:
        raise ValueError("empty bus list")
    if not protocols:
        raise ValueError("empty protocol list")
    resolved: Dict[str, Tuple[int, int]] = {}
    for protocol in protocols:
        if protocol not in DEFAULT_ID_RANGE:
            raise ValueError(f"unsupported discover protocol {protocol!r}")
        if ranges is not None and protocol in ranges:
            start, end = ranges[protocol]
        else:
            start, end = DEFAULT_ID_RANGE[protocol]
        if start > end:
            raise ValueError(f"{protocol}: id start {start} > end {end}")
        resolved[protocol] = (int(start), int(end))

    results: List[dict] = []
    bus_list = [int(b) for b in buses]

    for protocol in protocols:
        start, end = resolved[protocol]
        id_start: object = _hex_id(start) if protocol == "robstride" else start
        id_end: object = _hex_id(end) if protocol == "robstride" else end

        # RobStride: one overlapping multi-bus probe when ≥2 buses.
        if protocol == "robstride" and len(bus_list) > 1:
            try:
                by_bus = _robstride.discover_all_by_bus(
                    connection,
                    telemetry,
                    buses=bus_list,
                    start=start,
                    end=end,
                )
                for bus_i in bus_list:
                    ids = list(by_bus.get(bus_i, []))
                    results.append(
                        {
                            "bus": bus_i,
                            "protocol": protocol,
                            "id_start": id_start,
                            "id_end": id_end,
                            "ids": [_hex_id(i) for i in ids],
                            "ok": True,
                            "multi_bus": True,
                        }
                    )
            except Exception as exc:  # noqa: BLE001
                for bus_i in bus_list:
                    results.append(
                        {
                            "bus": bus_i,
                            "protocol": protocol,
                            "id_start": id_start,
                            "id_end": id_end,
                            "ids": [],
                            "ok": False,
                            "error": str(exc),
                            "multi_bus": True,
                        }
                    )
            continue

        for bus_i in bus_list:
            if not (1 <= bus_i <= 6):
                results.append(
                    {
                        "bus": bus_i,
                        "protocol": protocol,
                        "id_start": id_start,
                        "id_end": id_end,
                        "ids": [],
                        "ok": False,
                        "error": f"bus {bus_i} out of range 1..6",
                    }
                )
                continue
            try:
                ids = _discover_one(
                    connection,
                    telemetry,
                    protocol=protocol,
                    bus=bus_i,
                    start=start,
                    end=end,
                    listen_ms=listen_ms,
                )
            except Exception as exc:  # noqa: BLE001 — surface per-cell failure
                results.append(
                    {
                        "bus": bus_i,
                        "protocol": protocol,
                        "id_start": id_start,
                        "id_end": id_end,
                        "ids": [],
                        "ok": False,
                        "error": str(exc),
                    }
                )
                continue
            results.append(
                {
                    "bus": bus_i,
                    "protocol": protocol,
                    "id_start": id_start,
                    "id_end": id_end,
                    "ids": [_hex_id(i) for i in ids],
                    "ok": True,
                }
            )
    return results


def summarize_queued(results: Sequence[Mapping]) -> dict:
    """Compact summary for JSON / CLI (hit_count + buses touched)."""
    hits = [r for r in results if r.get("ok") and r.get("ids")]
    buses = list(dict.fromkeys(int(r["bus"]) for r in results))
    protocols = list(dict.fromkeys(str(r["protocol"]) for r in results))
    return {
        "buses": buses,
        "protocols": protocols,
        "results": list(results),
        "hit_count": sum(len(r["ids"]) for r in hits),
        "note": (
            "RobStride multi-bus overlaps listen; Damiao still per-bus. "
            "ids are hex strings"
        ),
    }
