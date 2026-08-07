"""``discover_panel`` — thin dashboard wrapper around ``DebugAPI.discover_queued``.

Routes through ``proxy.hub.debug.discover_queued`` (see
``deft_controls_sdk/debug/__init__.py``) rather than calling
``deft_controls_sdk/debug/discover.py:discover_queued`` directly against the
raw connection — ``DebugAPI`` is the one place that already normalizes the
``require_debug_mode`` gating and the per-plugin (robstride / damiao /
cubemars / zeroerr) discover/probe signature differences underneath, and it
is a clean 1:1 forward to the real primitive (same keyword args, same return
shape) so there is nothing this wrapper needs to reimplement.

Synchronous / blocking by design — a queued discover across several buses
and protocols can take seconds to minutes ("FW allows one bench session bus
at a time", per ``discover_queued``'s own docstring). ``estimated_s`` in the
returned dict is a rough heuristic so a dashboard UI can show a "this may
take about N seconds" hint before kicking it off — it is not a promise.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Mapping, Optional, Sequence, Tuple

from deft_controls_sdk.debug.discover import DEFAULT_ID_RANGE, summarize_queued

if TYPE_CHECKING:
    from deft_controls_sdk.host_proxy import HostProxy

__all__ = ["discover_panel"]

# Rough per-ID probe cost, seconds. RobStride discover doesn't take a
# listen_ms knob (fixed-ish FW timeout per id in the sweep); the other
# protocols are driven directly by the caller's listen_ms /
# sdo_timeout_ms, so we use that for them instead.
_ROBSTRIDE_PER_ID_S = 0.05
# Fixed per-(protocol,bus) overhead: SESSION_BEGIN/END, USB round trips,
# reg-scan fallback slack, etc. Deliberately generous — better to over-
# estimate a "this may take a while" hint than under-promise it.
_PER_CELL_OVERHEAD_S = 0.5
# Small extra per additional bus in a RobStride multi-bus overlapping probe
# (bus-mask setup, more replies to sort per listen window) — much less than
# treating each extra bus as a full separate sweep.
_MULTI_BUS_OVERHEAD_S = 0.3


def _resolve_range(
    protocol: str, ranges: Optional[Mapping[str, Tuple[int, int]]]
) -> Tuple[int, int]:
    if ranges is not None and protocol in ranges:
        start, end = ranges[protocol]
    else:
        start, end = DEFAULT_ID_RANGE.get(protocol, (1, 16))
    return int(start), int(end)


def estimate_discover_seconds(
    *,
    buses: Sequence[int],
    protocols: Sequence[str],
    ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
    listen_ms: int = 40,
) -> float:
    """Rough wall-clock estimate for a :func:`discover_panel` call.

    Heuristic only (see module docstring) — sums a per-cell cost across the
    ``(protocol, bus)`` queue, honouring the one case that isn't per-bus
    sequential: RobStride with 2+ buses runs a single overlapping multi-bus
    probe instead of one sweep per bus (see ``discover_queued``'s docstring).
    """
    bus_list = list(buses)
    n_buses = max(1, len(bus_list))
    per_id_ms = max(1, int(listen_ms)) / 1000.0
    total = 0.0
    for protocol in protocols:
        start, end = _resolve_range(protocol, ranges)
        id_count = max(0, end - start + 1)
        per_id_s = _ROBSTRIDE_PER_ID_S if protocol == "robstride" else per_id_ms
        cell_s = id_count * per_id_s + _PER_CELL_OVERHEAD_S
        if protocol == "robstride" and n_buses > 1:
            # One overlapping multi-bus probe, not N sequential sweeps — but
            # still a little slower than a single bus (session bus-mask
            # setup, more replies to sort per listen window).
            total += cell_s + _MULTI_BUS_OVERHEAD_S * (n_buses - 1)
        else:
            total += cell_s * n_buses
    return round(total, 1)


def discover_panel(
    proxy: "HostProxy",
    *,
    buses: List[int],
    protocols: List[str],
    ranges: Optional[Dict[str, Tuple[int, int]]] = None,
    listen_ms: int = 40,
) -> dict:
    """Queued multi-bus / multi-protocol discover, wrapped for the dashboard.

    Blocking — this can take anywhere from a couple seconds (one bus, one
    protocol, a narrow range) to minutes (``all`` buses × ``all`` protocols
    at the default id ranges). Not parallel across buses; see
    ``DebugAPI.discover_queued`` / ``debug/discover.py``.

    Returns
    -------
    dict with:
        ``estimated_s`` — heuristic duration hint (seconds), computed before
            the call so a caller could in principle show it before the
            blocking call even returns (e.g. a naive two-phase HTTP flow).
        ``buses`` / ``protocols`` / ``listen_ms`` — echoed request params.
        ``results`` — raw per-cell list from ``discover_queued``.
        ``summary`` — ``summarize_queued(results)`` (hit_count, buses,
            protocols touched, hex-string ids).
    """
    estimated_s = estimate_discover_seconds(
        buses=buses, protocols=protocols, ranges=ranges, listen_ms=listen_ms
    )
    results = proxy.hub.debug.discover_queued(
        buses=buses,
        protocols=protocols,
        ranges=ranges,
        listen_ms=listen_ms,
    )
    summary = summarize_queued(results)
    return {
        "estimated_s": estimated_s,
        "buses": list(buses),
        "protocols": list(protocols),
        "listen_ms": int(listen_ms),
        "results": results,
        "summary": summary,
    }
