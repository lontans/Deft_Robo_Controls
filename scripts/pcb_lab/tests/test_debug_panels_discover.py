"""Unit tests for ``deft_controls_sdk.debug.panels.discover.discover_panel``.

No hardware / no real COM port: ``proxy.hub.debug`` is a small recording
fake that stands in for ``DebugAPI`` — the panel is only responsible for
calling through with the right kwargs, attaching an ``estimated_s`` hint,
and shaping the return dict; the real discover behaviour is already covered
elsewhere (``debug/discover.py`` and its own protocol modules).
"""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug.panels.discover.discover_panel import (
    discover_panel,
    estimate_discover_seconds,
)


class _FakeDebugAPI:
    """Records the call it received; returns canned ``discover_queued`` rows."""

    def __init__(self, results=None, *, raise_exc: Exception | None = None):
        self._results = results if results is not None else []
        self._raise_exc = raise_exc
        self.calls: list[dict] = []

    def discover_queued(self, *, buses, protocols, ranges=None, listen_ms=40):
        self.calls.append(
            {
                "buses": list(buses),
                "protocols": list(protocols),
                "ranges": dict(ranges) if ranges else None,
                "listen_ms": listen_ms,
            }
        )
        if self._raise_exc is not None:
            raise self._raise_exc
        return self._results


class _FakeHub:
    def __init__(self, debug_api):
        self.debug = debug_api


class _FakeProxy:
    def __init__(self, debug_api):
        self.hub = _FakeHub(debug_api)


def _row(bus, protocol, ids=(), ok=True, **extra):
    row = {
        "bus": bus,
        "protocol": protocol,
        "id_start": 1,
        "id_end": 16,
        "ids": list(ids),
        "ok": ok,
    }
    row.update(extra)
    return row


# -- discover_panel: call-through / shape -----------------------------------------


def test_discover_panel_forwards_kwargs_to_debug_api():
    api = _FakeDebugAPI(results=[_row(1, "robstride", ["0x70"])])
    proxy = _FakeProxy(api)

    discover_panel(
        proxy,
        buses=[1, 2],
        protocols=["robstride", "damiao"],
        ranges={"robstride": (0x40, 0x50)},
        listen_ms=25,
    )

    assert len(api.calls) == 1
    call = api.calls[0]
    assert call["buses"] == [1, 2]
    assert call["protocols"] == ["robstride", "damiao"]
    assert call["ranges"] == {"robstride": (0x40, 0x50)}
    assert call["listen_ms"] == 25


def test_discover_panel_uses_default_listen_ms_when_omitted():
    api = _FakeDebugAPI(results=[])
    proxy = _FakeProxy(api)
    discover_panel(proxy, buses=[1], protocols=["robstride"])
    assert api.calls[0]["listen_ms"] == 40
    assert api.calls[0]["ranges"] is None


def test_discover_panel_return_shape_has_required_keys():
    rows = [_row(1, "robstride", ["0x70", "0x71"]), _row(1, "damiao", [])]
    api = _FakeDebugAPI(results=rows)
    proxy = _FakeProxy(api)

    out = discover_panel(proxy, buses=[1], protocols=["robstride", "damiao"])

    assert set(out.keys()) >= {
        "estimated_s",
        "buses",
        "protocols",
        "listen_ms",
        "results",
        "summary",
    }
    assert out["buses"] == [1]
    assert out["protocols"] == ["robstride", "damiao"]
    assert out["listen_ms"] == 40
    assert out["results"] == rows
    assert isinstance(out["estimated_s"], float)
    assert out["estimated_s"] > 0


def test_discover_panel_summary_matches_summarize_queued():
    """``summary`` must be exactly what ``summarize_queued`` would produce —
    the panel isn't allowed to invent its own summarization logic."""
    from deft_controls_sdk.debug.discover import summarize_queued

    rows = [
        _row(1, "robstride", ["0x70", "0x71"]),
        _row(2, "robstride", []),
        _row(1, "damiao", ["0x03"]),
    ]
    api = _FakeDebugAPI(results=rows)
    proxy = _FakeProxy(api)

    out = discover_panel(proxy, buses=[1, 2], protocols=["robstride", "damiao"])
    assert out["summary"] == summarize_queued(rows)
    assert out["summary"]["hit_count"] == 3


def test_discover_panel_propagates_underlying_errors():
    """Failures must surface as plain exceptions, not be swallowed."""
    api = _FakeDebugAPI(raise_exc=RuntimeError("hub.debug.discover_queued needs mode='debug'"))
    proxy = _FakeProxy(api)
    with pytest.raises(RuntimeError, match="mode='debug'"):
        discover_panel(proxy, buses=[1], protocols=["robstride"])


def test_discover_panel_propagates_value_error_from_empty_lists():
    api = _FakeDebugAPI(raise_exc=ValueError("empty bus list"))
    proxy = _FakeProxy(api)
    with pytest.raises(ValueError, match="empty bus list"):
        discover_panel(proxy, buses=[], protocols=["robstride"])


# -- estimate_discover_seconds: heuristic sanity, not exact value -----------------


def test_estimate_scales_with_more_buses():
    one_bus = estimate_discover_seconds(buses=[1], protocols=["damiao"], listen_ms=40)
    two_bus = estimate_discover_seconds(buses=[1, 2], protocols=["damiao"], listen_ms=40)
    assert two_bus > one_bus


def test_estimate_scales_with_more_protocols():
    one_proto = estimate_discover_seconds(buses=[1], protocols=["damiao"], listen_ms=40)
    two_proto = estimate_discover_seconds(
        buses=[1], protocols=["damiao", "cubemars"], listen_ms=40
    )
    assert two_proto > one_proto


def test_estimate_scales_with_wider_range():
    narrow = estimate_discover_seconds(
        buses=[1], protocols=["damiao"], ranges={"damiao": (1, 2)}, listen_ms=40
    )
    wide = estimate_discover_seconds(
        buses=[1], protocols=["damiao"], ranges={"damiao": (1, 16)}, listen_ms=40
    )
    assert wide > narrow


def test_estimate_robstride_multibus_is_not_multiplied_per_bus():
    """RobStride with 2+ buses runs one overlapping probe (see
    discover_queued's docstring) — the estimate must reflect that, not
    naively multiply a single-bus cost by bus count."""
    one_bus = estimate_discover_seconds(buses=[1], protocols=["robstride"], listen_ms=40)
    three_bus = estimate_discover_seconds(
        buses=[1, 2, 3], protocols=["robstride"], listen_ms=40
    )
    # Must grow far less than linearly (not 3x) since it's one shared sweep.
    assert one_bus < three_bus < one_bus * 3


def test_estimate_is_positive_and_finite_for_default_all_sweep():
    est = estimate_discover_seconds(
        buses=[1, 2, 3, 4, 5, 6],
        protocols=["robstride", "damiao", "cubemars", "zeroerr"],
        listen_ms=40,
    )
    assert est > 0
    assert est < 10_000  # sanity ceiling, not a precise bound
