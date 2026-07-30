"""Bandwidth load-matrix helpers (scenarios × host Hz).

Product slot map mirrors YAM enabled actuators (platform HostProxy constants —
no ``vbeta`` import). Used by the bandwidth TUI / ``test --bandwidth``.

Note: virtual bandwidth (``rx_sim=True``) synthesizes ACTUATOR RX and does **not**
exercise real MCP SPI RX — USB/plant-TX stress only. Hardware bandwidth keeps
``rx_sim=False``.
"""
from __future__ import annotations

import statistics
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from deft_controls_sdk.config import (
    BASE_DRIVE_SLOTS,
    BASE_STEER_SLOTS,
    LEFT_ARM_SLOTS,
    RIGHT_ARM_SLOTS,
)

# Enabled product slots per host bus 1..6 (CH3 lift disabled → empty).
PRODUCT_BY_BUS: Dict[int, List[int]] = {
    1: list(LEFT_ARM_SLOTS),
    2: list(RIGHT_ARM_SLOTS),
    3: [],
    4: [BASE_STEER_SLOTS["BwC"], BASE_DRIVE_SLOTS["BpC"]],
    5: [BASE_STEER_SLOTS["BwR"], BASE_DRIVE_SLOTS["BpR"]],
    6: [BASE_STEER_SLOTS["BwL"], BASE_DRIVE_SLOTS["BpL"]],
}

SCENARIO_NAMES: Tuple[str, ...] = (
    "idle",
    "ch1",
    "ch2",
    "ch3",
    "ch4",
    "ch5",
    "ch6",
    "fdcan",
    "mcp",
    "arms",
    "all",
)

SCENARIO_BLURBS: Dict[str, str] = {
    "idle": "no actuator hold (USB / empty plant)",
    "ch1": "left arm FDCAN (slots 0–6)",
    "ch2": "right arm FDCAN (slots 7–13)",
    "ch3": "lift bus (empty in product CFG)",
    "ch4": "base MCP rail CH4",
    "ch5": "base MCP rail CH5",
    "ch6": "base MCP rail CH6",
    "fdcan": "CH1–3 union (product-enabled)",
    "mcp": "CH4–6 union — SPI-CAN critical path",
    "arms": "CH1+CH2 arms only",
    "all": "every product-enabled slot",
}

DEFAULT_HZ_LIST: Tuple[float, ...] = (40.0, 100.0, 200.0, 500.0)


def parse_hz_list(text: str) -> List[float]:
    raw = (text or "").replace(",", " ").split()
    if not raw:
        raise ValueError("empty hz list")
    out = [float(tok) for tok in raw]
    if any(h <= 0 for h in out):
        raise ValueError("hz values must be > 0")
    return out


def scenario_slots(
    name: str,
    by_bus: Optional[Mapping[int, Sequence[int]]] = None,
) -> List[int]:
    """Resolve scenario name → sorted unique slot list."""
    key = (name or "").strip().lower()
    buses = by_bus if by_bus is not None else PRODUCT_BY_BUS

    def _bus(n: int) -> List[int]:
        return [int(s) for s in buses.get(n, ())]

    if key == "idle":
        return []
    if key in ("ch1", "ch2", "ch3", "ch4", "ch5", "ch6"):
        return sorted(_bus(int(key[-1])))
    if key == "fdcan":
        return sorted(_bus(1) + _bus(2) + _bus(3))
    if key == "mcp":
        return sorted(_bus(4) + _bus(5) + _bus(6))
    if key == "arms":
        return sorted(_bus(1) + _bus(2))
    if key == "all":
        seen: List[int] = []
        for n in sorted(buses.keys()):
            for s in _bus(n):
                if s not in seen:
                    seen.append(s)
        return sorted(seen)
    raise ValueError(f"unknown scenario {name!r}; known: {', '.join(SCENARIO_NAMES)}")


def render_report(scenario: str, results: Sequence[Mapping]) -> str:
    """Markdown-ish aggregate table grouped by host Hz."""
    by_hz: Dict[float, List[Mapping]] = {}
    for row in results:
        hz = float(row.get("hz", 0.0))
        by_hz.setdefault(hz, []).append(row)

    lines = [
        f"## Scenario: {scenario}",
        "",
        "| hz | n | ok | fb_hz (mean) | ack_lag_max | act_lap_ms | act_lap_max | periph_max |",
        "|---:|---:|:---|---:|---:|---:|---:|---:|",
    ]
    for hz in sorted(by_hz.keys()):
        rows = by_hz[hz]
        n = len(rows)
        ok_n = sum(1 for r in rows if r.get("ok"))
        fb = [float(r["raw_fb_hz"]) for r in rows if r.get("raw_fb_hz") is not None]
        lags = [int(r["ack_lag_max"]) for r in rows if r.get("ack_lag_max") is not None]
        laps = [float(r["lap_ms_mean"]) for r in rows if r.get("lap_ms_mean") is not None]
        lap_max = [int(r["lap_max_ms"]) for r in rows if r.get("lap_max_ms") is not None]
        periph = [
            int(r["periph_lap_max_ms"])
            for r in rows
            if r.get("periph_lap_max_ms") is not None
        ]
        lines.append(
            "| {hz:g} | {n} | {ok} | {fb} | {lag} | {lap} | {lmax} | {pmax} |".format(
                hz=hz,
                n=n,
                ok=f"{ok_n}/{n}",
                fb=f"{statistics.mean(fb):.1f}" if fb else "n/a",
                lag=str(max(lags)) if lags else "n/a",
                lap=f"{statistics.mean(laps):.2f}" if laps else "n/a",
                lmax=str(max(lap_max)) if lap_max else "n/a",
                pmax=str(max(periph)) if periph else "n/a",
            )
        )
    lines.append("")
    return "\n".join(lines)


def render_matrix_summary(
    by_scenario: Mapping[str, Sequence[Mapping]],
    *,
    rx_sim: bool,
) -> str:
    """One section per scenario + footer note on rx_sim."""
    parts = [render_report(name, rows) for name, rows in by_scenario.items()]
    if rx_sim:
        parts.append(
            "_Note: rx_sim=ON — ACTUATOR RX is synthesized; MCP SPI RX cost is not "
            "measured. Re-run with rx_sim off for wire-path critique._\n"
        )
    return "\n".join(parts)


def default_scenarios_for_matrix() -> List[str]:
    """Scenarios worth comparing FDCAN vs MCP vs full load."""
    return ["idle", "ch1", "mcp", "arms", "all"]
