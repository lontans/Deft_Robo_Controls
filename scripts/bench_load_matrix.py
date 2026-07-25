#!/usr/bin/env python3
"""Durable host-rate / bus-scenario load matrix — successor to the retired
`_tmp_mcp_timing_probe.py` / `_tmp_rate_rx_sweep.py` / `_tmp_load_matrix_report.py`
(see docs/scripts-hygiene.md "Already gone"). Wraps
`deft_controls_sdk.bench.metrics.measure_hold` (same helper
`rs02_channel_bringup.py` / `damiao_channel_bringup.py` already use) across
host TX rates and bus-group scenarios, and writes a report matching the
table shape of `docs/legacy/bench-load-matrix-*.md`.

Plan: docs/bench-optimize-and-load-matrix-plan.md

  cd scripts
  python bench_load_matrix.py --port COM5 --hz 40,100,200,500 --scenario all
  python bench_load_matrix.py --port COM5 --hz 40 --scenario idle
  python bench_load_matrix.py --port COM5 --hz 40,500 --scenario mcp --trials 3 --seconds 8 \
      --report ../docs/bench-load-matrix-<date>.md

Pass gates (see plan §2.3): 40 Hz is the hard gate (ack_lag_max<=2, healthy
fb_hz, plain plant pdu tag). 500 Hz is a capability note, not a hard fail —
the known `host_link_poll_rx()` coalesce-to-newest behavior fails
`cmd_seq_lag`/`ack_lag` there by design, not by regression.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from deft_controls_sdk.bench.metrics import measure_hold
from deft_controls_sdk.vbeta.cfg import ensure_yam_product_cfg

SCENARIOS = ("idle", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "mcp", "all")
DEFAULT_HZ = (40.0, 100.0, 200.0, 500.0)

# Matches rs02_channel_bringup.py's default hold gains — non-idle (kp/kd >
# 0.01) so rx_sim_actuator_on_apply() (App/Src/plant/rx_sim/rx_sim_actuator.c)
# actually synthesizes feedback instead of silently no-op'ing on a blank desire.
DEFAULT_KP = 8.0
DEFAULT_KD = 0.5


def parse_hz_list(raw: str) -> List[float]:
    out = [float(x) for x in raw.split(",") if x.strip()]
    if not out:
        raise ValueError(f"empty --hz list: {raw!r}")
    return out


def scenario_slots(scenario: str, by_bus: Dict[int, List[int]]) -> List[int]:
    """Slots to hold for ``scenario`` given the CFG's bus->enabled-slots map
    (from `ensure_yam_product_cfg`'s return value)."""
    if scenario == "idle":
        return []
    if scenario == "mcp":
        return sorted({s for bus in (4, 5, 6) for s in by_bus.get(bus, [])})
    if scenario == "all":
        return sorted({s for slots in by_bus.values() for s in slots})
    if scenario.startswith("ch") and scenario[2:].isdigit():
        bus = int(scenario[2:])
        if 1 <= bus <= 6:
            return sorted(by_bus.get(bus, []))
    raise ValueError(f"unknown scenario {scenario!r} (choices: {SCENARIOS})")


def run_matrix(
    hub: "ControlsPcbHub",
    *,
    hz_list: Sequence[float],
    scenario: str,
    trials: int,
    seconds: float,
    kp: float = DEFAULT_KP,
    kd: float = DEFAULT_KD,
) -> List[dict]:
    """Run one scenario across every rate in ``hz_list``. Returns the list of
    `measure_hold` result dicts (one per trial per rate), each tagged with
    scenario/hz/trial. Applies product CFG once up front (RAM, no persist);
    toggles ACTUATOR rx_sim on for every scenario except ``idle``."""
    by_bus = ensure_yam_product_cfg(hub, quiet=True)
    slots = scenario_slots(scenario, by_bus)
    desires = {s: ActuatorDesire(position=0.0, velocity=0.0, kp=kp, kd=kd) for s in slots}

    rx_sim = scenario != "idle"
    hub.set_rx_sim(rx_sim)
    try:
        results: List[dict] = []
        for hz in hz_list:
            for trial in range(1, trials + 1):
                label = f"{scenario}@{hz:g}Hz t{trial}"
                metrics = measure_hold(
                    hub, label, desires, seconds=seconds, hz=hz, print_report=True
                )
                metrics["scenario"] = scenario
                metrics["hz"] = hz
                metrics["trial"] = trial
                results.append(metrics)
        return results
    finally:
        hub.set_rx_sim(False)


def _fmt(v: Optional[float], nd: int = 1) -> str:
    return "n/a" if v is None else f"{v:.{nd}f}"


def render_report(scenario: str, results: Sequence[dict]) -> str:
    """Aggregate-per-rate table in the same shape as
    `docs/legacy/bench-load-matrix-*.md`'s bandwidth-baseline section."""
    lines = [
        f"## Scenario: {scenario}",
        "",
        "| tx Hz | n | fb_hz | ack_max | act_mn | act_pk | periph_mn | periph_pk | ok |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    by_hz: Dict[float, List[dict]] = {}
    for m in results:
        by_hz.setdefault(m["hz"], []).append(m)
    for hz in sorted(by_hz):
        trials = by_hz[hz]
        n = len(trials)
        fb = [t["raw_fb_hz"] for t in trials if t["raw_fb_hz"] is not None]
        ack = [t["ack_lag_max"] for t in trials if t["ack_lag_max"] is not None]
        act = [t["lap_ms_mean"] for t in trials if t["lap_ms_mean"] is not None]
        actpk = [t["lap_max_ms"] for t in trials if t["lap_max_ms"] is not None]
        per = [t["periph_lap_ms_mean"] for t in trials if t["periph_lap_ms_mean"] is not None]
        perpk = [t["periph_lap_max_ms"] for t in trials if t["periph_lap_max_ms"] is not None]
        ok_n = sum(1 for t in trials if t["ok"])
        lines.append(
            "| {hz:g} | {n} | {fb} | {ack} | {act} | {actpk} | {per} | {perpk} | {okn}/{n} |".format(
                hz=hz,
                n=n,
                fb=_fmt(sum(fb) / len(fb) if fb else None),
                ack=max(ack) if ack else "n/a",
                act=_fmt(sum(act) / len(act) if act else None),
                actpk=max(actpk) if actpk else "n/a",
                per=_fmt(sum(per) / len(per) if per else None),
                perpk=max(perpk) if perpk else "n/a",
                okn=ok_n,
            )
        )
    return "\n".join(lines) + "\n"


def write_report(path: Path, scenario: str, results: Sequence[dict]) -> None:
    header = (
        f"# Load matrix — {scenario}\n\nGenerated: {time.strftime('%Y-%m-%d %H:%M %Z')}\n\n"
    )
    body = render_report(scenario, results)
    existing = path.read_text(encoding="utf-8") if path.exists() else ""
    path.write_text(existing + ("\n" if existing else "") + header + body, encoding="utf-8")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="CDC port (default: auto-find)")
    ap.add_argument("--hz", default="40,100,200,500", help="Comma list of host TX rates")
    ap.add_argument("--scenario", default="all", choices=SCENARIOS)
    ap.add_argument("--trials", type=int, default=3, help="Repeats per rate")
    ap.add_argument("--seconds", type=float, default=3.0, help="Hold window per trial")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--report", default=None, help="Path to append a markdown report to")
    args = ap.parse_args(argv)

    hz_list = parse_hz_list(args.hz)

    with ControlsPcbHub.connect(args.port) as hub:
        hub.recover()
        results = run_matrix(
            hub,
            hz_list=hz_list,
            scenario=args.scenario,
            trials=args.trials,
            seconds=args.seconds,
            kp=args.kp,
            kd=args.kd,
        )

    hard_gate_hz = min(hz_list)
    hard_gate_ok = all(r["ok"] for r in results if r["hz"] == hard_gate_hz)

    if args.report:
        write_report(Path(args.report), args.scenario, results)
        print(f"\nReport appended: {args.report}")

    print(f"\n{args.scenario}: hard gate ({hard_gate_hz:g} Hz) {'PASS' if hard_gate_ok else 'FAIL'}")
    return 0 if hard_gate_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
