"""Analyze teleop_trace.csv — ack lag / host rate / kp windows."""
from __future__ import annotations

import csv
import datetime
import os
import statistics
from pathlib import Path

path = Path(__file__).resolve().parents[1] / "teleop_trace.csv"
st = path.stat()
print("path", path)
print("mtime", datetime.datetime.fromtimestamp(st.st_mtime))
print("size", st.st_size)

with path.open(newline="") as f:
    rows = list(csv.DictReader(f))
print("rows", len(rows))


def lag(tx: str, ack: str) -> int:
    return (int(tx) - int(ack)) & 0xFF


lags = [lag(r["tx"], r["ack"]) for r in rows]
lags_ok = [x for x in lags if x <= 128]
print(
    "ack_lag: min=%d p50=%.1f p95=%.1f max=%d mean=%.1f"
    % (
        min(lags_ok),
        statistics.median(lags_ok),
        sorted(lags_ok)[int(0.95 * (len(lags_ok) - 1))],
        max(lags_ok),
        statistics.mean(lags_ok),
    )
)

ts = [float(r["t_s"]) for r in rows]
dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
print(
    "host_dt_ms: p50=%.2f p95=%.2f max=%.2f mean=%.2f  hz~%.1f"
    % (
        1000 * statistics.median(dts),
        1000 * sorted(dts)[int(0.95 * (len(dts) - 1))],
        1000 * max(dts),
        1000 * statistics.mean(dts),
        1 / statistics.median(dts),
    )
)

for label, pred in [
    ("kp=0", lambda r: float(r["kp"]) == 0),
    ("kp>0", lambda r: float(r["kp"]) > 0),
    ("moving |rate|>0.1", lambda r: abs(float(r["rate"])) > 0.1),
]:
    sub = [r for r in rows if pred(r)]
    if not sub:
        continue
    lg = [x for x in (lag(r["tx"], r["ack"]) for r in sub) if x <= 128]
    print(
        "%s: n=%d lag p50=%.1f p95=%.1f max=%d mean=%.1f"
        % (
            label,
            len(sub),
            statistics.median(lg),
            sorted(lg)[int(0.95 * (len(lg) - 1))],
            max(lg),
            statistics.mean(lg),
        )
    )

acks = [int(r["ack"]) for r in rows]
adv = sum(1 for i in range(1, len(acks)) if acks[i] != acks[i - 1])
print("ack_value_changes: %d / %d rows (%.1f%%)" % (adv, len(acks) - 1, 100 * adv / (len(acks) - 1)))

chg_t = []
last = None
for r in rows:
    a = int(r["ack"])
    t = float(r["t_s"])
    if last is None:
        last = (a, t)
        continue
    if a != last[0]:
        chg_t.append(t - last[1])
        last = (a, t)
if chg_t:
    print(
        "ack_change_dt_ms: p50=%.1f p95=%.1f max=%.1f mean=%.1f n=%d  implied_hz~%.1f"
        % (
            1000 * statistics.median(chg_t),
            1000 * sorted(chg_t)[int(0.95 * (len(chg_t) - 1))],
            1000 * max(chg_t),
            1000 * statistics.mean(chg_t),
            len(chg_t),
            1 / statistics.median(chg_t),
        )
    )

print("--- lag by 2s windows ---")
for t0 in range(0, 22, 2):
    win = [r for r in rows if t0 <= float(r["t_s"]) < t0 + 2]
    if not win:
        continue
    lg = [x for x in (lag(r["tx"], r["ack"]) for r in win) if x <= 128]
    kp_on = sum(1 for r in win if float(r["kp"]) > 0)
    print(
        "t=%02d-%02ds n=%3d kp>0=%3d lag_p50=%4.0f lag_max=%3d kp_mean=%.1f"
        % (
            t0,
            t0 + 2,
            len(win),
            kp_on,
            statistics.median(lg),
            max(lg),
            statistics.mean(float(r["kp"]) for r in win),
        )
    )

print("rows lag<=2:", sum(1 for x in lags_ok if x <= 2))
print("rows lag 3-19:", sum(1 for x in lags_ok if 3 <= x <= 19))
print("rows lag>=20:", sum(1 for x in lags_ok if x >= 20))
print("rows lag>=26:", sum(1 for x in lags_ok if x >= 26))

# fb freshness: how often does fb position change while streaming
fb_chg = sum(1 for i in range(1, len(rows)) if rows[i]["fb"] != rows[i - 1]["fb"])
print("fb_position_changes: %d / %d (%.1f%%)" % (fb_chg, len(rows) - 1, 100 * fb_chg / (len(rows) - 1)))

# no lap_ms column in this file
print("has lap_ms column:", "lap_ms" in rows[0])
print("block values:", sorted({r["block"] for r in rows}))
