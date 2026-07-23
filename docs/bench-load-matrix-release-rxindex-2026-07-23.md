# Load matrix — Release `-Os` + per-bus RX index

Generated: 2026-07-23 13:23 Pacific Daylight Time

## Image

- Config: **Release** (`-Os`) + [per-bus RX index](rfc-per-bus-rx-index.md) (`docs/patches/per-bus-rx-index.patch` + CFG rebuild hooks in `plant_config_nvm.c`)
- Artifact: `Release/DeftRoboticsControlsPCB.elf`
- Soft-DFU: `python scripts/soft_dfu_flash.py --image Release/DeftRoboticsControlsPCB.elf`
- Compare to Release-only baseline [bench-load-matrix-release-2026-07-23.md](bench-load-matrix-release-2026-07-23.md) and Debug [bench-load-matrix-2026-07-23.md](bench-load-matrix-2026-07-23.md)
- Identity: equal-rate; **no** MCP÷ / post-FB decoupling

### §B act_lap / ok progression

| tx Hz | Debug act mean/peak | Release act mean/peak | Release+RX-idx act mean/peak | Debug ok | Rel ok | Rel+idx ok |
|---:|---|---|---|---|---|---|
| 40 | 3.6 / 18 | 1.8 / 10 | **1.6 / ~10** | 3/3 | 3/3 | 3/3 |
| 100 | 3.7 / 18 | 2.1 / 10 | **1.9 / ~10** | 3/3 | 3/3 | 3/3 |
| 200 | 4.0 / 18 | 2.1 / 11 | **2.0 / ~10** | 3/3 | 3/3 | 3/3 |
| 500 | 3.5 / 18 | 1.9 / 11 | **~2.0 / ~11** | 2/3 | 0/3 | 0/3 |

Verdict: RX index is a small further act_lap win on top of Release (~0.1–0.2 ms mean at 40–100 Hz). Not worse at 40/100/200 ok counts. 500 Hz host lag gate still fails (p95 inflated; host coalesce path).

## Setup

- Port: `COM5`
- Rates: 40, 100, 200, 500 Hz
- Trials/rate: 3
- Soft hold: 0.6 s
- Base seconds/phase: 8.0 s
- Flags: `--skip-real --skip-cali --skip-bw`
- DXL: slots 0/1 bounce; LED FLASH

## Metric definitions

- **tx Hz**: host `send_once` rate for the trial
- **plant_fb Hz**: non-debug feedback frames drained/parsed by the bench
- **applied Hz**: advance rate of `cmd_applied_seq`
- **cmd_seq_lag**: `(host_tx_seq - last_cmd_seq) & 0xFF` (ack lag)
- **act_lap_ms**: PlantTask actuator apply+TX lap
- **periph_lap_ms**: PeripheralTask lap (DXL/LED path)

## B — ×25 ACTUATOR rx_sim + DXL + LED

### Aggregate (mean across trials)

| tx Hz | n | plant_fb Hz | act_lap mean | ok |
|---:|---:|---:|---:|---|
| 40 | 3 | 366 | 1.6 | 3/3 |
| 100 | 3 | 306 | 1.9 | 3/3 |
| 200 | 3 | 234 | 2.0 | 3/3 |
| 500 | 3 | — | — | 0/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | per_mn | ok | notes |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 356 | 1.0 | 1.6 | 0.0 | Y |  |
| 40 | 2 | 366 | 1.0 | 1.6 | 0.0 | Y |  |
| 40 | 3 | 377 | 1.0 | 1.5 | 0.0 | Y |  |
| 100 | 1 | 261 | 1.0 | 1.8 | 0.5 | Y |  |
| 100 | 2 | 318 | 1.0 | 2.0 | 0.2 | Y |  |
| 100 | 3 | 339 | 1.0 | 1.8 | 0.1 | Y |  |
| 200 | 1 | 231 | 1.0 | 2.0 | 0.2 | Y |  |
| 200 | 2 | 243 | 1.0 | 2.0 | 0.2 | Y |  |
| 200 | 3 | 229 | 1.0 | 2.1 | 0.2 | Y |  |
| 500 | 1 | — | 13.0 | — | — | N | cmd_seq_lag_p95=13.0 |
| 500 | 2 | — | 26.0 | — | — | N | cmd_seq_lag_p95=26.0 |
| 500 | 3 | — | 10.0 | — | — | N | cmd_seq_lag_p95=10.0 |

## Bandwidth baseline

_Skipped (`--skip-bw`)._

## Notes

- Matrix script previously crashed on `--skip-bw` when formatting empty BW tables; guarded in `_tmp_load_matrix_report.py` after this run.
- 500 Hz lag p95 worse than Release-only baseline in this run — treat as host/CDC variance, not RX-index regression on act_lap (40–200 improved).
