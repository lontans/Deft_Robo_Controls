# Load matrix — Release `-Os` vs Debug baseline (×25 rx_sim + DXL + LED)

Generated: 2026-07-23 13:20 Pacific Daylight Time

## Image

- Config: **Release** (`-Os`, no `DEBUG`)
- Artifact: `Release/DeftRoboticsControlsPCB.elf` (text 73616 B vs Debug 139620 B)
- Soft-DFU: `python scripts/soft_dfu_flash.py --image Release/DeftRoboticsControlsPCB.elf`
- Code: equal-rate `main@2eb6eb2` (no MCP÷ / post-FB decoupling)
- Compare to Debug `-O0` §B in [bench-load-matrix-2026-07-23.md](bench-load-matrix-2026-07-23.md)

### §B act_lap / ok vs Debug (mean)

| tx Hz | Debug act_lap mean/peak | Release act_lap mean/peak | Debug ok | Release ok |
|---:|---|---|---|---|
| 40 | 3.6 / 18 | **1.8 / 10** | 3/3 | 3/3 |
| 100 | 3.7 / 18 | **2.1 / 10** | 3/3 | 3/3 |
| 200 | 4.0 / 18 | **2.1 / 11** | 3/3 | 3/3 |
| 500 | 3.5 / 18 | **1.9 / 11** | 2/3 | 0/3 |

Verdict: Release cuts mean act_lap ~**2×** and peaks 18→10–11 ms at equal-rate. 500 Hz host lag gate still fails (worse ok count than Debug — coalesce/host path, not act_lap).

## Setup

- Port: `COM5`
- Rates: 40, 100, 200, 500 Hz
- Trials/rate: 3
- Soft hold: 0.6 s
- Base seconds/phase: 8.0 s (real RS auto-extends for trapezoid)
- RS02: CH6 id=0x70 slot=23
- DXL: slots 0/1 bounce @ π/4 rad/s; LED FLASH
- Cali before real matrix: False
- Flags: `--skip-real --skip-cali`

## Metric definitions

- **tx Hz**: host `send_once` rate for the trial
- **plant_fb Hz**: non-debug feedback frames drained/parsed by the bench
- **raw_fb Hz**: all CDC reader frames (includes debug)
- **applied Hz**: advance rate of `cmd_applied_seq`
- **cmd_seq_lag**: `(host_tx_seq - last_cmd_seq) & 0xFF` (ack lag)
- **act_lap_ms**: PlantTask actuator apply+TX lap
- **periph_lap_ms**: PeripheralTask lap (DXL/LED path)
- **RS Δ/cmd**: measured vs planned MIT travel (real-RS cfg only)

## A — Real actuator on bus 6 (no ×25 rx_sim) + DXL + LED

Single CFG slot on CH6; other actuator slots quieted. Soft-engage + trapezoid teleop, direction planned to stay inside MIT ±12.57.

### Aggregate (mean across trials)

| tx Hz | n | plant_fb Hz | raw_fb Hz | applied Hz | cmd_seq_lag mean/p95/max | act_lap mean/peak | periph_lap mean/peak | s0/s1 span | RS Δ/cmd | ok |
|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|

## B — No real actuator on bus 6 (full ×25 ACTUATOR rx_sim) + DXL + LED

Product CFG ×25 with ACTUATOR rx_sim mask; no live RS02 MIT. Same DXL+LED load.

### Aggregate (mean across trials)

| tx Hz | n | plant_fb Hz | raw_fb Hz | applied Hz | cmd_seq_lag mean/p95/max | act_lap mean/peak | periph_lap mean/peak | s0/s1 span | RS Δ/cmd | ok |
|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|
| 40 | 3 | 372 | 427 | 40 | 0.11 / 1.0 / 3 | 1.8 / 10 | 0.2 / 9 | 1916/1776 | 0.00/0.00 | 3/3 |
| 100 | 3 | 302 | 403 | 100 | 0.32 / 1.0 / 2 | 2.1 / 10 | 0.3 / 9 | 1884/1745 | 0.00/0.00 | 3/3 |
| 200 | 3 | 242 | 417 | 199 | 0.88 / 1.3 / 10 | 2.1 / 11 | 0.2 / 9 | 1921/1764 | 0.00/0.00 | 3/3 |
| 500 | 3 | 39 | 361 | 411 | 1.86 / 3.7 / 12 | 1.9 / 11 | 0.2 / 13 | 1796/1771 | 0.00/0.00 | 0/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 366 | 1.0 | 1.8 | 9 | 0.2 | 9 | 40 | 1915 | 1768 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 378 | 1.0 | 1.8 | 10 | 0.1 | 9 | 40 | 1913 | 1782 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 372 | 1.0 | 1.8 | 10 | 0.2 | 9 | 40 | 1919 | 1777 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 282 | 1.0 | 2.2 | 10 | 0.3 | 9 | 100 | 1799 | 1729 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 331 | 1.0 | 1.9 | 10 | 0.1 | 9 | 100 | 1927 | 1732 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 294 | 1.0 | 2.1 | 10 | 0.4 | 9 | 100 | 1927 | 1775 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 265 | 1.0 | 2.1 | 10 | 0.0 | 9 | 200 | 1922 | 1740 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 232 | 2.0 | 2.0 | 10 | 0.2 | 9 | 198 | 1922 | 1775 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 229 | 1.0 | 2.1 | 11 | 0.2 | 9 | 198 | 1920 | 1778 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 36 | 3.0 | 1.8 | 11 | 0.2 | 9 | 410 | 1558 | 1739 | 0.00 | 0.00 | N | cmd_seq_lag_p95=3.0 |
| 500 | 2 | 40 | 4.0 | 1.9 | 11 | 0.2 | 13 | 413 | 1916 | 1790 | 0.00 | 0.00 | N | cmd_seq_lag_p95=4.0 |
| 500 | 3 | 42 | 4.0 | 2.0 | 11 | 0.2 | 13 | 410 | 1913 | 1784 | 0.00 | 0.00 | N | cmd_seq_lag_p95=4.0 |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | 8 | Y |
| 100 | off | 513 | 0 | 0 | 100 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 514 | 0 | 0 | 100 | 0 | 11 | 0 | 13 | 255 | 8 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 11 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0 | 11 | 0.0 | 13 | 255 | 8 | Y |
| 500 | off | 513 | 3 | 2 | 425 | 0 | 11 | 0.0 | 13 | 255 | n/a | N |
| 500 | on | 508 | 8 | 3 | 426 | 0 | 11 | 0.0 | 13 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 1 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | 2 | Y |
| 100 | off | 513 | 0 | 0 | 100 | 0.0 | 11 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 514 | 0 | 0 | 100 | 0.1 | 11 | 0 | 13 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0.0 | 11 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 508 | 1 | 1 | 199 | 0.0 | 11 | 0.0 | 13 | 255 | 2 | Y |
| 500 | off | 500 | 3 | 3 | 448 | 0.0 | 11 | 0.1 | 13 | 255 | n/a | N |
| 500 | on | 499 | 3 | 3 | 433 | 0.0 | 11 | 0.1 | 13 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 512 | 1 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | 2 | Y |
| 100 | off | 514 | 1 | 0 | 100 | 0 | 11 | 0.0 | 13 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 0 | 11 | 0 | 13 | 255 | 2 | Y |
| 200 | off | 506 | 1 | 1 | 198 | 0 | 11 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 516 | 1 | 1 | 200 | 0.0 | 11 | 0.0 | 13 | 255 | 2 | Y |
| 500 | off | 498 | 3 | 3 | 444 | 0.0 | 11 | 0.1 | 13 | 255 | n/a | N |
| 500 | on | 499 | 3 | 3 | 425 | 0.0 | 11 | 0.1 | 13 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 13 | 255 | 19 | Y |
| 100 | off | 513 | 0 | 0 | 100 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 512 | 2 | 1 | 100 | 0.6 | 11 | 0 | 13 | 255 | 19 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 11 | 0 | 13 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 200 | 0.6 | 11 | 0.0 | 13 | 255 | 19 | Y |
| 500 | off | 514 | 3 | 3 | 447 | 0 | 11 | 0 | 13 | 255 | n/a | N |
| 500 | on | 513 | 3 | 2 | 428 | 1.0 | 11 | 0.0 | 13 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 509 | 1 | 0 | 40 | 1.0 | 11 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 506 | 0 | 0 | 40 | 1.0 | 11 | 0 | 13 | 255 | 6 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 1.0 | 11 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 512 | 0 | 0 | 100 | 1.1 | 11 | 0 | 13 | 255 | 6 | Y |
| 200 | off | 497 | 1 | 1 | 199 | 1.0 | 11 | 0.1 | 13 | 255 | n/a | Y |
| 200 | on | 506 | 1 | 1 | 199 | 1.0 | 11 | 0.0 | 13 | 255 | 6 | Y |
| 500 | off | 495 | 3 | 2 | 444 | 1.0 | 11 | 0.2 | 13 | 255 | n/a | N |
| 500 | on | 494 | 3 | 2 | 423 | 1.0 | 11 | 0.2 | 13 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 492 | 0 | 0 | 40 | 1 | 11 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 476 | 2 | 0 | 40 | 1.0 | 11 | 0 | 13 | 255 | 25 | Y |
| 100 | off | 495 | 0 | 0 | 100 | 1 | 11 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 457 | 1 | 0 | 100 | 1.2 | 11 | 0 | 13 | 255 | 25 | Y |
| 200 | off | 494 | 1 | 1 | 199 | 1.0 | 11 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 449 | 1 | 1 | 199 | 1.5 | 11 | 0 | 13 | 255 | 25 | Y |
| 500 | off | 492 | 3 | 3 | 459 | 1.0 | 11 | 0.1 | 13 | 255 | n/a | N |
| 500 | on | 466 | 3 | 3 | 435 | 2.2 | 11 | 0.0 | 13 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -16 | +2 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -38 | +1 | +0.2 | +0.0 | +0 | Y | Y |
| 200 | -45 | +0 | +0.5 | -0.0 | +0 | Y | Y |
| 500 | -26 | +0 | +1.1 | -0.0 | +0 | N | N |

## Notes / anomalies

- rx_sim_x25 @500Hz t1: cmd_seq_lag_p95=3.0
- rx_sim_x25 @500Hz t2: cmd_seq_lag_p95=4.0
- rx_sim_x25 @500Hz t3: cmd_seq_lag_p95=4.0

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈372; lag_p95≈1.0; act_lap≈1.8; periph_lap≈0.2
- **100 Hz**: ok 3/3; plant_fb≈302; lag_p95≈1.0; act_lap≈2.1; periph_lap≈0.3
- **200 Hz**: ok 3/3; plant_fb≈242; lag_p95≈1.3; act_lap≈2.1; periph_lap≈0.2
- **500 Hz**: ok 0/3; plant_fb≈39; lag_p95≈3.7; act_lap≈1.9; periph_lap≈0.2
