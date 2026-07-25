# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 13:38 Pacific Daylight Time

## Image / context

- Firmware: **Release `-Os` + per-bus RX index** (same image as prior Cursor sprint)
- Live motor: CH6 RS02 id=`0x70` probed OK before run; trapezoid Δ≈2π tracked (RS Δ/cmd ≈ 6.26)

## Scaling verdict (why this run)

| Question | Answer from this matrix |
|----------|-------------------------|
| Does one real RS02 keep FB fresh under DXL+LED? | **Yes** @40–200 Hz (plant_fb 250–440, act_lap ≈0.0–0.1 ms, 3/3 ok). 500 Hz still host-lag limited (1/3). |
| Is ×25 rx_sim “fake load”? | **No** — rx_sim only fakes RX; TX is real. act_lap jumps **0.1 → 1.4–1.9 ms** when enabling all 25 slots. |
| Does fake RX dominate vs real TX? | **TX dominates.** Hold BW: CH6×2 sim on/off both act≈0; MCP×6 act≈0.5–1.0 TX-only; all×25 TX-only ≈1.0, +sim only +0–0.2 ms until 500 Hz (+1.0). |
| Base (CH4–6 ×6 MCP) equal-rate? | **Yes as a proxy** — hold act_mn ≈1.0 ms with rx_sim, fb≈500 raw, rx_fresh_max=6. Comfortable vs 2 ms plant tick. |
| Full product ×25 equal-rate? | **Borderline but real** — hold ~1–2 ms; teleop+DXL+LED §B ~1.4–1.9 ms mean. Predicts multi-motor **TX cost**, slightly **pessimistic** on RX (synthetic RX ≥ typical quiet bus). |

**Expectation for N real RS02s:** act_lap scales with **enabled TX slots / bus drivers**, not with “1× real × N”. Use BW ladder (CH6×2 → MCP×6 → all×25) as the budget; single-motor teleop only proves the motor/protocol path.

## Setup

- Port: `COM5`
- Rates: 40, 100, 200, 500 Hz
- Trials/rate: 3
- Soft hold: 0.6 s
- Base seconds/phase: 8.0 s (real RS auto-extends for trapezoid)
- RS02: CH6 id=0x70 slot=23
- DXL: slots 0/1 bounce @ π/4 rad/s; LED FLASH
- Cali before real matrix: False (auto-retry cali on 500 Hz fail)

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
| 40 | 3 | 437 | 493 | 40 | 0.15 / 1.0 / 5 | 0.0 / 1 | 0.0 / 9 | 1902/1784 | 6.26/6.28 | 3/3 |
| 100 | 3 | 381 | 495 | 100 | 0.38 / 1.0 / 4 | 0.1 / 2 | 0.1 / 9 | 1890/1752 | 6.26/6.28 | 3/3 |
| 200 | 3 | 251 | 465 | 198 | 0.84 / 1.3 / 4 | 0.1 / 2 | 0.1 / 10 | 1893/1761 | 6.26/6.28 | 3/3 |
| 500 | 3 | 22 | 420 | 410 | 1.70 / 2.7 / 6 | 0.1 / 4 | 0.2 / 10 | 1619/1736 | 6.26/6.28 | 1/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 448 | 1.0 | 0.0 | 1 | 0.0 | 6 | 40 | 1918 | 1812 | 6.26 | 0.02 | Y |  |
| 40 | 2 | 433 | 1.0 | 0.0 | 1 | 0.0 | 9 | 40 | 1907 | 1779 | 6.26 | 0.02 | Y |  |
| 40 | 3 | 430 | 1.0 | 0.0 | 1 | 0.0 | 7 | 40 | 1882 | 1762 | 6.26 | 0.02 | Y |  |
| 100 | 1 | 386 | 1.0 | 0.1 | 2 | 0.1 | 5 | 100 | 1892 | 1767 | 6.26 | 0.02 | Y |  |
| 100 | 2 | 377 | 1.0 | 0.1 | 1 | 0.1 | 7 | 100 | 1886 | 1723 | 6.26 | 0.02 | Y |  |
| 100 | 3 | 379 | 1.0 | 0.0 | 2 | 0.0 | 9 | 100 | 1893 | 1765 | 6.26 | 0.02 | Y |  |
| 200 | 1 | 257 | 1.0 | 0.1 | 2 | 0.1 | 9 | 198 | 1890 | 1759 | 6.26 | 0.02 | Y |  |
| 200 | 2 | 256 | 2.0 | 0.0 | 1 | 0.0 | 6 | 198 | 1892 | 1767 | 6.26 | 0.02 | Y |  |
| 200 | 3 | 240 | 1.0 | 0.1 | 2 | 0.1 | 10 | 198 | 1898 | 1757 | 6.26 | 0.02 | Y |  |
| 500 | 1 | 20 | 3.0 | 0.1 | 2 | 0.2 | 6 | 397 | 1505 | 1723 | 6.26 | 0.02 | N | retry+cali; cmd_seq_lag_p95=3.0; plant_fb_hz=19.571160788744063 |
| 500 | 2 | 21 | 3.0 | 0.1 | 4 | 0.2 | 5 | 414 | 1633 | 1741 | 6.26 | 0.02 | N | retry+cali; cmd_seq_lag_p95=3.0 |
| 500 | 3 | 24 | 2.0 | 0.1 | 2 | 0.3 | 10 | 419 | 1719 | 1743 | 6.26 | 0.02 | Y |  |

## B — No real actuator on bus 6 (full ×25 ACTUATOR rx_sim) + DXL + LED

Product CFG ×25 with ACTUATOR rx_sim mask; no live RS02 MIT. Same DXL+LED load.

### Aggregate (mean across trials)

| tx Hz | n | plant_fb Hz | raw_fb Hz | applied Hz | cmd_seq_lag mean/p95/max | act_lap mean/peak | periph_lap mean/peak | s0/s1 span | RS Δ/cmd | ok |
|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|
| 40 | 3 | 354 | 408 | 40 | 0.12 / 1.0 / 2 | 1.4 / 11 | 0.1 / 12 | 1866/1781 | 0.00/0.00 | 3/3 |
| 100 | 3 | 284 | 381 | 100 | 0.38 / 1.0 / 2 | 1.7 / 11 | 0.2 / 12 | 1903/1748 | 0.00/0.00 | 3/3 |
| 200 | 3 | 251 | 428 | 199 | 0.83 / 1.0 / 3 | 1.9 / 11 | 0.1 / 12 | 1941/1765 | 0.00/0.00 | 3/3 |
| 500 | 3 | 33 | 373 | 407 | 1.79 / 3.0 / 9 | 1.5 / 13 | 0.2 / 14 | 1647/1785 | 0.00/0.00 | 1/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 351 | 1.0 | 1.4 | 10 | 0.2 | 12 | 40 | 1920 | 1785 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 357 | 1.0 | 1.5 | 10 | 0.0 | 12 | 40 | 1741 | 1780 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 354 | 1.0 | 1.3 | 11 | 0.0 | 12 | 40 | 1938 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 297 | 1.0 | 1.6 | 11 | 0.2 | 12 | 100 | 1944 | 1726 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 277 | 1.0 | 1.8 | 11 | 0.3 | 12 | 100 | 1810 | 1739 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 279 | 1.0 | 1.6 | 11 | 0.2 | 12 | 100 | 1954 | 1778 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 247 | 1.0 | 1.9 | 11 | 0.2 | 12 | 199 | 1939 | 1778 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 250 | 1.0 | 1.9 | 11 | 0.1 | 12 | 199 | 1945 | 1743 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 255 | 1.0 | 1.9 | 11 | 0.1 | 12 | 199 | 1940 | 1775 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 33 | 4.0 | 1.5 | 11 | 0.3 | 13 | 406 | 1496 | 1783 | 0.00 | 0.00 | N | cmd_seq_lag_p95=4.0 |
| 500 | 2 | 34 | 2.0 | 1.6 | 11 | 0.2 | 13 | 408 | 1512 | 1787 | 0.00 | 0.00 | Y |  |
| 500 | 3 | 32 | 3.0 | 1.6 | 13 | 0.2 | 14 | 406 | 1934 | 1785 | 0.00 | 0.00 | N | cmd_seq_lag_p95=3.0 |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 514 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | 8 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 512 | 1 | 0 | 100 | 0 | 13 | 0 | 14 | 255 | 8 | Y |
| 200 | off | 514 | 1 | 1 | 197 | 0 | 13 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0 | 13 | 0 | 14 | 255 | 8 | Y |
| 500 | off | 513 | 4 | 2 | 431 | 0 | 13 | 0.0 | 14 | 255 | n/a | N |
| 500 | on | 514 | 3 | 3 | 442 | 0 | 13 | 0.0 | 14 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | 2 | Y |
| 100 | off | 522 | 0 | 0 | 100 | 0.0 | 13 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 0.0 | 13 | 0 | 14 | 255 | 2 | Y |
| 200 | off | 514 | 2 | 1 | 197 | 0.0 | 13 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 508 | 2 | 1 | 198 | 0.0 | 13 | 0.0 | 14 | 255 | 2 | Y |
| 500 | off | 504 | 3 | 3 | 431 | 0.0 | 13 | 0.1 | 14 | 255 | n/a | N |
| 500 | on | 501 | 3 | 3 | 443 | 0.0 | 13 | 0.1 | 14 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 514 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | 2 | Y |
| 100 | off | 514 | 1 | 0 | 100 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 512 | 0 | 0 | 100 | 0 | 13 | 0 | 14 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 13 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 200 | 0 | 13 | 0.1 | 14 | 255 | 2 | Y |
| 500 | off | 513 | 3 | 2 | 442 | 0 | 13 | 0.0 | 14 | 255 | n/a | N |
| 500 | on | 513 | 3 | 2 | 435 | 0 | 13 | 0.0 | 14 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 13 | 0 | 14 | 255 | 19 | Y |
| 100 | off | 514 | 1 | 0 | 100 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0.1 | 13 | 0 | 14 | 255 | 19 | Y |
| 200 | off | 514 | 2 | 1 | 199 | 0 | 13 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0.3 | 13 | 0 | 14 | 255 | 19 | Y |
| 500 | off | 514 | 3 | 3 | 443 | 0 | 13 | 0.0 | 14 | 255 | n/a | N |
| 500 | on | 514 | 3 | 3 | 425 | 0.9 | 13 | 0.0 | 14 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0.5 | 13 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 1.0 | 13 | 0 | 14 | 255 | 6 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0.7 | 13 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 1 | 13 | 0 | 14 | 255 | 6 | Y |
| 200 | off | 506 | 1 | 1 | 199 | 0.9 | 13 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 1.0 | 13 | 0.2 | 14 | 255 | 6 | Y |
| 500 | off | 494 | 3 | 2 | 446 | 1.0 | 13 | 0.3 | 14 | 255 | n/a | N |
| 500 | on | 494 | 4 | 2 | 428 | 1.0 | 13 | 0.2 | 14 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 1.0 | 13 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 493 | 0 | 0 | 40 | 1 | 13 | 0 | 14 | 255 | 25 | Y |
| 100 | off | 494 | 1 | 0 | 100 | 1 | 13 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 493 | 1 | 0 | 100 | 1.1 | 13 | 0 | 14 | 255 | 25 | Y |
| 200 | off | 494 | 2 | 1 | 199 | 1 | 13 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 490 | 1 | 1 | 199 | 1.2 | 13 | 0 | 14 | 255 | 25 | Y |
| 500 | off | 494 | 3 | 3 | 438 | 1.0 | 13 | 0.0 | 14 | 255 | n/a | N |
| 500 | on | 472 | 3 | 3 | 424 | 2.0 | 13 | 0.0 | 14 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -20 | +0 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -1 | +0 | +0.1 | +0.0 | +0 | Y | Y |
| 200 | -4 | -1 | +0.2 | -0.0 | +0 | Y | Y |
| 500 | -23 | +0 | +1.0 | -0.0 | +0 | N | N |

## Notes / anomalies

- skipped initial cali (--skip-cali)
- real_ch6 @500Hz t1: cmd_seq_lag_p95=3.0
-   → recalibrate CH6 and retry once
- real_ch6 @500Hz t2: cmd_seq_lag_p95=4.0
-   → recalibrate CH6 and retry once
- rx_sim_x25 @500Hz t1: cmd_seq_lag_p95=4.0
- rx_sim_x25 @500Hz t3: cmd_seq_lag_p95=3.0

## Takeaways

### Real CH6
- **40 Hz**: ok 3/3; plant_fb≈437; lag_p95≈1.0; act_lap≈0.0; periph_lap≈0.0
- **100 Hz**: ok 3/3; plant_fb≈381; lag_p95≈1.0; act_lap≈0.1; periph_lap≈0.1
- **200 Hz**: ok 3/3; plant_fb≈251; lag_p95≈1.3; act_lap≈0.1; periph_lap≈0.1
- **500 Hz**: ok 1/3; plant_fb≈22; lag_p95≈2.7; act_lap≈0.1; periph_lap≈0.2

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈354; lag_p95≈1.0; act_lap≈1.4; periph_lap≈0.1
- **100 Hz**: ok 3/3; plant_fb≈284; lag_p95≈1.0; act_lap≈1.7; periph_lap≈0.2
- **200 Hz**: ok 3/3; plant_fb≈251; lag_p95≈1.0; act_lap≈1.9; periph_lap≈0.1
- **500 Hz**: ok 1/3; plant_fb≈33; lag_p95≈3.0; act_lap≈1.5; periph_lap≈0.2
