# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 14:10 Pacific Daylight Time

## Cursor note (post-run)

PlantTask `osPriorityHigh` (Host/Peripheral `AboveNormal`): **act_lap peak
11→3 ms** (success vs mean ~1.2), but Host `cmd_seq_lag` blew up at 200/500
(p95 12–122) and periph_lap mean/peak ballooned (~6 / 42–76 ms) — DXL
fragmented under Plant preemption. **Reverted** to same-band priorities;
next: exclude `vTaskSuspendAll` intervals from act_lap (keep Host healthy,
keep peak honest).

## Setup

- Port: `COM5`
- Rates: 40, 100, 200, 500 Hz
- Trials/rate: 3
- Soft hold: 0.6 s
- Base seconds/phase: 8.0 s (real RS auto-extends for trapezoid)
- RS02: CH6 id=0x70 slot=23
- DXL: slots 0/1 bounce @ π/4 rad/s; LED FLASH
- Cali before real matrix: False

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
| 40 | 3 | 364 | 419 | 40 | 0.40 / 1.0 / 2 | 1.2 / 3 | 4.7 / 42 | 1868/1777 | 0.00/0.00 | 3/3 |
| 100 | 3 | 315 | 407 | 100 | 1.11 / 2.0 / 5 | 1.2 / 3 | 5.7 / 42 | 1781/1677 | 0.00/0.00 | 3/3 |
| 200 | 3 | 217 | 395 | 198 | 3.99 / 12.7 / 36 | 1.2 / 3 | 5.8 / 55 | 1779/1750 | 0.00/0.00 | 0/3 |
| 500 | 3 | 39 | 403 | 386 | 59.46 / 122.0 / 128 | 0.9 / 3 | 7.1 / 76 | 1317/1567 | 0.00/0.00 | 0/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 359 | 1.0 | 1.2 | 3 | 3.5 | 36 | 40 | 1920 | 1768 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 354 | 1.0 | 1.2 | 3 | 5.5 | 42 | 40 | 1926 | 1784 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 378 | 1.0 | 1.2 | 3 | 5.1 | 42 | 40 | 1757 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 316 | 2.0 | 1.2 | 3 | 5.4 | 42 | 100 | 1645 | 1696 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 311 | 2.0 | 1.2 | 3 | 5.8 | 42 | 100 | 1771 | 1654 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 319 | 2.0 | 1.2 | 3 | 5.9 | 42 | 100 | 1928 | 1680 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 218 | 10.0 | 1.2 | 3 | 5.6 | 55 | 198 | 1754 | 1752 | 0.00 | 0.00 | N | cmd_seq_lag_p95=10.0 |
| 200 | 2 | 215 | 11.0 | 1.2 | 3 | 5.5 | 55 | 198 | 1657 | 1736 | 0.00 | 0.00 | N | cmd_seq_lag_p95=11.0 |
| 200 | 3 | 218 | 17.0 | 1.2 | 3 | 6.2 | 55 | 198 | 1927 | 1763 | 0.00 | 0.00 | N | cmd_seq_lag_p95=17.0 |
| 500 | 1 | 43 | 120.0 | 0.9 | 3 | 7.3 | 55 | 375 | 1505 | 1637 | 0.00 | 0.00 | N | cmd_seq_lag_p95=120.0 |
| 500 | 2 | 37 | 118.0 | 0.9 | 3 | 7.2 | 76 | 388 | 1123 | 1403 | 0.00 | 0.00 | N | cmd_seq_lag_p95=118.0 |
| 500 | 3 | 38 | 128.0 | 0.8 | 3 | 6.7 | 76 | 393 | 1322 | 1662 | 0.00 | 0.00 | N | cmd_seq_lag_p95=128.0 |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | 8 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 100 | on | 440 | 18 | 1 | 98 | 0 | 3 | 0 | 76 | 255 | 8 | N |
| 200 | off | 513 | 1 | 1 | 199 | 0 | 3 | 0.0 | 76 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 200 | 0 | 3 | 0 | 76 | 255 | 8 | Y |
| 500 | off | 514 | 3 | 2 | 422 | 0 | 3 | 0.0 | 76 | 255 | n/a | N |
| 500 | on | 515 | 3 | 3 | 446 | 0 | 3 | 0 | 76 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 1 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | 2 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 0.0 | 3 | 0 | 76 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0.1 | 3 | 0 | 76 | 255 | 2 | Y |
| 200 | off | 510 | 2 | 1 | 199 | 0.0 | 3 | 0.0 | 76 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0 | 3 | 0 | 76 | 255 | 2 | Y |
| 500 | off | 505 | 3 | 2 | 421 | 0.0 | 3 | 0.1 | 76 | 255 | n/a | N |
| 500 | on | 508 | 3 | 3 | 431 | 0.0 | 3 | 0 | 76 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 40 | on | 476 | 5 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | 2 | N |
| 100 | off | 513 | 0 | 0 | 100 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0 | 3 | 0 | 76 | 255 | 2 | Y |
| 200 | off | 513 | 1 | 1 | 200 | 0 | 3 | 0.1 | 76 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 200 | 0.0 | 3 | 0 | 76 | 255 | 2 | Y |
| 500 | off | 514 | 3 | 2 | 421 | 0 | 3 | 0.1 | 76 | 255 | n/a | N |
| 500 | on | 512 | 3 | 2 | 444 | 0 | 3 | 0 | 76 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 40 | on | 515 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | 19 | Y |
| 100 | off | 513 | 0 | 0 | 100 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 100 | on | 509 | 2 | 1 | 100 | 0.0 | 3 | 0 | 76 | 255 | 19 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0 | 3 | 0.0 | 76 | 255 | n/a | Y |
| 200 | on | 514 | 2 | 1 | 199 | 0.0 | 3 | 0 | 76 | 255 | 19 | Y |
| 500 | off | 512 | 3 | 2 | 422 | 0 | 3 | 0.0 | 76 | 255 | n/a | N |
| 500 | on | 514 | 3 | 3 | 438 | 0.7 | 3 | 0 | 76 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 3 | 0 | 76 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0.2 | 3 | 0 | 76 | 255 | 6 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0.8 | 3 | 0 | 76 | 255 | n/a | Y |
| 100 | on | 514 | 1 | 0 | 100 | 0.3 | 3 | 0 | 76 | 255 | 6 | Y |
| 200 | off | 514 | 1 | 1 | 197 | 0.7 | 3 | 0.0 | 76 | 255 | n/a | Y |
| 200 | on | 506 | 1 | 1 | 199 | 0.7 | 3 | 0.0 | 76 | 255 | 6 | Y |
| 500 | off | 503 | 4 | 3 | 442 | 1.0 | 3 | 0.0 | 76 | 255 | n/a | N |
| 500 | on | 506 | 4 | 3 | 437 | 1 | 3 | 0.0 | 76 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 493 | 0 | 0 | 40 | 1.0 | 3 | 0 | 76 | 255 | n/a | Y |
| 40 | on | 489 | 1 | 0 | 40 | 1.0 | 3 | 0 | 76 | 255 | 25 | Y |
| 100 | off | 508 | 1 | 1 | 100 | 1.1 | 3 | 0 | 76 | 255 | n/a | Y |
| 100 | on | 489 | 2 | 1 | 99 | 1.0 | 3 | 0 | 76 | 255 | 25 | Y |
| 200 | off | 502 | 2 | 1 | 199 | 1.0 | 3 | 0 | 76 | 255 | n/a | Y |
| 200 | on | 492 | 3 | 1 | 199 | 1.1 | 3 | 0 | 76 | 255 | 25 | N |
| 500 | off | 495 | 3 | 3 | 423 | 1.0 | 3 | 0 | 76 | 255 | n/a | N |
| 500 | on | 494 | 15 | 7 | 435 | 1.1 | 3 | 0.2 | 76 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -4 | +1 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -20 | +1 | -0.0 | +0.0 | +0 | Y | Y |
| 200 | -10 | +1 | +0.0 | +0.0 | +0 | Y | N |
| 500 | -1 | +12 | +0.1 | +0.2 | +0 | N | N |

## Notes / anomalies

- rx_sim_x25 @200Hz t1: cmd_seq_lag_p95=10.0
- rx_sim_x25 @200Hz t2: cmd_seq_lag_p95=11.0
- rx_sim_x25 @200Hz t3: cmd_seq_lag_p95=17.0
- rx_sim_x25 @500Hz t1: cmd_seq_lag_p95=120.0
- rx_sim_x25 @500Hz t2: cmd_seq_lag_p95=118.0
- rx_sim_x25 @500Hz t3: cmd_seq_lag_p95=128.0

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈364; lag_p95≈1.0; act_lap≈1.2; periph_lap≈4.7
- **100 Hz**: ok 3/3; plant_fb≈315; lag_p95≈2.0; act_lap≈1.2; periph_lap≈5.7
- **200 Hz**: ok 0/3; plant_fb≈217; lag_p95≈12.7; act_lap≈1.2; periph_lap≈5.8
- **500 Hz**: ok 0/3; plant_fb≈39; lag_p95≈122.0; act_lap≈0.9; periph_lap≈7.1
