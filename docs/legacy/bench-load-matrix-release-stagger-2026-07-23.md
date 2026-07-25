# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 13:56 Pacific Daylight Time

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
| 40 | 3 | 355 | 398 | 40 | 0.14 / 1.0 / 2 | 1.5 / 11 | 0.2 / 12 | 1789/1756 | 0.00/0.00 | 3/3 |
| 100 | 3 | 339 | 421 | 100 | 0.43 / 1.0 / 7 | 1.6 / 11 | 0.1 / 12 | 1878/1779 | 0.00/0.00 | 3/3 |
| 200 | 3 | 256 | 412 | 199 | 0.85 / 1.3 / 8 | 1.9 / 11 | 0.2 / 12 | 1888/1755 | 0.00/0.00 | 3/3 |
| 500 | 3 | 43 | 372 | 410 | 1.96 / 4.0 / 25 | 1.7 / 14 | 0.3 / 14 | 1757/1721 | 0.00/0.00 | 1/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 354 | 1.0 | 1.6 | 11 | 0.1 | 12 | 40 | 1752 | 1763 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 360 | 1.0 | 1.3 | 11 | 0.1 | 12 | 40 | 1679 | 1727 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 352 | 1.0 | 1.5 | 11 | 0.3 | 12 | 40 | 1935 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 333 | 1.0 | 1.6 | 11 | 0.0 | 12 | 100 | 1723 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 360 | 1.0 | 1.6 | 11 | 0.3 | 12 | 100 | 1938 | 1779 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 325 | 1.0 | 1.6 | 11 | 0.1 | 12 | 100 | 1972 | 1779 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 238 | 1.0 | 1.9 | 11 | 0.3 | 12 | 199 | 1942 | 1780 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 260 | 1.0 | 1.8 | 11 | 0.2 | 12 | 198 | 1953 | 1743 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 269 | 2.0 | 1.9 | 11 | 0.2 | 12 | 199 | 1768 | 1743 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 40 | 3.0 | 1.7 | 11 | 0.2 | 12 | 413 | 1536 | 1750 | 0.00 | 0.00 | N | cmd_seq_lag_p95=3.0 |
| 500 | 2 | 41 | 2.0 | 1.7 | 14 | 0.5 | 14 | 408 | 1782 | 1631 | 0.00 | 0.00 | Y |  |
| 500 | 3 | 47 | 7.0 | 1.8 | 14 | 0.3 | 14 | 407 | 1954 | 1783 | 0.00 | 0.00 | N | cmd_seq_lag_p95=7.0 |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | 8 | Y |
| 100 | off | 506 | 1 | 1 | 100 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 514 | 1 | 0 | 100 | 0 | 14 | 0 | 14 | 255 | 8 | Y |
| 200 | off | 514 | 2 | 1 | 196 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0 | 14 | 0.0 | 14 | 255 | 8 | Y |
| 500 | off | 514 | 3 | 2 | 432 | 0 | 14 | 0.0 | 14 | 255 | n/a | N |
| 500 | on | 514 | 3 | 3 | 421 | 0 | 14 | 0.0 | 14 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 502 | 1 | 1 | 40 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 511 | 0 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | 2 | Y |
| 100 | off | 514 | 1 | 0 | 100 | 0.0 | 14 | 0.0 | 14 | 255 | n/a | Y |
| 100 | on | 512 | 1 | 0 | 100 | 0.0 | 14 | 0 | 14 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0.0 | 14 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0 | 14 | 0.0 | 14 | 255 | 2 | Y |
| 500 | off | 496 | 3 | 3 | 449 | 0.0 | 14 | 0.1 | 14 | 255 | n/a | N |
| 500 | on | 504 | 3 | 3 | 428 | 0.0 | 14 | 0.1 | 14 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 1 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | 2 | Y |
| 100 | off | 514 | 1 | 0 | 100 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 514 | 1 | 0 | 99 | 0 | 14 | 0 | 14 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0 | 14 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0 | 14 | 0.0 | 14 | 255 | 2 | Y |
| 500 | off | 514 | 3 | 2 | 422 | 0 | 14 | 0.0 | 14 | 255 | n/a | N |
| 500 | on | 514 | 3 | 2 | 434 | 0 | 14 | 0.0 | 14 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 514 | 0 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 0 | 14 | 0 | 14 | 255 | 19 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0 | 14 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 514 | 1 | 0 | 100 | 0.0 | 14 | 0 | 14 | 255 | 19 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 14 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 515 | 1 | 1 | 197 | 0.1 | 14 | 0.0 | 14 | 255 | 19 | Y |
| 500 | off | 515 | 3 | 2 | 419 | 0 | 14 | 0 | 14 | 255 | n/a | N |
| 500 | on | 513 | 3 | 3 | 414 | 0.8 | 14 | 0.1 | 14 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0.4 | 14 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 1 | 14 | 0 | 14 | 255 | 6 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 0.9 | 14 | 0.0 | 14 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 1.0 | 14 | 0 | 14 | 255 | 6 | Y |
| 200 | off | 512 | 2 | 1 | 198 | 0.9 | 14 | 0.1 | 14 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 1.0 | 14 | 0.0 | 14 | 255 | 6 | Y |
| 500 | off | 498 | 3 | 2 | 434 | 1.0 | 14 | 0.3 | 14 | 255 | n/a | N |
| 500 | on | 494 | 3 | 2 | 408 | 1.0 | 14 | 0.2 | 14 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 494 | 0 | 0 | 40 | 1 | 14 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 492 | 0 | 0 | 40 | 1 | 14 | 0 | 14 | 255 | 25 | Y |
| 100 | off | 494 | 1 | 0 | 100 | 1 | 14 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 494 | 1 | 0 | 100 | 1.0 | 14 | 0 | 14 | 255 | 25 | Y |
| 200 | off | 496 | 1 | 1 | 200 | 1.0 | 14 | 0.0 | 14 | 255 | n/a | Y |
| 200 | on | 488 | 2 | 1 | 199 | 1.2 | 14 | 0.0 | 14 | 255 | 25 | Y |
| 500 | off | 493 | 3 | 3 | 444 | 1.0 | 14 | 0.1 | 14 | 255 | n/a | N |
| 500 | on | 476 | 7 | 3 | 451 | 2.0 | 14 | 0.0 | 14 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -2 | +0 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -0 | +0 | +0.0 | +0.0 | +0 | Y | Y |
| 200 | -8 | +1 | +0.2 | +0.0 | +0 | Y | Y |
| 500 | -18 | +4 | +1.0 | -0.0 | +0 | N | N |

## Notes / anomalies

- rx_sim_x25 @500Hz t1: cmd_seq_lag_p95=3.0
- rx_sim_x25 @500Hz t3: cmd_seq_lag_p95=7.0

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈355; lag_p95≈1.0; act_lap≈1.5; periph_lap≈0.2
- **100 Hz**: ok 3/3; plant_fb≈339; lag_p95≈1.0; act_lap≈1.6; periph_lap≈0.1
- **200 Hz**: ok 3/3; plant_fb≈256; lag_p95≈1.3; act_lap≈1.9; periph_lap≈0.2
- **500 Hz**: ok 1/3; plant_fb≈43; lag_p95≈4.0; act_lap≈1.7; periph_lap≈0.3
