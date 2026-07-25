# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 13:05 Pacific Daylight Time

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
| 40 | 3 | 247 | 268 | 40 | 0.12 / 1.0 / 3 | 3.6 / 18 | 0.0 / 12 | 1875/1757 | 0.00/0.00 | 3/3 |
| 100 | 3 | 206 | 254 | 100 | 0.36 / 1.0 / 3 | 3.7 / 18 | 0.1 / 12 | 1864/1778 | 0.00/0.00 | 3/3 |
| 200 | 3 | 135 | 231 | 197 | 0.81 / 1.3 / 6 | 4.0 / 18 | 0.1 / 12 | 1903/1779 | 0.00/0.00 | 3/3 |
| 500 | 3 | 31 | 195 | 420 | 1.82 / 3.3 / 18 | 3.5 / 18 | 0.1 / 12 | 1571/1721 | 0.00/0.00 | 2/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 245 | 1.0 | 3.6 | 18 | 0.0 | 12 | 40 | 1944 | 1780 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 251 | 1.0 | 3.6 | 18 | 0.0 | 12 | 40 | 1765 | 1752 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 246 | 1.0 | 3.7 | 18 | 0.0 | 12 | 40 | 1916 | 1739 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 217 | 1.0 | 3.7 | 18 | 0.1 | 12 | 100 | 1916 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 204 | 1.0 | 3.7 | 18 | 0.1 | 12 | 100 | 1909 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 195 | 1.0 | 3.7 | 18 | 0.1 | 12 | 100 | 1768 | 1778 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 124 | 2.0 | 3.9 | 18 | 0.1 | 12 | 196 | 1934 | 1780 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 142 | 1.0 | 4.0 | 18 | 0.0 | 12 | 195 | 1942 | 1778 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 140 | 1.0 | 4.1 | 18 | 0.0 | 12 | 199 | 1834 | 1779 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 34 | 2.0 | 3.7 | 18 | 0.1 | 12 | 418 | 1576 | 1777 | 0.00 | 0.00 | Y |  |
| 500 | 2 | 30 | 6.0 | 3.3 | 18 | 0.0 | 12 | 427 | 1463 | 1631 | 0.00 | 0.00 | N | cmd_seq_lag_p95=6.0 |
| 500 | 3 | 29 | 2.0 | 3.4 | 18 | 0.0 | 12 | 416 | 1674 | 1755 | 0.00 | 0.00 | Y |  |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 480 | 3 | 0 | 39 | 1 | 18 | 0 | 12 | 255 | n/a | N |
| 40 | on | 512 | 0 | 0 | 40 | 1 | 18 | 0 | 12 | 255 | 8 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 1 | 18 | 0 | 12 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 1.0 | 18 | 0 | 12 | 255 | 8 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 1 | 18 | 0 | 12 | 255 | n/a | Y |
| 200 | on | 502 | 7 | 1 | 202 | 1.1 | 18 | 0 | 12 | 255 | 8 | N |
| 500 | off | 513 | 3 | 3 | 468 | 1 | 18 | 0 | 12 | 255 | n/a | N |
| 500 | on | 514 | 3 | 2 | 425 | 1.7 | 18 | 0 | 12 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 494 | 0 | 0 | 40 | 1 | 18 | 0 | 12 | 255 | n/a | Y |
| 40 | on | 493 | 0 | 0 | 40 | 1 | 18 | 0 | 12 | 255 | 2 | Y |
| 100 | off | 494 | 1 | 0 | 100 | 1.0 | 18 | 0 | 12 | 255 | n/a | Y |
| 100 | on | 494 | 1 | 0 | 100 | 1 | 18 | 0 | 12 | 255 | 2 | Y |
| 200 | off | 488 | 3 | 1 | 197 | 1.0 | 18 | 0 | 12 | 255 | n/a | N |
| 200 | on | 496 | 2 | 1 | 196 | 1 | 18 | 0 | 12 | 255 | 2 | Y |
| 500 | off | 494 | 3 | 3 | 425 | 1.1 | 18 | 0 | 12 | 255 | n/a | N |
| 500 | on | 487 | 3 | 3 | 434 | 1.5 | 18 | 0 | 12 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 481 | 2 | 0 | 40 | 1 | 18 | 0 | 12 | 255 | n/a | Y |
| 40 | on | 490 | 1 | 0 | 40 | 1 | 18 | 0 | 12 | 255 | 2 | Y |
| 100 | off | 494 | 1 | 0 | 100 | 1 | 18 | 0 | 12 | 255 | n/a | Y |
| 100 | on | 493 | 1 | 1 | 100 | 1 | 18 | 0 | 12 | 255 | 2 | Y |
| 200 | off | 494 | 2 | 1 | 198 | 1.0 | 18 | 0 | 12 | 255 | n/a | Y |
| 200 | on | 487 | 6 | 1 | 196 | 1.0 | 18 | 0 | 12 | 255 | 2 | N |
| 500 | off | 495 | 4 | 3 | 430 | 1.1 | 18 | 0 | 12 | 255 | n/a | N |
| 500 | on | 495 | 3 | 3 | 453 | 1.5 | 18 | 0 | 12 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 504 | 1 | 0 | 40 | 1 | 18 | 0 | 12 | 255 | n/a | Y |
| 40 | on | 474 | 0 | 0 | 40 | 2 | 18 | 0 | 12 | 255 | 19 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 1.0 | 18 | 0 | 12 | 255 | n/a | Y |
| 100 | on | 436 | 1 | 0 | 100 | 1.9 | 18 | 0 | 12 | 255 | 19 | Y |
| 200 | off | 514 | 2 | 1 | 199 | 1.0 | 18 | 0 | 12 | 255 | n/a | Y |
| 200 | on | 476 | 1 | 1 | 198 | 2.0 | 18 | 0 | 12 | 255 | 19 | Y |
| 500 | off | 514 | 3 | 3 | 459 | 1.4 | 18 | 0 | 12 | 255 | n/a | N |
| 500 | on | 438 | 3 | 3 | 433 | 2.3 | 18 | 0.0 | 12 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 473 | 0 | 0 | 40 | 2.0 | 18 | 0 | 12 | 255 | n/a | Y |
| 40 | on | 480 | 0 | 0 | 40 | 2 | 18 | 0 | 12 | 255 | 6 | Y |
| 100 | off | 440 | 1 | 0 | 100 | 2.2 | 18 | 0 | 12 | 255 | n/a | Y |
| 100 | on | 471 | 1 | 0 | 100 | 2.1 | 18 | 0 | 12 | 255 | 6 | Y |
| 200 | off | 463 | 4 | 1 | 200 | 2.0 | 18 | 0 | 12 | 255 | n/a | N |
| 200 | on | 446 | 1 | 1 | 199 | 2.1 | 18 | 0 | 12 | 255 | 6 | Y |
| 500 | off | 418 | 3 | 3 | 459 | 2.4 | 18 | 0 | 12 | 255 | n/a | N |
| 500 | on | 404 | 3 | 3 | 436 | 2.5 | 18 | 0 | 12 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 455 | 0 | 0 | 40 | 2.2 | 18 | 0 | 12 | 255 | n/a | Y |
| 40 | on | 300 | 0 | 0 | 40 | 3.3 | 18 | 0 | 12 | 255 | 25 | Y |
| 100 | off | 444 | 1 | 0 | 100 | 2.4 | 18 | 0 | 12 | 255 | n/a | Y |
| 100 | on | 284 | 1 | 0 | 100 | 3.7 | 18 | 0 | 12 | 255 | 25 | Y |
| 200 | off | 408 | 2 | 1 | 199 | 2.4 | 18 | 0 | 12 | 255 | n/a | Y |
| 200 | on | 268 | 2 | 1 | 199 | 3.8 | 18 | 0.0 | 12 | 255 | 25 | Y |
| 500 | off | 346 | 3 | 3 | 464 | 3.0 | 18 | 0 | 12 | 255 | n/a | N |
| 500 | on | 244 | 3 | 3 | 433 | 4.2 | 18 | 0.0 | 12 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -155 | +0 | +1.2 | +0.0 | +0 | Y | Y |
| 100 | -160 | +0 | +1.3 | +0.0 | +0 | Y | Y |
| 200 | -140 | +0 | +1.4 | +0.0 | +0 | Y | Y |
| 500 | -102 | +0 | +1.3 | +0.0 | +0 | N | N |

## Notes / anomalies

- rx_sim_x25 @500Hz t2: cmd_seq_lag_p95=6.0

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈247; lag_p95≈1.0; act_lap≈3.6; periph_lap≈0.0
- **100 Hz**: ok 3/3; plant_fb≈206; lag_p95≈1.0; act_lap≈3.7; periph_lap≈0.1
- **200 Hz**: ok 3/3; plant_fb≈135; lag_p95≈1.3; act_lap≈4.0; periph_lap≈0.1
- **500 Hz**: ok 2/3; plant_fb≈31; lag_p95≈3.3; act_lap≈3.5; periph_lap≈0.1
