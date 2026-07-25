# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 14:22 Pacific Daylight Time

## Cursor note (result)

**Peak kill success (equal-rate):** sticky `act_lap` **11→3–4 ms** (mean
~1.0–1.6) at 40/100/200/500; **12/12** load trials OK (500 Hz lag_p95=2).

Image: Release + RX-index + stagger + maintain budget≤2/tick + **act_lap
excludes DXL `vTaskSuspendAll` intervals** (same-band Host/Plant/Peripheral).
Plant-only High was tried and reverted (Host lag / periph blow-up).

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
| 40 | 3 | 354 | 404 | 40 | 0.11 / 1.0 / 2 | 1.3 / 3 | 0.1 / 13 | 1968/1802 | 0.00/0.00 | 3/3 |
| 100 | 3 | 274 | 371 | 100 | 0.35 / 1.0 / 2 | 1.4 / 4 | 0.3 / 13 | 1883/1758 | 0.00/0.00 | 3/3 |
| 200 | 3 | 242 | 428 | 199 | 0.85 / 1.0 / 3 | 1.6 / 4 | 0.2 / 13 | 1916/1776 | 0.00/0.00 | 3/3 |
| 500 | 3 | 26 | 380 | 390 | 1.61 / 2.0 / 4 | 1.0 / 4 | 0.1 / 13 | 1631/1745 | 0.00/0.00 | 3/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 361 | 1.0 | 1.3 | 3 | 0.0 | 9 | 40 | 1976 | 1806 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 342 | 1.0 | 1.2 | 3 | 0.1 | 9 | 40 | 1974 | 1774 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 360 | 1.0 | 1.3 | 3 | 0.3 | 13 | 40 | 1954 | 1827 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 253 | 1.0 | 1.3 | 4 | 0.4 | 13 | 100 | 1856 | 1740 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 307 | 1.0 | 1.4 | 4 | 0.1 | 13 | 100 | 1845 | 1778 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 262 | 1.0 | 1.3 | 4 | 0.4 | 13 | 100 | 1949 | 1756 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 260 | 1.0 | 1.6 | 4 | 0.1 | 13 | 200 | 1911 | 1775 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 235 | 1.0 | 1.6 | 4 | 0.2 | 13 | 199 | 1916 | 1779 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 232 | 1.0 | 1.6 | 4 | 0.2 | 13 | 199 | 1922 | 1775 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 25 | 2.0 | 0.9 | 4 | 0.2 | 13 | 389 | 1542 | 1708 | 0.00 | 0.00 | Y |  |
| 500 | 2 | 25 | 2.0 | 1.0 | 4 | 0.1 | 13 | 390 | 1428 | 1767 | 0.00 | 0.00 | Y |  |
| 500 | 3 | 27 | 2.0 | 1.0 | 4 | 0.1 | 13 | 391 | 1922 | 1759 | 0.00 | 0.00 | Y |  |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 514 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | 8 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 512 | 0 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | 8 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0 | 4 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0 | 4 | 0.0 | 13 | 255 | 8 | Y |
| 500 | off | 513 | 3 | 2 | 452 | 0 | 4 | 0.0 | 13 | 255 | n/a | N |
| 500 | on | 514 | 3 | 2 | 431 | 0 | 4 | 0.0 | 13 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 516 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | 2 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0 | 4 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0.0 | 4 | 0.0 | 13 | 255 | 2 | Y |
| 500 | off | 500 | 3 | 3 | 443 | 0.0 | 4 | 0.0 | 13 | 255 | n/a | N |
| 500 | on | 503 | 3 | 3 | 427 | 0.0 | 4 | 0.0 | 13 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 514 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | 2 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 512 | 1 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | 2 | Y |
| 200 | off | 513 | 1 | 1 | 199 | 0 | 4 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0 | 4 | 0.0 | 13 | 255 | 2 | Y |
| 500 | off | 514 | 3 | 2 | 447 | 0 | 4 | 0.0 | 13 | 255 | n/a | N |
| 500 | on | 514 | 3 | 2 | 434 | 0 | 4 | 0.0 | 13 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 4 | 0 | 13 | 255 | 19 | Y |
| 100 | off | 514 | 0 | 0 | 100 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 514 | 0 | 0 | 100 | 0.0 | 4 | 0 | 13 | 255 | 19 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0 | 4 | 0 | 13 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0.1 | 4 | 0.0 | 13 | 255 | 19 | Y |
| 500 | off | 513 | 3 | 3 | 437 | 0 | 4 | 0.0 | 13 | 255 | n/a | N |
| 500 | on | 514 | 3 | 3 | 444 | 0.9 | 4 | 0.0 | 13 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0.5 | 4 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 1.0 | 4 | 0 | 13 | 255 | 6 | Y |
| 100 | off | 514 | 0 | 0 | 100 | 0.9 | 4 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 1 | 4 | 0 | 13 | 255 | 6 | Y |
| 200 | off | 512 | 1 | 1 | 200 | 0.9 | 4 | 0.0 | 13 | 255 | n/a | Y |
| 200 | on | 512 | 1 | 1 | 199 | 1.0 | 4 | 0.0 | 13 | 255 | 6 | Y |
| 500 | off | 498 | 3 | 2 | 438 | 1.0 | 4 | 0.3 | 13 | 255 | n/a | N |
| 500 | on | 494 | 3 | 2 | 439 | 1.0 | 4 | 0.2 | 13 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 1.0 | 4 | 0 | 13 | 255 | n/a | Y |
| 40 | on | 492 | 0 | 0 | 40 | 1 | 4 | 0 | 13 | 255 | 25 | Y |
| 100 | off | 498 | 0 | 0 | 100 | 1 | 4 | 0 | 13 | 255 | n/a | Y |
| 100 | on | 493 | 1 | 0 | 100 | 1.0 | 4 | 0 | 13 | 255 | 25 | Y |
| 200 | off | 482 | 10 | 1 | 199 | 1.0 | 4 | 0 | 13 | 255 | n/a | N |
| 200 | on | 491 | 1 | 1 | 199 | 1.2 | 4 | 0.0 | 13 | 255 | 25 | Y |
| 500 | off | 494 | 3 | 3 | 441 | 1.0 | 4 | 0.1 | 13 | 255 | n/a | N |
| 500 | on | 466 | 11 | 3 | 451 | 2.0 | 4 | 0.0 | 13 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -20 | +0 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -4 | +1 | +0.0 | +0.0 | +0 | Y | Y |
| 200 | +10 | -9 | +0.2 | +0.0 | +0 | N | Y |
| 500 | -28 | +8 | +1.0 | -0.1 | +0 | N | N |

## Notes / anomalies

- None recorded.

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈354; lag_p95≈1.0; act_lap≈1.3; periph_lap≈0.1
- **100 Hz**: ok 3/3; plant_fb≈274; lag_p95≈1.0; act_lap≈1.4; periph_lap≈0.3
- **200 Hz**: ok 3/3; plant_fb≈242; lag_p95≈1.0; act_lap≈1.6; periph_lap≈0.2
- **500 Hz**: ok 3/3; plant_fb≈26; lag_p95≈2.0; act_lap≈1.0; periph_lap≈0.1
