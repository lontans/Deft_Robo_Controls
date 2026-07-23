# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 14:04 Pacific Daylight Time

## Cursor note (post-run)

Firmware under test: Release + RX-index + stagger + **`ROBSTRIDE_MAINTAIN_MAX_PER_TICK=2`**
(equal-rate defer of enable/run_mode pairs). Sticky `act_lap` peaks still
~7–11 ms under DXL+LED teleop, but bandwidth ×25 (no DXL/LED) shows
`act_lap_ms` sample max ≈ **1 ms** — so remaining sticky peak is Plant
wall-clock stretched by Peripheral `vTaskSuspendAll` (same-priority
preempt), not 25× MIT work. Next Cursor step: raise PlantTask above
Peripheral/Host.

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
| 40 | 3 | 359 | 400 | 40 | 0.12 / 1.0 / 1 | 1.3 / 9 | 0.0 / 9 | 1916/1764 | 0.00/0.00 | 3/3 |
| 100 | 3 | 299 | 394 | 100 | 0.36 / 1.0 / 2 | 1.5 / 11 | 0.2 / 15 | 1888/1763 | 0.00/0.00 | 3/3 |
| 200 | 3 | 234 | 419 | 198 | 0.89 / 1.3 / 9 | 1.9 / 11 | 0.2 / 15 | 1770/1765 | 0.00/0.00 | 3/3 |
| 500 | 3 | 32 | 373 | 404 | 1.63 / 2.0 / 4 | 1.5 / 11 | 0.3 / 15 | 1920/1692 | 0.00/0.00 | 3/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 344 | 1.0 | 1.2 | 7 | 0.0 | 9 | 40 | 1908 | 1768 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 358 | 1.0 | 1.2 | 9 | 0.0 | 9 | 40 | 1917 | 1781 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 374 | 1.0 | 1.3 | 9 | 0.0 | 9 | 40 | 1924 | 1743 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 317 | 1.0 | 1.3 | 10 | 0.0 | 12 | 100 | 1921 | 1768 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 325 | 1.0 | 1.4 | 11 | 0.2 | 15 | 100 | 1793 | 1744 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 255 | 1.0 | 1.9 | 11 | 0.4 | 15 | 100 | 1951 | 1778 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 259 | 2.0 | 1.8 | 11 | 0.1 | 15 | 199 | 1922 | 1744 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 212 | 1.0 | 1.8 | 11 | 0.3 | 15 | 197 | 1649 | 1779 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 230 | 1.0 | 1.9 | 11 | 0.2 | 15 | 198 | 1739 | 1771 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 33 | 2.0 | 1.5 | 11 | 0.2 | 15 | 406 | 1924 | 1751 | 0.00 | 0.00 | Y |  |
| 500 | 2 | 32 | 2.0 | 1.5 | 11 | 0.2 | 15 | 409 | 1919 | 1540 | 0.00 | 0.00 | Y |  |
| 500 | 3 | 31 | 2.0 | 1.3 | 11 | 0.4 | 15 | 398 | 1918 | 1784 | 0.00 | 0.00 | Y |  |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 514 | 0 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | 8 | Y |
| 100 | off | 512 | 0 | 0 | 100 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 490 | 6 | 1 | 99 | 0 | 11 | 0 | 15 | 255 | 8 | N |
| 200 | off | 513 | 1 | 1 | 199 | 0 | 11 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0 | 11 | 0 | 15 | 255 | 8 | Y |
| 500 | off | 514 | 3 | 2 | 445 | 0 | 11 | 0.0 | 15 | 255 | n/a | N |
| 500 | on | 515 | 3 | 3 | 415 | 0 | 11 | 0.0 | 15 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 508 | 1 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | 2 | Y |
| 100 | off | 513 | 0 | 0 | 100 | 0 | 11 | 0.0 | 15 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0.0 | 11 | 0 | 15 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0.0 | 11 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 509 | 2 | 1 | 198 | 0.0 | 11 | 0.0 | 15 | 255 | 2 | Y |
| 500 | off | 500 | 3 | 2 | 440 | 0.0 | 11 | 0.1 | 15 | 255 | n/a | N |
| 500 | on | 504 | 3 | 3 | 414 | 0.0 | 11 | 0.0 | 15 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | 2 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 514 | 0 | 0 | 100 | 0 | 11 | 0 | 15 | 255 | 2 | Y |
| 200 | off | 513 | 1 | 1 | 199 | 0.0 | 11 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 516 | 1 | 1 | 200 | 0 | 11 | 0.0 | 15 | 255 | 2 | Y |
| 500 | off | 514 | 3 | 2 | 450 | 0 | 11 | 0.0 | 15 | 255 | n/a | N |
| 500 | on | 514 | 3 | 2 | 444 | 0.0 | 11 | 0.1 | 15 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 512 | 1 | 0 | 40 | 0 | 11 | 0 | 15 | 255 | 19 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0.0 | 11 | 0 | 15 | 255 | 19 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 11 | 0 | 15 | 255 | n/a | Y |
| 200 | on | 514 | 2 | 1 | 199 | 0.2 | 11 | 0 | 15 | 255 | 19 | Y |
| 500 | off | 513 | 3 | 2 | 441 | 0 | 11 | 0 | 15 | 255 | n/a | N |
| 500 | on | 515 | 3 | 3 | 441 | 0.9 | 11 | 0.1 | 15 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0.3 | 11 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 511 | 0 | 0 | 40 | 1.0 | 11 | 0 | 15 | 255 | 6 | Y |
| 100 | off | 512 | 0 | 0 | 100 | 0.8 | 11 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 1 | 11 | 0 | 15 | 255 | 6 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0.8 | 11 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 506 | 1 | 1 | 199 | 1 | 11 | 0.0 | 15 | 255 | 6 | Y |
| 500 | off | 494 | 3 | 2 | 450 | 1.0 | 11 | 0.3 | 15 | 255 | n/a | N |
| 500 | on | 496 | 3 | 2 | 446 | 1.0 | 11 | 0.3 | 15 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 1.0 | 11 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 492 | 0 | 0 | 40 | 1 | 11 | 0 | 15 | 255 | 25 | Y |
| 100 | off | 496 | 0 | 0 | 100 | 1.0 | 11 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 494 | 1 | 0 | 100 | 1.1 | 11 | 0 | 15 | 255 | 25 | Y |
| 200 | off | 494 | 1 | 1 | 199 | 1.0 | 11 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 489 | 1 | 1 | 199 | 1.2 | 11 | 0.0 | 15 | 255 | 25 | Y |
| 500 | off | 494 | 3 | 3 | 443 | 1 | 11 | 0.1 | 15 | 255 | n/a | N |
| 500 | on | 478 | 3 | 3 | 437 | 2.0 | 11 | 0.0 | 15 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -20 | +0 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -2 | +1 | +0.1 | +0.0 | +0 | Y | Y |
| 200 | -5 | +0 | +0.2 | +0.0 | +0 | Y | Y |
| 500 | -16 | +0 | +1.0 | -0.0 | +0 | N | N |

## Notes / anomalies

- None recorded.

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈359; lag_p95≈1.0; act_lap≈1.3; periph_lap≈0.0
- **100 Hz**: ok 3/3; plant_fb≈299; lag_p95≈1.0; act_lap≈1.5; periph_lap≈0.2
- **200 Hz**: ok 3/3; plant_fb≈234; lag_p95≈1.3; act_lap≈1.9; periph_lap≈0.2
- **500 Hz**: ok 3/3; plant_fb≈32; lag_p95≈2.0; act_lap≈1.5; periph_lap≈0.3
