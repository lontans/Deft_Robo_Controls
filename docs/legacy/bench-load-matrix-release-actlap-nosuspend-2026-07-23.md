# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-23 14:16 Pacific Daylight Time

## Cursor note (post-run)

**Bug:** `act_lap` peak/mean hit **65535** (uint16 wrap). Cause: unlock called
`xTaskResumeAll()` *before* `plant_timing_scheduler_suspend_end()`, so Plant
could open/close a lap with `s_suspend_depth>0` and underflow the subtract.
Fixed order + saturating subtract; re-matrix next.

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
| 40 | 3 | 352 | 406 | 40 | 0.13 / 1.0 / 1 | 32.9 / 65535 | 0.0 / 8 | 2255/2028 | 0.00/0.00 | 3/3 |
| 100 | 3 | 307 | 400 | 100 | 0.38 / 1.0 / 5 | 1.4 / 65535 | 0.2 / 11 | 1875/1766 | 0.00/0.00 | 3/3 |
| 200 | 3 | 268 | 431 | 200 | 0.80 / 1.0 / 8 | 1.6 / 65535 | 0.1 / 11 | 1869/1755 | 0.00/0.00 | 3/3 |
| 500 | 3 | 33 | 374 | 415 | 1.76 / 2.3 / 4 | 1.2 / 65535 | 0.3 / 15 | 1671/1774 | 0.00/0.00 | 2/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 345 | 1.0 | 96.3 | 65535 | 0.0 | 7 | 40 | 2942 | 2500 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 352 | 1.0 | 1.2 | 65535 | 0.0 | 8 | 40 | 1869 | 1760 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 358 | 1.0 | 1.3 | 65535 | 0.0 | 8 | 40 | 1954 | 1824 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 263 | 1.0 | 1.3 | 65535 | 0.4 | 8 | 100 | 1810 | 1735 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 331 | 1.0 | 1.4 | 65535 | 0.0 | 8 | 100 | 1947 | 1780 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 326 | 1.0 | 1.4 | 65535 | 0.1 | 11 | 100 | 1867 | 1782 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 272 | 1.0 | 1.6 | 65535 | 0.1 | 11 | 199 | 1953 | 1782 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 271 | 1.0 | 1.6 | 65535 | 0.2 | 11 | 200 | 1701 | 1742 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 262 | 1.0 | 1.6 | 65535 | 0.1 | 11 | 200 | 1953 | 1742 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 32 | 2.0 | 1.2 | 65535 | 0.3 | 15 | 417 | 1665 | 1752 | 0.00 | 0.00 | Y |  |
| 500 | 2 | 39 | 3.0 | 1.3 | 65535 | 0.4 | 15 | 423 | 1594 | 1806 | 0.00 | 0.00 | N | cmd_seq_lag_p95=3.0 |
| 500 | 3 | 28 | 2.0 | 1.1 | 65535 | 0.2 | 15 | 407 | 1755 | 1763 | 0.00 | 0.00 | Y |  |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | 8 | Y |
| 100 | off | 514 | 1 | 0 | 100 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0 | 65535 | 0 | 15 | 255 | 8 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0 | 65535 | 0.0 | 15 | 255 | 8 | Y |
| 500 | off | 514 | 3 | 2 | 448 | 0 | 65535 | 0.0 | 15 | 255 | n/a | N |
| 500 | on | 514 | 3 | 2 | 448 | 0 | 65535 | 0.0 | 15 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 513 | 1 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | 2 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0.0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 513 | 1 | 0 | 100 | 0.1 | 65535 | 0 | 15 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 200 | 0.0 | 65535 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0.0 | 65535 | 0.0 | 15 | 255 | 2 | Y |
| 500 | off | 499 | 3 | 3 | 434 | 0.0 | 65535 | 0.1 | 15 | 255 | n/a | N |
| 500 | on | 498 | 3 | 3 | 458 | 0.0 | 65535 | 0.1 | 15 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 514 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | 2 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 513 | 0 | 0 | 100 | 0 | 65535 | 0 | 15 | 255 | 2 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 65535 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 0 | 65535 | 0.0 | 15 | 255 | 2 | Y |
| 500 | off | 514 | 3 | 2 | 448 | 0.0 | 65535 | 0.1 | 15 | 255 | n/a | N |
| 500 | on | 516 | 3 | 2 | 434 | 0 | 65535 | 0.0 | 15 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 0 | 65535 | 0 | 15 | 255 | 19 | Y |
| 100 | off | 513 | 1 | 0 | 100 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 512 | 1 | 0 | 100 | 0.2 | 65535 | 0 | 15 | 255 | 19 | Y |
| 200 | off | 514 | 1 | 1 | 199 | 0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 200 | on | 514 | 1 | 1 | 199 | 0.2 | 65535 | 0.0 | 15 | 255 | 19 | Y |
| 500 | off | 514 | 3 | 3 | 444 | 0 | 65535 | 0.0 | 15 | 255 | n/a | N |
| 500 | on | 513 | 3 | 3 | 446 | 0.9 | 65535 | 0.0 | 15 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 1.0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 513 | 0 | 0 | 40 | 1.0 | 65535 | 0 | 15 | 255 | 6 | Y |
| 100 | off | 513 | 0 | 0 | 100 | 1.0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 510 | 0 | 0 | 100 | 1.0 | 65535 | 0 | 15 | 255 | 6 | Y |
| 200 | off | 504 | 1 | 1 | 199 | 1 | 65535 | 0.0 | 15 | 255 | n/a | Y |
| 200 | on | 513 | 1 | 1 | 199 | 1.0 | 65535 | 0.0 | 15 | 255 | 6 | Y |
| 500 | off | 496 | 3 | 2 | 454 | 1.0 | 65535 | 0.3 | 15 | 255 | n/a | N |
| 500 | on | 498 | 3 | 3 | 444 | 1.0 | 65535 | 0.2 | 15 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 1 | 65535 | 0 | 15 | 255 | n/a | Y |
| 40 | on | 492 | 0 | 0 | 40 | 1 | 65535 | 0 | 15 | 255 | 25 | Y |
| 100 | off | 512 | 0 | 0 | 100 | 1 | 65535 | 0 | 15 | 255 | n/a | Y |
| 100 | on | 493 | 0 | 0 | 100 | 1.2 | 65535 | 0 | 15 | 255 | 25 | Y |
| 200 | off | 496 | 1 | 1 | 199 | 1.0 | 65535 | 0 | 15 | 255 | n/a | Y |
| 200 | on | 492 | 1 | 1 | 199 | 1.1 | 65535 | 0.0 | 15 | 255 | 25 | Y |
| 500 | off | 494 | 3 | 2 | 443 | 1.0 | 65535 | 0.1 | 15 | 255 | n/a | N |
| 500 | on | 482 | 3 | 3 | 443 | 2.1 | 65535 | 0.0 | 15 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -20 | +0 | +0.0 | +0.0 | +0 | Y | Y |
| 100 | -20 | +0 | +0.2 | +0.0 | +0 | Y | Y |
| 200 | -4 | +0 | +0.1 | +0.0 | +0 | Y | Y |
| 500 | -13 | +0 | +1.0 | -0.1 | +0 | N | N |

## Notes / anomalies

- rx_sim_x25 @500Hz t2: cmd_seq_lag_p95=3.0

## Takeaways

### Real CH6

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈352; lag_p95≈1.0; act_lap≈32.9; periph_lap≈0.0
- **100 Hz**: ok 3/3; plant_fb≈307; lag_p95≈1.0; act_lap≈1.4; periph_lap≈0.2
- **200 Hz**: ok 3/3; plant_fb≈268; lag_p95≈1.0; act_lap≈1.6; periph_lap≈0.1
- **500 Hz**: ok 2/3; plant_fb≈33; lag_p95≈2.3; act_lap≈1.2; periph_lap≈0.3
