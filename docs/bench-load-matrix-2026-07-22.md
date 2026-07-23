# Load matrix — bus6 real RS02 vs ×25 rx_sim (+ DXL + LED)

Generated: 2026-07-22 17:22 Pacific Daylight Time

## Executive findings

1. **Real CH6 RS02 + DXL + LED is clean through 200 Hz.** Act lap ~0.8–0.9 ms, periph lap ~0.2 ms, `cmd_seq_lag_p95` ≤ 1, full 2π teleop (`Δ≈6.26/6.28`, `end_err≈0.02`) on all 12 trials after encoder cali.
2. **×25 ACTUATOR rx_sim costs ~+2.7–3.3 ms act lap** vs real-single-slot (~3.6–4.2 ms vs ~0.9 ms) and cuts plant_fb roughly in half at the same TX rate.
3. **500 Hz host TX does not sustain 500 Hz apply** under either load: applied ≈430–440 Hz; plant_fb collapses (~50 Hz real, ~57 Hz rx_sim). Lag p95 sits at the gate edge (2–3).
4. **Bandwidth baseline (hold-only, no DXL teleop):** all×25 RX-sim drops fb by ~125–155 Hz vs TX-only and adds ~+1.1–1.4 ms act lap. 500 Hz fails the ack/fb gates in the hold matrix even without peripherals thrashing.
5. **One anomaly:** rx_sim ×25 @ 500 Hz trial 3 had `cmd_seq_lag_p95=3` (gate ≤2). Not RS-related; no cali retry.

## Setup

- Port: `COM5`
- Rates: 40, 100, 200, 500 Hz
- Trials/rate: 3
- Soft hold: 0.6 s
- Base seconds/phase: 8.0 s (real RS auto-extends for trapezoid)
- RS02: CH6 id=0x70 slot=23
- DXL: slots 0/1 bounce @ π/4 rad/s; LED FLASH
- Cali before real matrix: True

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
| 40 | 3 | 457 | 491 | 40 | 0.11 / 1.0 / 4 | 0.8 / 4 | 0.2 / 10 | 1753/1776 | 6.26/6.28 | 3/3 |
| 100 | 3 | 416 | 489 | 100 | 0.36 / 1.0 / 4 | 0.9 / 4 | 0.2 / 10 | 1898/1763 | 6.26/6.28 | 3/3 |
| 200 | 3 | 266 | 478 | 200 | 0.81 / 1.0 / 3 | 0.9 / 4 | 0.2 / 11 | 1897/1761 | 6.26/6.28 | 3/3 |
| 500 | 3 | 50 | 428 | 435 | 1.74 / 2.0 / 8 | 0.9 / 6 | 0.5 / 14 | 1768/1627 | 6.26/6.28 | 3/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 457 | 1.0 | 0.9 | 4 | 0.1 | 9 | 40 | 1433 | 1785 | 6.26 | 0.02 | Y |  |
| 40 | 2 | 452 | 1.0 | 0.8 | 4 | 0.2 | 10 | 40 | 1910 | 1781 | 6.27 | 0.02 | Y |  |
| 40 | 3 | 461 | 1.0 | 0.8 | 4 | 0.2 | 8 | 40 | 1916 | 1761 | 6.26 | 0.02 | Y |  |
| 100 | 1 | 423 | 1.0 | 0.9 | 4 | 0.2 | 10 | 100 | 1922 | 1764 | 6.26 | 0.02 | Y |  |
| 100 | 2 | 420 | 1.0 | 0.8 | 4 | 0.3 | 8 | 100 | 1907 | 1760 | 6.26 | 0.02 | Y |  |
| 100 | 3 | 405 | 1.0 | 1.0 | 2 | 0.0 | 10 | 99 | 1865 | 1765 | 6.26 | 0.02 | Y |  |
| 200 | 1 | 249 | 1.0 | 0.8 | 4 | 0.2 | 11 | 200 | 1918 | 1759 | 6.26 | 0.02 | Y |  |
| 200 | 2 | 275 | 1.0 | 0.9 | 4 | 0.1 | 9 | 200 | 1916 | 1765 | 6.26 | 0.02 | Y |  |
| 200 | 3 | 274 | 1.0 | 0.9 | 2 | 0.2 | 7 | 200 | 1856 | 1758 | 6.26 | 0.02 | Y |  |
| 500 | 1 | 47 | 2.0 | 0.9 | 2 | 0.4 | 6 | 434 | 1855 | 1635 | 6.26 | 0.02 | Y |  |
| 500 | 2 | 47 | 2.0 | 1.0 | 5 | 0.5 | 9 | 436 | 1546 | 1598 | 6.26 | 0.02 | Y |  |
| 500 | 3 | 55 | 2.0 | 1.0 | 6 | 0.6 | 14 | 434 | 1904 | 1647 | 6.26 | 0.02 | Y |  |

## B — No real actuator on bus 6 (full ×25 ACTUATOR rx_sim) + DXL + LED

Product CFG ×25 with ACTUATOR rx_sim mask; no live RS02 MIT. Same DXL+LED load.

### Aggregate (mean across trials)

| tx Hz | n | plant_fb Hz | raw_fb Hz | applied Hz | cmd_seq_lag mean/p95/max | act_lap mean/peak | periph_lap mean/peak | s0/s1 span | RS Δ/cmd | ok |
|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|
| 40 | 3 | 257 | 272 | 40 | 0.10 / 1.0 / 2 | 3.6 / 13 | 0.0 / 14 | 1919/1732 | 0.00/0.00 | 3/3 |
| 100 | 3 | 222 | 263 | 100 | 0.31 / 1.0 / 10 | 3.6 / 14 | 0.1 / 14 | 1919/1747 | 0.00/0.00 | 3/3 |
| 200 | 3 | 155 | 241 | 200 | 0.78 / 1.0 / 16 | 3.9 / 18 | 0.1 / 14 | 1924/1754 | 0.00/0.00 | 3/3 |
| 500 | 3 | 57 | 199 | 440 | 1.78 / 2.3 / 37 | 4.2 / 18 | 0.1 / 14 | 1689/1712 | 0.00/0.00 | 2/3 |

### Per-trial

| tx | trial | plant_fb | lag_p95 | act_mn | act_pk | per_mn | per_pk | applied | s0 | s1 | rs_Δ | end_err | ok | notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 40 | 1 | 251 | 1.0 | 3.6 | 13 | 0.0 | 14 | 40 | 1910 | 1714 | 0.00 | 0.00 | Y |  |
| 40 | 2 | 260 | 1.0 | 3.5 | 13 | 0.0 | 14 | 40 | 1927 | 1754 | 0.00 | 0.00 | Y |  |
| 40 | 3 | 259 | 1.0 | 3.6 | 13 | 0.1 | 14 | 40 | 1920 | 1727 | 0.00 | 0.00 | Y |  |
| 100 | 1 | 214 | 1.0 | 3.6 | 13 | 0.1 | 14 | 100 | 1929 | 1747 | 0.00 | 0.00 | Y |  |
| 100 | 2 | 230 | 1.0 | 3.6 | 13 | 0.0 | 14 | 100 | 1909 | 1712 | 0.00 | 0.00 | Y |  |
| 100 | 3 | 222 | 1.0 | 3.6 | 14 | 0.1 | 14 | 100 | 1918 | 1782 | 0.00 | 0.00 | Y |  |
| 200 | 1 | 163 | 1.0 | 3.8 | 17 | 0.0 | 14 | 199 | 1928 | 1742 | 0.00 | 0.00 | Y |  |
| 200 | 2 | 137 | 1.0 | 4.0 | 18 | 0.1 | 14 | 200 | 1923 | 1787 | 0.00 | 0.00 | Y |  |
| 200 | 3 | 165 | 1.0 | 3.8 | 18 | 0.0 | 14 | 200 | 1920 | 1732 | 0.00 | 0.00 | Y |  |
| 500 | 1 | 54 | 2.0 | 4.1 | 18 | 0.1 | 14 | 439 | 1565 | 1763 | 0.00 | 0.00 | Y |  |
| 500 | 2 | 54 | 2.0 | 4.2 | 18 | 0.1 | 14 | 439 | 1953 | 1688 | 0.00 | 0.00 | Y |  |
| 500 | 3 | 63 | 3.0 | 4.2 | 18 | 0.1 | 14 | 442 | 1549 | 1684 | 0.00 | 0.00 | N | cmd_seq_lag_p95=3.0 |

## Bandwidth baseline (hold matrix, separate from teleop load)

TX-only vs RX-sim hold on product CFG slots. No DXL/LED teleop in this section.


### 1_CH1_x8

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 513 | 0 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | 8 | Y |
| 100 | off | 512 | 0 | 0 | 100 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 514 | 1 | 0 | 100 | 1.0 | 18 | 0 | 14 | 255 | 8 | Y |
| 200 | off | 513 | 2 | 1 | 200 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 512 | 1 | 1 | 199 | 1.1 | 18 | 0 | 14 | 255 | 8 | Y |
| 500 | off | 502 | 19 | 3 | 468 | 1 | 18 | 0 | 14 | 255 | n/a | N |
| 500 | on | 515 | 3 | 2 | 447 | 1.8 | 18 | 0 | 14 | 255 | 8 | N |

### 4_CH4_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 492 | 0 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 490 | 0 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | 2 | Y |
| 100 | off | 493 | 0 | 0 | 100 | 1.0 | 18 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 493 | 1 | 0 | 100 | 0.9 | 18 | 0 | 14 | 255 | 2 | Y |
| 200 | off | 494 | 1 | 1 | 198 | 1.0 | 18 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 493 | 1 | 1 | 200 | 1.0 | 18 | 0 | 14 | 255 | 2 | Y |
| 500 | off | 493 | 3 | 3 | 464 | 1.2 | 18 | 0 | 14 | 255 | n/a | N |
| 500 | on | 494 | 3 | 3 | 446 | 1.4 | 18 | 0 | 14 | 255 | 2 | N |

### 6_CH6_x2

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 512 | 0 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 512 | 0 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | 2 | Y |
| 100 | off | 504 | 4 | 0 | 98 | 1 | 18 | 0 | 14 | 255 | n/a | N |
| 100 | on | 512 | 0 | 0 | 100 | 1 | 18 | 0 | 14 | 255 | 2 | Y |
| 200 | off | 513 | 1 | 1 | 199 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 512 | 1 | 1 | 200 | 1 | 18 | 0 | 14 | 255 | 2 | Y |
| 500 | off | 514 | 3 | 3 | 454 | 1 | 18 | 0 | 14 | 255 | n/a | N |
| 500 | on | 512 | 3 | 3 | 444 | 1 | 18 | 0 | 14 | 255 | 2 | N |

### 7_CH1-3_fdcan

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 493 | 3 | 0 | 40 | 1 | 18 | 0 | 14 | 255 | n/a | N |
| 40 | on | 474 | 0 | 0 | 40 | 2.0 | 18 | 0 | 14 | 255 | 19 | Y |
| 100 | off | 512 | 1 | 0 | 100 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 438 | 1 | 0 | 100 | 1.9 | 18 | 0 | 14 | 255 | 19 | Y |
| 200 | off | 513 | 1 | 1 | 199 | 1 | 18 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 476 | 1 | 1 | 200 | 1.9 | 18 | 0 | 14 | 255 | 19 | Y |
| 500 | off | 512 | 3 | 3 | 443 | 1.4 | 18 | 0 | 14 | 255 | n/a | N |
| 500 | on | 436 | 3 | 3 | 450 | 2.3 | 18 | 0 | 14 | 255 | 19 | N |

### 8_CH4-6_mcp

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 487 | 0 | 0 | 40 | 2.0 | 18 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 472 | 0 | 0 | 40 | 2 | 18 | 0 | 14 | 255 | 6 | Y |
| 100 | off | 498 | 1 | 0 | 100 | 2.1 | 18 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 438 | 0 | 0 | 100 | 2.0 | 18 | 0 | 14 | 255 | 6 | Y |
| 200 | off | 486 | 1 | 1 | 200 | 2.1 | 18 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 466 | 8 | 1 | 199 | 2.0 | 18 | 0 | 14 | 255 | 6 | N |
| 500 | off | 450 | 3 | 3 | 445 | 2.2 | 18 | 0 | 14 | 255 | n/a | N |
| 500 | on | 414 | 3 | 3 | 446 | 2.5 | 18 | 0 | 14 | 255 | 6 | N |

### 9_all_CH1-6_x25

| tx Hz | rx_sim | fb_hz | ack_max | ack_p95 | applied_hz | act_mn | act_pk | per_mn | per_pk | pend_max | rx_fresh_max | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:---|---|
| 40 | off | 460 | 0 | 0 | 40 | 2.1 | 18 | 0 | 14 | 255 | n/a | Y |
| 40 | on | 316 | 0 | 0 | 40 | 3.2 | 18 | 0 | 14 | 255 | 25 | Y |
| 100 | off | 462 | 0 | 0 | 100 | 2.2 | 18 | 0 | 14 | 255 | n/a | Y |
| 100 | on | 305 | 1 | 0 | 100 | 3.5 | 18 | 0 | 14 | 255 | 25 | Y |
| 200 | off | 438 | 1 | 1 | 200 | 2.3 | 18 | 0 | 14 | 255 | n/a | Y |
| 200 | on | 288 | 2 | 1 | 198 | 3.6 | 18 | 0 | 14 | 255 | 25 | Y |
| 500 | off | 368 | 3 | 3 | 451 | 2.8 | 18 | 0 | 14 | 255 | n/a | N |
| 500 | on | 244 | 3 | 3 | 445 | 4.2 | 18 | 0.0 | 14 | 255 | 25 | N |

### all×25 delta (RX-sim ON − TX-only)

| tx Hz | Δfb | Δack_max | Δact_mn | Δper_mn | Δpend_max | tx_ok | rx_ok |
|---:|---:|---:|---:|---:|---:|---|---|
| 40 | -144 | +0 | +1.1 | +0.0 | +0 | Y | Y |
| 100 | -156 | +1 | +1.3 | +0.0 | +0 | Y | Y |
| 200 | -150 | +1 | +1.3 | +0.0 | +0 | Y | Y |
| 500 | -124 | +0 | +1.4 | +0.0 | +0 | N | N |

## Notes / anomalies

- rx_sim_x25 @500Hz t3: cmd_seq_lag_p95=3.0

## Takeaways

### Real CH6
- **40 Hz**: ok 3/3; plant_fb≈457; lag_p95≈1.0; act_lap≈0.8; periph_lap≈0.2
- **100 Hz**: ok 3/3; plant_fb≈416; lag_p95≈1.0; act_lap≈0.9; periph_lap≈0.2
- **200 Hz**: ok 3/3; plant_fb≈266; lag_p95≈1.0; act_lap≈0.9; periph_lap≈0.2
- **500 Hz**: ok 3/3; plant_fb≈50; lag_p95≈2.0; act_lap≈0.9; periph_lap≈0.5

### ×25 rx_sim
- **40 Hz**: ok 3/3; plant_fb≈257; lag_p95≈1.0; act_lap≈3.6; periph_lap≈0.0
- **100 Hz**: ok 3/3; plant_fb≈222; lag_p95≈1.0; act_lap≈3.6; periph_lap≈0.1
- **200 Hz**: ok 3/3; plant_fb≈155; lag_p95≈1.0; act_lap≈3.9; periph_lap≈0.1
- **500 Hz**: ok 2/3; plant_fb≈57; lag_p95≈2.3; act_lap≈4.2; periph_lap≈0.1
