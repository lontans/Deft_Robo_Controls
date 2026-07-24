# PDU × plant integrated test

Date: 2026-07-24 10:43
Port: COM5  hz=40.0  RS slot=24 id=0x70

Host reacts to PDU kill: NORMAL → stream motion; kill≠0 → `McuState.ESTOP` (servos cleared / RS desires cleared). LEDs follow PDU via firmware override.

Motion: slot0 full-range bounce @ π/4; slot1 ±80 ticks around initial present @ π/16; RS02 soft-hold.

| Phase | kill | led | host | fb_hz | lag_mean | act_lap | s0 | s1 | rsΔ | pack_v | result |
|-------|-----:|----:|------|------:|---------:|--------:|---:|---:|----:|--------|--------|
| NORMAL | 0 | 3 | NORMAL | 429.1 | 0.08856607310215557 | 0.18126747437092264 | 1726 | 56 | 0.001 | [48.00, 48.00, 0.00, 0.00] | PASS |
| SOFT_KILL_REQ | 1 | 6 | ESTOP | 451.9 | 0.11827354260089686 | 0.0005530973451327434 | 0 | 0 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| SOFT_KILL_READY | 2 | 5 | ESTOP | 445.2 | 0.12357630979498861 | 0.0 | 0 | 0 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| HARD_ESTOP | 3 | 7 | ESTOP | 445.4 | 0.12585421412300685 | 0.0 | 0 | 0 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| NORMAL_RESTORE | 0 | 3 | NORMAL | 405.9 | 0.08288110508140109 | 0.15014720314033367 | 1209 | 55 | 0.001 | [48.00, 48.00, 0.00, 0.00] | PASS |

## Per-phase detail

### NORMAL (`normal`)

- `ok_kill`: True
- `ok_led`: True
- `ok_motion`: True
- `ok_host`: True
- `lag_p95`: 1.0
- `lag_max`: 2
- `act_lap_peak_max`: 1.0
- `periph_lap_mean`: 0.1896551724137931
- `raw_fb_hz`: 482.9410425583673
- `rail_v_V`: [48.0, 19.0, 12.0, 5.0]
- `pack_i_A`: [1.2, 0.8, 0.0, 0.0]
- `rail_i_A`: [0.5, 0.3, 0.2, 0.1]
- `local_estop_mode`: 0

### SOFT_KILL_REQ (`soft_kill_req`)

- `ok_kill`: True
- `ok_led`: True
- `ok_motion`: True
- `ok_host`: True
- `lag_p95`: 1.0
- `lag_max`: 1
- `act_lap_peak_max`: 2.0
- `periph_lap_mean`: 0.09623893805309734
- `raw_fb_hz`: 498.18872278447697
- `rail_v_V`: [48.0, 19.0, 12.0, 5.0]
- `pack_i_A`: [1.2, 0.8, 0.0, 0.0]
- `rail_i_A`: [0.5, 0.3, 0.2, 0.1]
- `local_estop_mode`: 0

### SOFT_KILL_READY (`soft_kill_ready`)

- `ok_kill`: True
- `ok_led`: True
- `ok_motion`: True
- `ok_host`: True
- `lag_p95`: 1.0
- `lag_max`: 1
- `act_lap_peak_max`: 2.0
- `periph_lap_mean`: 0.08759124087591241
- `raw_fb_hz`: 500.1652719982586
- `rail_v_V`: [48.0, 19.0, 12.0, 5.0]
- `pack_i_A`: [1.2, 0.8, 0.0, 0.0]
- `rail_i_A`: [0.5, 0.3, 0.2, 0.1]
- `local_estop_mode`: 0

### HARD_ESTOP (`hard_estop`)

- `ok_kill`: True
- `ok_led`: True
- `ok_motion`: True
- `ok_host`: True
- `lag_p95`: 1.0
- `lag_max`: 1
- `act_lap_peak_max`: 2.0
- `periph_lap_mean`: 0.08922558922558922
- `raw_fb_hz`: 489.6506621208022
- `rail_v_V`: [48.0, 19.0, 12.0, 5.0]
- `pack_i_A`: [1.2, 0.8, 0.0, 0.0]
- `rail_i_A`: [0.5, 0.3, 0.2, 0.1]
- `local_estop_mode`: 0

### NORMAL_RESTORE (`normal`)

- `ok_kill`: True
- `ok_led`: True
- `ok_motion`: True
- `ok_host`: True
- `lag_p95`: 1.0
- `lag_max`: 2
- `act_lap_peak_max`: 2.0
- `periph_lap_mean`: 0.19381746810598627
- `raw_fb_hz`: 458.8885639341164
- `rail_v_V`: [48.0, 19.0, 12.0, 5.0]
- `pack_i_A`: [1.2, 0.8, 0.0, 0.0]
- `rail_i_A`: [0.5, 0.3, 0.2, 0.1]
- `local_estop_mode`: 0

## Summary: failed=0/5
