# PDU × plant integrated test

Date: 2026-07-24 11:55
Port: COM5  hz=40.0  RS slot=24 id=0x70

Host reacts to PDU kill: NORMAL → stream motion; kill≠0 → `McuState.ESTOP` (servos cleared / RS desires cleared). LEDs follow PDU via firmware override.

Motion: slot0 full-range bounce @ π/4; slot1 ±80 ticks around initial present @ π/16; RS02 soft-hold.

| Phase | kill | led | host | fb_hz | lag_mean | act_lap | s0 | s1 | rsΔ | pack_v | result |
|-------|-----:|----:|------|------:|---------:|--------:|---:|---:|----:|--------|--------|
| NORMAL | 0 | 8 | NORMAL | 430.5 | 0.11425917324663261 | 0.20901068276823037 | 1730 | 41 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| SOFT_KILL_REQ | 1 | 6 | ESTOP | 446.7 | 0.10380034032898469 | 0.0 | 0 | 0 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| SOFT_KILL_READY | 2 | 5 | ESTOP | 449.7 | 0.10321489001692047 | 0.0 | 0 | 0 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| HARD_ESTOP | 3 | 7 | ESTOP | 449.5 | 0.11349520045172219 | 0.0011123470522803114 | 0 | 0 | 0.000 | [48.00, 48.00, 0.00, 0.00] | PASS |
| NORMAL_RESTORE | 0 | 8 | NORMAL | 412.5 | 0.07894736842105263 | 0.1662627241880756 | 1229 | 24 | 0.001 | [48.00, 48.00, 0.00, 0.00] | PASS |

## Per-phase detail

### NORMAL (`normal`)

- `ok_kill`: True
- `ok_led`: True
- `ok_motion`: True
- `ok_host`: True
- `lag_p95`: 1.0
- `lag_max`: 2
- `act_lap_peak_max`: 1.0
- `periph_lap_mean`: 0.18485833720390155
- `raw_fb_hz`: 484.29390089616163
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
- `periph_lap_mean`: 0.14269725797425853
- `raw_fb_hz`: 495.66659170390415
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
- `lag_max`: 2
- `act_lap_peak_max`: 2.0
- `periph_lap_mean`: 0.10450250138966093
- `raw_fb_hz`: 499.9522420621876
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
- `periph_lap_mean`: 0.11457174638487208
- `raw_fb_hz`: 489.96562891322105
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
- `lag_max`: 1
- `act_lap_peak_max`: 2.0
- `periph_lap_mean`: 0.20067862336403297
- `raw_fb_hz`: 472.5334200457371
- `rail_v_V`: [48.0, 19.0, 12.0, 5.0]
- `pack_i_A`: [1.2, 0.8, 0.0, 0.0]
- `rail_i_A`: [0.5, 0.3, 0.2, 0.1]
- `local_estop_mode`: 0

## Summary: failed=0/5
