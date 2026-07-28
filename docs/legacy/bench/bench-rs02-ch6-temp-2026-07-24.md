# Bench — RS02 CH6 temperature study

Date: 2026-07-24T191001Z

## Setup

| Item | Value |
|------|--------|
| Port | COM5 |
| Bus / ID / slot | CH6 / 0x70 / 24 |
| Recalibrate | ok |
| Start pose | -0.0000 rad |
| kp / kd | 8.0 / 1.0 |
| v_max motion | 1.5708 rad/s |
| Phase durations (s) | disable=180.0 enable=180.0 motion=300.0 |

## Temperature by phase

| Phase | n | t span (s) | start °C | end °C | min | max | mean |
|-------|--:|-----------:|---------:|-------:|----:|----:|-----:|
| disabled | 351 | 0.0–179.8 | 40.0 | 40.0 | 40.0 | 40.0 | 40.0 |
| enabled_hold | 351 | 180.9–360.7 | 39.0 | 39.0 | 39.0 | 39.0 | 39.0 |
| slow_motion | 584 | 361.9–661.7 | 39.0 | 40.0 | 39.0 | 40.0 | 39.6 |

CSV: `bench-rs02-ch6-temp-2026-07-24.csv`

## Notes

- Temperature from plant MIT FB (RobStride raw/10 → °C).
- Disabled = kp=0 idle at measured start; enabled = MIT hold; motion = sine ≤ v_max.
- Heating allowed; motor left durable — no thermal abort.
- Recalibrate PASS (mechPos ≈ 0, VBUS ≈ 47.3 V). Start resolved from probe+plant (no p=0 snap).
- **No meaningful thermal rise** this run: FB stayed ~39–40 °C across ~11 min. Motion torque peaks were only ~±0.3 N·m (unloaded ±1 rad sine @ π/2). Prior heating issues likely need load / higher duty / longer dwell — data still logged in CSV (1286 samples).
