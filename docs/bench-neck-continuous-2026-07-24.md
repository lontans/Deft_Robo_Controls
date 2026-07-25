# Bench — neck DXL + yam_continuous_all live

Date: 2026-07-24 (evening)  
Owner: Cursonier  
Board: Jetson `192.168.50.48`, `/dev/ttyACM0`, serial `3167376F3435`  
Constraint: Claudistic dashboard/GUI left alone (no `debug_dashboard` sync; CDC not held after)

## 1. Isolated neck (Dynamixel slots 0/1)

Torque-off discover → hold present → +80-step pitch nudge → clear, via `PcbRobotSession` (same write path as continuous).

```text
DXL present pitch=2570 yaw=1138
pitch nudge goal=2650 fb=2532
NECK_TRACK_OK
EXIT=0  CDC_FREE
```

| Check | Result |
|-------|--------|
| Present discover (both IDs) | PASS |
| Pitch track after nudge | PASS (`|fb−goal|≈118` steps) |
| CDC release | PASS |

Note: an earlier hub-only draft failed (`ControlsPcbHub` has no `poll_feedback`); continuous + the session-based prove above are the valid evidence.

## 2. Continuous cruise (`yam_continuous_all.py`)

Synced continuous stack only (not dashboard). Launched with PDU UART sim + `--record --duration 45`, soft-kill via follow-mode `.deft_session/soft_kill_request` after latch+cruise.

| Check | Result |
|-------|--------|
| CH1 Damiao discover 1–7 | PASS — all FOUND |
| Progressive arm latch | PASS — `faults=[1×7]` |
| DXL present phase | Reported `pitch=0 yaw=0` (false-zero accept); cruise FB later showed real neck motion |
| Base spare IDs probe | PASS — `0x70`/`0x74`/`0x75` found; CH6 Damiao `0x06` |
| Cruise J2 CLEAR bounce | PASS — reverse stuck then reverse arrived |
| Base continuous spin + rail reverse | PASS — s22–s25 cmd/fb tracking |
| DXL cruise track | PASS — e.g. `dxl=2934/2817\|2120/2414` |
| Stream health | `fb≈470–500` Hz, `tx=20`, `ack=0`, `gap95≈50 ms` |
| Soft-kill park (flag file) | PASS — `dashboard soft_kill_request → soft_kill_park()` → cleanup → done |
| Recording | `record_20260724T225538.ndjson` |
| Post CDC | FREE |

PDU line showed `pdb=hard_estop/COMMS_LOSS` during cruise (UART sim / peer link axis) — peripherals still healthy; do not treat as arm/DXL/base failure (see `docs/peripherals/continuous-ops.md`).

## Handoff

- CDC free; `pdb_uart_sim` restored on `/dev/ttyTHS1` with `--control-port 8767` (prior Claudistic-shaped sim).
- Neck live under continuous + isolated nudge. Continuous full stack (left arm + neck + bench-spare base) re-proven with soft-kill park.
