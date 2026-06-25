# Known issues and upgrade backlog

Bench is functional for four-actuator plant teleop and per-bus RS2 calibrate/discover. Items below are open gaps or operational quirks — not blockers for current learning bench work.

## Operational — calibration

### Encoder cal reports **NOISE** on daisy-chain bus (CH1)

| | |
|---|---|
| **Symptom** | `comm 0x05` encoder calibration fails with a **NOISE** fault when calibrating RS01 motors on the CH1 daisy chain (`0x74` behind `0x70`), or after repeated cal attempts without a clean bus idle period. |
| **Likely cause** | Innocuous bus contention — traffic from another motor on the shared branch, stale drive state, or residual frames after an interrupted probe/session. Not necessarily a wiring defect. |
| **Workaround** | **Power-cycle** the affected motor(s) or the bench supply, ensure no host script is streaming commands, run `--recovery` on CH1, then calibrate **one motor at a time** (`--bus 1 --target 0x74`). CH2/CH3 single-motor branches are less affected. |
| **Recal** | After NOISE, a power cycle is required before calibration will succeed; retrying cal in software alone is unreliable. |

## Medium priority (firmware / host)

| Issue | Where | Impact |
|-------|-------|--------|
| **UART TX blocks main loop** | `host_transport_uart.c` | Blocking TX can delay RX on Jetson UART path; USB CDC bench is unaffected. |
| **Silent CAN TX drop** | `actuator_apply_desire` | Full TX queue → frame skipped with no fault flag in feedback. |
| **Both transports always linked** | USB + UART objects in project | Duplicate RX ring BSS when only one mode is active. |
| **3× MOTOR_CTRL per motor per 2 ms** | `robstride_apply_cycle` | Reliability repeat on 500 Hz path; increases CAN load with four motors. |
| **RS2 session blocks plant loop** | `plant_diag_skip_actuator_can` | Intentional for bench probes; do not mix RS2 session with plant teleop. |
| **Ctrl+C mid-probe can wedge MCU** | Blocking `robstride_probe_id` | Host may need `--recovery` or brief reset on `0x70` before next session. |

## Low priority / not implemented yet

| Item | Notes |
|------|-------|
| System / servo / LED staging | Wire layout defined; only actuator commands consumed on normal path |
| Feedback `header.seq` | Not incremented |
| ROTS / apply-history ring | TODO in `host_link_poll_tx` |
| NVM config loader | `plant_config` is compile-time RAM table |
| RobStride model limits table | RS-02/RS-01 limits hardcoded in `robstride.c` |
| Auto-pause 500 Hz when USB idle | Would reduce background CAN when no fresh host commands |

## Closed (fixed in current tree)

| Issue | Resolution |
|-------|------------|
| No MCU magic resync on RX | `host_link` hunts `CMDH` in partial buffer |
| Main ↔ TIM6 staging races | Short `__disable_irq()` around staging copies |
| Stale feedback while TX pending | Rebuild feedback when `tx_ready()` each poll |
| `float_to_uint(…, 16)` UB | Unsigned shift in `robstride.c` |
| Out-of-range `protocol` index | `protocol >= PROTO_COUNT` guard |
| Enable OK when enqueue fails | Fixed in `control_loop_init` |
| FDCAN2/3 not routed | Three-bus `can_router` + CH2/CH3 actuators in `plant_config` |
| `ACTUATOR_COUNT = 1` only | Now **4** slots wired; 25 wire slots unchanged |
| Cal timeout (`mms` stuck in cali) | Cal done on `mms=rest\|running`; per-bus probe routing via `pdu.data[11]` |
| Wrong bus for CH2/CH3 probes | `bus_handle` swap matches schematic |

When fixing an open item, move it to **Closed** with a short note or delete from this file.
