# Known issues and upgrade backlog

RobStride CH1 bench is functional for plant teleop and per-bus RS2 calibrate/discover. **Damiao CH3 discovery is blocked** on CAN RX (see below). Other items are open gaps or operational quirks.

## High priority — RS02 plant teleop chunking + CH2 cali (Jul 2026)

### Grouped motion on FDCAN CH2 and MCP CH4 (regression)

| | |
|---|---|
| **Symptom** | Plant teleop moves in ~0.2–0.4 s lumps; `fb` jumps ~0.35 rad; `lead` pins at ±0.35. **CH2 used to be smooth** before MCP teleop commits. |
| **Telemetry** | `pend=6`, `lap≈205ms` (CH2) / `368ms` (CH4), `block=none`. See `docs/bringup.md` §7. |
| **Regression** | Git `d9ce9e6` / `c700c78` after known-good `5df1f04`. Partial revert **did not fix** (Jul 2026). |
| **Handoff** | [handoff-plant-superloop-regression.md](handoff-plant-superloop-regression.md) |

### CH2 cali (`--bus 2`) no shaft spin after teleop

| | |
|---|---|
| **Symptom** | `calibrate --bus 2 --id 0x70`: prep OK, `0x05` issued, no `... cali listen` lines, no spin. |
| **Note** | Distinct from chunking path (bench PDU). CH4 MCP cali on same motor ID worked earlier. See `docs/bringup.md` §8. |

## High priority — Damiao CH3 (Jul 2026)

### CAN TX OK, motor RX silent (`rx_raw=0`)

| | |
|---|---|
| **Symptom** | `python scripts/damiao_scan.py --discover --bus 3` reports `tx>0` but `rx_raw=0` on every ESC_ID; no `FOUND`. USB `--link-test` and DM0 session path pass. |
| **Ruled out** | CAN H/L swap; missing termination **on controls PCB** (PCB has board-side 120 Ω). |
| **Likely cause** | **Missing 120 Ω at motor end** of a two-node bus. DM-J4310 has **no register** to enable onboard termination — external resistor across CAN_H/CAN_L at the far connector is required. Single-ended termination → frames leave MCU but nothing reflects back to RX. |
| **Verify** | Power off: measure CAN_H–CAN_L at PCB connector (~120 Ω = one terminator; ~60 Ω = both ends). Damiao Assistant + USB2CAN on motor with same harness. |
| **Workaround** | Splice 120 Ω across H/L at motor-side XT30 (or unused daisy-chain port). Re-run discover. |
| **Firmware** | `scripts/damiao_scan.py` + `plant_diag.c` / `damiao.c` — DM0 sync probe, REG_SCAN, `'m'` PDU. Reflash before bench. |
| **After fix** | Update `plant_config.c` slot 2 `motor_id` + `master_id` from reg scan; confirm with `--probe-id <N> --bus 3`. |

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
| Per-LED RGB from host | `leds[0]` is mode/brightness/count only; `g_pixels[]` filled in firmware |
| Feedback `header.seq` | Not incremented |
| ROTS / apply-history ring | TODO in `host_link_poll_tx` |
| NVM config loader | `plant_config` is compile-time RAM table |
| RobStride model limits table | RS-02/RS-01 limits hardcoded in `robstride.c` |
| Auto-pause 500 Hz when USB idle | Would reduce background CAN when no fresh host commands |
| MCP2518 CH4–CH6 full integration | SPI-CAN backends partially explored — see [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md) |

## Closed — Dynamixel neck servos (Jul 2026)

Bench path: **2× XL330** (ID 1 bottom, ID 2 top) on **UART5** @ 2M, host teleop @ ~40 Hz.

| Issue | Symptom | Fix |
|-------|---------|-----|
| Sync read/write on bus | `rd_rx=0`, frozen feedback, no hold | **Unicast** goal write + present-position read only; one bus op per TIM6 tick (`servo.c` + `control_loop.c`) |
| Oscillation / hunting | Goal chased lagging feedback every frame | Teleop **idle latch**: snap cmd to present on arrow entry; do not track fb every frame (`dynamixel_teleop.py`) |
| Position limits rejected sync | fb at ~1265/2623 outside old 1536–2560 window | Limits **1024–3072** (slot 1 down to **512**) in `plant_config.c` + teleop |
| HW error after stress / no recovery | Torque off until power cycle; overload at extreme `--arrow-vel` | Poll **HW Error Status (70)**; **REBOOT** + torque-on state machine; SVD diag exposes `hw_err0/1` |
| CAN LEDs blink during servo teleop | All three activity LEDs sync-blink, no motors connected | **Servo session** skips actuator CAN while `leds`/servo host commands active (`servo_host_session_active` + `plant_diag_skip_actuator_can`) |
| SVD missing on host | `diag=none(SVD missing — reflash firmware?)` | `servo_diag_feedback_fill` in `plant_feedback.c` when PDU not DXL |

**Scripts:** `scripts/dynamixel_teleop.py`, `scripts/dynamixel_scan.py`  
**Teleop defaults:** arrow vel **900**, instant stop on release (no ramp-down), `--arrow-vel` capped at 1500 in script.

## Closed — SK9822 LED strip (Jul 2026)

Bench path: full uncut strip on **SPI3** (PB3 SCK, PB5 MOSI), **5 V** PSU, common GND.

| Issue | Symptom | Fix |
|-------|---------|-----|
| Duplicate `led_table` definition | Linker error or wrong config | Single definition in **`plant_config.c`** (same pattern as `servo_table`); `extern` in `sk9822.h` |
| `LED_COUNT` vs `LED_STRIP_COUNT` | Undefined / mismatched symbol | Unified on **`LED_STRIP_COUNT`** in `sk9822.h` |
| Garbled / wrong LED colors | SPI frame bytes wrong | **`u32_to_bytes_be`** was missing `out[2]` (green channel shifted) in `sk9822.c` |
| `led_init` memset typo | Staged commands never cleared | `memset(&g_cmd_stage, …)` not `g_cmd_live` twice (`led.c`) |
| `led.h` typo | Build failure on mount prototype | `host_command_iamge_t` → **`host_command_image_t`** |
| Missing include in command router | `implicit declaration of led_command_mount` | **`#include "plant/led.h"`** in `plant_command.c` |
| Missing feedback include | Implicit declaration of `led_feedback_snapshot` | **`#include "plant/led.h"`** in `plant_feedback.c` |
| End frame length | Strip latch / wrong tail behavior | **`0xFF` + (5 + N/16)** zero bytes per SK9822/Pololu note (not `ceil(N/16)` alone) |

**Host `leds[0]`** (offset **528**, uint16 LE): mode 5b \| brightness 5b \| led_count 6b. Count **0** → `led_table[0].default_count` (`LED_STRIP_MAX`, default **120**).  
**Runtime:** `led_command_mount` in `plant_command.c`; **`led_service()` in `app_run()` @ ~30 Hz** (not TIM6). Modes: **0** = knight-rider test, **1** = off.  
**Test:** `python scripts/sk9822_led_test.py --port COM9`

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
| `ACTUATOR_COUNT = 1` only | Now **6** slots wired; 25 wire slots unchanged |
| Damiao plugin + DM0 PDU | `damiao.c`, `plant_diag.c`, `scripts/damiao_scan.py` — USB path OK; CAN RX pending termination |
| Cal timeout (`mms` stuck in cali) | Cal done on `mms=rest\|running`; per-bus probe routing via `pdu.data[11]` |
| Wrong bus for CH2/CH3 probes | `bus_handle` swap matches schematic |

When fixing an open item, move it to **Closed** with a short note or delete from this file.
