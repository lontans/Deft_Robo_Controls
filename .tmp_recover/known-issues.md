# Known issues and upgrade backlog

RobStride CH1 bench is functional for plant teleop and per-bus RS2 calibrate/discover. **Damiao CH3 discover** had a host scan-order bug (fixed in `damiao.discover()`); see below. Other items are open gaps or operational quirks.

## High priority ΓÇö Damiao CH3 (Jul 2026)

### Discover scans wrong IDs first (host) ΓÇö fixed Jul 2026

| | |
|---|---|
| **Symptom** | `control_hub.py discover --protocol damiao --bus 3 --start 1 --end 16` ΓåÆ no hit; `--start 6 --end 6` or `damiao_scan.py --discover` finds ESC_ID **0x06**. |
| **Cause** | Per-ID `REG_SCAN` from ID **1** upward sends ~60 CAN frames per wrong ID before reaching the motor at **0x06**; the drive stops replying until a quiet period. Not missing termination ΓÇö **scan order + TX flood**. |
| **Fix** | `damiao.discover()` runs MCU `ID_SWEEP` first, then per-ID fallback with configured Damiao slot IDs (default **6**) before 1..5. Firmware: lighter reg-scan TX burst, RX drain after each probe. |
| **Verify** | `python scripts/control_hub.py discover --port COM5 --protocol damiao --bus 3` ΓåÆ `FOUND ΓÇª esc_id=0x06`. Expert path: `python scripts/damiao_scan.py --port COM5 --discover --bus 3`. |
| **Config** | Slot 2: `motor_id=0x06`, `master_id=auto` (RX master **0x16** on bench). |

### CAN TX OK, motor RX silent (`rx_raw=0`) ΓÇö deprioritized

| | |
|---|---|
| **Symptom** | Expert scan reports `tx>0` but `rx_raw=0` on every ESC_ID after the host-order bug is ruled out. |
| **Note** | If discover finds **0x06** with `damiao_scan.py --discover` or single-ID probe but never with `1..16` reg-scan only, treat as scan-order issue (above), not wiring. |
| **Still check** | Motor-end 120 ╬⌐ on a two-node bus if **all** probe styles show `rx_raw=0` including direct `--probe-id 0x06` after power cycle. |
| **Firmware** | `damiao.c` / `diag_dm.c` ΓÇö DM0 session, REG_SCAN, `ID_SWEEP`. Reflash after bench changes. |

## Operational ΓÇö calibration

### Encoder cal reports **NOISE** on daisy-chain bus (CH1)

| | |
|---|---|
| **Symptom** | `comm 0x05` encoder calibration fails with a **NOISE** fault when calibrating RS01 motors on the CH1 daisy chain (`0x74` behind `0x70`), or after repeated cal attempts without a clean bus idle period. |
| **Likely cause** | Innocuous bus contention ΓÇö traffic from another motor on the shared branch, stale drive state, or residual frames after an interrupted probe/session. Not necessarily a wiring defect. |
| **Workaround** | **Power-cycle** the affected motor(s) or the bench supply, ensure no host script is streaming commands, run `--recovery` on CH1, then calibrate **one motor at a time** (`--bus 1 --target 0x74`). CH2/CH3 single-motor branches are less affected. |
| **Recal** | After NOISE, a power cycle is required before calibration will succeed; retrying cal in software alone is unreliable. |

## Medium priority (firmware / host)

| Issue | Where | Impact |
|-------|-------|--------|
| **UART TX blocks main loop** | `host_transport_uart.c` | Blocking TX can delay RX on Jetson UART path; USB CDC bench is unaffected. |
| **Silent CAN TX drop** | `actuator_apply_desire` | Full TX queue ΓåÆ frame skipped with no fault flag in feedback. |
| **Mixed-bus bitrate** | CH3 std+ext | All nodes on a shared branch must use the same nominal bit timing (1 Mbps); mismatched devices will not decode each other. |
| **Both transports always linked** | USB + UART objects in project | Duplicate RX ring BSS when only one mode is active. |
| **3├ù MOTOR_CTRL per motor per 2 ms** | `robstride_apply_cycle` | Reliability repeat on 500 Hz path; increases CAN load with four motors. |
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
| MCP2518 CH4ΓÇôCH6 full integration | SPI-CAN backends partially explored ΓÇö see [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md) |

## Closed ΓÇö Dynamixel neck servos (Jul 2026)

Bench path: **2├ù XL330** (ID 1 bottom, ID 2 top) on **UART5** @ 2M, host teleop @ ~40 Hz.

| Issue | Symptom | Fix |
|-------|---------|-----|
| Sync read/write on bus | `rd_rx=0`, frozen feedback, no hold | **Unicast** goal write + present-position read only; one bus op per TIM6 tick (`servo.c` + `control_loop.c`) |
| Oscillation / hunting | Goal chased lagging feedback every frame | Teleop **idle latch**: snap cmd to present on arrow entry; do not track fb every frame (`dynamixel_teleop.py`) |
| Position limits rejected sync | fb at ~1265/2623 outside old 1536ΓÇô2560 window | Limits **1024ΓÇô3072** (slot 1 down to **512**) in `plant_config.c` + teleop |
| HW error after stress / no recovery | Torque off until power cycle; overload at extreme `--arrow-vel` | Poll **HW Error Status (70)**; **REBOOT** + torque-on state machine; SVD diag exposes `hw_err0/1` |
| CAN LEDs blink during servo teleop | All three activity LEDs sync-blink, no motors connected | **Servo session** skips actuator CAN while `leds`/servo host commands active (`servo_host_session_active` + `plant_diag_skip_actuator_can`) |
| SVD missing on host | `diag=none(SVD missing ΓÇö reflash firmware?)` | `servo_diag_feedback_fill` in `plant_feedback.c` when PDU not DXL |

**Scripts:** `scripts/dynamixel_teleop.py`, `scripts/dynamixel_scan.py`  
**Teleop defaults:** arrow vel **900**, instant stop on release (no ramp-down), `--arrow-vel` capped at 1500 in script.

## Closed ΓÇö SK9822 LED strip (Jul 2026)

Bench path: full uncut strip on **SPI3** (PB3 SCK, PB5 MOSI), **5 V** PSU, common GND.

| Issue | Symptom | Fix |
|-------|---------|-----|
| Duplicate `led_table` definition | Linker error or wrong config | Single definition in **`plant_config.c`** (same pattern as `servo_table`); `extern` in `sk9822.h` |
| `LED_COUNT` vs `LED_STRIP_COUNT` | Undefined / mismatched symbol | Unified on **`LED_STRIP_COUNT`** in `sk9822.h` |
| Garbled / wrong LED colors | SPI frame bytes wrong | **`u32_to_bytes_be`** was missing `out[2]` (green channel shifted) in `sk9822.c` |
| `led_init` memset typo | Staged commands never cleared | `memset(&g_cmd_stage, ΓÇª)` not `g_cmd_live` twice (`led.c`) |
| `led.h` typo | Build failure on mount prototype | `host_command_iamge_t` ΓåÆ **`host_command_image_t`** |
| Missing include in command router | `implicit declaration of led_command_mount` | **`#include "plant/led.h"`** in `plant_command.c` |
| Missing feedback include | Implicit declaration of `led_feedback_snapshot` | **`#include "plant/led.h"`** in `plant_feedback.c` |
| End frame length | Strip latch / wrong tail behavior | **`0xFF` + (5 + N/16)** zero bytes per SK9822/Pololu note (not `ceil(N/16)` alone) |

**Host `leds[0]`** (offset **528**, uint16 LE): mode 5b \| brightness 5b \| led_count 6b. Count **0** ΓåÆ `led_table[0].default_count` (`LED_STRIP_MAX`, default **120**).  
**Runtime:** `led_command_mount` in `plant_command.c`; **`led_service()` in `app_run()` @ ~30 Hz** (not TIM6). Modes: **0** = knight-rider test, **1** = off.  
**Test:** `python scripts/sk9822_led_test.py --port COM9`

## Closed ΓÇö RS02 plant teleop + CH2 cali (Jul 2026)

Bench verified: CH2 FDCAN (`teleop --slot 1`) and CH4 MCP (`teleop --slot 3`) with `lapΓëê0ΓÇô1 ms`, smooth `cmd`/`fb`; CH2 cali after teleop spins. Details: [bringup.md](bringup.md) ┬º7ΓÇô┬º8, [handoff-plant-superloop-regression.md](handoff-plant-superloop-regression.md).

| Issue | Symptom | Fix |
|-------|---------|-----|
| Grouped motion on CH2 / CH4 | ~0.2ΓÇô0.4 s motion lumps; `lap` 50ΓÇô370 ms, `pend` maxed, `lead` pinned | Firmware: restore burst 8, single `control_loop_service()`, DXL skip without servo session, eager MCP init, fire-and-forget MCP plant TX. Host: ack-only stale gate, cmd slew integration, `fb_age` on any fresh sample (`plant.py`) |
| CH2 cali after teleop | Prep HIT, `0x05` sent, no `cali listen`, no spin | `calibrate.py`: FDCAN gets same 250 ms settle + pre-`0x05` reset as MCP (removed `CALI_SKIP_RESET`); teleop exit runs `recovery_on_exit` |

## Closed (fixed in current tree)

| Issue | Resolution |
|-------|------------|
| No MCU magic resync on RX | `host_link` hunts `CMDH` in partial buffer |
| Main Γåö TIM6 staging races | Short `__disable_irq()` around staging copies |
| Stale feedback while TX pending | Rebuild feedback when `tx_ready()` each poll |
| `float_to_uint(ΓÇª, 16)` UB | Unsigned shift in `robstride.c` |
| Out-of-range `protocol` index | `protocol >= PROTO_COUNT` guard |
| Enable OK when enqueue fails | Fixed in `control_loop_init` |
| FDCAN2/3 not routed | Three-bus `can_router` + CH2/CH3 actuators in `plant_config` |
| CH3 mixed std+ext RX | `can_router.c` dual filters on `hfdcan2`; `actuator_dispatch_bus_rx` fan-out ΓÇö see [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) |
| `ACTUATOR_COUNT = 1` only | Now **6** slots wired; 25 wire slots unchanged |
| Damiao plugin + DM0 PDU | `damiao.c`, `plant_diag.c`, `scripts/damiao_scan.py` ΓÇö USB path OK; CAN RX pending termination |
| Cal timeout (`mms` stuck in cali) | Cal done on `mms=rest\|running`; per-bus probe routing via `pdu.data[11]` |
| Wrong bus for CH2/CH3 probes | `bus_handle` swap matches schematic |

When fixing an open item, move it to **Closed** with a short note or delete from this file.
