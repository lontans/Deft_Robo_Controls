# FreeRTOS bring-up — Controls PCB (STM32G474)

Technical notes on migrating Deft controls firmware from a bare-metal superloop (`14eb426`) to **FreeRTOS via CMSIS-RTOS v1** (CubeMX). Format follows [ch4-mcp2518-bringup-notes.txt](ch4-mcp2518-bringup-notes.txt): observable signals, debug timeline, root causes, and bench verification.

**Status (Jul 2026):** USB CDC enumerates (`COM5`), PC3 heartbeat ~1 Hz, scheduler runs two tasks. Regressions introduced in the first RTOS commits (solid heartbeat, Windows Code 43) are documented below with root cause and resolution.

**Reference:** ST UM1722 (*Developing Applications on STM32Cube with RTOS*), `External_Documentation/STM32/STM32 RTOS Documentation.pdf`.

---

## Ground truth signals

| Signal | Meaning |
|--------|---------|
| PC2 ×3 boot flashes | `main()` alive through early GPIO |
| PC2 solid ON | `CDC_Init_FS` — USB enumerated and configured by host |
| PC3 ~1 Hz toggle | TIM6 ISR running (`control_loop_tick`) |
| PC1 HIGH → LOW | Bring-up milestone: USB init returned → `app_init()` returned |
| CAN CH1–6 LEDs solid | `can_router_init()` finished (pre-scheduler; **not** proof RTOS runs) |
| PC2+PC3 slow alternate blink | `Error_Handler()` |
| PC2+PC3 fast alternate blink | `HardFault_Handler()` |
| Windows connect chime + Code 43 | USB stack started but descriptor/config timed out (typical when blocked in `app_init`) |
| No chime, no COM | USB never started or MCU hung before `MX_USB_Device_Init()` |

---

## Git anchors

| Commit | Label |
|--------|--------|
| `14eb426` | Last **pre-RTOS** commit — Damiao enable working, USB + heartbeat nominal |
| `cfecd2e` | First RTOS commit (“mid-migration”) — kernel integration; regressions begin |
| Working tree (Jul 2026) | Phase 2 task split + bring-up fixes (may be uncommitted) |

---

## Implementation overview

### Stack and CubeMX configuration

| Item | Value |
|------|--------|
| RTOS API | **CMSIS-RTOS v1** (`cmsis_os.h` / `osThreadCreate`) — not v2 |
| Kernel | FreeRTOS 10.3.1 (Cube FW_G4 1.6.3) |
| HAL timebase | **TIM7** @ 1 ms (`VP_SYS_VS_tim7` in `.ioc`) |
| Kernel tick | **SysTick** → `xPortSysTickHandler()` (FreeRTOS owns SysTick) |
| Heap | Dynamic only, `configTOTAL_HEAP_SIZE = 24 KiB` |
| FPU | `configENABLE_FPU = 0` (G474 has FPU; disabled for port simplicity) |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | **5** (ST UM1722 example; ISRs that call `FromISR` APIs must be priority **≥ 6**) |

### Boot sequence (`Core/Src/main.c`)

Init order matches the known-good pre-RTOS baseline:

```
HAL_Init → clock → GPIO (+ boot pulses) → FDCAN/SPI/UART/TIM6
→ control_loop_start()          // arms TIM6 + first HB edge
→ MX_USB_Device_Init()          // BEFORE blocking plant init
→ app_init()                    // MCP2518 SPI, mutexes, host_link, …
→ MX_FREERTOS_Init()
→ HAL_SuspendTick()             // stop TIM7 HAL tick during scheduler handoff
→ osKernelStart()               // never returns
```

USB enumeration and the heartbeat LED do **not** depend on the scheduler. Only the work drained from TIM6 ticks and `app_run()` requires task context.

### Task architecture (`Core/Src/app_freertos.c`)

| Task | Priority | Stack (words) | Role |
|------|----------|---------------|------|
| **ControlTask** | `osPriorityAboveNormal` (4) | 512 | Drains `g_control_ticks_pending` via `control_loop_service()` — actuator apply/capture + servo |
| **StartDefaultTask** (host link) | `osPriorityNormal` (2) | 1024 | `HAL_ResumeTick()` then `app_run()` + `osDelay(1)` — USB host link, diag, LEDs |

Pre-RTOS, a single `while(1) { app_run(); }` handled both control-loop service and host link. RTOS splits these so a blocking diagnostic probe on the host path cannot starve the 500 Hz plant loop.

### Control loop — ISR vs task

**Pre-RTOS (`14eb426`):**

- TIM6 ISR (`control_loop_tick`): increment counter, toggle PC3, bump `g_control_ticks_pending`
- Main superloop (`app_run` → `control_loop_service`): drain pending ticks, run actuator/servo

**Post-RTOS (resolved configuration):**

- Same ISR pattern — no FreeRTOS calls inside TIM6
- **ControlTask** replaces superloop drain (poll + `osDelay(1)`)

An intermediate design used `vTaskNotifyGiveFromISR()` from TIM6. That required TIM6 NVIC priority **6+** and introduced kernel calls into an ISR whose only bring-up responsibility is a GPIO toggle. The pending-flag pattern from `14eb426` was restored.

### Plant-layer RTOS changes

| Area | Change |
|------|--------|
| `can_router.c` | Per-FDCAN-bus `xSemaphoreCreateMutex()` (CH1–3 independent) |
| `spi_can_port.c` | One SPI1 mutex for CH4–6 MCP2518 rails (replaces `__disable_irq()` spinlock — deadlock hazard under preemption) |
| `actuator.c` / `servo.c` | `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` for shared desire/state |
| `control_loop.c` | RTOS-agnostic ISR; service runs in task context |

Mutexes are created in `app_init()` **before** `osKernelStart()`. This is valid in FreeRTOS; blocking `Take` with the scheduler not running would be problematic (none occur at init).

### Interrupt wiring (`Core/Src/stm32g4xx_it.c`, `FreeRTOSConfig.h`)

- Empty `SVC_Handler` / `PendSV_Handler` / `SysTick_Handler` stubs removed from `stm32g4xx_it.c` — port maps `vPortSVCHandler` → `SVC_Handler`, `xPortPendSVHandler` → `PendSV_Handler`
- `SysTick_Handler()` calls `xPortSysTickHandler()` only when scheduler ≠ `NOT_STARTED`
- `HAL_TIM_PeriodElapsedCallback` in `main.c` merged: TIM6 → `control_loop_tick()`, TIM7 → `HAL_IncTick()` (Cube regeneration may split these — re-merge required)

### NVIC priorities (bring-up vs RTOS rules)

| IRQ | Pre-RTOS (`14eb426`) | Regressed RTOS build | Resolved bring-up |
|-----|----------------------|----------------------|-------------------|
| TIM6 | 0 | 6 (for `FromISR`) | **0** (ISR has no FreeRTOS calls) |
| USB_LP | 0 | 6 | **0** (must preempt long `app_init` / MCP2518) |
| UART4 | 0 | 6 | 6 (TELEM mode — inert unless `HOST_TRANSPORT_UART`) |
| MCP2518 EXTI | — | 6 | 6 |
| TIM7 (HAL tick) | — | 15 | 15 |
| SysTick / PendSV | 15 | 15 | 15 |

Lowering USB/TIM6 to priority 6 for FreeRTOS syscall compatibility broke bring-up paths that do not involve the scheduler. USB should remain at **0** until MCP2518 init is deferred or made non-blocking.

---

## Debug timeline

### Phase 0 — first RTOS commit (`cfecd2e`)

- FreeRTOS added; single `defaultTask` running `app_run()`; USB init moved into task
- USB and heartbeat lost immediately
- Initial hypothesis: scheduler never started; solid CAN LEDs only indicated `app_init()` completed pre-scheduler

### Phase 1–2 — task split + notify-from-ISR

- **ControlTask** + host link task (`StartDefaultTask`)
- `vTaskNotifyGiveFromISR` from TIM6
- NVIC priorities lowered to 6 for TIM6/USB/UART
- Observed: PC3 solid (GPIO set, no toggle), Code 43 or no COM port

### Phase 3 — decouple USB/heartbeat from scheduler

- `control_loop_start()` + `MX_USB_Device_Init()` moved back to `main()`
- UART4 `DAMIAO_BRIDGE` → `TELEM` to eliminate spurious UART4 IRQ during bench
- PC3 remained solid; USB still failed — ruled out “USB inside a task that never runs” as sole cause

### Phase 4 — init order + diagnostic milestones

- USB init restored **before** `app_init()` (match `14eb426`)
- PC1 milestone LED; `HAL_SuspendTick` / `HAL_ResumeTick`
- Symptoms unchanged: solid heartbeat + Code 43

### Phase 5 — root cause resolution

1. **NVIC:** TIM6 + USB_LP restored to priority **0** (`tim.c`, `usbd_conf.c`)
2. **Control loop:** `14eb426` ISR pattern restored (pending flag + toggle; no FreeRTOS in ISR)
3. **ControlTask:** `control_loop_service()` + `osDelay(1)` instead of task notify

**Outcome:** COM5 enumeration, PC3 toggling, connect chime without Code 43.

---

## Identified issues — ranked by impact

### 1. USB NVIC priority 0 → 6 (Code 43)

| | |
|---|---|
| **Configuration** | USB_LP at priority 6 while `app_init()` blocks for seconds in MCP2518 SPI + `HAL_Delay` |
| **Mechanism** | Host begins enumeration immediately after `MX_USB_Device_Init()`. Descriptor reads overlap `mcp2518_init_all()`. At priority 6, USB LP IRQ does not reliably preempt that work → SETUP/DESCRIPTOR timeout → **Code 43** |
| **Resolution** | `HAL_NVIC_SetPriority(USB_LP_IRQn, 0, 0)` in `USB_Device/Target/usbd_conf.c` |
| **Note** | RTOS syscall priority ceiling applies only to ISRs that call `FromISR` APIs — not to USB or GPIO heartbeat |

### 2. FreeRTOS calls inside TIM6 ISR (solid heartbeat)

| | |
|---|---|
| **Configuration** | `vTaskNotifyGiveFromISR` in `control_loop_tick()`; TIM6 at priority ≥ 6 |
| **Mechanism** | Unnecessary kernel coupling for a GPIO blink; priority constraint conflicts with bring-up observability |
| **Resolution** | Pending flag in ISR; `control_loop_service()` in ControlTask (equivalent to pre-RTOS `app_run` drain) |
| **Note** | ISRs should remain minimal; 500 Hz plant work needs only a counter in the ISR |

### 3. False indicator: “CAN LEDs solid = RTOS works”

| | |
|---|---|
| **Observation** | Solid CAN activity LEDs after boot |
| **Actual meaning** | `can_router_init()` runs in `main()` before `osKernelStart()` — does not confirm task execution |

### 4. USB init order relative to `app_init`

USB must start **before** the long MCP2518 path (match `14eb426`). USB-after-`app_init` delays enumeration until after multi-second blocking, degrading bench results even when NVIC priorities are correct.

### 5. UART4 Damiao bridge IRQ (latent)

`UART4_MODE_DAMIAO_BRIDGE` arms 921600 baud RX interrupt at NVIC 6. A floating RX line can generate interrupt storms and interfere with scheduler bootstrap (SVC/PendSV at 15). Bench builds use `UART4_MODE_TELEM`; restore bridge mode only when the debug UART is physically connected.

### 6. `taskENTER_CRITICAL` masks TIM6 at priority 6

`configMAX_SYSCALL_INTERRUPT_PRIORITY = 5` masks IRQs with NVIC priority **≥ 5**. TIM6 at priority 6 is masked during critical sections. Acceptable when critical sections are short; additional reason to keep TIM6 at priority 0 for bring-up signals.

### 7. CubeMX regeneration hazards

- Duplicate `HAL_TIM_PeriodElapsedCallback` — merge TIM6 + TIM7 into USER CODE 0
- `.ioc` may reset TIM6/USB NVIC to 6 — verify `tim.c` / `usbd_conf.c` after regeneration
- Do not re-add empty `SVC_Handler` / `PendSV_Handler` in `stm32g4xx_it.c`

---

## RTOS verification procedure

### Level 0 — Pre-scheduler (power-on, no host tools)

After flash, wait **2–3 seconds**:

| Check | Pass criterion |
|-------|----------------|
| PC2 ×3 boot flashes | `main()` early path |
| PC3 ~1 Hz blink | TIM6 ISR (independent of scheduler) |
| PC2 solid after USB cable | Host configured CDC (`CDC_Init_FS`) |
| PC1 LOW | `app_init()` returned |
| CAN CH1–6 LEDs solid | Router init complete |

PC3 blink plus PC2 solid indicates **pre-RTOS parity** before task-level checks.

### Level 1 — USB host link (confirms `StartDefaultTask` + scheduler)

**Important:** COM port enumeration only proves `CDC_Init_FS` ran. `link-test` requires
`StartDefaultTask` → `app_run()` → `host_link_poll_tx()`. If link-test times out with
no 562 B feedback, the scheduler is not running tasks (common cause: broken SysTick wiring
— `xPortSysTickHandler` must map to `SysTick_Handler` in `FreeRTOSConfig.h` with no
duplicate handler in `stm32g4xx_it.c`).

```powershell
pip install pyserial
python scripts/controls_pcb_host.py ports
python scripts/controls_pcb_host.py --port COM5 link-test
python scripts/controls_pcb_host.py --port COM5 status
```

| Command | Pass criterion |
|---------|----------------|
| `link-test` | Exits 0; firmware returns 562 B feedback with valid magic |
| `status` | Prints `mcu_state`, actuator slot summary |

A passing `link-test` confirms **`StartDefaultTask` is running** under the scheduler (`app_run` → `host_link_poll_*`).

### Level 2 — Control loop under RTOS (confirms `ControlTask`)

Indirect checks without JTAG:

1. `link-test` and `status` remain reliable during CH1/CH3 motor activity — host and control tasks coexist.
2. RobStride probe/teleop on CH1 — sustained 500 Hz command path requires `host_link_command_is_fresh()` and ControlTask draining ticks:

   ```powershell
   python scripts/controls_pcb_host.py --port COM5 probe --slot 0 --bus 1 --id 0x76
   ```

3. Scope / LED: PB15 (CH3) or PC7 (CH1) activity blink under host-driven CAN traffic while PC3 heartbeat continues.

### Level 3 — Debugger (optional)

Breakpoints in CubeIDE:

1. First line of `ControlTask` — should hit after `osKernelStart`
2. First line of `StartDefaultTask` — should hit after `osKernelStart`
3. `control_loop_service()` — regular hits at ~1 kHz poll rate (`osDelay(1)` batches tick bursts)

If (2) hits but (3) does not, inspect ControlTask creation or priority assignment.

### Level 4 — `configASSERT` visibility

`FreeRTOSConfig.h` maps `configASSERT` → `Error_Handler()` (PC2+PC3 slow blink). A silent `for(;;)` with IRQs disabled obscures assert trips during bring-up.

---

## Host tool: `controls_pcb_host` — Damiao enable on CH3

Package: `scripts/controls_pcb_host/` (CLI: `scripts/controls_pcb_host.py`).

Firmware: slot **2** = Damiao, `CAN_BUS_CH3`, `PROTO_DAMIAO` (`plant_config.c`). CH3 = **`hfdcan2`** standard CAN @ 1 Mbps — see [damiao-bringup.md](damiao-bringup.md) for termination and register map.

### Prerequisites

```powershell
pip install pyserial
# Motor 24 V on harness; USB to laptop; firmware built with HOST_TRANSPORT_USB
python scripts/controls_pcb_host.py --port COM5 link-test
```

### Discover motor ESC_ID (if unknown)

```powershell
python scripts/controls_pcb_host.py --port COM5 discover --protocol damiao --bus 3 --start 1 --end 16 --listen-ms 40
```

Uses DM0 PDU reg-scan (`plant_diag` session on CH3). Update `plant_config.c` slot 2 `motor_id` with the discovered ESC_ID.

### Probe one ID (register scan, no enable)

```powershell
python scripts/controls_pcb_host.py --port COM5 probe --slot 2 --bus 3 --id 0x01 --listen-ms 15
```

Explicit bus/id:

```powershell
python scripts/controls_pcb_host.py --port COM5 probe --bus 3 --id 0x06 --protocol damiao
```

### Enable Damiao (clear fault + enable + optional MIT hold)

```powershell
python scripts/controls_pcb_host.py --port COM5 probe --slot 2 --enable --listen-ms 15 --hold-ms 3000
```

Host sequence (`plugins/damiao.py`):

1. `dm_session_begin` — firmware `mcu_state = DIAG_ONLY`, CH3 dedicated to DM0 probes
2. `DM_PROBE_CLEAR_FAULT` then `DM_PROBE_ENABLE`
3. Expect **ERR nibble = 1** in probe PDU or actuator slot-2 mirror (`ERR=0x1` = enabled)
4. With `--hold-ms > 0`: periodic `DM_PROBE_MIT` bursts to prevent Damiao comm timeout before teleop
5. `dm_session_end` — return toward normal plant mode

**Pass criteria:** console prints `OK: motor reports enabled (ERR=0x1).` and/or `format_hit` with plausible `pos` / `err`.

**Electrical:** `tx>0` with `rx_raw=0` indicates a physical-layer issue (typically **120 Ω termination at motor end**), not RTOS — see [damiao-bringup.md](damiao-bringup.md).

### Return to normal plant mode after bench

```powershell
python scripts/controls_pcb_host.py --port COM5 recover --bus 3
```

### Legacy script (equivalent protocol)

```powershell
python scripts/damiao_scan.py --port COM5 --link-test
python scripts/damiao_scan.py --port COM5 --probe-id 0x01 --bus 3 --enable --hold-ms 3000
```

`controls_pcb_host` is preferred for new work; `damiao_scan.py` retains expert-only flags.

---

## Definition of done — FreeRTOS path

```
[ ] PC3 heartbeat ~1 Hz with USB cable connected
[ ] Windows COM port (no Code 43) — CDC stable after 3 s from power-on
[ ] controls_pcb_host link-test → exit 0
[ ] controls_pcb_host status → valid feedback header
[ ] CH1 RobStride probe still works (slot 0)
[ ] Damiao discover OR probe on CH3 (RX depends on termination, not RTOS)
[ ] Damiao --enable → ERR=0x1 (when motor + bus are verified)
[ ] CubeMX regen checklist: TIM6+TIM7 callback merged, USB/TIM6 NVIC 0, no duplicate SVC/PendSV
```

---

## Key firmware files (CubeMX regeneration)

| File | RTOS-related content |
|------|----------------------|
| `Core/Src/main.c` | Init order, merged TIM callback, `HAL_SuspendTick` |
| `Core/Src/app_freertos.c` | Tasks, stacks, priorities |
| `Core/Inc/FreeRTOSConfig.h` | Heap, assert, handler macros |
| `Core/Src/stm32g4xx_it.c` | `SysTick_Handler`, EXTI, no SVC/PendSV stubs |
| `Core/Src/stm32g4xx_hal_timebase_tim.c` | TIM7 HAL tick |
| `Core/Src/tim.c` | TIM6 NVIC priority |
| `USB_Device/Target/usbd_conf.c` | USB_LP NVIC priority |
| `App/Src/plant/control_loop.c` | ISR pending flag + heartbeat; no FreeRTOS in ISR |
| `App/Src/plant/can/can_router.c` | FDCAN mutexes |
| `App/Src/plant/can/spi_can_port.c` | SPI1 mutex |
| `App/Src/plant/actuator.c`, `servo.c` | Critical sections |
| `DeftRoboticsControlsPCB.ioc` | FREERTOS CMSIS_V1, TIM7 timebase |

---

## Summary

FreeRTOS splits the former `app_run()` superloop into a **high-priority ControlTask** (500 Hz plant work via TIM6 pending flag) and a **normal host-link task** (USB CDC, diag, LEDs). The kernel uses **SysTick**; HAL uses **TIM7**. The observed regressions were not intrinsic to RTOS adoption. Primary causes: (1) **USB and TIM6 NVIC priority reduced from 0 to 6**, allowing MCP2518 blocking in `app_init` to starve USB enumeration (Code 43), and (2) **FreeRTOS notifications wired into the TIM6 ISR** despite a GPIO toggle and byte counter sufficing for the pre-RTOS design. Restoring priority 0 for USB/TIM6, initializing USB before `app_init`, and draining ticks in ControlTask restored COM enumeration and heartbeat. Scheduler health is verified with `controls_pcb_host link-test` / `status`; Damiao bench follows with `probe --slot 2 --enable`.

---

## See also

- [bringup.md](bringup.md) — transport selection, motor map, legacy `damiao_scan.py` commands
- [damiao-bringup.md](damiao-bringup.md) — CH3 termination, register map, symptom tree
- [architecture.md](architecture.md) — module map
- `RTOS_BRINGUP_HANDOFF.txt` — detailed round-by-round debug log
