# FreeRTOS on this board — a working reference

Consolidates 8 web tutorials (`External_Documentation/STM32/freertos_1.pdf`
through `freertos_8.pdf`, controllerstech.com's CMSIS-RTOS series) plus ST's
official `External_Documentation/STM32/STM32 RTOS Documentation.pdf` (UM1722,
"Developing applications on STM32Cube with RTOS") into one document, with each
part mapped onto **this repo's actual, currently-live** FreeRTOS setup —
`USE_FREERTOS_SCHEDULER` is `1` today, not a future plan. Written to be fed
back as context, not read as a tutorial in its own right — every section
assumes you already know embedded C and just needs the RTOS-specific mental
model plus exactly where each concept shows up in this codebase.

---

## 0. The one thing to internalize before anything else: v1 vs v2

The 8-part tutorial series (Parts 1–8) teaches **CMSIS-RTOS v2** — `osThreadNew()`,
single-call attribute-struct creation. **This repo uses CMSIS-RTOS v1** —
`osThreadDef()` + `osThreadCreate()`, the two-step macro style — confirmed by
reading `Core/Src/app_freertos.c` directly, and it matches UM1722 (the ST
manual) exactly, not the tutorial series. Every code example below is
translated; don't copy v2 syntax into this project.

| Concept | Tutorial series (v2) | **This repo (v1, actual)** |
|---|---|---|
| Create a thread | `osThreadNew(func, arg, &attr)` | `osThreadDef(name, func, prio, 0, stack_words); h = osThreadCreate(osThread(name), arg);` |
| Create a semaphore | `osSemaphoreNew(max, init, &attr)` | `osSemaphoreDef(SEM); h = osSemaphoreCreate(osSemaphore(SEM), max);` |
| Create a mutex | `osMutexNew(&attr)` | `osMutexDef(MTX); h = osMutexCreate(osMutex(MTX));` |
| Create a queue | `osMessageQueueNew(n, size, &attr)` | `osMessageQDef(Q, n, type); h = osMessageCreate(osMessageQ(Q), NULL);` |
| Create a timer | `osTimerNew(cb, type, arg, &attr)` | `osTimerDef(T, cb); h = osTimerCreate(osTimer(T), type, arg);` |
| Acquire semaphore | `osSemaphoreAcquire(h, timeout)` | `osSemaphoreWait(h, timeout)` |
| Acquire mutex | `osMutexAcquire(h, timeout)` | `osMutexWait(h, timeout)` |
| Delay | `osDelay(ms)` | `osDelay(ms)` — unchanged |

**This repo uses CMSIS-RTOS v2** (`osThreadNew` / `cmsis_os.h` wrapping
`cmsis_os2.h`). CubeMX `.ioc` is CMSIS_V2; `configTOTAL_HEAP_SIZE=48KB`.
Host and Plant share `osPriorityAboveNormal` so time-slicing keeps USB ack
alive when Plant is continuously READY; Peripheral matches that band too
(DXL/LED starved at `BelowNormal` under ×25 load — polled UART uses
`dxl_port_bus_lock()` / `vTaskSuspendAll` so Plant cannot preempt mid-packet).
Plant TIM6 wake still uses native `ulTaskNotifyTake` / `vTaskNotifyGiveFromISR`
(no CMSIS wrapper).

---

## 1. This repo's live architecture (read this before the per-part sections)

Three tasks, created in `MX_FREERTOS_Init()` (`Core/Src/app_freertos.c`):

| Task | Priority | Wake mechanism | Body | Role |
|---|---|---|---|---|
| `StartDefaultTask` (Host) | `osPriorityAboveNormal` | `osDelay(1)` loop | `app_host_service()` | USB init + host_link RX, diag, PDB |
| `PlantTask` | `osPriorityAboveNormal` | `ulTaskNotifyTake` from TIM6 ISR, **no delay** | `app_plant_service()` | Actuator apply/capture, host FB TX — the **autonomy loop** |
| `PeripheralTask` | `osPriorityAboveNormal` | `osDelay(1)` loop | `app_peripheral_service()` | DXL bus poll, LED/thermo SPI3, CAN router diag poll |

This is the "autonomy loop vs peripheral loop" split discussed for this
project — implemented as **FreeRTOS task priorities**, not a second hardware
timer. `PlantTask` has no `osDelay` and is woken directly by TIM6 via task
notification specifically so it stays in the Ready state and can **preempt**
`PeripheralTask` mid-DXL-transaction the instant a tick arrives (see Part 2 —
this is the preemption rule in direct use: a higher-priority task interrupts a
lower one, never the reverse). `app_run()` still exists and calls all three
service functions in sequence, gated `#if !USE_FREERTOS_SCHEDULER` — the
bare-metal superloop fallback, same functions either way.

Config (`Core/Inc/FreeRTOSConfig.h`), each value cross-referenced below:
`configTICK_RATE_HZ=1000`, `configMAX_PRIORITIES=7`, `configTOTAL_HEAP_SIZE=24KB`
(dynamic-only, `configSUPPORT_STATIC_ALLOCATION=0`), `configCHECK_FOR_STACK_OVERFLOW=2`,
`configUSE_MUTEXES=1`, `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`. HAL
timebase is **TIM7** (SysTick owned entirely by the FreeRTOS port) — a
`Core/Src/stm32g4xx_hal_timebase_tim.c` override, not the CubeMX default.

**Prior bring-up history exists and is worth reading before touching any of
this** — `RTOS_BRINGUP_HANDOFF.txt` (repo root) documents a real, painful
first attempt: NVIC priority conflicts, a floating UART4 RX line firing
interrupts at priority 6 and starving the scheduler bootstrap, `MX_USB_Device_Init()`
blocked behind a slow `mcp2518_init_all()`, and a silent `configASSERT` hang
with zero LED indication. The current architecture (task split, TIM6 notify
guarded on scheduler state, deferred mutex creation) is the result of chasing
those down — every "why is it done this way" question below likely has an
answer in that file.

---

## 2. Part 1 — Setup, task states, CMSIS-RTOS concept

**Concept.** The superloop+`HAL_Delay()` problem: one blocking call stalls
everything. FreeRTOS gives each job its own task (own stack, priority,
execution point); the scheduler time-slices a single core so it *feels*
concurrent. Four task states: **Running** (owns the CPU — only one at a time
here, single Cortex-M4 core), **Ready** (could run, waiting for the scheduler
to pick it), **Blocked** (waiting on a delay/semaphore/queue/notification —
costs zero CPU, the whole point), **Terminated** (rare in practice; tasks loop
forever). CMSIS-RTOS itself is a *portability shim* over whichever kernel is
underneath (FreeRTOS here) — it doesn't schedule anything itself.

**Applies here:** `PlantTask`'s idle state *is* Blocked, sitting in
`ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` — zero CPU burned between ticks,
exactly the state-model benefit over the old bare-metal `while(1)` polling
`g_control_ticks_pending`. `PeripheralTask`/`StartDefaultTask`'s `osDelay(1)`
loops put them in Blocked for that 1 tick (1ms) each iteration — during which
`PlantTask` or a higher-priority ISR can run freely with zero contention.

**Also applies:** the SysTick/HAL-timebase conflict this tutorial and UM1722
both flag is already resolved here — HAL timebase is TIM7
(`stm32g4xx_hal_timebase_tim.c`), SysTick belongs entirely to the FreeRTOS
port, exactly per the tutorial's "move to TIM6/TIM7" instruction and UM1722's
implicit position (SysTick is the scheduler's, don't assume HAL can share it).

---

## 3. Part 2 — Multiple tasks, priorities, preemption

**Concept.** The Preemption Rule, stated plainly by the tutorial: *a higher
priority task can preempt a lower one at any point; a lower one can never do
the same in return.* Demonstrated two ways: a blocking loop in the
lowest-priority task barely affects the others (they preempt it on schedule);
the same loop in the *highest*-priority task freezes everything below it
completely, since nothing can preempt it back. Suspend (`osThreadSuspend`) vs
Terminate (`osThreadTerminate`): suspend preserves the task's stack/state for
later resume; terminate deletes it permanently (would need `osThreadCreate`
again from scratch) — suspend is almost always the right tool.

**Applies here — directly, this is the core design rationale for the split
in §1:** `PlantTask` at `osPriorityAboveNormal` above `PeripheralTask` at
`osPriorityBelowNormal` is precisely "put the blocking-risk-prone work at
lower priority than the time-critical work" — if `PeripheralTask` is mid-DXL
transaction (bounded, but non-zero time) when a TIM6 tick fires, `PlantTask`
preempts it immediately per the Preemption Rule; `PeripheralTask` resumes once
`PlantTask` blocks again on the next `ulTaskNotifyTake`. This is exactly why
the architecture works without needing every single peripheral driver to
individually prove itself non-blocking-relative-to-the-tick — the *priority
difference itself* is the isolation mechanism, on top of (not instead of) the
individual drivers already being reasonably bounded (DXL's ~800µs async poll,
LED's blocking SPI transmit still pending a fix — see the SK9822 plan
elsewhere).

**Caution worth carrying forward:** the tutorial's "highest-priority blocking
task freezes everything" failure mode is a live risk for `PlantTask` — if
`app_plant_service()` ever grows a genuinely long blocking call, `PeripheralTask`
*and* `StartDefaultTask` starve completely, silently, with no watchdog telling
you why. Nothing in the current code does this deliberately, but it's the
one architectural invariant to protect above all others.

Suspend/Terminate: not currently used anywhere in this codebase's task
management — all three tasks are permanent, running forever. Worth knowing the
tools exist if a future bring-up/self-test task needs to run once and free its
stack (Part 2's "one-shot init task terminates itself" pattern).

---

## 4. Part 3 — Queues, race conditions

**Concept.** Global variables shared across tasks race — the scheduler can
switch mid-write, and `volatile` does **not** fix this (it only stops the
compiler from caching a value in a register; it says nothing about atomicity
across a preemption). A queue gives each message its own slot, FIFO-ordered
(with an optional priority parameter that can jump a message ahead — unused,
priority 0, in the tutorial's own examples). Timeout choices: `0` (fail
immediately if full/empty), N ms, or `osWaitForever`.

**Applies here:** this repo doesn't currently use FreeRTOS queues anywhere —
communication between `PlantTask`/`PeripheralTask`/`StartDefaultTask` is via
plain shared state (`actuator_desire_live[]`, `g_plant_pending_image`, etc.)
protected by **critical sections** (`plant_crit_enter/exit`, §6) rather than
queues. This is a deliberate, reasonable choice for this shape of problem —
these are "latest value wins" desires (a new actuator command supersedes the
old one, it doesn't need to be queued and processed in order) rather than
discrete, must-not-drop events. If a future need arises for genuinely ordered,
must-not-drop messages between tasks (e.g. a fault log that every entry must
reach), a queue — not a shared struct with a critical section — would be the
right tool per this tutorial's own race-condition argument.

---

## 5. Part 4 — Semaphores, priority inversion

**Concept.** Binary semaphore = 1-token lock, no ownership concept — any task
can release it, not just the one that took it. Counting semaphore = N tokens
for N interchangeable resource instances. Neither protects against **priority
inversion**: the tutorial's demo has a low-priority task (LPT) hold the
semaphore, a medium-priority task (MPT) that doesn't even touch the semaphore
preempt LPT anyway (since MPT > LPT), and a high-priority task (HPT) block on
the semaphore indefinitely — HPT, the highest-priority task in the system, is
effectively held hostage by MPT, which has nothing to do with the resource at
all. This is exactly why Part 5 exists.

**Applies here:** CMSIS/FreeRTOS semaphores aren't used directly in this
codebase's plant code — mutexes are used instead (`can_router.c`,
`spi_can_port.c`, see Part 5). The one native-FreeRTOS mechanism playing the
"semaphore" role is `PlantTask`'s task notification (§0) — a lighter-weight
binary-semaphore-equivalent for the single TIM6-ISR-to-PlantTask link, chosen
over a real semaphore because there's only ever one giver (the ISR) and one
taker (PlantTask), so the extra kernel object a semaphore would need is pure
overhead here.

---

## 6. Part 5 — Mutex, priority inheritance, recursive mutex

**Concept.** A mutex is a semaphore with **ownership** — only the acquiring
task can release it — which is what enables **priority inheritance**: the
instant a higher-priority task blocks on a mutex held by a lower-priority
task, FreeRTOS temporarily boosts the holder's priority to match, so a
medium-priority task can no longer preempt it out from under the high-priority
waiter. Priority drops back the instant the mutex is released. Recursive
mutex: same object, but the *same* task can re-acquire it (nested calls) with
an internal count, avoiding a self-deadlock a plain mutex would cause.
**Mutexes must never be used from ISR context at all** (Part 6 states this
explicitly and definitively — no `FromISR` variant exists for a reason: a
mutex can block, and blocking is illegal in an ISR).

**Applies here — this is directly live code, not a hypothetical:**
`App/Src/plant/can/can_router.c` gives each FDCAN bus (CH1–3) its own
`SemaphoreHandle_t bus_mutex[bus]` so `PlantTask` and `StartDefaultTask` (or
future concurrent callers) can touch `tx_queues[]`/`rx_rings[]` safely —
exactly the "protect a shared resource between tasks of different priorities"
case this tutorial pair is about. `App/Src/plant/can/spi_can_port.c` does the
same for the shared SPI1 bus (CH4–6 MCP2518FD rails), replacing what used to
be a raw `__disable_irq()` spinlock — a real deadlock hazard under preemption,
per that file's own comment.

**A genuine, already-hit gotcha, not from the tutorials — worth its own
callout:** both files **defer mutex creation** until the first lock attempt
*after* `osKernelStart()`, rather than creating the mutex in `_init()`. The
comment in `spi_can_port.c` explains why: *"creating a mutex before the
scheduler runs uses `portENTER_CRITICAL` while `uxCriticalNesting` is still
`0xaaaaaaaa`, which leaves BASEPRI raised and deadlocks `HAL_Delay` (TIM7) in
`mcp2518_init`."* This is the exact same class of hazard `plant_crit.h`
guards against (next paragraph) — creating or entering FreeRTOS synchronization
primitives before `osKernelStart()` has actually run is not safe by default,
even though the API doesn't warn you at compile time. Neither the tutorial
series nor UM1722 mentions this specific pre-scheduler mutex-creation trap —
it's a real bug this project found the hard way, worth remembering before
adding any new mutex/semaphore that might get created during `app_init()`.

**`plant_crit.h`** is the same lesson applied to critical sections directly:
```c
static inline void plant_crit_enter(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        __disable_irq();
    else
        taskENTER_CRITICAL();
}
```
`taskENTER_CRITICAL()` before `osKernelStart()` leaves BASEPRI raised forever
(same root cause). The fix: use raw `__disable_irq()`/`__enable_irq()` until
the scheduler is confirmed running, then switch to the real FreeRTOS primitive.
**Any new pre-scheduler-safe synchronization code in this project should
follow this same "check `xTaskGetSchedulerState()` first" pattern** — it's
the established, working convention here, not a one-off.

---

## 7. Part 6 — Event flags

**Concept.** An event group is a 32-bit value, each bit an independent
condition; a task can wait for *any* one (`osFlagsWaitAny`) or *all*
(`osFlagsWaitAll`) of a set of bits, through one kernel object. Only 24 bits
(0–23) are usable — FreeRTOS reserves the top 8. **Explicitly, definitively
safe to call `osEventFlagsSet`/`xEventGroupSetBitsFromISR` from an ISR** —
the tutorial calls this out as event flags' key architectural advantage over
queues (which need careful `FromISR` timeout handling) and mutexes (never
ISR-safe, full stop). The real-hardware example sets flags directly from a
GPIO EXTI callback and a `HAL_UART_RxCpltCallback`, dispatching to two
independent tasks off the same event group with zero cross-talk.

**Applies here:** event groups aren't currently used in this codebase — TIM6's
ISR-to-task handoff uses a task notification instead (§0/§5), and there's
only ever one waiter per ISR source today (PlantTask on TIM6; the MCP2518FD
EXTI lines just set a flag byte checked later, not an RTOS primitive at all —
see `mcp2518_isr_rx_pending()`, confirmed in `RTOS_BRINGUP_HANDOFF.txt` as
"only sets a flag, no blocking/RTOS calls from real ISR context," which is
the correct call regardless of which primitive is used). **Where this would
become directly relevant:** if a future ISR (e.g. the PDB link's UART RX/TX
complete callbacks, or a future hard-ESTOP sense interrupt) needs to wake
*more than one* task, or a task needs to wait on *either of two* independent
interrupt sources at once, an event group is the correct tool — cleaner than
adding a second task notification (a task only has one notification value in
this FreeRTOS version) or a semaphore per source.

---

## 8. Part 7 — Software timers

**Concept.** Software timer callbacks run in a dedicated **timer daemon
task**, not in the caller's task or an ISR — so callbacks must stay short and
non-blocking (a slow callback delays every other timer in the system, since
they share one daemon task and one internal queue). Periodic vs one-shot.
Calling `osTimerStart()` on an already-running timer **resets** it rather than
creating a second one — used deliberately in the tutorial for a
debounce/inactivity pattern: keep pushing a one-shot's deadline forward on
every new event; it only fires once activity actually stops.

**Applies here:** no FreeRTOS software timers are used in this codebase today
— all periodic behavior (LED chase-mode 50ms refresh, DXL's 5ms rate limit,
PDB's 20ms TX period, servo watchdog timeouts) is hand-rolled via
`HAL_GetTick()` comparisons inside the relevant service function, called from
`PeripheralTask`'s 1ms loop. This works fine at the current scale but is
worth knowing as an alternative: a software timer could replace some of these
ad-hoc `HAL_GetTick()` throttles with a formal, independently-scheduled
callback — though note the daemon-task caveat above means it wouldn't
automatically solve anything about *priority* (the daemon task has its own
fixed priority, separate from `PlantTask`/`PeripheralTask`) — this is a
tidiness option, not a performance one, for this codebase's current shape.

---

## 9. Part 8 — Stack management

**Concept.** Every task's stack is a fixed allocation; overflow silently
corrupts adjacent memory rather than crashing immediately, making it a nasty
delayed-symptom bug class. High water mark = the historical *minimum* free
stack ever observed for a task — monotonically non-increasing, a direct
measurement of "how close has this task actually come to overflowing."
Practical sizing rule the tutorial lands on: run worst-case load, observe the
high water mark, then permanently allocate that minimum plus **20–30%
headroom** — never ship a task that hit 0 free bytes in testing.
`configCHECK_FOR_STACK_OVERFLOW`: method 1 checks the stack pointer only at
context switches (fast, can miss overflows between switches); **method 2**
(pattern-fill + continuous check) catches more, at some cost — the tutorial
recommends method 2 for most projects.

**Applies here — already following this exact recommendation:**
`FreeRTOSConfig.h` has `configCHECK_FOR_STACK_OVERFLOW 2`, with a comment
explicitly citing the same reasoning as the tutorial ("catches overflow at the
point of the offending context switch/task-check instead of leaving it as
silent corruption"). `vApplicationStackOverflowHook()` (`app_freertos.c`)
currently just calls `Error_Handler()` — matching the tutorial's own guidance
that the hook should be a terminal diagnostic stop, not attempt recovery, though
without the tutorial's UART-message-identifying-the-task step; **worth
considering **adding that** (transmit `pcTaskName` before halting) given how
much this project's own bring-up history (`RTOS_BRINGUP_HANDOFF.txt`) has
struggled with silent, undiagnosable hangs — a stack overflow that just calls
`Error_Handler()` with no task-name output is exactly the kind of "which
component died" ambiguity that handoff document is full of.**

All three tasks are currently sized at a flat **1024 words** (`app_freertos.c`)
— no high-water-mark measurement has been done yet per this doc's research
(no `osThreadGetStackSpace`/`uxTaskGetStackHighWaterMark` calls exist anywhere
in this codebase currently). Given `PlantTask` calls into the actuator/CAN
stack (deep call chains through `robstride.c`/`mcp2518fd.c`) and
`PeripheralTask` calls into DXL/LED/SPI drivers, measuring actual high water
marks under a real bench load (all buses commanded, LED chase active, DXL
armed) before trusting the flat 1024-word guess would be the direct
application of this section's practical rule — a natural companion task
alongside the SK9822 fix and the PDB bring-up, not a separate project.

---

## 10. UM1722 (ST's official manual) — the parts specific to this exact chip/HAL

This is the authoritative document (v1 API, matches this repo) underlying
several of `FreeRTOSConfig.h`'s exact numeric choices — cross-referenced
directly, not paraphrased:

- **`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`** — this repo's value
  is UM1722's own literal reference example (Figure 4). Not a guess.
- **SVC_Handler / PendSV_Handler must not be redefined** in `stm32g4xx_it.c` —
  UM1722 states this explicitly ("must be removed... to avoid a duplicate
  definition"); confirmed absent in this repo's `stm32g4xx_it.c` per
  `RTOS_BRINGUP_HANDOFF.txt`'s own audit.
- **`xPortSysTickHandler` must NOT also be defined as `SysTick_Handler`
  elsewhere** if HAL is generating its own — this repo's resolution (TIM7 HAL
  timebase, SysTick owned by the FreeRTOS port exclusively) is precisely
  UM1722's documented safe configuration, and `FreeRTOSConfig.h`'s own comment
  cites UM1722 directly for this.
- **ISR → task handoff via `osSemaphoreRelease` (Section 3.2.2)** is UM1722's
  sanctioned pattern for interrupt-to-thread signaling. This repo doesn't use
  a semaphore for its one ISR→task link (TIM6→PlantTask) — it uses a task
  notification instead (§0), a lighter native-FreeRTOS mechanism UM1722
  doesn't cover at all (it predates or falls outside the CMSIS wrapper). The
  underlying rule UM1722 is protecting — *ISRs hand off to tasks via a
  non-blocking, ISR-safe primitive; they never do the work themselves* — is
  still fully honored; the specific primitive chosen is just better-suited to
  this repo's exact ISR→task topology (always exactly one receiver).
- **Heap scheme**: `configSUPPORT_STATIC_ALLOCATION=0` / dynamic-only implies
  one of `heap_1`–`heap_4`; given tasks are created once at boot and never
  deleted in this codebase (no `vTaskDelete` calls anywhere), even the
  simplest `heap_1` (never frees) would technically suffice — but confirm
  which `heapN.c` is actually linked before relying on that assumption if
  anything starts creating/destroying RTOS objects at runtime later (e.g. a
  future one-shot self-test task per Part 2's suggestion).
- **Priority-inheritance mutex demo (Section 3.4)** is UM1722's own version of
  the exact scenario `can_router.c`/`spi_can_port.c`'s mutexes are protecting
  against in this codebase — same mechanism, same reasoning, just the
  official-doc citation for it.

---

## 11. Cross-cutting gotcha checklist (pulled together from all 9 documents + this repo's own bug history)

- **Never call a blocking API from an ISR** — no `osMutexAcquire`/`osMutexWait`
  ever from ISR context (Part 5/6, UM1722). This repo's ISRs (TIM6, MCP2518FD
  EXTI, UART RX/TX completes) all just set flags/counters or hand off via
  task notification — confirmed clean, keep it that way.
- **Any ISR calling an RTOS-safe API must be configured at NVIC priority ≥
  `configMAX_SYSCALL_INTERRUPT_PRIORITY`** (numerically — higher number = lower
  logical priority on Cortex-M) — UM1722's central rule, and the exact bug
  class `RTOS_BRINGUP_HANDOFF.txt` hit with UART4 at priority 6 vs SVCall/PendSV
  at 15. Check this explicitly for any new ISR touching FreeRTOS state (the
  new PDB UART4 RX/TX-complete callbacks included).
- **`SPI3_IRQHandler` must call `HAL_SPI_IRQHandler(&hspi3)`** — Cube often
  enables SPI3 NVIC in `HAL_SPI_MspInit` without generating the vector body.
  Missing handler → `Default_Handler` infinite loop on the first
  `HAL_SPI_Transmit_IT` from `led_service()`, which kills Host/Plant (0 FB).
  Handler lives in `stm32g4xx_it.c` USER CODE; `.ioc` has `NVIC.SPI3_IRQn` at
  priority 6.
- **Don't create or lock a FreeRTOS synchronization object before
  `osKernelStart()` has actually run** — the `spi_can_port.c` mutex bug (§6).
  Check `xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED` first, same
  pattern as `plant_crit.h`.
- **The highest-priority task must never contain an unbounded blocking call**
  (Part 2) — here, that's `PlantTask`/`app_plant_service()`. This is the
  single architectural invariant most worth protecting as this codebase grows.
- **`volatile` is not synchronization** (Part 3) — for anything shared across
  the three tasks, use a critical section (`plant_crit_enter/exit`), a mutex,
  or a queue; never rely on `volatile` alone.
- **Stack sizes are currently unmeasured guesses** (§9) — worth doing the
  high-water-mark pass before this matters in the field, not after a silent
  overflow.
- **HAL callbacks that self-disarm must be re-armed inside the callback**
  (Part 6's `HAL_UART_Receive_IT` example) — worth double-checking on any HAL
  IT-mode callback added to this project (the PDB link and SK9822 fix both
  add new ones).

---

## Related

- `RTOS_BRINGUP_HANDOFF.txt` (repo root) — the real bring-up history behind
  most of the choices documented above.
- `Core/Src/app_freertos.c`, `Core/Inc/FreeRTOSConfig.h`, `App/Inc/plant/plant_crit.h`,
  `App/Src/plant/can/can_router.c`, `App/Src/plant/can/spi_can_port.c` — the
  live source this guide is grounded in.
- `External_Documentation/STM32/freertos_1.pdf` – `freertos_8.pdf` — the full
  tutorial series (CMSIS-RTOS v2; translate via §0 before using as copy-paste
  reference for this repo).
- `External_Documentation/STM32/STM32 RTOS Documentation.pdf` — UM1722, the
  ST manual, CMSIS-RTOS v1 (matches this repo's actual API).
