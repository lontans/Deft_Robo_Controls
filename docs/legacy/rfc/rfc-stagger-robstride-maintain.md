# RFC: stagger `robstride_maintain_enable` re-arm phase across slots

Status: patch drafted and verified (`git apply --check`), ready to apply.
Author: Agent 2 (Claude), offline — no build/flash performed by this agent.
Executor: Cursor (owns `App/`/`Core/`, COM5, soft-DFU).
Patch: [`docs/patches/stagger-robstride-maintain.patch`](patches/stagger-robstride-maintain.patch).
Follow-on to: [rfc-release-build.md](rfc-release-build.md),
[rfc-per-bus-rx-index.md](rfc-per-bus-rx-index.md) — both already applied and
matrixed (see [legacy/bench-load-matrix-release-2026-07-23.md](../bench-load-matrix-release-2026-07-23.md),
[legacy/bench-load-matrix-release-rxindex-2026-07-23.md](../bench-load-matrix-release-rxindex-2026-07-23.md)).

**500 Hz narrative (short):** Release/RX-index cut mean lap time; this stagger
targets the rate-independent **peak**. The remaining 500 Hz `cmd_seq_lag_p95`
fail is the intentional coalesce-to-latest in `host_link_poll_rx()` (see
"Related" below and the cross-link in
[rfc-release-build.md](rfc-release-build.md)) — measure stagger alone before
touching host-link scheduling.

## Problem: act_lap peak is stuck at ~9-18 ms regardless of tx Hz

Line up every §B (×25 rx_sim + DXL + LED) bench doc so far:

| Build | 40 Hz peak | 100 Hz peak | 200 Hz peak | 500 Hz peak |
|---|---:|---:|---:|---:|
| Debug `-O0` (07-23) | 18 | 18 | 18 | 18 |
| Release `-Os` | 10 | 10 | 11 | 11 |
| Release + RX-index | ~10 | ~10 | ~10 | ~11 |

The **mean** dropped ~2× from Release and a further ~5-10% from the RX
index (both real, expected wins). The **peak** did not move nearly as much,
and — this is the tell — **it does not scale with tx Hz at all**. If a peak
were caused by "occasionally two things line up in the same tick" driven by
*host* timing, you'd expect it to shift with tx Hz (more command opportunities
per second at 500 Hz than 40 Hz). A peak that is flat across a 12.5×
range of command rates is a signature of a **fixed-period architectural
event**, not a host-driven one — something in the *firmware's own* timebase
fires on a wall-clock schedule independent of how often the host talks to it.

## Root cause: unstaggered 2-second keepalive re-arm

[`App/Src/plant/plugins/robstride.c`](../App/Src/plant/plugins/robstride.c)
`robstride_maintain_enable()` (~L608) re-sends `run_mode` + `enable` (2 CAN
frames) for a slot once every `2000 ms`, tracked via a per-slot
`last_maintain_ms[slot]` static array populated by `robstride_apply_cycle`
(~L826). The **first** arm always fires immediately when a slot transitions
from idle to commanded (`last_ms == 0` short-circuits the 2 s gate). In every
bench run so far, all commanded slots receive their *first* non-idle desire
from the **same host command frame**, in the **same plant tick** — so they
all first-arm together, and since `*last_ms = now` unconditionally on every
arm, **every subsequent 2-second re-arm also lands in the same tick, for
every commanded slot, for the life of the run.**

That means every ~2 s, one single plant tick does 2 extra CAN
frames (pack + `can_tx_enqueue`, plus an MCP flush-rail mark when
applicable) **per commanded RobStride slot** on top of its normal 1 MIT
frame — up to 3× the steady-state per-tick frame count in that one tick,
across however many buses are commanded. That is exactly consistent with a
peak 5-6× the mean, recurring on a fixed ~2 s cadence unrelated to tx Hz —
and plausibly also the source of the sporadic `cmd_seq_lag_p95` "anomaly"
notes scattered through the bench docs (a tick that runs several ms long is
a tick where the USB host-link task is more likely to get starved long
enough for a few queued host frames to coalesce away at once — see the
"Related: 500 Hz gate" note below).

## Fix

Stagger the phase of the periodic re-arm across slots, without changing
*when the first enable goes out* (still immediate — no bringup latency
added) or *whether* the 2 s cadence applies (still exactly once per period,
same reliability guarantee). On `first_arm`, instead of storing `now`,
store `now - stagger_ms` where `stagger_ms = slot * 2000 / ACTUATOR_COUNT`
(80 ms per slot step at `ACTUATOR_COUNT=25`). This only shifts *when the
next* 2 s window closes for that slot — up to 25 commanded slots spread
their re-arms across the full 2 s period (~1 slot re-arming every ~80 ms)
instead of stacking all of them into one tick.

```c
if (first_arm) {
	uint32_t stagger_ms = ((uint32_t)slot * ROBSTRIDE_MAINTAIN_PERIOD_MS) /
	                      ACTUATOR_COUNT;
	*last_ms = now - stagger_ms;
} else {
	*last_ms = now;
}
```

Backdating relies on the same wraparound-safe `now - last_ms` unsigned
subtraction idiom already used everywhere else in this file (e.g.
`robstride_interp_desire`'s `since_ms = HAL_GetTick() - h->t_curr_ms`) — if
`now < stagger_ms` near boot, the subtraction wraps mod 2³², and the later
`(now2 - *last_ms) < ROBSTRIDE_MAINTAIN_PERIOD_MS` comparison still resolves
correctly under the same modular arithmetic. No new failure mode introduced.

Full diff (also touches 3 call sites in single-motor bringup/probe/cali
paths that share one `ctrl_fast_maintain_ms` variable, not the ×25 array —
those pass `slot=0u`, i.e. no stagger, identical behavior to before since
there's only ever one motor active in those flows):
[stagger-robstride-maintain.patch](patches/stagger-robstride-maintain.patch).

**Zero protocol change.** Every slot still gets exactly one `run_mode` +
`enable` pair every 2 s while commanded, and still gets it immediately on
first arm. Only *which tick* the periodic ones land in changes.

## Explicit non-goals

- No change to the 2 s period itself, to MIT/pararead cadence, to
  `RS02_MCP_APPLY_DIV`/`RS02_FDCAN_APPLY_DIV_HEAVY` (both stay `1u`), or to
  any bus-priority/skip logic — this sprint's identity lock (equal-rate FB
  freshness, no MCP÷/post-FB/priority-actuator tricks) is untouched.
- Does not address the 500 Hz `cmd_seq_lag_p95` gate failure directly (see
  below) — that gate fails on a host-link mechanism this patch doesn't
  touch, though flattening the periodic act_lap spikes may reduce how often
  that mechanism gets triggered (untested — needs a matrix run to confirm).

## How to apply

```powershell
git apply docs/patches/stagger-robstride-maintain.patch
```

Verified clean against `main`'s current `robstride.c` (post Release+RX-index
apply) via `git apply --check` — no working-tree changes made by this agent.

## Matrix checklist

Same shape as the prior two RFCs — apply on top of the current
Release+RX-index image so this run isolates the stagger's effect:

- Host rates: 40, 100, 200, 500 Hz.
- `--skip-real --skip-cali`.
- 3 trials/rate, **each trial ≥ 4-5 s** (needs to span at least two 2-second
  re-arm windows to actually observe whether the peak flattened — the prior
  bench runs used 8 s phases, which is already enough; just don't shrink it
  for this comparison).
- Compare against
  [legacy/bench-load-matrix-release-rxindex-2026-07-23.md](../bench-load-matrix-release-rxindex-2026-07-23.md).

## Success metric

`act_lap` **peak** at every rate should drop from ~9-11 ms toward something
much closer to the mean (~1.6-2.2 ms) — if the theory is right, the peak
column stops being a fixed ~5-6× multiple of the mean and starts tracking
it. Mean should be flat or very slightly better (fewer extra frames spread
out is marginally cheaper in aggregate than a periodic backlog that likely
also forces a couple of extra SPI-flush passes to drain). No regression in
`ok` counts at 40/100/200 Hz.

## Result (Cursor matrix, 2026-07-23 13:56 PDT)

Patch applied and matrixed:
[legacy/bench-load-matrix-release-stagger-2026-07-23.md](../bench-load-matrix-release-stagger-2026-07-23.md).
Partial confirmation, not a full one — flagging honestly rather than
overclaiming:

| tx Hz | Release+RX-idx act_lap mean/peak | +Stagger act_lap mean/peak | ok |
|---:|---|---|---|
| 40 | 1.6 / ~10 | **1.5 / 11** | 3/3 |
| 100 | 1.9 / ~10 | **1.6 / 11** | 3/3 |
| 200 | 2.0 / ~10 | **1.9 / 11** | 3/3 |
| 500 | ~2.0 / ~11 | **1.7 / 14** | 1/3 (was 0/3) |

Mean improved slightly further at every rate (as expected — spreading the
re-arm cost is marginally cheaper in aggregate, not just flatter). The
**peak did not flatten to near the mean** the way the theory predicted —
it's still ~6-9× the mean, not ~1-2×.

The more interesting datum is in the same doc's bandwidth-baseline section:
`act_pk` reads **exactly 14 across every single bus group** — `1_CH1_x8`
(pure Damiao, zero RobStride slots), `4_CH4_x2`, `6_CH6_x2`, `7_CH1-3_fdcan`,
`8_CH4-6_mcp`, `9_all_CH1-6_x25` — regardless of rx_sim on/off or tx Hz. A
peak that's identical across a Damiao-only group and every RobStride group
alike cannot be the maintain-enable burst this patch targets (Damiao has no
such re-arm mechanism at all). That means **the stagger fix was real and
worth keeping** (mean is down, and it removes a genuine synchronized-burst
mechanism this doc diagnosed correctly), but **a separate, still-unexplained
periodic cost of ~11-14 ms exists somewhere shared across all buses/protocols**
— a candidate for a follow-up investigation (ISR/scheduler jitter, a
periodic flash/NVM touch, or something in the common plant-tick path rather
than any one protocol plugin). Not chased further here — out of scope for
this RFC's claim, flagging for whoever picks up the next optimization pass.

500 Hz `ok` improved from 0/3 to 1/3 (small sample — 3 trials — treat as
directional, not conclusive) but is still failing; consistent with the
"Related" section below being the dominant 500 Hz blocker, not act_lap.

## Related: why the 500 Hz `cmd_seq_lag_p95` gate still fails (read-only finding, no patch)

Every bench doc to date — Debug, Release, Release+RX-index — fails the
500 Hz `ok` gate on `cmd_seq_lag_p95`, even though act_lap itself is well
inside the 2 ms TIM6 budget at 500 Hz post-Release (mean ~2 ms). Traced the
mechanism (read-only, [`App/Src/host/host_link.c`](../App/Src/host/host_link.c) +
[`Core/Src/app_freertos.c`](../Core/Src/app_freertos.c)) rather than leaving
it as "coalesce artifact, don't chase it":

- `host_link_poll_rx()` drains the whole USB RX ring in one call, but by
  design (documented, intentional) **coalesces multiple queued plant
  command frames down to only the newest** before staging them for
  `PlantTask` — see the comment at `host_link.c:113`, "coalesce plant
  images to the latest per lap so a 200-500 Hz host does not mount 25 slots
  for every queued frame." Every frame skipped by this coalesce is a frame
  whose sequence number never updates `g_last_command_seq` — which is
  exactly the field the bench's `cmd_seq_lag` metric reads back.
- `host_link_poll_rx()` itself only runs from `StartDefaultTask`
  (`Core/Src/app_freertos.c:136-147`), a plain `for (;;) { app_host_service();
  osDelay(1); }` loop — capped at roughly the RTOS tick rate (~1 kHz ceiling
  if `configTICK_RATE_HZ=1000`), and that task shares the *same*
  `osPriorityAboveNormal` band as `PlantTask` and `PeripheralTask`
  (`app_freertos.c:44-67`). Under FreeRTOS time-slicing this doesn't starve
  outright, but every extra millisecond `PlantTask` spends mid-burst (see
  the maintain-enable finding above) is a millisecond `StartDefaultTask`
  isn't draining the USB ring — and at 500 Hz host TX (2 ms/frame), a
  handful of ticks' worth of delay is enough for several frames to queue up
  and get collapsed into one, which is structurally a **larger relative
  lag at higher tx Hz** even with nothing going wrong, because the
  coalesce-to-latest design discards intermediate sequence numbers on
  purpose. This is very likely why `cmd_seq_lag_p95` scales specifically
  with tx Hz while `act_lap`/`plant_fb` do not — the same design that keeps
  ×25 slot-mounting cheap at high host rates trades away exactly the
  sequence-continuity number this particular bench gate checks.
- **Candidate experiment (not a patch — needs board measurement, flagging
  only):** the maintain-enable stagger above should reduce how often
  `PlantTask` runs long enough to let a multi-frame backlog build in the
  USB ring, which may reduce (not necessarily fix) the 500 Hz `lag_p95`
  spikes as a side effect. If it doesn't move the needle, the next lever
  isn't a firmware change at all — it's deciding whether the `cmd_seq_lag`
  gate threshold is the right thing to check at 500 Hz given the
  coalesce-to-latest design is intentional, versus loosening
  `StartDefaultTask`'s `osDelay(1)` (e.g. yielding without a full tick
  delay) to poll the USB ring more often. That's a scheduling change, not
  an algorithmic one — recommend measuring with the stagger patch alone
  first before touching task timing, since it's the smaller, safer, already
  fully-drafted change.
