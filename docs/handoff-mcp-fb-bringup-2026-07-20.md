# Handoff: MCP / FB / dashboard bringup (2026-07-20)

Resume context for another Cursor agent. Author of this note was the prior agent on the DeftRoboticsControlsPCB host SDK + live COM5 plant bringup.

## User position (accept as constraint)

The operator’s claim, which the prior agent under-weighted:

> **500 Hz plant path with non-blank teleop on CH1–CH6 (including MCP CH4–6) was already proven working** (legacy teleop / prior testing). SPI handling that load is not a new impossibility.

If the next agent’s model requires “MCP SPI cannot sustain non-blank 500 Hz,” that model conflicts with the operator’s lived result. Prefer: **something about this host/dashboard session differs from the working teleop path**, or **observability (USB FB / metrics / LEDs) is what collapsed**, not that MCP teleop never worked.

The operator is moving to another agent partly because the prior agent kept recentering on MCU SPI as the root cause while the operator insists the full-rail teleop case already worked.

## What was measured on this plant (COM5, flashed board, no reflash)

Hardware: dual YAM Damiao + MCP RobStride on CH4–6 (live CFG, not dual-YAM-only defaults). Host work in untracked `scripts/deft_controls_sdk/` (legacy under `scripts/legacy/`).

| Condition | Host TX | Raw USB `fb_hz` (FrameReader) | `ack_lag` @ 40 Hz host |
|-----------|---------|-------------------------------|-------------------------|
| Idle / blank MCP desires | ~40 Hz, gap p95 ~25 ms | ~900–1000 Hz | ~0 |
| Single non-blank MCP hold (Apply-style `pos≈1e-6, kp>0`) | ~40 Hz | **~2 Hz** | **steady ~20–21** |
| Three non-blank MCP holds (slots 3+4+5) | ~40 Hz, gaps still OK | ~1 Hz | **29 → 62 → 95** |
| CH1–3 / blank MCP | fine | high FB | lag ~0 |

Additional observations:

- Under single MCP, over ~0.5 s FB spacing: `host_delta ≈ ack_delta ≈ 21` while `lag` stays ~21 → **MCU `last_cmd_seq` keeps up with host**; lag is not “MCU stopped accepting cmds.”
- Stopping host TX lets lag drain to 0; resuming 40 Hz climbs back to ~21.
- Wall latency `lag/tx_hz ≈ 0.5 s` across host rates (40/20/5/2 Hz) in one sweep — consistent with FB sample period, not only “compute can’t do 40 Hz.”
- Operator: LED freeze visible under **GUI**, not when watching lean agent scripts; freeze correlated with **`ack_lag` spikes (~29)** more than steady ~20.
- Idle GUI: `gap_max` could spike ~50 ms (Windows sleep / sticky metric / telemetry-on-hot-path). Plant TX work (`send_ms`) stayed ~1.5–2 ms.

**Blank vs non-blank MCP A/B** (same host stream): blank → FB ~1000 Hz; non-blank → FB ~2 Hz. That still implicates the **non-blank MCP apply path on the MCU** for USB FB collapse under *this* session — but it does **not** by itself prove teleop never worked, nor that SPI is incapable of 500 Hz in the legacy configuration.

## Where prior agent reasoning is weak / contested

1. **“SPI can’t do 500 Hz”** — Too strong vs operator + firmware intent (`robstride.c` / `mcp2518fd.c` comments: non-blocking plant MIT so 500 Hz MCP is viable; blocking TX was the old peg). Prefer: under *this* hold pattern we see **USB FB starve**, while cmd ack still advances.

2. **Equating FB starve with “teleop broken”** — Legacy teleop is open-loop send@~40 Hz; motors can feel fine with sparse USB FB. Measuring `fb_hz`/`ack_lag` as primary health may overstate regression vs “working teleop.”

3. **Under-explored deltas vs working teleop** (next agent should prioritize):
   - Live CFG vs default table; which slots/buses/protocols are actually mounted.
   - Dashboard **Apply accumulates** multi-slot holds vs teleop’s active-bus / single-focus pattern.
   - Dashboard `HOME_POS_EPS` + always-on stream vs teleop loop structure.
   - `start_streaming` auto-recover was removed earlier (RECOVERY/`plant_recovery_all` was cross-lighting MCP); confirm teleop doesn’t do something equivalent.
   - Whether legacy truly held **all** CH4–6 non-blank simultaneously at plant rate, or “all buses in the system” with only one MCP active.
   - Whether “500 Hz” refers to **plant tick** (TIM6) vs **host USB CMD rate** (~40 Hz) — easy to conflate in conversation.

4. **GUI-only LED freeze** — Partially explained by host TX gaps / telemetry-on-stream-thread (later split); not fully closed against board LED timing vs ACT LED vs multi-hold.

## Firmware notes (for orientation, not as verdict)

- Superloop (`app_run`): `poll_rx` → `control_loop_service` (up to `CONTROL_TICK_BURST_MAX=8` applies) → … → `host_link_poll_tx`.
- Blank MCP desires **skip SPI** (`actuator.c` / `robstride_apply_cycle`).
- Non-blank MCP: `mcp2518_prepare_tx` + `spi_can_router_tx_flush` per apply.
- FDCAN CH1–3: mostly enqueue to HW FIFO (different CPU cost).
- `last_cmd_seq` updates on USB CMD dispatch in `host_link.c`, reported 8-bit in FB.

## Host SDK changes made this session (untracked / local)

Package: `scripts/deft_controls_sdk/`.

Intent of later fixes (after several wrong turns: credit pacing, blaming MCU compute, sticky metrics):

- Plant stream = **send → sleep only**; **telemetry on a side thread** (not in plant TX).
- `ControlsPcbHub.connect(..., persist_telemetry=False)` by default; dashboard opts into `state.json`.
- Compact opt-in logging: `deft_fb_v1` via `start_recording()` / `log_feedback()` (`telemetry/fb_log.py`) — not fat `SessionState` as flight log.
- Windows hybrid sleep / `timeBeginPeriod` remain host workarounds; Linux/Jetson should use plain sleep path.
- Dashboard: no auto-recover on connect; Apply `send=False`; Idle all; held snapshot; `/api/state` coalesce; gap_p95 + short-window gap_max.

Treat these as **host hygiene**, not proof that MCP teleop is impossible on-device.

## Suggested next-agent agenda

1. **Reproduce legacy teleop** on the same COM5/CFG with CH4–6 non-blank as the operator describes; log raw `FrameReader.total_frames` rate and ack deltas the same way as SDK probes.
2. If legacy shows high FB under multi-MCP non-blank → prior agent’s “MCP SPI starves USB” story is incomplete; diff host frame content, slot holds, recover, rate, flush.
3. If legacy also shows ~2 Hz FB but “feels fine” → reframe problem as **observability / GUI / multi-hold**, not “MCP broken.”
4. Instrument MCU lap (`lap_ms`, `ticks_pending` SVD) under single vs multi MCP hold — needs DEBUG PDU path.
5. Do not reintroduce plant-thread JSON/`state.json`/credit wait.

## Key files

- `App/Src/plant/plugins/robstride.c` — MCP vs FDCAN apply, SPI flush
- `App/Src/plant/actuator.c` — blank MCP skip
- `App/Src/plant/control_loop.c` — 500 Hz pending + burst max 8
- `App/Src/app.c` / `App/Src/host/host_link.c` — superloop + USB FB
- `scripts/deft_controls_sdk/link/connection.py` — plant vs telemetry threads
- `scripts/deft_controls_sdk/telemetry/` — cache, compact FB log, disk writer
- `scripts/deft_controls_sdk/debug_dashboard/` — GUI
- `scripts/legacy/control_hub/teleop/plant.py` — known-good teleop reference

## Tone for resume

Operator is frustrated after a long bringup; they asked for this handoff explicitly. Prioritize **diff vs working legacy teleop** over re-arguing SPI physics. Thank you from prior agent for the session; operator’s stubbornness on “CH1–6 teleop worked” is a feature for the next pass.

---

## Addendum 2026-07-20 (evening) — teleop_trace, ack-lag mechanism, 6× MCP @ 500 Hz

Operator pointed at repo-root `teleop_trace.csv` (mtime **2026-07-08**, from legacy `teleop --slot 3 --debug-trace`). Next agent (Claude): treat that file as evidence; do not assume “smooth teleop ⇒ ack_lag≈0.”

### What `teleop_trace.csv` shows (single MCP slot teleop)

| Metric | Value |
|--------|--------|
| Host cadence | ~36 Hz (dt p50 ~27.7 ms) |
| `ack_lag` = `(tx − ack) & 0xFF` | p50 **7**, p95 **14**, max **17** (never ≥20) |
| Same while `kp>0` / moving | p50 **8**, max **17** |
| How often `ack` value changes | ~**2.8 Hz** (p50 gap ~363 ms) |
| Motion quality | real tracking (`rate` ±3.5, `lead` up to 0.35, `block=none`) |
| `lap_ms` column | **absent** — not a deprecated lap dump |

Interpretation:

- Legacy RobStride `run_for_slot` mounts **`[slot]` only** → one MCP rail non-blank; other CH4–6 desires zero-filled → firmware **blank-skip** (no SPI) on those rails.
- Smooth teleop **coexisted with sparse USB FB** (~3 Hz ack updates, lag ~7–14). Open-loop send@~36 Hz does not need a 1 kHz FB flood for the shaft to feel fine.
- Dashboard **single** MCP Apply measured later (lean SDK probe) is in the same regime (~2–4 Hz FB, lag ~20 @ 40 Hz host). Dashboard **triple** MCP Apply (slots 3+4+5) is **worse** (FB ~0.7 Hz, lag max ~61) — that multi-hold pattern is **not** what `--slot 3` teleop did.
- Host TX under the SDK probe stayed healthy (`send_ms` ~1.6 ms, gap p95 ~26 ms) in blank and MCP cases → ack lag here is **not** a Windows plant-thread stall.

### How ack_lag relates to superloop (why “optimizations” help)

Causal chain (current architecture):

1. Host sends plant CMD @ ~40 Hz; MCU updates `last_cmd_seq` on USB RX dispatch.
2. USB FB is built/sent in `host_link_poll_tx()` at the **end of each `app_run` lap**.
3. `ack_lag` at a FB sample ≈ “how many host CMDs landed since the previous FB sample” ≈ `tx_hz × (time between FB samples)`.
4. If laps stretch (blocking work in plant apply), FB becomes sparse → lag climbs even while `last_cmd_seq` still advances when RX is drained.
5. `led_service()` also runs once per lap → long laps look like LED freeze.

So reducing ack lag means **keeping `app_run` laps short** under non-blank MCP, not pacing the host harder.

### What the unflashed plant-MCP fix changes (and what it does not)

Built into `Debug/DeftRoboticsControlsPCB.elf` but **not flashed** this session (no ST-Link). Intended fixes:

| Change | Why it cuts lap time / ack lag |
|--------|--------------------------------|
| `mcp2518_try_send`: no `force_ready` / no `HAL_Delay`; fail soft if TXQ busy or bus-off | Plant path was still able to block up to ~32 ms per flush via `mcp_txq_release_after_tx` |
| `mcp2518_prepare_tx`: clear ATIF only; bus-off recover ≤1 / 100 ms / rail | Recover+force_ready every MIT was a lap tax |
| MCP enable arming via enqueue+flush, not `mcp2518_send` | First Apply could block up to ~50 ms ×2 |
| Lap timing also on thermo `'t'` PDU bytes 16..21 | So `lap_ms` / `ticks_pending` visible while SPI3_ROLE_THERMO owns the mailbox |

**Does not by itself create free SPI budget for 6 actuators.** It removes accidental blocking so the remaining cost is closer to “real SPI + CAN work per non-blank MCP slot per plant tick.”

After flash, fair A/B:

1. Blank → expect lag ~0, FB flood.
2. Single MCP hold (teleop-equivalent) → compare to CSV max lag 17 / ~3 Hz ack cadence; target much closer to blank if fix works.
3. Triple MCP hold → must not climb to lag 29–61 / LED freeze.
4. Use `scripts/_tmp_mcp_timing_probe.py` (raw `FrameReader.total_frames`, ack_lag, lap_ms when present).

### Can we run 6 actuators on CH4–6 (2 per bus) on the 500 Hz inner loop?

**Hardware topology:** 3 MCP2518 rails (CH4–CH6), shared SPI1 (~10.6 MHz SCK). TXQ is configured **1-deep** (`MCP_TXQ_1X8`). Plant RobStride MCP path: **1 MIT frame per slot per 500 Hz tick**, then `prepare_tx` + up to 2× `tx_flush`, plus RX poll on commanded buses. Burst can service up to `CONTROL_TICK_BURST_MAX=8` pending ticks per lap.

**Rough load if all 6 are non-blank every 2 ms:**

- 6 MIT frames / 2 ms = **3 kHz** of SPI-CAN TX attempts across 3 rails (2 frames/rail/tick if both slots on a bus apply).
- Each frame still needs SPI load + chip select; rails are **serialized on one SPI**.
- 1-deep TXQ ⇒ cannot pipeline many frames per rail without software queue + later flush; if flush falls behind, frames backlog or drop.
- FDCAN CH1–3 is different: HW FIFO / enqueue-cheap. MCP is CPU+SPI synchronous work inside `actuator_apply_desire()`.

**Verdict (current architecture):**

| Goal | Realistic on today’s path? |
|------|----------------------------|
| 1 MCP slot non-blank @ plant 500 Hz intent, host ~40 Hz | Yes in principle (teleop already moved metal); after non-block fix, USB FB/ack should recover toward blank |
| 3 MCP slots (1 per bus) non-blank continuously | Possible but tight; was the dashboard failure case; needs post-flash lap_ms proof |
| **6 actuators (2 per CH4–6 bus) all non-blank every 500 Hz tick** | **Not a reliable plan on the current “flush SPI every plant tick per slot” path.** SPI serialization + 1-deep TXQ + superloop coupling to USB FB/LEDs will sag laps before you get true 500 Hz apply on all six |

Blank-skip helps only for **uncommanded** slots. Six live MIT holds cannot blank-skip.

### If 6× MCP @ 500 Hz is a product requirement — different path (recommended fork)

Do **not** keep stacking host workarounds. Pick an explicit plant architecture:

1. **Decimated MCP apply (most practical)**  
   Keep TIM6 / FDCAN at 500 Hz; apply each MCP slot at e.g. 100–250 Hz (phase-stagger 2 motors × 3 rails). Hold-last desires still update from USB at host rate. Document that MCP outer rate ≠ FDCAN inner rate.

2. **Async SPI-CAN DMA / IRQ completion**  
   Plant tick only enqueues; a background/ISR drains TXQ without `HAL_Delay` in `app_run`. Superloop stays free for USB FB + LEDs. Larger firmware project; enables denser MCP without lap pegging.

3. **Move high-count MCP off this MCU’s plant hot path**  
   Second controller / co-processor owns CH4–6 MIT; hub exchanges desires/feedback at a lower rate. Use when 6× true 1 kHz-class torque loops are required.

4. **Host policy (insufficient alone)**  
   Dashboard must not leave 6 slots ACTIVE by accident; teleop-style single-focus helps UX but **does not** create SPI budget if product needs six simultaneous stiff holds.

### Guidance for Claude (next)

1. Flash the already-built ELF (or rebuild) with ST-Link; re-run `_tmp_mcp_timing_probe.py`.
2. Treat teleop_trace.csv as the single-MCP baseline (lag max 17, ack ~3 Hz) — success is **beating that toward blank**, not merely matching “motors move.”
3. Decide product intent: (A) 1–3 MCP @ plant-rate with healthy FB, or (B) 6 MCP @ true 500 Hz. If (B), start a **decimate or async-SPI design**, do not only tune `try_send`.
4. Keep plant TX thread free of telemetry/`state.json` (already separated in SDK).
5. Operator success bar for localhost: no ack_lag~20 / spike~29 LED freeze when changing an MCP variable — that is the post-flash acceptance test for the non-block fix; 6×@500 Hz is a **separate** architecture decision.
