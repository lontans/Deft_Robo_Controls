# Bench: board optimization checklist + bandwidth/load-matrix test plan

Role: **Claude-Bench** (see [`.cursor/plans/four-agent_next_pass_c4b4205e.plan.md`](../.cursor/plans/four-agent_next_pass_c4b4205e.plan.md)).
Status: **plan + CLI skeleton, offline** — no COM opened by this pass. Live
matrix run is a follow-on for whoever owns CDC next.

Out of scope (per plan): `deft_vbeta/`, Claudistic dashboard teleop rewrite,
Soft-DFU flash.

---

## 0. What "board optimize" already covers — do not re-litigate

A prior offline pass ([`legacy/act-lap-bloat-deepdive-2026-07-23.md`](legacy/act-lap-bloat-deepdive-2026-07-23.md))
already identified and ranked the plant/MCP/host optimization candidates, and
— per [`decisions.md`](decisions.md) / [`rfc-stagger-robstride-maintain.md`](rfc-stagger-robstride-maintain.md)
— the top two are **already applied and matrixed**:

| # | Candidate | Status | Evidence |
|---|---|---|---|
| 1 | Release build (`-Os` vs `-O0`) | **Applied** — `Release/DeftRoboticsControlsPCB.elf` exists; Soft-DFU's `default_firmware_elf()` prefers it | [`rfc-release-build.md`](rfc-release-build.md), [`legacy/bench-load-matrix-release-2026-07-23.md`](legacy/bench-load-matrix-release-2026-07-23.md) |
| 2 | Per-bus RX-dispatch slot index (`actuator_dispatch_bus_rx`, was O(frames×25) scan) | **Applied** | [`rfc-per-bus-rx-index.md`](rfc-per-bus-rx-index.md), [`legacy/bench-load-matrix-release-rxindex-2026-07-23.md`](legacy/bench-load-matrix-release-rxindex-2026-07-23.md), [`legacy/bench-load-matrix-release-rxindex-real-2026-07-23.md`](legacy/bench-load-matrix-release-rxindex-real-2026-07-23.md) |
| 3 | Stagger `robstride_maintain_enable` re-arm phase | **Applied** | [`rfc-stagger-robstride-maintain.md`](rfc-stagger-robstride-maintain.md), [`legacy/bench-load-matrix-release-stagger-2026-07-23.md`](legacy/bench-load-matrix-release-stagger-2026-07-23.md), [`legacy/bench-load-matrix-release-maintain-budget-2026-07-23.md`](legacy/bench-load-matrix-release-maintain-budget-2026-07-23.md) |
| — | `PlantTask` → `osPriorityHigh` | **Tried, reverted** — cut act_lap peak 11→3 ms but blew up `cmd_seq_lag` p95 (12–122) at 200/500 Hz and ballooned `periph_lap` (DXL fragmented under preemption) | [`legacy/bench-load-matrix-release-plant-high-2026-07-23.md`](legacy/bench-load-matrix-release-plant-high-2026-07-23.md) |

Net effect on the all×25 hold (40 Hz host, RX-sim on): `act_lap` mean dropped
from **~3.6–4.2 ms** (Debug baseline, 07-23) to **~1.0–2.0 ms**
(release+rxindex+maintain-budget, 07-23). This checklist does **not** propose
redoing that work — it (a) lists what's left to confirm still holds under
current firmware, (b) names the two open, matrix-evidence-backed items, and
(c) gives a durable CLI so the *next* regression check doesn't reinvent a
`_tmp_` harness.

---

## 1. Board optimization checklist (static review, this pass)

### 1a. Confirm-still-true (regression watch, run when CDC is next free)

These aren't new work — they're the "did the last three landed changes hold"
sanity check before trusting the board for anything else:

- [ ] `Release/DeftRoboticsControlsPCB.elf` is still the flashed image
  (`soft_dfu_flash.py` prefers it automatically — confirm via
  `hub.debug.cfg_get_table()` isn't the story here; check build date /
  `Release/` mtime vs last `App/` edit instead).
- [ ] Per-bus RX-index and stagger patches are still present in
  `App/Src/plant/actuator.c` / `robstride.c` (i.e. nobody reverted them while
  chasing an unrelated bug) — `git log -p` on those files, not a live check.
- [ ] All×25 40 Hz hold still lands near the maintain-budget baseline
  (`act_lap` mean ≈1.0–1.3 ms, `ok` 3/3) — first thing `bench_load_matrix.py`
  (§2) should reproduce.

### 1b. Open candidates (matrix-evidence gated — do not implement speculatively)

| Candidate | Why it's not done yet | Gate before touching `App/` |
|---|---|---|
| Per-slot float work in `robstride_apply_cycle` (`robstride_interp_desire` + `robstride_maintain_enable` diff-compare) — cheap individually, ×25×500 Hz | Deepdive explicitly said "not worth touching yet — measure after 1a" (1a = RX-index, now done) | Re-run §2 matrix on current firmware first; only chase this if `act_lap` mean at 500 Hz is still the bottleneck (last measured: RX-index/maintain-budget already got 500 Hz `act_lap` mean down to ~1.5 ms — the 500 Hz failure is `cmd_seq_lag`, not `act_lap`, so this candidate may already be moot) |
| 500 Hz `cmd_seq_lag_p95` gate (host_link_poll_rx coalesces queued images to newest per lap — intermediate seqs never ack) | Known, intentional coalesce behavior, not a bug; `rfc-stagger-robstride-maintain.md` flags it as a separate follow-on | Needs its own RFC/patch (out of scope here) if 500 Hz is ever promoted from "capability note" to a hard product requirement — see §3 pass/fail, it isn't today |

### 1c. Explicitly not board optimization (do not fold into this checklist)

- Repo bloat (`External_Documentation/`, `docs/deft_vbeta_ref/`) —
  already tracked in [`legacy/act-lap-bloat-deepdive-2026-07-23.md`](legacy/act-lap-bloat-deepdive-2026-07-23.md) §2/§3
  as a separate, non-runtime concern (git history size, not plant timing).
- CubeMars scaling/feedback-ID P0s ([`lessons.md`](lessons.md)) — protocol
  correctness, not bandwidth.

---

## 2. Bandwidth / load-matrix test plan

### 2.1 Living CLI: `scripts/bench_load_matrix.py`

Skeleton checked in alongside this plan (see `scripts/bench_load_matrix.py`).
Wraps [`deft_controls_sdk/bench/metrics.py`](../scripts/deft_controls_sdk/bench/metrics.py)'s
`measure_hold` — the same helper `rs02_channel_bringup.py` already uses, so
this is not a new metrics implementation, just a durable multi-scenario/
multi-rate runner + report writer around the existing one.

**Replaces** (ghosts — do not resurrect these names, they're gone from the
tree per [`scripts-hygiene.md`](scripts-hygiene.md)): `_tmp_mcp_timing_probe.py`,
`_tmp_rate_rx_sweep.py`, `_tmp_load_matrix_report.py`.

CLI shape:

```powershell
cd scripts
python bench_load_matrix.py --port COM5 --hz 40,100,200,500 --scenario all
python bench_load_matrix.py --port COM5 --hz 40 --scenario idle
python bench_load_matrix.py --port COM5 --hz 40,500 --scenario ch1 --trials 3 --seconds 8 --report ../docs/bench-load-matrix-<date>.md
```

- `--hz`: comma list, default `40,100,200,500` (matches bringup §7a and the
  `docs/legacy/bench-load-matrix-*` baselines).
- `--scenario`: `idle` (blank product CFG, TX only), `ch1`/`ch2`/`ch3`/
  `ch4`/`ch5`/`ch6` (single-bus hold, RX-sim on that bus's slots), `mcp`
  (CH4–6 together — the Apply-accumulate footgun case, see §2.3), `all`
  (every enabled product-CFG slot, RX-sim on) — mirrors the bus-group
  breakdown already used in `docs/legacy/bench-load-matrix-*.md` (`1_CH1_x8`
  … `9_all_CH1-6_x25`, now ×26 slots per host-exchange-v3).
- `--trials`: repeats per rate (default 3, matching prior bench docs).
- `--report`: optional path; writes the same table shape as
  `docs/legacy/bench-load-matrix-*.md` so a new run is a direct row-for-row
  diff against a prior baseline. Without `--report`, just prints
  `measure_hold`'s existing stdout report per trial plus an aggregate table.

### 2.2 Scenarios

| Scenario | What's held | Purpose |
|---|---|---|
| `idle` | Blank desires, TX only, no RX-sim | Isolates pure USB/host TX cost from any plant apply cost |
| Single-bus (`ch1`…`ch6`) | RX-sim on that bus's enabled product-CFG slots only | Per-bus baseline — cheapest way to catch a regression on one channel without the all×26 noise floor |
| `mcp` (CH4–6 together) | RX-sim on all CH4–6 slots simultaneously | **Apply-accumulate footgun**: per `docs/api.md` — "Holds accumulate per slot — leaving many CH4–6 slots non-blank at once is expensive on the MCU." Blank MCP slots skip SPI entirely; this scenario is the one that actually exercises that cost, so it must never be silently merged into a "cheap" idle-style baseline |
| `all` | RX-sim on every enabled product-CFG slot (arms 0–13 + base 14–19; `yam_product_rows()` — 20 of 26 slots, lift/spare 20–25 stay disabled) | Product-realistic worst case, matches `9_all_CH1-6_x25`/`x26` in prior bench docs |

**Open question this plan does not resolve:** `docs/bench-pdb-sdk-contract-2026-07-24.md`
records a `9_all_CH1-6_x26` run with **CH3×4** enabled, but
[`vbeta/slots.py`](../scripts/deft_controls_sdk/vbeta/slots.py)'s
`yam_product_rows()` currently disables all of CH3 (lift slot 20 + 5 spares
21–25). Either that bench session used a different, non-`yam_product_rows`
CFG, or `slots.py` is stale relative to the live product layout. Flag to
whoever owns `deft_vbeta`/CFG next (Claude-Vbeta lane) — resolving it is
out of scope here (no speculative CFG changes from this pass).

### 2.3 Pass/fail (per plan §"Plan content must lock")

Mirrors `docs/bringup.md`'s acceptance numbers — do not invent new gates:

- **40 Hz is the hard gate**: must meet `ack_lag_max ≤ 2`, `fb_hz` healthy
  (≥20 per `measure_hold`'s own `ok_fb`, though product bench docs show
  hundreds-Hz in practice), `ok_plant_tag` (no stray DEBUG/bench tag on the
  plant stream).
- **100/200 Hz**: same gates, informational if they slip — no bench doc so
  far has failed these.
- **500 Hz**: **capability note, not a hard fail.** Every prior matrix run
  (07-22 through release/rxindex/maintain-budget/stagger) shows 500 Hz
  failing `cmd_seq_lag_p95`/`ack_lag` some fraction of trials — this is the
  known, intentional `host_link_poll_rx()` coalesce-to-newest behavior, not
  a regression. Record it; don't gate the pass on it.
- **TX floor**: keep host TX ≥ 1 Hz for the duration of any scenario so
  `ACTUATOR_HOST_STALE_MS` (500 ms, [`diag_gates.c`](../App/Src/plant/diag/diag_gates.c))
  never trips `plant_runtime_actuator_can_apply()` into `PLANT_BLOCK_HOST_STALE`
  mid-measurement — a stalled TX loop would silently zero out the apply gate
  and produce a misleading "healthy" FB read (feedback keeps flowing even
  while blocked).

### 2.4 Doc updates (this pass)

`debug_api.md` §6 and `bringup.md` §7a/§"Code map" still show example
commands for the three deleted ghost scripts — rewritten to point at
`bench_load_matrix.py` (see diffs in this same change). `api.md`'s one-line
bandwidth example and `rfc-release-build.md`'s matrix-checklist command
updated the same way.

---

## 3. Live-run readiness (for whoever runs this next, when CDC is free)

1. Announce COM ownership per the sprint's collision rule (`docs/rfc-release-build.md`
   §"Matrix checklist": "Leave COM5 idle before/after — announce `COM5: Cursor`
   while running").
2. Confirm no Claudistic dashboard / Claude-Vbeta smoke has the port open.
3. `python bench_load_matrix.py --port COM5 --scenario all --report ../docs/bench-load-matrix-<date>.md`
   as the baseline re-check (§1a), then targeted scenarios only if something
   regressed.
4. If §1a's "confirm-still-true" checklist fails (act_lap mean back above
   ~2 ms at 40 Hz all×26), that's a signal one of the three landed patches
   regressed — `git log` those files before assuming new optimization work
   is needed.
