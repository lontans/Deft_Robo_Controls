# Deep dive: act_lap under ×25 + repo bloat (2026-07-23)

Offline analysis (no board/COM touched). Scope: where the ×25 rx_sim act_lap
cost (~3.6–4.2 ms vs ~0.9 ms real-single-slot, per
[bench-load-matrix-2026-07-22.md](bench-load-matrix-2026-07-22.md)) actually
goes, and what in the repo is bloat vs load-bearing. Read-only pass; no App/
edits made yet — this is the recon for the equal-rate cut in
[vbeta_api_shape plan](../../../.cursor/plans) ("Claude: cut ×25+DXL+LED
act_lap on main without MCP÷4/post-FB").

---

## 1. Loop performance — where act_lap goes

`control_loop_service()` → `actuator_apply_desire()` runs the whole 25-slot
table every plant tick (`App/Src/plant/actuator.c:198`). Already-landed work
(Jul 22 MCP poll / host coalesce work) cut most of the *SPI/FDCAN* redundancy:
INT-gated MCP idle RX, batched 16 B RAM, TXQ STA dedupe, burst=1, end-of-lap
skips already-polled buses. What's left:

### 1a. Damiao RX dispatch is an unindexed O(frames × 25) scan — real, fixable

`actuator_dispatch_bus_rx()` (`actuator.c:150`) does, per bus, per tick:

```c
while (can_rx_pop(bus, &frame) == CAN_OK) {
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {   // scans all 25 slots
        if (!actuator_table[i].enabled) continue;
        if (actuator_table[i].bus != bus) continue;
        ...
    }
}
```

CH1/CH2 (Damiao arms) can return up to 7 RX frames per tick each (daisy
chain). That's up to **7 × 25 = 175** slot-table checks per bus per tick,
×2 arm buses, every 2 ms lap — pure waste since `actuator_table[i].bus`
is fixed at CFG-load time, not per-tick. `actuator_table` never changes
shape mid-run outside `plant_config`/CFG load.

**Fix:** precompute a per-bus slot list (small fixed array, built once in
`actuator_init()`/CFG-apply, not per-dispatch) and iterate only the slots
that live on that bus. Same idea already applied to MCP polling
(`poll_buses` bitmask) — just not yet to the RX dispatch fan-out. This is
the single most mechanical, lowest-risk win in the ×25 path: no protocol
behavior changes, pure lookup restructuring.

### 1b. Per-slot float work on every non-idle tick

`robstride_apply_cycle` calls `robstride_interp_desire` (host position
interp, MCP only) and `robstride_maintain_enable` (cheap `HAL_GetTick()`
diff-compare, only sends when `>2000 ms` or first-arm) on *every* non-idle
slot every tick. Individually cheap (a few float ops), but ×25 slots ×
500 Hz it's the kind of thing that shows up once 1a is fixed and the
profile shifts. Not worth touching yet — measure after 1a.

### 1c. Untried, zero-risk, potentially the biggest lever: Release build

**Confirmed: this firmware has never been built with optimization.**
`Debug/makefile` / `.cproject` show only a `Debug` config has ever produced
output (`Debug/DeftRoboticsControlsPCB.elf` is the only build artifact in
the tree). A `Release` configuration already exists in `.cproject`
(`com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.release.1049488396`)
but has **never been built** — no `Release/` directory exists anywhere.
The 2026-07-22 handoff doc's own "what to try next" lists "`-Os` / Release
compare — Debug is `-O0`" and it's still open.

Every number in both bench docs (act_lap 0.9–4.2 ms, fb_hz ceilings) was
measured at `-O0`. Float-heavy per-slot work (MIT pack/unpack, CRC16,
interp) on a Cortex-M4 typically drops substantially under `-O2`/`-Os` —
this could move the needle more than any of the SPI/dispatch hygiene above,
for zero logic risk (same code, different codegen). It just has never been
tried because Debug is what CubeIDE builds by default and nobody's swapped
configs.

**Why it's still open:** no `arm-none-eabi-gcc`/`make` on this machine's
PATH outside STM32CubeIDE's bundled toolchain — building/comparing Release
needs either the IDE (Project → Build Configurations → Set Active →
Release) or a headless CubeIDE invocation. Either way it's a build-only,
no-board action — squarely in the "Claude: offline until matrix" lane.
**Recommend doing this first**, before further hand-hygiene of the SPI
path, since it changes the baseline every other measurement is compared
against.

### 1d. Not a bug, just a note for interpretation

The all×25 rx_sim bench is exercising *real* CAN/SPI TX for all 25 slots
(rx_sim only fakes the RX/feedback side — see
`rx_sim_actuator_on_apply`, `App/Src/plant/rx_sim/rx_sim_actuator.c:6`,
which pushes synthetic feedback but does nothing to the TX path). So the
~3.6–4.2 ms is genuine bus-driver cost for 25 real frames/tick across 6
buses, not simulator overhead — 1a/1c are real fixes, not artifacts of the
bench harness.

---

## 2. Repo bloat

Cursor's own [scripts-hygiene.md](scripts-hygiene.md) (already
written, uncommitted) covers `scripts/_tmp_*` triage and `scripts/legacy/`
retirement well — not duplicating that here. Two bigger sources it doesn't
touch:

| Path | Size | Tracked? | Issue |
|---|---:|---|---|
| `External_Documentation/` | **~120 MB** after GUI removal (was 171 MB) | Yes (PDFs/samples) | Vendor PDFs still dominate clone size. **`RobStride/motor_toolV14L/` Qt GUI untracked + gitignored** (2026-07-23) — see [`External_Documentation/RobStride/README.md`](../External_Documentation/RobStride/README.md). History rewrite still needed if packed `.git` must shrink. |
| `docs/deft_vbeta_ref/deft_vbeta/` | **36 MB** | No (untracked) | Left alone for now (user call 2026-07-23). Still do not commit the full mirror; trim to cited files before any add. |
| `Debug/DeftRoboticsControlsPCB.elf` | 2.3 MB | Yes, deliberately (`.gitignore` un-ignores it) | Intentional — soft-DFU flashes from this path. Fine to keep, but every firmware commit now carries a full 2.3 MB binary diff in history. Not "bloat" so much as a known, accepted cost — flagging only because Cursor's draft `.gitignore` snippet in scripts-hygiene.md (`*.elf`) would silently conflict with the existing `!Debug/DeftRoboticsControlsPCB.elf` un-ignore rule if applied as-is. Worth a one-line note back to them, not a fix I should make in their file.

`.git` itself is currently 101 MB packed — the 171 MB `External_Documentation` and PDFs compress reasonably, but it's still the dominant cost of every clone.

**Recommendations, ranked:**

1. **`External_Documentation/RobStride/motor_toolV14L/`** — **done** (untracked + `.gitignore`; pointer README under `RobStride/`). Does not rewrite history.
2. **`External_Documentation/*.pdf`** (~120 MB of datasheets) — same call to make: keep a short README with vendor links, or move to a wiki/drive, vs. carrying every datasheet in every clone forever. Lower urgency than #1 since PDFs are at least genuinely reference material for this project.
3. **`docs/deft_vbeta_ref/`** — deferred (leave tree for now). Still: don't commit the whole 36 MB mirror; trim to cited files before any add.

Items 1–2 are already-tracked history — removing them from the working tree stops future growth but doesn't shrink existing `.git` without a history rewrite (`git filter-repo`/BFG), which is a separate, disruptive decision (rewrites SHAs, needs force-push, coordinates with anyone else with a clone). Flagging as a call for you, not something to do unprompted.

---

## 3. Priority order

| # | Action | Effort | Risk | Lane |
|---|---|---|---|---|
| 1 | Build `Release` config, compare act_lap/fb_hz against the 2026-07-22 matrix numbers | Low (build-only) | None — same code | Offline, no board |
| 2 | Index Damiao/actuator RX dispatch by bus instead of scanning all 25 slots | Low | Low — pure restructure, same semantics | Offline, no board (App/ edit + build) |
| 3 | Untrack `External_Documentation/motor_toolV14L/` (binaries) going forward | ~~Low~~ | **Done** 2026-07-23 | — |
| 4 | Trim `docs/deft_vbeta_ref/` to the ~2 files actually cited before it's ever committed | Low | None (uncommitted) | Deferred — leave mirror for now; do not `git add` |
| 5 | Decide on `External_Documentation/*.pdf` retention policy | Low decision, larger if history rewrite wanted | Medium if rewriting history | User call |
