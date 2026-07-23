# RFC: per-bus RX-dispatch slot index (`actuator.c`)

Status: patch drafted and verified (`git apply --check`), ready to apply.
Author: Agent 2 (Claude), offline — no build/flash performed by this agent.
Executor: Cursor (owns `App/`/`Core/`, COM5, soft-DFU).
Patch: [`docs/patches/per-bus-rx-index.patch`](patches/per-bus-rx-index.patch).

## Problem

[`App/Src/plant/actuator.c`](../App/Src/plant/actuator.c)
`actuator_dispatch_bus_rx()` (~L150) does, per bus, per plant tick:

```c
while (can_rx_pop(bus, &frame) == CAN_OK) {
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {   // scans all 25 slots
        if (!actuator_table[i].enabled) continue;
        if (actuator_table[i].bus != bus) continue;
        ...
    }
}
```

CH1/CH2 (Damiao arm daisy chains) can return up to ~7 RX frames per tick
each. That's up to **7 × 25 = 175** slot-table checks per bus per tick — on
top of the identical scan repeating for every other bus with traffic — pure
waste, because `actuator_table[i].bus` only changes on CFG apply (init,
factory defaults, NVM load, host CFG SET), never per-tick. Same shape of fix
already landed for MCP polling (`poll_buses` bitmask in
`actuator_apply_desire`) — this just extends the idea to the RX fan-out,
which never got it.

## Fix

Precompute, once per CFG-apply edge, a `[bus][slot]` index of which slots
live on which bus (`s_bus_slot_idx` / `s_bus_slot_count`, sized
`[CAN_BACKEND_COUNT][ACTUATOR_COUNT]`), and have `actuator_dispatch_bus_rx`
iterate only `s_bus_slot_count[bus]` entries instead of all 25. Full diff in
[per-bus-rx-index.patch](patches/per-bus-rx-index.patch); summary of the
shape:

```c
void actuator_rebuild_bus_index(void)
{
	memset(s_bus_slot_count, 0, sizeof(s_bus_slot_count));
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled) continue;
		can_bus_id_t bus = actuator_table[i].bus;
		if ((uint8_t)bus >= CAN_BACKEND_COUNT) continue;
		s_bus_slot_idx[bus][s_bus_slot_count[bus]] = i;
		s_bus_slot_count[bus]++;
	}
}
```

`actuator_dispatch_bus_rx` then loops `for (k = 0; k < count; k++) { i =
s_bus_slot_idx[bus][k]; ... }` — identical body otherwise, same protocol
dispatch, same `damiao_had_rx` post-loop (also now bus-scoped instead of
full-table).

**Zero protocol change.** This only changes which indices get visited and in
what order (ascending slot index within a bus, same as today since the table
is built in slot order) — no behavior differs for any enabled slot.

## Call-site wiring Cursor needs to add (patch only covers `actuator.c`/`.h`)

The patch calls `actuator_rebuild_bus_index()` once, at the end of
`actuator_init()` — correct for boot, insufficient for runtime CFG changes.
`actuator_table` is also mutated at these sites, all in
[`App/Src/plant/plant_config_nvm.c`](../App/Src/plant/plant_config_nvm.c),
which the patch does **not** touch (out of this agent's `docs/`+`scripts/`
lane):

| Site | Line (as read) | Why it needs the rebuild call |
|---|---|---|
| `plant_config_load_factory_defaults()` | `plant_config_nvm.c:319-357` | Writes `actuator_table[slot]` directly for the default 25-slot layout |
| `nvm_slots_to_table()` (called from `plant_config_nvm_load()`) | `plant_config_nvm.c:273-285`, called at `:367` | Overwrites the whole table from flash on `plant_config_init()` / `PLANT_CFG_OP_LOAD` |
| `plant_config_on_command()`, `case PLANT_CFG_OP_SET` | `plant_config_nvm.c:416-434` | Runtime per-slot CFG SET — this is what `ensure_product_cfg()` in the bench scripts calls over CDC before every matrix run |

Simplest correct wiring: call `actuator_rebuild_bus_index()` unconditionally
at the end of the `PLANT_CFG_OP_SET` case, at the end of
`plant_config_load_factory_defaults()`, and at the end of
`nvm_slots_to_table()` (or its caller `plant_config_nvm_load()`). Each
rebuild is O(25) — trivial next to a CFG command that's already going out
over USB — so there's no need for a dirty-flag/lazy-rebuild scheme; just
call it every time any of those three write to `actuator_table`.

**Confirmed:** `App/Src/app.c` calls `actuator_init()` at L29, then
`plant_config_init()` at L33 — so the patch's rebuild call inside
`actuator_init()` runs on an all-zeroed table (harmless: index ends up
empty) and is superseded once `plant_config_init()` populates
`actuator_table` via `plant_config_load_factory_defaults()` +
`plant_config_nvm_load()`. This makes the three `plant_config_nvm.c` rebuild
calls above load-bearing, not optional — without them the index stays empty
for the entire boot and `actuator_dispatch_bus_rx` silently stops
processing all RX (would show up immediately as zero feedback in any
matrix run, not a subtle bug, but worth getting right on the first flash
rather than debugging it live).

## Explicit non-goals

- No `RS02_MCP_APPLY_DIV` / post-FB / MCP-only priority changes — this
  sprint's identity lock (see `three_agent_sprint` plan) bans that class of
  change entirely; this patch doesn't touch TX timing or FDCAN÷ at all,
  only RX dispatch fan-out.
- No change to which buses get polled or in what order — `poll_buses`
  bitmask logic in `actuator_apply_desire` is untouched.
- No change to `damiao_had_rx` semantics — still per-slot, still only set
  on a real parsed match.

## How to apply

```powershell
git apply docs/patches/per-bus-rx-index.patch
# then wire the three plant_config_nvm.c call sites above by hand
```

Verified clean against `main`@ current tip via `git apply --check` (dry-run,
no working-tree changes made by this agent).

## Matrix checklist

Same as [rfc-release-build.md](rfc-release-build.md)'s appendix — apply
**after** the Release-build comparison lands, so this run isolates the
dispatch-index effect instead of conflating it with the `-O0`→`-Os` jump:

- Host rates: 40, 100, 200, 500 Hz.
- `--skip-real --skip-cali`.
- 3 trials/rate.
- Compare against the Release-baseline bench doc from
  [rfc-release-build.md](rfc-release-build.md), not the original Debug
  07-23 doc, so only one variable changes per comparison.

## Exit

Patch applies cleanly to current `main` `actuator.c`/`actuator.h` (confirmed
by this agent via `git apply --check`); Cursor adds the three
`plant_config_nvm.c` rebuild calls, rebuilds the same config used for the
Release baseline, re-flashes, and re-runs the matrix — `ok` counts not worse
than the Release baseline, `act_lap` flat or down (relief expected mainly on
CH1/CH2 Damiao phases, since that's where the RX-frame-count × table-scan
product is largest).
