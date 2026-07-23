# RFC: build/flash Release (`-Os`) instead of Debug (`-O0`)

Status: ready to execute. Author: Agent 2 (Claude), offline — no build/flash
performed by this agent. Executor: Cursor (owns `App/`/`Core/`, COM5, soft-DFU).

## Problem

Every act_lap / fb_hz number in
[bench-load-matrix-2026-07-22.md](bench-load-matrix-2026-07-22.md) and
[bench-load-matrix-2026-07-23.md](bench-load-matrix-2026-07-23.md) was
measured on the **Debug** configuration, which builds at `-O0`. A **Release**
configuration already exists in [`.cproject`](../.cproject)
(`com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.release.1049488396`,
optimization level already set to `-Os` for both the C and C++ compiler
tools) but has **never been built** — no `Release/` directory exists
anywhere in the tree, only `Debug/DeftRoboticsControlsPCB.elf`. This is a
zero-logic-risk lever (same source, different codegen) that no one has
pulled yet, and it's plausible it moves act_lap more than any SPI/dispatch
hygiene change, given how float- and memcpy-heavy the ×25 path is (MIT
pack/unpack, CRC16, host-position interp).

## What to do

1. **CubeIDE:** Project → Build Configurations → Set Active → **Release**,
   then Build (or Project → Build All with Release active). This generates
   `Release/DeftRoboticsControlsPCB.elf` alongside the existing `Debug/`
   output — it does not touch `Debug/`.
2. **Soft-DFU prefers Release, Debug is the explicit fallback** —
   `default_firmware_elf()` in
   [`scripts/deft_controls_sdk/bench/soft_dfu.py`](../scripts/deft_controls_sdk/bench/soft_dfu.py)
   returns `Release/DeftRoboticsControlsPCB.elf` when that file exists,
   otherwise `Debug/DeftRoboticsControlsPCB.elf`.

   ```powershell
   # After a Release CubeIDE build — no --image needed:
   python scripts/soft_dfu_flash.py
   # Force Debug while validating Release, or if Release/ is absent:
   python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
   # Or pin Release explicitly:
   python scripts/soft_dfu_flash.py --image Release/DeftRoboticsControlsPCB.elf
   ```

3. **Re-run the matrix** unchanged (same script/args as the 07-23 baseline):

   ```powershell
   cd scripts
   $env:PYTHONIOENCODING='utf-8'
   python _tmp_load_matrix_report.py --port COM5 --skip-real --skip-cali --trials 3 --seconds 8
   ```

4. **Write `docs/bench-load-matrix-release-<date>.md`** with the same table
   shape as the existing bench docs (§A real CH6, §B ×25 rx_sim, bandwidth
   baseline by bus group), so it's a direct row-for-row comparison against
   [bench-load-matrix-2026-07-23.md](bench-load-matrix-2026-07-23.md).

## Success metric

§B (×25 rx_sim + DXL + LED) `act_lap` mean/peak should drop from the 07-23
Debug baseline:

| tx Hz | Debug act_lap mean (07-23) |
|---:|---:|
| 40 | 3.6 |
| 100 | 3.7 |
| 200 | 4.0 |
| 500 | 3.5 (2/3 ok — 500 Hz already gates on ack_lag, not act_lap) |

Pass condition: Release act_lap mean is lower at every rate **and** the `ok`
column is not worse than Debug at the same rate (40/100/200 currently 3/3;
500 currently 2/3 for an unrelated coalesce-lag reason — don't let 500 Hz
regress further, but don't expect this change to fix that particular gate
either, since it's `ack_lag`, not lap time).

If `-Os` code-sizes something into failing a timing assumption (unlikely,
but `-O2`/`-Os` can occasionally reorder volatile access in ways `-O0` never
exercised) — check first for a missing `volatile` on anything read/written
across the ISR/main-loop boundary before assuming the optimization itself is
unsafe. Nothing in this RFC proposes touching those qualifiers; if the
Release build behaves differently in a way that looks like a memory-ordering
bug rather than "just faster," that's a separate, real bug this exposed, not
a reason to stay on `-O0`.

## Non-goals

- No source changes. This RFC is build-configuration only.
- Not a request to make Release the new default for day-to-day dev
  iteration (Debug's `-g0`/no-optimize is presumably still useful when
  actually debugging with a probe) — just to get one clean comparison
  number, and to make Release the flashed image for load-matrix runs going
  forward if it wins.
- Does not replace or block [rfc-per-bus-rx-index.md](rfc-per-bus-rx-index.md)
  — apply this one first per the matrix checklist below so the RX-index
  matrix run has a clean, already-Release baseline to compare against
  instead of conflating two changes in one before/after.

## Related: 500 Hz coalesce + stagger follow-on

Release improves mean `act_lap` but does **not** fix the 500 Hz
`cmd_seq_lag_p95` gate: `host_link_poll_rx()` intentionally coalesces queued
plant images to the newest per lap (see comment in `host_link.c`), so
intermediate `seq` values never ack. Long plant bursts make that worse.
Follow-on: [rfc-stagger-robstride-maintain.md](rfc-stagger-robstride-maintain.md)
(flatten periodic maintain-enable peaks that starve the USB drain task).
This RFC stays build-config only.

## Matrix checklist (shared with rfc-per-bus-rx-index.md)

- Host rates: 40, 100, 200, 500 Hz.
- `--skip-real --skip-cali` (no live RS02 needed for this comparison — ×25
  rx_sim is the load of interest).
- 3 trials/rate, matching prior bench docs.
- Leave COM5 idle before/after — announce `COM5: Cursor` while running per
  the sprint plan's collision rule.
