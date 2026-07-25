# Scripts hygiene — `_tmp_*`, `legacy/`, and `docs/legacy/`

Offline housekeeping. No board required.

## Scratch / dumps (do not commit)

Already in `.gitignore`:

- root `/*.csv`, `/tmp_*.txt`, `/RTOS_BRINGUP_HANDOFF.txt`, `.tmp_recover/`
- `docs/handoff-*.md` (agent resume notes — durable history lives in `bringup.md` / `lessons.md` / bench docs)
- `scripts/_tmp_*.log`, `scripts/.deft_session/`

Wipe local session dumps anytime:

```powershell
Remove-Item -Recurse -Force scripts\.deft_session -ErrorAction SilentlyContinue
Remove-Item -Force scripts\_tmp_*.log -ErrorAction SilentlyContinue
```

## `scripts/_tmp_*` — current (2026-07-24)

| File | Keep? | Why |
|------|-------|-----|
| `yam_continuous_all.py` | **Yes** (not `_tmp_`) | Continuous multi-peripheral driver — Claude 2 / live ops core |
| `pdb_uart_sim.py` | **Yes** (not `_tmp_`) | PDU UART simulator used by every live prove path |
| `soft_dfu_flash.py` | **Yes** — Cursor owns | Soft-DFU entrypoint; do not gut/edit outside Cursor's Soft-DFU work |
| `_tmp_stop_can.py` | **Yes** | One-shot blank-actuators + DIAG recovery after a killed session |
| `_tmp_launch_continuous.py` | **Yes** | Launches `yam_continuous_all.py` with the standard bench flags |
| `_tmp_base_bus56_lab.py` | **Yes** | Base RS bus5/6 lab — `--prove-360` / `--tx-smoke` / `--fix-74` harness |
| `_tmp_pdb_led_live_prove.py` | **Yes** (PDB prove trio) | Live PDU/LED prove |
| `_tmp_pdb_plant_integ_test.py` | **Yes** (PDB prove trio) | Plant + PDU integration prove |
| `_tmp_pdb_softkill_handshake_prove.py` | **Yes** (PDB prove trio) | Soft-kill handshake prove |
| `_tmp_dxl_one.py` | **Yes — load-bearing, not a one-off** | Single-DXL range check; `sample_servo_fb()` is imported by both `_tmp_pdb_plant_integ_test.py` and `_tmp_pdb_softkill_handshake_prove.py` (the kept PDB prove trio). An earlier pass in this doc called it a delete candidate — deleting it would break those two keepers, so it stays until `sample_servo_fb` is promoted into the SDK and both callers are repointed. |
| `_tmp_jetson_start_paced_sim.py` | Keep for now | Starts a paced `pdb_uart_sim` on the Jetson over SSH for Track B hygiene/prove; small, single-purpose, not folded into the runners below |

**Do not** delete the PDB prove trio or `_tmp_dxl_one.py` while Claude 2's live-prove work depends on them.

### Archived this pass → `scripts/legacy/`

- `_tmp_bus6_real_hw.py` — real-HW ×25 rx_sim + RS02 + DXL + LED load harness. Zero importers left in `scripts/`, and its own top-level `from _tmp_mcp_timing_probe import ensure_product_cfg` already points at a file that no longer exists — it was dead (import-broken) before this pass. Kept as reference, not deleted.

### Archived this pass → `scripts/legacy/tmp_runners/`

Five near-duplicate Windows→Jetson SSH deploy-and-run wrappers, folded into one archive folder instead of one helper (see [`scripts/legacy/tmp_runners/README.md`](../scripts/legacy/tmp_runners/README.md) for the fold-later plan): `_tmp_run_prove360.py`, `_tmp_poll_prove360.py`, `_tmp_run_tx_smoke.py`, `_tmp_run_fix74.py`, `_tmp_check_recording.py`.

### Already gone (stale entries removed from this doc)

A prior version of this table tracked `_tmp_load_matrix_report.py`, `_tmp_mcp_timing_probe.py`, `_tmp_rate_rx_sweep.py`, and `_tmp_image_id_verify.py` as keepers. None of the four exist anywhere in the tree anymore (already deleted/promoted before this pass) — `docs/debug_api.md` §"Related, narrower probes" still shows example commands for three of them plus the now-archived `_tmp_bus6_real_hw.py`; that section needs a rewrite by whoever next touches `debug_api.md` (out of scope for a hygiene-only pass).

## `docs/legacy/` — dated benches and superseded wire docs

Moved this pass (durable facts already folded into `lessons.md` / `decisions.md` / current bench docs; these are the raw dated records):

- All `docs/bench-load-matrix-*.md` (Debug/Release/rxindex/stagger/actlap variants, 2026-07-22/23)
- `docs/act-lap-bloat-deepdive-2026-07-23.md`
- `docs/handoff-jetson-pdb-uart-debug-2026-07-23.md`
- `docs/host-exchange-v1.md`, `docs/host-exchange-v2.md` — superseded by [`host-exchange-v3.md`](host-exchange-v3.md) (694 B, 26 actuator slots, layout v3; see `decisions.md` ADR-001's 2026-07-24 note)

Left in place (current, per plan): `bench-pdb-sdk-contract-2026-07-24.md`, `bench-pdb-plant-integ-2026-07-23.md`, `bench-pdb-plant-integ-2026-07-24.md`, `pdb-uart-v1.md`, `host-exchange-v3.md`, `ch4-mcp2518-bringup-postmortem.md`, `lessons.md`, `decisions.md`, `api.md`. Also untouched (not named in this pass, not clearly superseded): `bench-pdb-uart-prove-2026-07-23.md`, `bench-rs02-ch6-temp-2026-07-24.md`, `bench-vbeta-arm-2026-07-24.md`, `bench-yam-clear-left-2026-07-24.md`.

Cross-doc links into the moved files were repointed (`architecture.md`, `bringup.md`, `decisions.md`, `debug_api.md`, `pdb-uart-v1.md`, `rfc-release-build.md`, `rfc-stagger-robstride-maintain.md`, `vbeta-pcb-adapter.md`) so nothing 404s. Links *between* two files that both moved into `docs/legacy/` together were left as bare filenames — they still resolve since both sides live in the same directory now.

## `scripts/legacy/` retirement

Status: **frozen**; SDK has RobStride calibrate (`hub.debug.calibrate_robstride`).
Legacy calibrate note in `legacy/README.md` is stale.

### Prove-out checklist (before `git rm`)

1. Soft-DFU current ELF.
2. `ControlsPcbHub` connect + `cfg_get_table` / Damiao+RS discover.
3. Streaming hold on known slots; FB tracks.
4. Load matrix on current firmware (successor to the retired `_tmp_load_matrix_report.py` — none exists yet; write one before using this checklist).
5. Zero grep of production docs/scripts importing `scripts/legacy` (bringup still references legacy CLIs — rewrite those lines first).

### Remove from tracking

```powershell
# After prove-out + bringup/api no longer point at legacy:
Add-Content .gitignore "`nscripts/legacy/"
git rm -r --cached scripts/legacy
# Optional: keep a tarball/tag `archive/legacy-host-YYYY-MM-DD` before delete
```

Update: [bringup.md](bringup.md) teleop examples → SDK / vbeta smokes;
[api.md](api.md) drop legacy pointers; [legacy/README.md](../scripts/legacy/README.md) mark **retired**.
