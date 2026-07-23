# Scripts hygiene — `_tmp_*` and `legacy/`

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

## `scripts/_tmp_*` triage

| File | Keep? | Action |
|------|-------|--------|
| `_tmp_load_matrix_report.py` | **Yes** | Canonical ×25+DXL+LED matrix; promote later to `bench/load_matrix_report.py` |
| `_tmp_bus6_real_hw.py` | **Yes** | Shared teleop/hold helper for matrix; promote with matrix |
| `_tmp_mcp_timing_probe.py` | **Yes** (for now) | Product CFG helper + light probe; fold `ensure_product_cfg` into SDK when convenient |
| `_tmp_rate_rx_sweep.py` | Maybe | Bandwidth/rate sweep; keep until matrix covers rates |
| `_tmp_image_id_verify.py` | Thin | One-shot seq verify; delete after next green matrix or fold into tests |
| `_tmp_dxl_one.py` | Thin | Prefer `vbeta_neck_led_smoke.py` |
| `_tmp_*_run.log` / binary logs | **No** | gitignore; do not commit |

**Do not** delete matrix/bus6 while Claude’s equal-rate work depends on them.

## `scripts/legacy/` retirement

Status: **frozen**; SDK has RobStride calibrate (`hub.debug.calibrate_robstride`).
Legacy calibrate note in `legacy/README.md` is stale.

### Prove-out checklist (before `git rm`)

1. Soft-DFU current ELF.
2. `ControlsPcbHub` connect + `cfg_get_table` / Damiao+RS discover.
3. Streaming hold on known slots; FB tracks.
4. Load matrix (`_tmp_load_matrix_report.py --skip-real`) green on current firmware.
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
