# scripts/legacy — frozen

**Frozen.** Prefer [`../deft_controls_sdk/`](../deft_controls_sdk/README.md) and
[`../../docs/api.md`](../../docs/api.md). Do not extend this tree.

This tree holds the pre-SDK host packages and CLIs (`control_hub/`,
`controls_pcb_host/`, …). After a green **SDK-only** actuator prove-out, the
tree will be gitignored and removed from tracking.

Timing/health: use plant `system[]` (layout v2), not SVD PDU bytes. Standalone
scripts still hardcoding 562 B are marked UNMAINTAINED.

Calibrate (RS02 encoder) remains the only known holdout not in the SDK — use
already-calibrated motors for prove-out, or:

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/control_hub.py calibrate --port COM5 --bus N --id 0x..
```

## Tomorrow — SDK-only prove-out (no `scripts/legacy` imports)

Run from `scripts/` (+ `soft_dfu_flash.py` as needed):

1. Soft-DFU flash current ELF if firmware is stale.
2. `ControlsPcbHub.connect()` (auto-port or explicit).
3. `cfg_get_table` / optional `cfg_set_slot(..., persist=True)` + power-cycle check.
4. `discover_robstride` / `discover_damiao` on real buses.
5. `start_streaming` + `set_actuator` hold on a known slot; FB tracks.
6. `_tmp_mcp_timing_probe.py` all×25 still OK.
7. **Do not** invoke `scripts/legacy/*`.

If that passes: add `scripts/legacy/` to `.gitignore`,
`git rm -r --cached scripts/legacy`, and drop legacy from `api.md` / `bringup.md`
(calibrate note: archived / contact if needed).

## Running a legacy CLI (until retired)

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/control_hub.py --list-ports
```
