# scripts/legacy — frozen / pending retire

**Do not extend.** Prefer [`../deft_controls_sdk/`](../deft_controls_sdk/README.md) and [`../../docs/`](../../docs/README.md). Near-term product path: PlantProxy + `pcb_lab` ([integration.md](../../docs/integration.md)).

This tree holds:

1. **Pre-SDK packages** — `control_hub/`, `controls_pcb_host/`, `controls_hub_api/`, …
2. **Former root CLIs** (moved 2026-07-29) — bringup, continuous, PDB prove, vbeta smoke wrappers, teleop helpers, Jetson utilities

After PlantProxy / `pcb_lab` prove-out, gitignore and `git rm -r --cached scripts/legacy`.

## Run a legacy CLI

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/control_hub.py --help
python legacy/vbeta_smoke.py arm --hold
python legacy/yam_continuous_all.py --help
python legacy/rs02_channel_bringup.py --bus 4
python legacy/pdb_uart_sim.py --help
```

Jetson remote launches (dashboard / `launch_continuous.py`) still deploy selected files as **basenames** under remote `…/scripts/` so existing `pkill` / `cd` paths keep working.

## Inventory (high level)

| Cluster | Examples |
|---------|----------|
| Soft-realtime cruise | `yam_continuous_all.py`, `launch_continuous.py`, `stop_can.py` |
| Channel / product prove | `rs02_channel_bringup.py`, `damiao_channel_bringup.py`, `vbeta_smoke*.py`, `vbeta_product_prove.py`, `bench_load_matrix.py` |
| PDB | `pdb_uart_sim.py`, `pdb_*_prove.py`, `pdb_plant_integ_test.py` |
| Teleop / Quest / mouse | `mouse_*.py`, `quest_*.py`, `yam_*_clear_*.py`, `mission_impossible.py` |
| Jetson helpers | `jetson_*.py`, `lift_canopen_discover.py`, `base_bus56_lab.py`, `dxl_one.py` |
| Old hub packages | `control_hub*`, `controls_pcb_host*`, `controls_hub_*` |
| Scratch | `_tmp_*`, `tmp_runners/` |
