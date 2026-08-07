# Talk-through demos (Controls PCB)

Runnable scripts you can narrate live — notebook-level, not REPL paste.

Product story: **parent deft_vbeta / these demos → `HostProxy.set_section` → USB → plant**.  
Lab story (`proxy.actions`, dashboard keys) is separate — see [`../notebooks/`](../notebooks/).

## Setup

```bash
cd ~/deft_vbeta   # or host: …/DeftRoboticsControlsPCB
source .venv/bin/activate          # Jetson
cd external/controls_pcb/scripts   # Jetson submodule
# host: cd scripts

pip install -r requirements.txt    # once
# Jetson also: pip install -e ..   if not already editable-installed
```

One CDC owner: close dashboard / `pcb_lab.debug` / other HostProxy first.

## Run order (product pitch)

```bash
# from scripts/
python demo_scripts/01_board_alive.py --port /dev/ttyACM0
python demo_scripts/02_apply_yam_cfg.py --port /dev/ttyACM0
python demo_scripts/03_product_set_section_idle.py --port /dev/ttyACM0

# optional motion on THIS harness (spare-slot base 22–25), not product wheel IDs:
python demo_scripts/04_bench_base_hold.py --port /dev/ttyACM0
python demo_scripts/04_bench_base_hold.py --port /dev/ttyACM0 --nudge   # motors move
```

Omit `--port` to auto-pick STM32 CDC `0483:5740`. Windows: `--port COM4`.

Each script prints numbered steps, then **releases the port** when the `with HostProxy` block ends (`close()`).

## What each shows

| Script | Mode | Point |
|--------|------|--------|
| `01_board_alive` | scan/status | USB + plant stream healthy |
| `02_apply_yam_cfg` | debug + CFG | Product map into RAM (no torque) |
| `03_product_set_section_idle` | bandwidth | **`set_section("left_arm", IDLE)`** demux |
| `04_bench_base_hold` | bandwidth + bench assembly | Same API on spare-slot `base`; `--nudge` moves |

## Full YAM stack (not scripted here)

`deft_vbeta` → `install_pcb_backend(robot)` → `PcbRobotSession` → same `set_section`.  
See Jetson note `~/deft_vbeta/JETSON_SETUP_C1.md` and `amr/pcb_bridge.py`.
