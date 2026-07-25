# Scripts hygiene — `_tmp_*`, `legacy/`, and `docs/legacy/`

Offline housekeeping. No board required.

## Scratch / dumps (do not commit)

Already in `.gitignore`:

- root `/*.csv`, `/tmp_*.txt`, `/RTOS_BRINGUP_HANDOFF.txt`, `.tmp_recover/`
- `docs/handoff-*.md` (agent resume notes — durable history lives in `bringup.md` / `lessons.md` / bench docs)
- `scripts/_tmp_*.log`, `scripts/_tmp_*.txt`, `scripts/.deft_session/`

Wipe local session dumps anytime:

```powershell
Remove-Item -Recurse -Force scripts\.deft_session -ErrorAction SilentlyContinue
Remove-Item -Force scripts\_tmp_*.log, scripts\_tmp_*.txt -ErrorAction SilentlyContinue
```

## Canonical live ops CLIs (2026-07-24 streamline)

| File | Role |
|------|------|
| `yam_continuous_all.py` | Multi-peripheral cruise driver |
| `launch_continuous.py` | Remote one-shot continuous prove (SSH) |
| `stop_can.py` | Blank actuators + DIAG after hard-kill |
| `pdb_uart_sim.py` | PDU UART peer simulator |
| `soft_dfu_flash.py` / `.sh` | Soft-DFU entry |
| `bench_load_matrix.py` | Host-rate / scenario load matrix |
| `vbeta_smoke.py` | Unified arm/base/neck smoke (`arm\|base\|neck`) |
| `vbeta_product_prove.py` | Product-CFG ladder (one session) |
| `vbeta_smoke_lib.py` | Shared smoke implementations |
| `base_bus56_lab.py` | Base bus5/6 interactive lab |
| `dxl_one.py` | Single-DXL range check CLI |
| `pdb_led_live_prove.py` / `pdb_plant_integ_test.py` / `pdb_softkill_handshake_prove.py` | PDB prove trio |
| `rs02_channel_bringup.py` / `damiao_channel_bringup.py` | Channel bringup CLIs |
| `yam_arm_clear_range.py` / `yam_dxl_clear_teleop.py` | Clear-range characterization |
| `jetson_estop_*.py` / `jetson_uart_listen.py` | Jetson GPIO/UART helpers |

Shared helpers now live in the SDK (import these, do not duplicate):

- `deft_controls_sdk.bench.servo_fb.sample_servo_fb`
- `deft_controls_sdk.bench.rs02_motion` (`rs02_resolve_start`, `sample_position`, …)

Deprecated `_tmp_*.py` shims still forward to the promoted names for one release of muscle memory; prefer the table above.

## Archived this pass → `scripts/legacy/`

Overlaps retired after continuous + product prove + bus56 lab cover the same gates:

- `yam_rig_smoke_suite.py`
- `yam_base_rotate_prove.py`
- `dxl_servo_clear_range.py` (canon DXL clear = `yam_dxl_clear_teleop.py`)
- `yam_arm_clear_teleop.py` (canon arm clear = `yam_arm_clear_range.py`)
- `_tmp_jetson_gpio_diag.sh`, `_tmp_jetson_pinmux_hold.sh` (use `jetson_estop_*`)

Earlier archives: `tmp_runners/`, `_tmp_bus6_real_hw.py`, `_tmp_jetson_start_paced_sim.py`.

## `docs/legacy/` — dated benches and superseded wire docs

See prior passes; load-matrix dated benches and host-exchange v1/v2 live there.

## `scripts/legacy/` retirement

Status: **frozen**; SDK has RobStride calibrate (`hub.debug.calibrate_robstride`).
Legacy calibrate note in `legacy/README.md` is stale.

### Prove-out checklist (before `git rm`)

1. Soft-DFU current ELF.
2. `ControlsPcbHub` connect + `cfg_get_table` / Damiao+RS discover.
3. Streaming hold on known slots; FB tracks.
4. Load matrix on current firmware — `python bench_load_matrix.py --port COM5 --scenario all`.
5. Zero grep of production docs/scripts importing `scripts/legacy` (bringup still references legacy CLIs — rewrite those lines first).
