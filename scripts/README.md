# scripts/

Thin living host surface for the Controls PCB. Prefer the SDK; treat root CLIs as gone.

## Living (keep)

| Path | Role |
|------|------|
| [`deft_controls_sdk/`](deft_controls_sdk/README.md) | Preferred host API (`ControlsPcbHub`, vbeta adapters, Soft-DFU impl, dashboard) |
| [`tests/`](tests/) | Offline SDK tests (`pytest`) |
| [`soft_dfu_flash.py`](soft_dfu_flash.py) / [`.sh`](soft_dfu_flash.sh) | One-liner Soft-DFU entry |
| [`udev/`](udev/) | Linux udev rules for CDC/DFU |
| [`requirements.txt`](requirements.txt) / [`requirements-dev.txt`](requirements-dev.txt) | Deps |
| [`legacy/`](legacy/README.md) | Deprecated CLIs + frozen pre-SDK packages |

```powershell
cd scripts
pip install -r requirements.txt
python soft_dfu_flash.py
python -m deft_controls_sdk.debug_dashboard --port COM5
pytest tests
```

## Deprecated → `legacy/`

Everything that used to sit at `scripts/*.py` (bringup, continuous, PDB prove, mouse/quest teleop, vbeta smoke wrappers, Jetson helpers, …) now lives under [`legacy/`](legacy/README.md). Still runnable for bench continuity; **do not extend**. Near-term replacement: PlantProxy + lerobot-shaped `pcb_lab` (see [`../docs/integration.md`](../docs/integration.md)).

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/vbeta_smoke.py arm --hold
python legacy/yam_continuous_all.py --help
```

Dashboard continuous launch still syncs `legacy/yam_continuous_all.py` (and peers) to a **flat** Jetson `scripts/` tree for path compatibility.

## Not source (ignored / local only)

| Path | Notes |
|------|--------|
| `.deft_session/` | Live telemetry mirror |
| `__pycache__/`, `.pytest_cache/`, `*.egg-info/` | Build/test artifacts |
| `_tmp_*` | Scratch — do not commit |

Docs map: [`../docs/README.md`](../docs/README.md).
