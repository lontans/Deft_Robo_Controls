# scripts/

Thin living host surface for the Controls PCB.

## Living

| Path | Role |
|------|------|
| [`deft_controls_sdk/`](deft_controls_sdk/README.md) | Hub + HostProxy + vbeta + **debug** + dashboard |
| [`pcb_lab/`](pcb_lab/README.md) | Lab app + **tests** (+ optional local `legacy/`, gitignored) |
| [`soft_dfu_flash.py`](soft_dfu_flash.py) / [`.sh`](soft_dfu_flash.sh) | Soft-DFU entry |
| [`udev/`](udev/) | Linux udev rules |
| [`requirements.txt`](requirements.txt) / [`requirements-dev.txt`](requirements-dev.txt) | Deps |

```powershell
cd scripts
pip install -r requirements.txt
python soft_dfu_flash.py
python -m deft_controls_sdk.debug_dashboard --port COM5
python -m pcb_lab doctor
pytest pcb_lab/tests
```

## pcb_lab layout

```text
pcb_lab/
  lab.py          # hold / step / blank / doctor → HostProxy
  tests/          # offline SDK + lab tests
  legacy/         # gitignored local CLIs (not in repo)
```

Docs: [`../docs/README.md`](../docs/README.md).
