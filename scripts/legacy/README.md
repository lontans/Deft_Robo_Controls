# scripts/legacy — frozen host stack

This tree holds the pre-`deft_controls_sdk` host packages and CLIs:

- `control_hub/`, `controls_pcb_host/`, `controls_hub_controller/`, `controls_hub_api/`
- Top-level bench scripts (`control_hub.py`, `damiao_scan.py`, teleop helpers, …)
- Older unit tests under `tests/`

**Do not extend this tree.** New host work goes in [`../deft_controls_sdk/`](../deft_controls_sdk/README.md).

```powershell
cd scripts
python -m deft_controls_sdk.debug_dashboard --port COM5
# or
python -c "from deft_controls_sdk import ControlsPcbHub"
```

Paths inside these packages still assume they live under `scripts/` on `sys.path`. To run a legacy CLI:

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/control_hub.py --list-ports
```
