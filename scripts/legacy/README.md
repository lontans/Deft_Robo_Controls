# scripts/legacy — frozen / pending retire

**Do not extend.** Prefer [`../deft_controls_sdk/`](../deft_controls_sdk/README.md) and
[`../../docs/api.md`](../../docs/api.md).

RobStride encoder calibrate is in the SDK: `hub.debug.calibrate_robstride(...)`.
Retirement checklist: [`../../docs/scripts-hygiene.md`](../../docs/scripts-hygiene.md).

This tree holds the pre-SDK host packages (`control_hub/`, `controls_pcb_host/`, …).
After SDK-only prove-out + bringup examples rewritten, gitignore and
`git rm -r --cached scripts/legacy`.

## Running a legacy CLI (until retired)

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/control_hub.py --help
```

Prefer SDK / `vbeta_*_smoke.py` for new work.
