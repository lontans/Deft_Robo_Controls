# pcb_lab

Plant lab on **HostProxy**, plus parked tests and deprecated CLIs.

```text
pcb_lab/
  lab.py / __main__.py   → doctor | hold | step | blank
  tests/                 → pytest (offline)
  legacy/                → gitignored local CLIs (optional; not tracked)
```

```powershell
cd scripts
python -m pcb_lab doctor --port COM5
python -m pcb_lab hold --component left_arm --hold-s 3
pytest pcb_lab/tests
```

```text
pcb_lab → HostProxy → ControlsPcbHub → USB
```

YAM / teleop uses `deft_controls_sdk.vbeta` on the same HostProxy — see [`../deft_controls_sdk/README.md`](../deft_controls_sdk/README.md).

**Not** a replacement for `deft_controls_sdk.debug` (`hub.debug` CFG / discover / Soft-DFU).
