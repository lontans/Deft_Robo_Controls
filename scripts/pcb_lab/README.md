# pcb_lab

**Charter:** CLI for proving the Controls PCB after flash/bringup.  
**Programmatic API:** always [`deft_controls_sdk`](../deft_controls_sdk/README.md) — `HostProxy`, `actions`, `config`, `hub.debug`. Do not import helpers from `pcb_lab`.

| Surface | Owns |
|---------|------|
| `python -m pcb_lab` | Board: scan / status / leave / flash / images / build |
| `python -m pcb_lab.debug` | CLI alias of `deft_controls_sdk.debug.suite` → **show** · **set** · **test** |
| SDK | Everything notebooks / scripts call |

Works on **Windows** and **Ubuntu/Jetson** (USB CDC). Omit `--port` to auto-pick STM32 CDC `0483:5740`, or pass `COM5` / `/dev/ttyACM0`.

```bash
cd scripts
pip install -r requirements.txt

python -m pcb_lab
python -m pcb_lab scan
python -m pcb_lab status                  # or --port COM5 / --port /dev/ttyACM0
python -m pcb_lab flash                   # Soft-DFU; Linux: see scripts/README.md

python -m pcb_lab.debug show --pcb
python -m pcb_lab.debug set --cfg
python -m pcb_lab.debug test              # board + peripheral entry
python -m pcb_lab.debug test --actuators  # actuators menu only
python -m pcb_lab.debug test --servos     # servos menu only
python -m pcb_lab.debug test --led        # LED menu only
python -m pcb_lab.debug test --inventory --preset bench --buses 5,6
```

Bare ``test``: board (``p`` doctor · ``c``/``C`` CFG · ``i`` inventory · ``o`` observe)
then peripherals (``a`` actuators · ``s`` servos · ``l`` led). Flags open that
peripheral menu directly.


## Notebooks

Use the SDK (see [`deft_controls_sdk/README.md`](../deft_controls_sdk/README.md) quick start). Optional thin façade only if you want it:

```python
from pcb_lab.lab import LabRobot  # wraps HostProxy; prefer HostProxy directly
```

## Layout

```text
pcb_lab/
  lab.py     # board CLI + optional LabRobot façade
  board.py   # scan / status / flash / menu
  debug/     # __main__ → suite.main (CLI alias only)
  tests/     # offline SDK + lab tests
```
