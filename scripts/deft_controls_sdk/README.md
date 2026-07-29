# deft_controls_sdk

Host SDK for the Deft Robotics controls PCB (USB CDC).

## Shape

```text
Hub (controls_pcb_hub.py)  →  board USB (slots, hub.debug, telemetry)
HostProxy (host_proxy.py)  →  components (left_arm / base / …)
vbeta/                     →  YAM / deft_vbeta drivers → HostProxy
debug/                     →  hub.debug toolkit (discover, CFG, Soft-DFU)
link/                      →  Connection + 694 B + Desire types (+ cubemars_mit)
telemetry/                 →  live FB cache / optional logs
pdb/                       →  power-board kill helpers
debug_dashboard/           →  human UI → Hub (owns COM while open)
```

Lab app + tests + deprecated CLIs: [`../pcb_lab/`](../pcb_lab/).

Docs: [`docs/host-contract.md`](../../docs/host-contract.md), [`docs/integration.md`](../../docs/integration.md).

## Quick start

```python
from deft_controls_sdk import HostProxy, ControlsPcbHub, ActuatorDesire

with HostProxy.connect() as proxy:
    proxy.component("left_arm").hold([0.0] * 7, kp=8.0, kd=0.5)
    proxy.send_once()

with ControlsPcbHub.connect() as hub:
    hub.debug.cfg_get_table()
```

**USB flash:** `python soft_dfu_flash.py`  
**Dashboard:** `python -m deft_controls_sdk.debug_dashboard`  
**Lab:** `python -m pcb_lab doctor`  
**Tests:** `pytest pcb_lab/tests`
