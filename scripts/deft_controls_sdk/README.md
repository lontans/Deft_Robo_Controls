# deft_controls_sdk

Host SDK for the Deft Robotics controls PCB (USB CDC).

## Shape

```text
actions/          ComponentAction / LedAction / ServoAction (plant CMDH)
config/           Profile, slot maps, CFG rows, LED/servo/pdu identity, firmware paths
debug/            hub.debug toolkit (discover, CFG RPC, Soft-DFU, suite)
telemetry/        FB cache / recorder
link/             Connection + 694 B + Desire types
ControlsPcbHub    board USB (slots, hub.debug, telemetry)
HostProxy         component demux → actions.ComponentAction
vbeta/            YAM / deft_vbeta drivers → HostProxy
debug_dashboard/  human UI → Hub (owns COM while open)
```

Lab app + tests: [`../pcb_lab/`](../pcb_lab/).

Docs: [`docs/host-contract.md`](../../docs/host-contract.md), [`docs/integration.md`](../../docs/integration.md).

## Quick start

```python
from deft_controls_sdk import HostProxy, ControlsPcbHub, ActuatorDesire
from deft_controls_sdk.actions import ComponentAction
from deft_controls_sdk.config import yam_product_profile

with HostProxy.connect() as proxy:
    proxy.component("left_arm").hold([0.0] * 7, kp=8.0, kd=0.5)
    proxy.send_once()

# Same ComponentAction type with hub sink (no HostProxy policy):
with ControlsPcbHub.connect() as hub:
    ComponentAction(hub, yam_product_profile(), "base").blank(send=True)
    hub.debug.cfg_get_table()
```

**USB flash:** `python soft_dfu_flash.py`  
**Dashboard:** `python -m deft_controls_sdk.debug_dashboard`  
**Lab:** `python -m pcb_lab inventory` / `python -m pcb_lab doctor`  
**Tests:** `pytest pcb_lab/tests`
