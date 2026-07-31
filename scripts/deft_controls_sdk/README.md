# deft_controls_sdk

Host SDK for the Deft Robotics controls PCB (USB CDC).

## Shape

```text
actions/          PlantAction → ActuatorAction / LedAction / ServoAction / PduLinkAction
                  + TeleopEngine / operate (spin, move_arm) / cfg_identity (NVM gate)
config/           Assembly + ActuatorProfile/ServoProfile; Profile demux shim; CFG rows
debug/            hub.debug toolkit (discover, CFG RPC, Soft-DFU, suite)
                  suite/workshop.py = Assembly workshop (bare `test`)
telemetry/        FB cache / recorder
link/             Connection + 694 B + Desire types
ControlsPcbHub    board USB (slots, hub.debug, telemetry)
HostProxy         profile demux → actions.ActuatorAction
vbeta/            YAM / deft_vbeta drivers → HostProxy
debug_dashboard/  human UI → Hub (owns COM while open; can import actions.teleop later)
ros/              optional ROS 2 adapter (HostProxy as a node); needs rclpy
                  only when imported — python -m deft_controls_sdk.ros
```

Lab app + tests: [`../pcb_lab/`](../pcb_lab/).

Docs: [`docs/host-contract.md`](../../docs/host-contract.md), [`docs/integration.md`](../../docs/integration.md).

## Quick start

```python
from deft_controls_sdk import HostProxy, ControlsPcbHub, ActuatorDesire
from deft_controls_sdk.actions import ActuatorAction
from deft_controls_sdk.config import yam_product_profile

with HostProxy.connect() as proxy:
    proxy.actuators("left_arm").hold([0.0] * 7, kp=8.0, kd=0.5)
    proxy.send_once()

# Same ActuatorAction type with hub sink (no HostProxy policy):
with ControlsPcbHub.connect() as hub:
    ActuatorAction(hub, yam_product_profile(), "base").blank(send=True)
    hub.debug.cfg_get_table()
```

**USB flash:** `python soft_dfu_flash.py`  
**Dashboard:** `python -m deft_controls_sdk.debug_dashboard`  
**Lab:** `python -m pcb_lab inventory` / `python -m pcb_lab doctor`  
**Tests:** `pytest pcb_lab/tests`

## ROS 2 (optional)

`ros/` wraps one `HostProxy` as a node — actuators/led/servo topics over the
same `ActuatorAction`/`LedAction`/`ServoAction` every other host app uses.
Requires `rclpy` + `sensor_msgs`/`std_msgs` (ROS 2 workspace or `pip install
rclpy`); nothing else in this SDK needs them.

```powershell
python -m deft_controls_sdk.ros --help
python -m deft_controls_sdk.ros --profile product
```

CFG / discover / cal are not exposed on this node — use `mode="debug"` via
`pcb_lab` / `hub.debug` for that.
