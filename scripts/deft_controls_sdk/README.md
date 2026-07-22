# deft_controls_sdk

Host SDK for the Deft Robotics controls PCB (USB CDC). Canonical call surface:
[`docs/api.md`](../../docs/api.md). Wire: [`docs/host-exchange-v2.md`](../../docs/host-exchange-v2.md)
(672 B layout v2). DEBUG frames: [`docs/host-debug-v1.md`](../../docs/host-debug-v1.md).

Does **not** import `scripts/legacy/`.

## Quick start

```python
from deft_controls_sdk import ControlsPcbHub, ActuatorDesire, find_cdc_port, flash_firmware

print(find_cdc_port())  # COM5 / /dev/ttyACM0

with ControlsPcbHub.connect() as hub:  # auto CDC, or connect("COM5")
    hub.recover()
    hub.start_streaming(hz=40.0)
    hub.set_actuator(0, ActuatorDesire(position=0.2, kp=8.0, kd=0.5), send=False)
    print(hub.telemetry.snapshot())
```

**DEBUG** (CFG / discover / calibrate / soft-DFU enter):

```python
with ControlsPcbHub.connect() as hub:
    table = hub.debug.cfg_get_table()
    hit = hub.debug.discover_robstride(bus=4)
    hub.debug.cfg_set_slot(slot=19, bus=4, protocol=1, motor_id=hit or 0, persist=True)
    hub.debug.calibrate_robstride(bus=4, motor_id=hit)  # shaft free, 24–60 V
```

**Single-actuator channel bringup** (move cable between CH1–CH6, change `--bus`):

```powershell
python rs02_channel_bringup.py --bus 4
python rs02_channel_bringup.py --bus 1 --motor-id 0x70 --skip-cali
```

Runs discover → CFG → ack_lag/lap/fb_hz hold → cali → tiny teleop. PASS/FAIL summary at end.

**USB flash** (no ST-Link):

```powershell
python soft_dfu_flash.py
# or: flash_firmware(confirm=True)
```

**Dashboard:** `python -m deft_controls_sdk.debug_dashboard` → http://127.0.0.1:8765

## Layout

```
controls_pcb_hub.py     # façade
link/exchange/          # 672 B plant + DEBUG frame pack/parse
bench/                  # discover, CFG, calibrate, soft_dfu, metrics
telemetry/              # snapshot / record
debug_dashboard/        # localhost UI
```

