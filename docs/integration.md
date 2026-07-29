# Integration — SDK, vbeta, i2rt

Two host stacks share one plant USB/session. Do **not** edit YAMAIMobile for plant glue; keep MotorsBus-shaped APIs.

## Stack A — direct SDK / vbeta

```
PcbArmDriver / PcbPlatformClient / vbeta_smoke
  → ControlsPcbHub session → USB 694 B → STM32 (packs MIT)
```

Live adapters: `scripts/deft_controls_sdk/vbeta/`. Cameras / episode packing stay in vbeta. Smoke (deprecated CLI): `python legacy/vbeta_smoke.py arm|base|neck`, `legacy/vbeta_product_prove.py`.

Parity vs reference I2RT/Feather surfaces: methods YAMAIMobile actually calls are matched. Neck pitch: forward `neck_cmd` unmodified (offset already baked upstream).

**Base ID gap:** product CFG uses `0x01`/`0x02` per CH4–6 rail; some benches answer to spare IDs (`0x70`/`0x74`/…). That is HW/CFG mismatch, not an adapter bug — do not silently remap.

## Stack B — i2rt over UDS

```
i2rt pcb: → UDS → pcb_mit_relay / pcb_bridge → same session → USB → STM32
```

Bridge owns COM; Jetson i2rt clients talk MIT-shaped desires over the socket. Prefer pointing the bridge at **PlantProxy** once it exists (`use_plant_proxy=False` as rollback).

## Platform direction

| Layer | Role |
|-------|------|
| Controls PCB firmware | Thin plant: CFG slots, pack/unpack, kill, Soft-DFU |
| PlantProxy (SDK, near-term) | One COM demux; component MIT API; profile |
| pcb_lab | lerobot-shaped hold/step/doctor/parity (+ DIAG) |
| ROS peripheral drivers | Later — many nodes, one plant proxy |
| YAMAIMobile | lerobot robot object (often under DeftRecorderNode); untouched for this work |

`ros2_control` is a future ROS lane (needs MIT kp/kd); it does not replace the plant client and is not a drop-in for current lerobot.

## Reference checkouts

`docs/deft_vbeta_ref/` submodule pointers (if present) are read-only contract mirrors — never edit from this repo. Prefer living code under `scripts/deft_controls_sdk/vbeta/`.
