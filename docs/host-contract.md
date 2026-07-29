# Host contract

Fixed **694-byte** images both directions (USB CDC or UART). Source of truth: `App/Inc/host/host_exchange_schema.h`, `scripts/deft_controls_sdk/link/exchange/wire_layout.py`.

## Plant frames (layout v3)

| | Command | Feedback |
|--|---------|----------|
| Magic | `0x434D4448` (`CMDH`) | `0x46424848` (`HBHF`) |
| `layout_version` | `3` | `3` |
| `byte_size` | `694` | `694` |

v2 (672 B) is rejected by `host_command_image_valid()`.

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 12 | header — magic, layout_version, byte_size, seq |
| 12 | 32 | system — health, timing, seq readbacks, kill mirror |
| 44 | 572 | `actuator_*[26]` — 22 B each (20 B MIT + 2 B meta) |
| 616 | 12 | `servos[2]` |
| 628 | 2 | `leds[1]` |
| 630 | 64 | `pdb[]` — power-board mirror on PLANT (mailbox unused) |

Factory CFG slot budget: CH1×8, CH2×8, CH3×4, CH4–6×2 (= 26).

### Actuator slot (22 B)

20 B MIT desire/state (pos, vel, kp, kd, torque — SI) + 2 B meta. Feedback meta packs protocol/bus/motor_id/flags; command meta reserved (host writes 0).

### Rates

Host stream ~30–100 Hz design point; MCU plant **500 Hz** hold-last. USB FS headroom is comfortable at 694 B duplex in that band.

## DEBUG frames

Tagged bench ops (CFG, RS2, DM, DFU, …) use separate frames — same 694 B size.

| | Command | Feedback |
|--|---------|----------|
| Magic | `0x44424743` (`DBGC`) | `0x46424744` (`DBGF`) |

Mailbox at offset **630** (32 B): tags `CFG`, `RS2`, `DM0`, `DFU!`, …. Plant TX never puts tags in `HBHF.pdb`.

Host rules: plant stream = `CMDH`/`HBHF` only; `hub.debug.*` and soft-DFU send `DBGC` and wait for `DBGF`.

SDK: `wire_layout.py`, `link/exchange/bench.py`. Firmware: `host_link.c`.

## Soft-DFU (USB flash, no ST-Link)

```powershell
python scripts/soft_dfu_flash.py
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
python scripts/soft_dfu_flash.py --require-usb-dfu   # prove loops
python scripts/soft_dfu_flash.py scan
```

Pass = `flash ok — CDC at …` **without** `(SWD)`.

1. Host sends DEBUG tag `DFU!` on app CDC (`0483:5740`).
2. Firmware sets option byte **nBOOT0=0**, resets → ROM DFU (`0483:DF11`).
3. Host programs ELF/BIN (CubeProgrammer / `dfu-util`).
4. AN3156 Leave → trampoline `0x0803F800` restores **nBOOT0=1** → app CDC.

Soft MEMRMP into system memory is unreliable here; option-byte boot is the supported path. ST-Link SWD = recovery only. App CDC must be `usbser` (COM port), not WinUSB.

## Host API (SDK)

```python
from deft_controls_sdk import ControlsPcbHub, ActuatorDesire

with ControlsPcbHub.connect("COM5") as hub:
    hub.recover()
    hub.start_streaming(hz=40.0)
    hub.set_actuator(0, ActuatorDesire(position=0.2, kp=8.0, kd=0.5), send=False)
```

- One COM owner; prefer `send=False` while streaming.
- Plant top-level on hub; `hub.debug` = bench lease; `hub.telemetry` = FB cache / recording.
- Flash host **and** firmware together after a layout bump.

Package layout: `ControlsPcbHub` → `link/` → `debug/` → `telemetry/` → `debug_dashboard`; plant demux = `HostProxy`; lab = `pcb_lab/`.
