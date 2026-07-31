# Host contract

Fixed **694-byte** images both directions (USB CDC or UART). Source of truth: `App/Inc/host/host_exchange_schema.h`, `scripts/deft_controls_sdk/link/exchange/wire_layout.py`.

## Link mode (`stm32_mode`)

Chosen at `ControlsPcbHub.connect(..., mode=...)`. Change mode only by **disconnect + reconnect**. Distinct from `mcu_state` (safety: NORMAL / RECOVERY / ESTOP) and from **`plant_apply`** (observe vs control).

| `stm32_mode` | SDK `mode=` | Wire behavior |
|-------------:|-------------|---------------|
| **0** | `bandwidth` | Plant `CMDH`/`HBHF` only; full rate; `pdb[630+]` = **PDU mirror** |
| **1** | `debug` | Plant frames continue; host may also send **debug lanes** (`DBGC`/`DBGF` lane map); `hub.debug.*` allowed |
| **2** | Soft-DFU enter | Leave app CDC → ROM DFU (preferred). Legacy mailbox tag `DFU!` still accepted |

**Bandwidth:** never arm debug lanes — timing metrics must stay trustworthy.

**Soft-DFU:** enter only via plant `CMDH` with `stm32_mode=2` (or legacy `DFU!` tag).
Never decode Soft-DFU from a debug-lanes `DL` header — those bytes overlay the
plant system word and accidentally look like mode 2.

Wire packing: `stm32_mode` lives in plant **system command** word bits **9–10** (after `mcu_state` + rx_sim bits). Feedback echoes it in `system.reserved0` bits **1:0**. Echo updates when CMDH is **USB-RX'd** (not only on TIM6 apply) so reconnect after debug does not leave a sticky mode; Soft-DFU **enter** still happens only on plant apply.

### `plant_apply` (observe vs control)

System command **wire bit 11** (`system.reserved` bit 6):

| Value | Meaning |
|------:|---------|
| **0** | Observe — plant stream OK; do **not** mount actuator desires or tear down a bench lease |
| **1** | Control — mount/apply desires (still gated by lease / probe / host_stale) |

Replaces `mcu_state=DIAG_ONLY` as the observe toggle. Legacy hosts that still send `mcu_state=2` (DIAG_ONLY) are treated as `plant_apply=0`. Feedback `plant_block=4` is `apply_off` (alias of the old `diag_only` code).

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
| 12 | 32 | system — health, timing, seq readbacks, kill mirror, **stm32_mode** |
| 44 | 572 | `actuator_*[26]` — 22 B each (20 B MIT + 2 B meta) |
| 616 | 12 | `servos[2]` |
| 628 | 2 | `leds[1]` |
| 630 | 64 | `pdb[]` — **power-board / PDU mirror on PLANT** (never DEBUG tags) |

Factory CFG slot budget: CH1×8, CH2×8, CH3×4, CH4–6×2 (= 26).

### Actuator slot (22 B)

20 B MIT desire/state (pos, vel, kp, kd, torque — SI) + 2 B meta. Feedback meta packs protocol/bus/motor_id/flags; command meta reserved (host writes 0).

### Rates

Host stream ~30–200 Hz design point; MCU plant **500 Hz** hold-last. USB FS headroom is comfortable at 694 B duplex in that band.

## Debug lanes frames (mode 1)

Exclusive RPC (discover, CFG, cal, …) without overlaying plant `pdb[]`.

| | Command | Feedback |
|--|---------|----------|
| Magic | `0x44424743` (`DBGC`) | `0x46424744` (`DBGF`) |

### Debug-lanes v1 map (when header tag = `DL\x01`)

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 12 | image header |
| 12 | 6 | debug-lanes header: `'D' 'L' ver=1`, flags, `arm_mask` LE u16 |
| 18 | 320 | lanes 0..9 × 32 B |
| 338 | … | pad to 694 B |

| Lane | Subsystem |
|-----:|-----------|
| 0 | RobStride |
| 1 | CubeMars |
| 2 | ZeroErr |
| 3 | Damiao |
| 4 | SK9822 |
| 5 | Servo / DXL |
| 6 | PDU lab |
| 7–9 | reserved |

`arm_mask` bit *i* = lane *i* armed. FW/host ignore un-armed lanes. Plant apply trusts actuator desires only on **CMDH** frames.

Lane 0 (RobStride) / lane 3 (Damiao) / lane 7 (CFG) use the same 32 B layouts previously carried in the DEBUG mailbox at offset 630 (tags `RS2`, `DM0`, `CFG`, …) so parsers can be shared during migration.

**Multi-bus discover (RS2):** `SESSION_BEGIN` data[5] = `bus_mask` (bit0=CH1 … bit5=CH6). FW TX enable/promisc on all masked buses, one round-robin listen, progress DBGF hits with data[27]=host bus 1..6. Damiao session mask uses data[9] (data[5] remains master_id); Damiao discover is still per-bus on the host until a multi-bus ID_SWEEP lands.

### Legacy mailbox (deprecated)

Old path: tags in `pdb[0..31]` at offset 630 on `DBGC`/`DBGF`. Still accepted by FW when debug-lanes header is absent; **SDK host TX no longer emits this path**. Soft-DFU tag `DFU!` remains a deprecated alias for mode 2.

Hardware inventory: `python -m pcb_lab inventory` / `hub.debug.inventory(preset=…)` — DEBUG discover over lanes (ID ranges required) plus servos/PDU.

## Soft-DFU (USB flash, no ST-Link)

```powershell
python scripts/soft_dfu_flash.py
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
python scripts/soft_dfu_flash.py --require-usb-dfu   # prove loops
python scripts/soft_dfu_flash.py scan
```

Pass = `flash ok — CDC at …` **without** `(SWD)`.

1. Host connects / sends plant or DEBUG frame with **`stm32_mode=2`**, or legacy DEBUG tag `DFU!` on app CDC (`0483:5740`).
2. Firmware sets option byte **nBOOT0=0**, resets → ROM DFU (`0483:DF11`).
3. Host programs ELF/BIN (CubeProgrammer / `dfu-util`).
4. AN3156 Leave → trampoline `0x0803F800` restores **nBOOT0=1** → app CDC.

Soft MEMRMP into system memory is unreliable here; option-byte boot is the supported path. ST-Link = recovery only. App CDC must be `usbser` (COM port), not WinUSB.

## Host API (SDK)

```python
from deft_controls_sdk import ControlsPcbHub, HostProxy, ActuatorDesire
from deft_controls_sdk.actions import ActuatorAction
from deft_controls_sdk.config import yam_product_profile

# Plant / bandwidth — no debug-lanes frame
with ControlsPcbHub.connect("COM5", mode="bandwidth") as hub:
    hub.start_streaming(hz=200.0)

# Named plant motion (shared ActuatorAction; proxy or hub sink)
with HostProxy.connect("COM5", mode="bandwidth") as proxy:
    proxy.actuators("left_arm").hold([0.0] * 7, kp=8.0, kd=0.5)

# Debug — debug_lanes + hub.debug.*
with ControlsPcbHub.connect("COM5", mode="debug") as hub:
    ids = hub.debug.discover_robstride_all(bus=1)
```

- One COM owner; prefer `send=False` while streaming.
- Plant top-level on hub; `hub.debug` requires `mode="debug"`; `hub.telemetry` = FB cache.
- **actions** = plant behaviour (`ActuatorAction` / LED / servo / PDU link); **config** = profiles & identity; **debug** = board RPC (may call actions for normal behaviour).
- Flash host **and** firmware together after a layout bump.

Package layout: `actions/` · `config/` · `debug/` · `telemetry/` · `link/` · façades `ControlsPcbHub` / `HostProxy`; lab = `pcb_lab/`.
