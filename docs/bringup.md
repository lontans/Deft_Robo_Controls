# Bring-up

## 1. Select host transport (firmware)

Edit `App/Inc/host/host_transport.h` before building:

```c
#define HOST_TRANSPORT_UART 0   // controls PCB: USB CDC (laptop bench)
// #define HOST_TRANSPORT_UART 1   // dev board / Jetson: UART4
```

| Board | `HOST_TRANSPORT_UART` | Physical link |
|-------|----------------------|---------------|
| Controls PCB (laptop) | `0` | USB FS CDC → `COM*` / `/dev/ttyACM*` |
| Dev / Jetson UART | `1` | UART4 PC10/11 @ 115200 8N1 |

Rebuild and flash from STM32CubeIDE (Debug).

## 2. Motor and CAN

`plant_config.c` enables **four** RobStride actuators (`ACTUATOR_COUNT = 4`):

| Slot | Bus | Motor ID | Model |
|------|-----|----------|-------|
| 0 | CH1 | `0x70` | RS02 |
| 1 | CH1 | `0x74` | RS01 (daisy on CH1) |
| 2 | CH2 | `0x73` | RS01 |
| 3 | CH3 | `0x75` | RS01 |

- FDCAN1/2/3 @ 1 Mbit/s, extended IDs, per-bus TX queue + RX ring (depth 128)
- `can_router.c` maps schematic CH2 → `hfdcan3` (PA8/PA15), CH3 → `hfdcan2` (PB12/PB13)
- Activity LEDs: PC7 (CH1), PC6 (CH2), PB15 (CH3)

On boot, `control_loop_init()` starts TIM6 @ 500 Hz. Motors are woken by host bench probes (`--recovery`, calibrate preamble, or plant teleop with prior probe).

## 3. Laptop USB bench (Windows / Linux)

```powershell
pip install -r scripts/requirements.txt
python scripts/host_teleop_laptop_usb.py --list-ports
python scripts/host_teleop_laptop_usb.py --port COM9 --plant-teleop
```

### Plant teleop (`--plant-teleop`) — recommended runtime path

- All four slots in one 562 B frame; MCU applies at **500 Hz** (no RS2 PDU)
- Auto-syncs feedback, **slow homing to 0.00 rad**, then arrow-key velocity on all motors
- Gentle defaults: kp 8–12 (gated — **0 at rest**), 5 rad/s, slow ramps → low bench current
- Keys: **Left/Right** move active bus selection, **0** = all buses, **1/2/3** = CH1/CH2/CH3 only, **r** re-sync, **q** quit

```powershell
# Even gentler
python scripts/host_teleop_laptop_usb.py --port COM9 --plant-teleop --plant-arrow-vel 3 --plant-home-slew 0.15
```

Motors must be woken once per branch before plant teleop (recovery or calibrate on that bus).

### Launch demo (`--launch-seq`)

Sequential capability demo with **15% stagger** outbound and return: **0x70 → 0x74 → 0x73 → 0x75** sweep to end; when **0x75** finishes, **0x70** begins return to 0 with the same stagger.

```powershell
python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq
python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq --launch-ccw   # if direction is wrong
python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq --launch-vel 10
```

### RS2 PDU path — calibrate, discover, single-motor teleop

Uses `pdu.data[0..2] = 'R','S','2'` and `pdu.data[11]` for schematic bus (`1` = CH1 … `3` = CH3). Pauses the 500 Hz actuator loop while an RS2 session is active.

```powershell
# Discover / recovery / scan
python scripts/rs02_can_scan.py --port COM9 --bench-cmds --bus 1 --target 0x70

# Encoder cal on a specific branch
python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 1 --motor-id 0x70
python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 2 --motor-id 0x73
python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 3 --motor-id 0x75

# RS2 arrow teleop (higher kp default — one motor per round-robin frame)
python scripts/host_teleop_laptop_usb.py --port COM9 --motor-ids 0x70,0x74
```

See [known-issues.md](known-issues.md) for daisy-chain cal **NOISE** fault.

## 4. Jetson / UART teleop

```bash
cd /path/to/DeftRoboticsControlsPCB
pip3 install -r scripts/requirements.txt
python3 scripts/host_teleop.py
```

When prompted: **1** = USB, **0** = UART. Or `--transport usb` / `--transport uart`.

This script targets the original single-slot position-step teleop; for multi-motor bench use `host_teleop_laptop_usb.py` over USB on the controls PCB.

## 5. What success looks like

| Check | Expected |
|-------|----------|
| Heartbeat LED (PC3) | Toggles ~2 Hz |
| CAN activity LEDs | Blink on traffic per branch |
| Plant teleop status | `cmd=` tracks `fb=`; kp=0 at rest, non-zero while moving |
| `ack_seq` | Tracks low 8 bits of command seq |
| `tick` | Increments (12-bit plant counter) |
| All four slots | Feedback populates after wake; homing completes → arrow keys enabled |

## 6. Common mismatches

| Symptom | Likely cause |
|---------|----------------|
| No feedback on one bus | Motor not woken; wrong `--bus` vs schematic branch |
| Plant teleop, no motion | kp=0 until feedback sync; run recovery on that branch |
| Cal reports **NOISE** | Daisy-chain bus issue — power-cycle drives, retry (see known-issues) |
| MCU stuck after Ctrl+C mid-probe | Short `0x70` reset or `--recovery` on affected bus |
| Wrong bus / LED | CH2/CH3 Cube instance swap — use schematic bus in scripts, not Cube name |
| Garbage / no apply | USB port mismatch, or RX desync (magic hunt in `host_link`) |

## 7. Size check (optional)

After build:

```bash
arm-none-eabi-size Debug/DeftRoboticsControlsPCB.elf
```
