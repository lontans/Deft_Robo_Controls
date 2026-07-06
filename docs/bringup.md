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

`plant_config.c` enables **six** actuators (`ACTUATOR_COUNT = 6`):

| Slot | Bus | Motor ID | Protocol |
|------|-----|----------|----------|
| 0 | CH1 | `0x76` | RobStride RS02 |
| 1 | CH1 | `0x74` | RobStride RS01 (daisy on CH1) |
| 2 | **CH3** | `0x01` (placeholder) | **Damiao** DM-J4310 |
| 3 | CH4 | `0x70` | RobStride (MCP2518) |
| 4 | CH5 | `0x70` | RobStride (MCP2518) |
| 5 | CH6 | `0x70` | RobStride (MCP2518) |

- FDCAN1/2/3 @ 1 Mbit/s, per-bus TX queue + RX ring (depth 128)
- **CH1 / CH2:** extended CAN (RobStride)
- **CH3:** **standard** CAN (Damiao) — `hfdcan2` @ PB12/PB13, accept-all std filter
- `can_router.c` maps schematic CH2 → `hfdcan3` (PA8/PA15), CH3 → `hfdcan2` (PB12/PB13)
- Activity LEDs: PC7 (CH1), PC6 (CH2), PB15 (CH3)
- **CH4–CH6:** MCP2518FD SPI-CAN — see [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md)

On boot, `control_loop_init()` starts TIM6 @ 500 Hz. RobStride motors are woken by host bench probes (`--recovery`, calibrate preamble, or plant teleop with prior probe).

### Damiao CH3 (in progress — Jul 2026)

**Goal:** Discover DM-J4310 ESC_ID / Master ID on CH3 over MCU USB (no USB-UART adapter).

| Check | Current result |
|-------|----------------|
| USB feedback | OK — magic, `ack_seq`, `mcu_state=DIAG_ONLY` |
| DM0 session (`--link-test`) | OK after latest firmware flash |
| CAN TX on CH3 | OK — `tx>0` on probes; PB15 activity LED may blink |
| CAN RX from motor | **FAIL** — `rx_raw=0` on all scanned ESC_IDs (`--discover --start 1 --end 16`) |

**Likely blocker:** two-node CAN bus needs **120 Ω between CAN_H and CAN_L at each end**. Controls PCB has termination on its side; **DM-J4310 has no software termination register** — add a physical **120 Ω resistor across CAN_H and CAN_L at the motor-end connector** (or unused daisy-chain XT30 port). Measure H–L with power off: ~60 Ω = both ends terminated; ~120 Ω = one end only.

**Gold-standard isolate test:** Damiao Assistant + USB2CAN on the motor (same 24 V harness). If Assistant works but MCU path does not, focus on CH3 harness/termination; if both fail, check motor power and connector.

```powershell
pip install pyserial
python scripts/damiao_scan.py --port COM5 --link-test
python scripts/damiao_scan.py --port COM5 --ack-debug --bus 3 --probe-id 1
python scripts/damiao_scan.py --port COM5 --discover --bus 3 --start 1 --end 16
python scripts/damiao_scan.py --port COM5 --probe-id 6 --bus 3   # if prior hint at id 6
```

Discovery uses **register read** (`DM_PROBE_REG_SCAN`): TX `0x7FF` read ESC_ID (`0x08`) + MST_ID (`0x07`). Works while motor is **disabled** (red LED). After FOUND, update slot 2 in `plant_config.c` with discovered `motor_id` and Master ID.

Extended notes: local `docs/damiao-bringup.md` (gitignored). See [known-issues.md](known-issues.md).

## 3. Laptop USB bench (Windows / Linux)

```powershell
pip install -r scripts/requirements.txt
python scripts/host_teleop_laptop_usb.py --list-ports
python scripts/host_teleop_laptop_usb.py --port COM9 --plant-teleop
```

### Plant teleop (`--plant-teleop`) — recommended runtime path

- All enabled RobStride slots in one 562 B frame; MCU applies at **500 Hz** (no RS2/DM PDU)
- Auto-syncs feedback, **slow homing to 0.00 rad**, then arrow-key velocity on all motors
- Gentle defaults: kp 8–12 (gated — **0 at rest**), 5 rad/s, slow ramps → low bench current
- Keys: **Left/Right** move active bus selection, **0** = all buses, **1/2/3** = CH1/CH2/CH3 only, **r** re-sync, **q** quit

```powershell
# Even gentler
python scripts/host_teleop_laptop_usb.py --port COM9 --plant-teleop --plant-arrow-vel 3 --plant-home-slew 0.15
```

Motors must be woken once per branch before plant teleop (recovery or calibrate on that bus).

### Launch demo (`--launch-seq`)

Sequential capability demo with **15% stagger** on CH1 daisy chain (`0x76 → 0x74`).

```powershell
python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq
python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq --launch-ccw
python scripts/host_teleop_laptop_usb.py --port COM9 --launch-seq --launch-vel 10
```

### RS2 PDU path — calibrate, discover, single-motor teleop

Uses `pdu.data[0..2] = 'R','S','2'` and `pdu.data[11]` for schematic bus (`1` = CH1 … `3` = CH3). Pauses the 500 Hz actuator loop while an RS2 session is active.

```powershell
# Discover / recovery / scan (CH1 example)
python scripts/rs02_can_scan.py --port COM9 --bench-cmds --bus 1 --target 0x76

# Encoder cal on CH1
python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 1 --motor-id 0x76
python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 1 --motor-id 0x74

# RS2 arrow teleop (CH1 motors)
python scripts/host_teleop_laptop_usb.py --port COM9 --motor-ids 0x76,0x74
```

See [known-issues.md](known-issues.md) for daisy-chain cal **NOISE** fault.

## 3b. Dynamixel neck servos (UART5)

```powershell
python scripts/dynamixel_scan.py --port COM9 --start 1 --end 2
python scripts/dynamixel_teleop.py --port COM9
python scripts/dynamixel_teleop.py --port COM9 --debug   # SVD diag line
```

Slot 0 = ID 1 (bottom), slot 1 = ID 2 (top). Unicast wr/rd @ 500 Hz on MCU; host @ ~40 Hz. See **Closed — Dynamixel neck** in [known-issues.md](known-issues.md).

## 3c. SK9822 LED strip (SPI3)

```powershell
python scripts/sk9822_led_test.py --port COM9
python scripts/sk9822_led_test.py --port COM9 --mode 0 --brightness 8 --count 0
python scripts/sk9822_led_test.py --port COM9 --mode 1   # off
```

5 V on strip, GND common with MCU. Mode 0 = red dot scan from input end. Tune `LED_STRIP_MAX` in `App/Inc/plant/plugins/sk9822.h`. See **Closed — SK9822** in [known-issues.md](known-issues.md).

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
| All RobStride slots | Feedback populates after wake; homing completes → arrow keys enabled |
| Damiao `--discover` | `FOUND` with `esc_id` + `master_rx`; `rx_raw > 0` |

## 6. Common mismatches

| Symptom | Likely cause |
|---------|----------------|
| No feedback on one bus | Motor not woken; wrong `--bus` vs schematic branch |
| Plant teleop, no motion | kp=0 until feedback sync; run recovery on that branch |
| Cal reports **NOISE** | Daisy-chain bus issue — power-cycle drives, retry (see known-issues) |
| MCU stuck after Ctrl+C mid-probe | Short `0x70` reset or `--recovery` on affected bus |
| Wrong bus / LED | CH2/CH3 Cube instance swap — use schematic bus in scripts, not Cube name |
| Damiao `tx>0` `rx_raw=0` | Missing motor-end 120 Ω termination; 24 V; see [known-issues.md](known-issues.md) |
| Garbage / no apply | USB port mismatch, or RX desync (magic hunt in `host_link`) |

## 7. Size check (optional)

After build:

```bash
arm-none-eabi-size Debug/DeftRoboticsControlsPCB.elf
```
