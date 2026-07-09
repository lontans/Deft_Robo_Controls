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

On boot, `control_loop_start()` in `main()` arms TIM6 @ 500 Hz (before the FreeRTOS scheduler). RobStride motors are woken by host bench probes (`--recovery`, calibrate preamble, or plant teleop with prior probe). See [free_rtos-bringup.md](free_rtos-bringup.md) for RTOS task layout and verification.

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

## 7. Plant teleop cadence (FDCAN + MCP) — **resolved Jul 2026**

Regression after commits **`d9ce9e6`** / **`c700c78`** (last known-good CH2: **`5df1f04`**). Restoring `CONTROL_TICK_BURST_MAX=8` and a single `control_loop_service()` per lap was **necessary but not sufficient** — several independent bugs stacked on top of the MCP superloop work.

### Symptom (failing builds)

- `cmd` updates smoothly but `fb` jumps in lumps; motion feels stepped.
- Telemetry: `pend` pegged, `lap` **~50–370 ms**, `ptick` ≪ burst max, `lead` often pinned at **±0.35 rad**.
- MCP slot 3: no CH4 LED during teleop, `kp=0` while arrow held, homing completes on `cmd` but `fb` unchanged.

### Root causes (fix-focused)

| Layer | Cause | Fix |
|-------|--------|-----|
| **Superloop** | `d9ce9e6`/`c700c78`: burst 1, 4× service loop, all 6 slots + full `can_router_poll()` every tick | Restore burst **8**, single `control_loop_service()`; scope polls to commanded buses; skip blank MCP slots |
| **Dynamixel** | `servo_bus_service()` ran every plant burst with no servo host session → UART RX **~50 ms** timeout → `lap≈52 ms`, `pend=255` on **all** plant teleop (including CH2) | Skip DXL bus unless `g_servo_host_session` |
| **MCP init** | Lazy MCP rail init (one rail per `spi_can_router_hw_step`) never finished once end-of-lap stopped walking all SPI rails | **Eager** `mcp2518_reinit_rail()` for CH4–6 in `spi_can_router_init()` |
| **MCP plant TX** | `enqueue` + `try_send` with 3 ms TXQ wait × burst 8 → `lap≈327 ms`; blocking `probe_tx` (50 ms) same problem | Plant MIT: `enqueue` + `prepare_tx` + **fire-and-forget** `mcp2518_try_send` (load TXQ, no wait). Probes/recovery still use blocking `send_now` |
| **MCP enable** | `maintain_enable` skipped when `kp>0`; post-recover motor never armed for MIT teleop | Always run enable on **first** non-idle entry (`last_maintain_ms==0`), then skip while `kp>0` |
| **Recovery LEDs** | `recover` runs `plant_recovery_all()` on **all** enabled RS02 slots (CH1–3 FDCAN + CH4–6 MCP), not only `--bus N` | Expected: CH1–2 blink on any recover; MCP reset uses `send_now` so CH4 blinks when rail is init’d |
| **Host teleop** | `FB_STALE` gate treated flat `fb` as dead → `kp=0` with arrow held; blank MCP desire skipped firmware SPI | Gate on **ack** age only; active slot sends `HOME_POS_EPS` so MCP is non-blank |
| **Host cmd slew** | `cmd = fb + lead` each tick rebased onto lagged `fb` → visible snaps | Integrate `cmd` from previous `cmd`; clamp lead widening only |
| **`fb_age` metric** | Host only reset timer when position moved **>1e-4 rad** — flat rest looked “stale” during motion | Reset `fb_age` on **any** fresh actuator sample for the slot |

**Discrepancy note:** Early traces showed `lap≈52 ms` with `pend=255` and were attributed to CAN/MCP polling alone. Profiling showed **`lap` matched `DXL_RX_TIMEOUT_MS` (50)** — Dynamixel was the dominant cost until the servo skip. After that, CH2 reached `lap≈0–1 ms`. MCP then failed for different reasons (init, host gating, blocking TX), not superloop burst settings.

### Good telemetry (post-fix, reflash required)

| Bus | `lap` | `pend` | `fb_age` (motion) | Notes |
|-----|-------|--------|-------------------|-------|
| CH2 FDCAN slot 1 | 0–1 ms | 0 | 0 ms | Smooth `cmd`/`fb` tracking |
| CH4 MCP slot 3 | 0–1 ms | 0–1 | 0 ms | CH4 ACT LED blinks during arrow hold |

At rest, `fb_age` may climb while `fb` is flat — that means position has not changed, not that USB feedback stopped.

### Bringup check

```powershell
python scripts/control_hub.py --port COM5 recover --bus 2
python scripts/control_hub.py --port COM5 teleop --slot 1   # CH2 FDCAN

python scripts/control_hub.py --port COM5 recover --bus 4
python scripts/control_hub.py --port COM5 teleop --slot 3   # CH4 MCP
```

Optional trace:

```powershell
python scripts/control_hub.py --port COM5 teleop --slot 3 --debug-trace teleop_trace.csv
```

### Key files (firmware + host)

| Area | Files |
|------|--------|
| Superloop | `App/Src/app.c`, `App/Src/plant/control_loop.c` |
| Actuator scope | `App/Src/plant/actuator.c` |
| RS02 apply | `App/Src/plant/plugins/robstride.c` |
| MCP SPI | `App/Src/plant/can/spi_can_router.c`, `App/Src/plant/can/mcp2518fd.c` |
| Servo skip | `App/Src/plant/servo.c` |
| Host teleop | `scripts/control_hub/teleop/plant.py`, `scripts/control_hub/teleop/defaults.py` |

### Distinguish from key-input glitches

- `dir` briefly `0` while arrow held → host latch (`RELEASE_CONFIRM_S` in `teleop/input.py`). Separate from plant cadence.

**Historical handoff:** [handoff-plant-superloop-regression.md](handoff-plant-superloop-regression.md) (superseded for bringup status).

## 8. CH2 cali after teleop — **resolved Jul 2026**

### Symptom (was)

After plant teleop on CH2, `calibrate --bus 2 --id 0x70` showed prep HIT but no `... cali listen` lines and no shaft spin. CH4 MCP cali on the same motor ID had worked earlier.

### Cause

FDCAN cali path set **`CALI_SKIP_RESET`** so firmware skipped the pre-`0x05` reset that MCP gets. After MIT teleop the drive often needs that reset immediately before `comm 0x05`.

### Fix

`scripts/control_hub/rs02/calibrate.py`: FDCAN uses the same **250 ms settle** and firmware reset before `0x05` as MCP (removed `CALI_SKIP_RESET` for FDCAN). RS02 teleop exit runs **`recovery_on_exit`** to clear bench state.

### Bringup check

```powershell
python scripts/control_hub.py --port COM5 recover --bus 2
python scripts/control_hub.py --port COM5 teleop --slot 1
python scripts/controls_pcb_host.py --port COM5 calibrate --bus 2 --id 0x70
```

Expect `... cali listen` lines and shaft spin. If cali fails cold (no teleop), check harness/termination — not this handoff path.

## 9. Size check (optional)

After build:

```bash
arm-none-eabi-size Debug/DeftRoboticsControlsPCB.elf
```
