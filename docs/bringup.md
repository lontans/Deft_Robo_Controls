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

`plant_config` enables **seven** actuators (`ACTUATOR_COUNT = 7`):

| Slot | Bus | Motor ID | Protocol |
|------|-----|----------|----------|
| 0–6 | CH1 | `0x01`…`0x07` (factory placeholders) | Damiao |

Runtime: `config set` after discover. Wire image still 562 B (`HOST_EXCHANGE_ACTUATOR_SLOTS=25`); plant applies slots `0..ACTUATOR_COUNT-1` only.

- FDCAN1/2/3 @ 1 Mbit/s, per-bus TX queue + RX ring (depth 128)
- **CH1:** **mixed** standard + extended when Damiao and RobStride share the branch (see [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md)); Damiao daisy-chain bench Jul 2026
- **CH2:** extended CAN (RobStride)
- **CH3:** **mixed** standard + extended classic CAN — `hfdcan2` @ PB12/PB13 (`StdFiltersNbr=1`, `ExtFiltersNbr=1`, accept-all dual filter). Damiao (std) and RobStride (ext) may share the branch when configured on different actuator slots. See [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md).
- `can_router.c` maps schematic CH2 → `hfdcan3` (PA8/PA15), CH3 → `hfdcan2` (PB12/PB13)
- Activity LEDs: PC7 (CH1), PC6 (CH2), PB15 (CH3)
- **CH4–CH6:** MCP2518FD SPI-CAN — see [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md)

On boot, `control_loop_start()` in `main()` arms TIM6 @ 500 Hz (before the FreeRTOS scheduler). RobStride motors are woken by host bench probes (`--recovery`, calibrate preamble, or plant teleop with prior probe). See [free_rtos-bringup.md](free_rtos-bringup.md) for RTOS task layout and verification.

### Damiao CH1 daisy chain (in progress — Jul 2026)

**Goal:** Discover and run plant teleop on a daisy-chained Damiao branch on CH1 (FDCAN1, mixed std+ext when RS shares the bus).

| Check | Current result |
|-------|----------------|
| USB / DM0 session | OK |
| Discover **DM-J4310** | **OK** — register scan finds 4310 ESC_IDs on CH1 |
| Discover **DM-J4340P-2EC** (isolated CH1) | **OK** — same MIT + `0x7FF` reg-scan as 4310 (docs register map / read-param identical). Bench unit: `ESC_ID=0x01`, `Master=0x11`. Enable (`0xFC`) + MIT feedback OK (`ERR=1`, pos/temp). |
| Plant slot map (RAM) | Slot 0 → Damiao CH1 `0x01`; other slots disabled for isolate test. **`--persist` flash SAVE currently `flash_err`** (RAM survives until power cycle; needs NVM/flash fix + reflash). |
| Plant feedback (slot 0) | **OK** — `status` shows `slot0 pos≈-0.75 err=0x1` while motor enabled |
| Plant teleop jog | **Pending** — run `teleop --slot 0` interactively |
| Multi-motor daisy teleop | **Not yet** — previously 4310s faulted when un-enabled 4340s sat mid-chain; re-test after each 4340 ID is mapped+enabled |

**Doc note:** Extended manuals (`DMJ4310 Documentation Extended.pdf` vs `DM-J4340P-2EC V1.1 Documentation Extended.pdf`) share the same CAN 2.0B @ 1 Mbps MIT / `0x7FF` param protocol and register addresses. No separate `PROTO_DAMIAO_4340`. If a unit never answers, check Assistant baud (codes >1M → CAN FD feedback the MCU will not decode) and ESC_ID range — not a different wire format.

`control_hub.py discover` returns **one** motor per run (first responder). To list every ID on the branch:

```powershell
python scripts/control_hub.py discover --port COM5 --protocol damiao --bus 1          # first hit only
python scripts/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 32 --listen-ms 60
python scripts/damiao_scan.py --port COM5 --probe-id 0x01 --bus 1 --enable --hold-ms 2000
```

Discovery uses **register read** (`DM_PROBE_REG_SCAN`): TX `0x7FF` read ESC_ID (`0x08`) + MST_ID (`0x07`). Works while motor is **disabled** (red LED). Map each FOUND `esc_id` / `master_rx` to a plant slot with `config set --bus 1 --motor-id …` (add `--persist` once flash SAVE works).

**Isolate-bench map (Jul 2026):**

| Joint / unit | Slot | Bus | ESC_ID | Master | Notes |
|--------------|------|-----|--------|--------|-------|
| DM-J4340P-2EC (isolate) | 0 | CH1 | `0x01` | `0x11` (AUTO ok) | Discover+enable+wave OK |
| YAM 7-motor daisy | 0–6 | CH1 | TBD after discover | | `ACTUATOR_COUNT=7` — reflash required; then `damiao_scan --discover --host-only --bus 1` |

```powershell
# After reflash (ACTUATOR_COUNT=7):
python scripts/control_hub.py config show --port COM5   # expect 7 slots
python scripts/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 32 --listen-ms 60
python scripts/control_hub.py --port COM5 --plant-teleop --plant-slots 0,1,2,3,4,5,6
python scripts/control_hub.py --hello-world --port COM5 --slot N   # AI jog when teleop not holding COM
```

**Agent plan:** [plan-damiao-4340-bringup.md](plan-damiao-4340-bringup.md). Arm shared-teleop: expand to 7 slots, discover all ESC_IDs, shared plant. Extended notes: local `docs/damiao-bringup.md` (gitignored). See [known-issues.md](known-issues.md).

### YAM joint-slot-ESC_ID map and command policy (Jul 2026)

Joint-space host commands (single-joint AI jog/status, user multi-joint teleop) are
specified in [plan-yam-joint-commands.md](plan-yam-joint-commands.md); it is the source
of truth for the command surface, soft-limit rules, and calibration gap. Summary:

| Joint | Slot | Bus | ESC_ID | Master | Soft limit (rad, motor frame until zeroed) |
|-------|------|-----|--------|--------|----------------------------------------------|
| J1 | 0 | CH1 | `0x01` | `0x11` | `[-2.618, 3.130]` |
| J2 | 1 | CH1 | `0x02` | `0x12` | `[0, 3.650]` |
| J3 | 2 | CH1 | `0x03` | `0x13` | `[0, 3.130]` |
| J4 | 3 | CH1 | `0x04` | `0x14` | `±1.5708` |
| J5 | 4 | CH1 | `0x05` | `0x15` | `±1.5708` |
| J6 | 5 | CH1 | `0x06` | `0x16` | `±2.094` |
| J7 (EE, bench-only) | 6 | CH1 | `0x07` | `0x17` | `[1.10, 2.80]` provisional, not in `yam.xml` |

ESC_ID/Master column is the nominal factory-placeholder mapping (`0x01`…`0x07` /
`0x11`…`0x17`); confirm against the actual daisy chain with `damiao_scan --discover`
(see the isolate-bench map above) before trusting slot assignment. J1–J6 ranges come
from `External_Documentation/yam_arm_damiao/yam.xml`; J7 is bench-derived and
intentionally loose. Both sourced live via `control_hub.py hello-world --limits`.

**Who holds COM5:**

| Who | Command |
|-----|---------|
| User (interactive) | `--plant-teleop --plant-slots 0,1,2,3,4,5,6` or `teleop --slot N` |
| AI (single-joint, non-interactive) | `hello-world --joint N …` / `joint goto --joint N …` / `joint status --joint N` |

User stops teleop (`q`) before handing the port to AI commands; AI prefers one joint per
invocation and runs `--dry-run` first when a joint/delta is ambiguous.

```powershell
python scripts/control_hub.py hello-world --limits
python scripts/control_hub.py joint status --port COM5 --joint 1
python scripts/control_hub.py joint goto --port COM5 --joint 7 --delta -0.3 --dry-run
python scripts/control_hub.py joint goto --port COM5 --joint 5 --to 0.4 --absolute --i-know-zeros --dry-run
```

`joint goto --to --absolute` is refused without `--i-know-zeros` — motor encoder frame
is not yet aligned to the MuJoCo model frame (calibration steps in the plan §6).
`joint home` is P2 and not implemented yet (stub returns non-zero).

### Dual-arm YAM bring-up (Jul 2026)

Scaled from one arm (7 slots, CH1) to two identical arms (14 slots): arm1 = slots 0-6 /
J1-J7 on CH1 (unchanged), **arm2 = slots 7-13 / J8-J14 on CH2**. Arm2 mirrors arm1's
soft limits and gains exactly (identical hardware) — see `arm_local_joint()` /
`load_yam_limits()` in `scripts/control_hub/yam_limits.py`. Bench-verified both arms
alive and moving via `joint goto`/`config show`/`discover` on real hardware.

**Firmware fixes required to get here** (all in this session's diff, all need
rebuild + reflash — see git history on `App/`):

| Bug | Symptom | Fix |
|-----|---------|-----|
| CFG PDU packing overflowed at 14 slots | `_Static_assert` build failure (6+14×3 > 32 B PDU) | Paginated CFG GET: firmware replies ≤8 slots/page + `[total_count][start_slot]` trailer; host (`plugins/plant_config.py::fetch_table`) loops pages automatically. `plant_config_nvm.c` |
| Damiao TX tripled per slot per tick | J6/J7 (last-enqueued slots) silently stopped tracking MIT commands once 6-7 Damiao slots were enabled at once — CH1 bus oversubscribed (~10.5k frames/sec demand vs ~7.7-9.3k capacity) | `damiao_apply_cycle()`: 3× redundant MIT send → 1×. `plugins/damiao.c` |
| Thermo diag reply clobbered CFG replies every tick | `config show`/`discover` hard-timeout (`pdu=t` on every frame, even after `recover`) | `thermo_feedback_fill()` now only writes if nothing else claimed the shared PDU slot that tick. `thermo.c` |
| CH2 was ext-ID-only (RobStride-provisioned) | `discover --bus 2` found nothing on arm2 even fully powered/wired — CH2's global filter rejected all standard-ID (Damiao) frames | `fdcan_rx_mode_for_bus()`: CH2 now mixed std+ext like CH1/CH3. Verified safe via the STM32G4 HAL source — each FDCAN instance has its own independent message-RAM block, so this doesn't touch CH1/CH3's RAM. `can_router.c` |

**Host-side actuator table** (`scripts/controls_pcb_host/actuator_config.py` /
`protocol/schema.py`): `ACTUATOR_COUNT=14`, `_DEFAULT_TABLE` extended arm2 CH2 entries,
`session.py`'s `status` display fixed to show all slots (was hardcoded to 6).

**Teleop rework** (`scripts/control_hub/teleop/`):

- **No more gravity sag on release** — `_update_slot_motion` used to drop to a weak
  `idle_kp` (or 0, fully backdrivable) once the arrow was released or homing finished.
  It now holds at each slot's full tuned `SLOT_KP` at all times once synced.
- **Non-target joints no longer go limp** — a plain single-slot `send_plant({slot: ...})`
  zero-fills every other slot in that frame (firmware copies the whole staged image into
  `actuator_desire_live[]` each tick), so `teleop --slot N` used to drop every other joint
  to kp=0. It now pulls in every enabled slot and braces all but the target one at their
  current position, full stiffness (`target_slot` param, `run_for_slot`).
- **Home to current position, not a fixed 0 rad** — `_home()` takes a per-slot
  `home_target` dict (defaults to each slot's fb right after wake) instead of driving
  everything to `D.HOME_TARGET=0.0`. Avoids a large forced move on arms that are nowhere
  near a calibrated zero pose; "already at home" fires immediately since target==current.
- **Group selection by joint number across arms** — `--plant-teleop --joints 1,2,3,4`
  expands arm-local joint numbers (1..7) to the matching slot on *every* arm at once
  (`slots_for_arm_local_joints()` in `controls_pcb_host/teleop/delegate.py`); e.g.
  `--joints 1,2,3,4` → slots `0,1,2,3,7,8,9,10`. `--plant-slots` (raw slot list) still
  works for asymmetric picks. Bus-select keys (`0`/`1`/`2`) now usefully split by arm
  since arm2 is on its own bus — `1`=arm1 only, `2`=arm2 only, `0`=both together.

**Gain/velocity tuning history** (bench feedback, Jul 2026 — see `teleop/defaults.py`):

| Setting | History | Current |
|---|---|---|
| `SLOT_KP` (per-joint) | flat 12.0 → (30,50,90,50,12,12,12) → (40,60,90,60,25,25,20), mirrored per arm | wrist/EE joints bumped once they started holding at full stiffness continuously (weren't firm enough for that duty at 12.0) |
| `DM_KD` | 0.5 → 0.8 → 1.0 | more damping to match higher `SLOT_KP` |
| `DM_ARROW_VEL` | 3.0 → 0.6 → 0.25 → 0.12 rad/s | still "a little high" per bench feedback — next lever to pull if whip persists |
| `MAX_CMD_LEAD` | 0.35 → 0.18 rad | caps how far cmd can run ahead of fb before capping — was contributing to "whipping" (stored torque released suddenly) |

**Commands:**

```powershell
# Enable check / current position, any joint 1-14
python scripts/control_hub.py joint status --port COM5 --joint 8

# Single joint (braces all other 13 automatically)
python scripts/control_hub.py teleop --port COM5 --slot 0

# One full arm
python scripts/control_hub.py --plant-teleop --plant-slots 0,1,2,3,4,5,6 --port COM5   # arm1
python scripts/control_hub.py --plant-teleop --plant-slots 7,8,9,10,11,12,13 --port COM5 # arm2

# Joint group across both arms at once
python scripts/control_hub.py --plant-teleop --joints 1,2,3,4 --port COM5
```

**Not yet done:** arm2's limits/gains are mirrored from arm1, not independently
bench-verified per-joint (flagged in `yam_limits.reasonableness_notes()`). Velocity may
need another cut. Full session trace (live commands + output):
[bench-log-2026-07-10-gain-retest.md](bench-log-2026-07-10-gain-retest.md).

### Damiao CH3 (earlier bench — Jul 2026)

Single DM-J4310 on CH3: USB/DM0 OK, CAN TX OK; **`rx_raw=0`** on register scan until motor-end **120 Ω** termination is added (4310 has no software termination register). Superseded for multi-motor work by CH1 daisy-chain bench above.

### Mixed std/ext on CH1 / CH3 (Jul 2026)

Firmware accepts **both** 11-bit and 29-bit classic CAN on schematic CH1 and CH3 (`FDCAN_RX_STD_AND_EXT`). Plant teleop fans out RX to every enabled actuator slot on the bus so Damiao and RobStride can coexist when assigned to different slots on the same harness.

**Architecture (FIFOs, filter masks, demux):** [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) §0 — HW RX FIFO0 is shared; std vs ext is identified by HAL `IdType` / `can_frame_t.id_type`; software fan-out replaces per-plugin ring drains.

**Bench verified (Jul 2026):** four-slot plant teleop `0,1,2,3` — Damiao + RS on CH1–CH3, including Damiao + RS both on CH1. Smooth arrow motion after homing when all configured motors are enabled. **Damiao-only CH1 daisy chain:** homing OK on DM-J4310 slots; teleop still blocked when un-enabled DM-J4340 units sit between configured 4310s — see § Damiao CH1 daisy chain.

```powershell
python scripts/control_hub.py config set --port COM5 --slot 1 --protocol robstride --bus 3 --motor-id 0x75
python scripts/control_hub.py config set --port COM5 --slot 2 --protocol damiao   --bus 3 --motor-id 0x06
python scripts/control_hub.py recover --port COM5 --bus 3
python scripts/control_hub.py --plant-teleop --plant-slots 1,2 --port COM5
```

**Bench verify (user-run):**

1. `discover --protocol damiao --bus 3` → FOUND (std frames).
2. `discover --protocol robstride --bus 3` → FOUND when RS motor on CH3 (ext frames).
3. Mixed `--plant-slots`: both slots show feedback; arrow motion moves the selected motor only.
4. Regression: Damiao-only CH3 teleop; RS CH1/CH2 teleop unchanged.
5. Damiao CH1 daisy chain: isolated DM-J4340P-2EC discover/enable OK (`0x01`/`0x11`); multi-motor daisy teleop still pending full-chain enable.

All devices on a mixed branch must share **1 Mbps** nominal bit timing and proper **120 Ω** termination at each end.

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
| Damiao `--discover` | `FOUND` with `esc_id` + `master_rx`; `rx_raw > 0` (4310 OK; isolated 4340P `0x01`/`0x11` OK) |
| Damiao CH1 daisy homing | Configured 4310 slots reach home; teleop motion pending full chain enable |

## 6. Common mismatches

| Symptom | Likely cause |
|---------|----------------|
| No feedback on one bus | Motor not woken; wrong `--bus` vs schematic branch |
| Plant teleop, no motion | kp=0 until feedback sync; run recovery on that branch |
| Cal reports **NOISE** | Daisy-chain bus issue — power-cycle drives, retry (see known-issues) |
| MCU stuck after Ctrl+C mid-probe | Short `0x70` reset or `--recovery` on affected bus |
| Wrong bus / LED | CH2/CH3 Cube instance swap — use schematic bus in scripts, not Cube name |
| Damiao `tx>0` `rx_raw=0` | Missing motor-end 120 Ω termination; 24 V; see [known-issues.md](known-issues.md) |
| Damiao 4310 teleop fault mid-chain | Un-enabled DM-J4340 between configured 4310 slots — discover/enable 4340 or re-chain |
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
