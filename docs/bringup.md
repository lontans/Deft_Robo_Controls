# Bring-up

**As of Jul 2026.** Dual YAM Damiao arms are the live plant path. Prefer
[`scripts/deft_controls_sdk/`](../scripts/deft_controls_sdk/README.md) and
[`api.md`](api.md). Legacy is frozen ([`scripts/legacy/README.md`](../scripts/legacy/README.md))
pending SDK-only prove-out, then gitignore. Wire: [host-exchange-v3.md](host-exchange-v3.md),
DEBUG: [host-debug-v1.md](host-debug-v1.md). Compact bug list: [lessons.md](lessons.md).

This file keeps **how to run** plus the **bench stories** worth not losing
(Damiao daisy, dual-arm firmware fixes, plant-cadence regression, CH4 MCP2518FD +
MCP2562 timeline, USB FB rate on the 26-slot CH1–6 plant). Full CH4 postmortem:
[ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md). USB FB
matrix: §7a.

---

## 1. Transport + flash

Edit `App/Inc/host/host_transport.h` before building:

| Board | `HOST_TRANSPORT_UART` | Link |
|-------|----------------------|------|
| Controls PCB (laptop) | `0` | USB FS CDC → `COM*` |
| Jetson / UART | `1` | UART4 PC10/11 @ 115200 |

Rebuild/flash from STM32CubeIDE (Debug). PC3 ≈ 2 Hz heartbeat when the plant is alive.

---

## 2. Plant map (dual-arm — current)

| Arm | Slots | Joints | Bus |
|-----|-------|--------|-----|
| Arm1 | 0–6 | J1–J7 | CH1 (FDCAN1) |
| Arm2 | 7–13 | J8–J14 | CH2 (FDCAN3) |

- Firmware `ACTUATOR_COUNT` matches host exchange (**25** slots today). Dual-arm
  Damiao maps below still use slots 0–13; unused slots stay disabled in CFG.
- Soft limits: `yam_limits.py` / `yam.xml` (J1–J6); J7 provisional. Absolute goto needs `--i-know-zeros` until encoder↔model zeros exist.
- Nominal Damiao ESC `0x01`…`0x07`, Master `0x11`…`0x17` — **confirm with discover** before trusting CFG.
- CH1–CH3: mixed std+ext when Damiao + RobStride share a branch. Detail: [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md).
- Schematic: CH2 → `hfdcan3`, CH3 → `hfdcan2`.
- **CH4–CH6:** MCP2518FD + MCP2562 SPI-CAN — see §8 and [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md).

### Joint ↔ slot (arm1; arm2 mirrors +7)

| Joint | Slot | Bus | ESC (nominal) | Master | Soft limit (rad, motor frame until zeroed) |
|-------|------|-----|---------------|--------|----------------------------------------------|
| J1 | 0 | CH1 | `0x01` | `0x11` | `[-2.618, 3.130]` |
| J2 | 1 | CH1 | `0x02` | `0x12` | `[0, 3.650]` |
| J3 | 2 | CH1 | `0x03` | `0x13` | `[0, 3.130]` |
| J4 | 3 | CH1 | `0x04` | `0x14` | `±1.5708` |
| J5 | 4 | CH1 | `0x05` | `0x15` | `±1.5708` |
| J6 | 5 | CH1 | `0x06` | `0x16` | `±2.094` |
| J7 (EE) | 6 | CH1 | `0x07` | `0x17` | `[1.10, 2.80]` provisional |

Arm2: J8–J14 → slots 7–13 on CH2, same local soft limits/gains.

---

## 3. Host SDK (preferred)

```powershell
cd scripts
pip install -r requirements.txt

# Single RS02 — move cable between CH1–CH6, only change --bus
python rs02_channel_bringup.py --bus 4
python rs02_channel_bringup.py --bus 1 --motor-id 0x70 --skip-cali

# Telemetry UI (owns COM)
python -m deft_controls_sdk.debug_dashboard --port COM5

# Plant control (script)
python -c "
from deft_controls_sdk import ControlsPcbHub, ActuatorDesire
with ControlsPcbHub.connect('COM5') as hub:
    hub.recover()
    hub.start_streaming()
    hub.set_actuator(0, ActuatorDesire(position=0.0, kp=8.0, kd=0.5))
    print(hub.telemetry.snapshot())
"

# Bring-up / CFG / discover (same COM — not concurrent with dashboard)
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    print(hub.debug.cfg_get_table())
    print(hub.debug.discover_damiao(bus=1))
"
```

One process owns COM. Plant motion = top-level hub methods (`set_actuator`, `start_streaming`, `recover`). Discover/CFG/calibrate = `hub.debug.*`. There is **no** `hub.plant` namespace.

**MCP CH4–6 idle quirk:** firmware skips SPI when the slot desire is blank (`kp/kd/vel/τ≈0` and `position==0`). Idle stream blinks CH1–3 but not CH4–6 until a non-blank desire or a probe runs on that bus.

---

## 4. Legacy teleop / joint CLI (frozen — prefer SDK/vbeta for new work)

New teleop work should use `vbeta_smoke.py arm --side left|right` (see §3) or
`hub.set_actuator(...)` directly. The commands below are the legacy
`scripts/legacy/control_hub.py` CLI — kept only for the full daisy-chain
Damiao discover (`--discover --host-only`, lists **every** ID on the bus;
`hub.debug.discover_damiao()` is first-hit only, no SDK replacement for the
full listing yet):

```powershell
cd scripts
$env:PYTHONPATH = "legacy;."
python legacy/control_hub.py joint status --port COM5 --joint 8
python legacy/control_hub.py teleop --port COM5 --slot 0
python legacy/control_hub.py --plant-teleop --plant-slots 0,1,2,3,4,5,6 --port COM5   # arm1
python legacy/control_hub.py --plant-teleop --plant-slots 7,8,9,10,11,12,13 --port COM5 # arm2
python legacy/control_hub.py --plant-teleop --joints 1,2,3,4 --port COM5               # both arms
python legacy/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 32 --listen-ms 60
```

User teleop releases COM (`q`) before AI / `joint goto`. Absolute `--to` refused without `--i-know-zeros`.

**Gain / velocity direction** (bench Jul 2026, legacy `teleop/defaults.py` — not ported to the SDK):

| Setting | History | Current |
|---------|---------|---------|
| `SLOT_KP` | flat 12 → (30,50,90,…) → | ~(40,60,90,60,25,25,20) mirrored |
| `DM_KD` | 0.5 → 0.8 → | ≈1.0 |
| `DM_ARROW_VEL` | 3.0 → 0.6 → 0.25 → | ≈0.12 rad/s (still a bit high) |
| `MAX_CMD_LEAD` | 0.35 → | ≈0.18 rad |

RS02 calibrate (SDK):

```powershell
python rs02_channel_bringup.py --bus N
# or: hub.debug.calibrate_robstride(bus=N, motor_id=0x..)
```

---

## 5. Damiao stories (keep)

### 5.1 Scan-order / termination / 4310 vs 4340

- Probing ESC 1→N with heavy `REG_SCAN` before the real ID can silence the drive. Prefer MCU `ID_SWEEP`, then known slot IDs, then range.
- After scan-order is ruled out: `tx>0` / `rx_raw=0` → motor-end **120 Ω** (4310/4340 have no software termination). ~60 Ω H–L if both ends terminated.
- Master ID typically `ESC_ID + 0x10`; confirm via regs `0x08` / `0x07`.
- **4310 vs 4340:** same MIT / `0x7FF` map — no separate protocol. Silence → baud (CAN FD), ID, termination — not a different wire format.
- Gold-standard isolate: Damiao Assistant + USB2CAN; 24 V + common GND with MCU.

### 5.2 Daisy chain

Un-enabled motors mid-harness block teleop behind them — map+enable every unit (harness lesson, not FIFO). Isolated DM-J4340P-2EC on CH1 (`0x01`/`0x11`) discover+enable+MIT OK before multi-motor work.

Discover (list all IDs — hub discover is first-hit only; legacy CLI, no SDK replacement yet):

```powershell
python legacy/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 32 --listen-ms 60
```

### 5.3 Dual-arm firmware bugs that got us here

Scaled 7→14 slots (arm2 on CH2). Fixes required:

| Bug | Symptom | Fix |
|-----|---------|-----|
| CFG PDU overflow at 14 slots | Build / host CFG failure | Paginated CFG GET (≤8 slots/page + trailer) |
| Damiao 3× MIT TX/tick | Last slots starved (~10.5k demand vs ~7.7–9.3k capacity) | `damiao_apply_cycle`: 1× MIT/tick |
| Thermo diag clobbered CFG | `config show` / discover hard-timeout | Thermo writes PDU only if unclaimed that tick |
| CH2 ext-only | Arm2 Damiao discover silent | CH2 mixed std+ext (same as CH1/CH3) |

**Teleop durability lessons:** hold full `SLOT_KP` on release (no gravity sag); brace non-target joints at current fb; home to **current fb**, not fixed 0; `--joints 1..7` expands across both arms.

---

## 6. Mixed std/ext FDCAN (CH1–CH3)

Damiao = 11-bit std; RobStride = 29-bit ext. One HW RX FIFO; demux by `IdType`. Install std+ext accept-all filters; fan-out each frame to all slots on that bus — do **not** reintroduce per-plugin exclusive `while (can_rx_pop)`.

Deep reference: [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) (§0 as-built; ignore any older “CH2 ext-only” line).

```powershell
# Example mixed plant on CH3 — SDK
python -c "
from deft_controls_sdk import ControlsPcbHub, ActuatorDesire
with ControlsPcbHub.connect('COM5') as hub:
    hub.recover()
    hub.start_streaming()
    hub.set_actuator(1, ActuatorDesire(position=0.0, kp=8.0, kd=0.5))
    hub.set_actuator(2, ActuatorDesire(position=0.0, kp=8.0, kd=0.5))
"
```

All nodes on a shared branch: **1 Mbps** nominal + 120 Ω termination at each end.

---

## 7. Plant teleop cadence (FDCAN + MCP) — resolved Jul 2026

Regression after `d9ce9e6` / `c700c78` (last known-good CH2: `5df1f04`). Restoring burst=8 + single `control_loop_service()` was **necessary but not sufficient**.

### Symptom

- `cmd` smooth, `fb` lumpy; `pend` pegged; `lap` ~50–370 ms; MCP slot: no CH4 LED, `kp=0` while arrow held.

### Root causes (stacked)

| Layer | Cause | Fix |
|-------|--------|-----|
| Superloop | Burst 1 / over-service / poll everything | Burst **8**, single service; scope polls; skip blank MCP |
| Dynamixel | UART RX ~50 ms every plant burst | Skip DXL unless `g_servo_host_session` |
| MCP init | Lazy rail init never finished | Eager `mcp2518_reinit_rail()` for CH4–6 at boot |
| MCP plant TX | Blocking TXQ wait × burst → `lap≈327 ms` | Fire-and-forget `try_send` on MIT path |
| Host gate | `FB_STALE` on flat position → `kp=0` | Gate on **ack** age; non-blank MCP desire |
| Cmd slew | Rebase onto lagged `fb` → snaps | Integrate from previous `cmd` |
| `fb_age` | Only reset if pos moved | Reset on any fresh sample |

Early `lap≈52 ms` looked like CAN/MCP polling — it was **Dynamixel** (`DXL_RX_TIMEOUT_MS`). After the skip, CH2 `lap≈0–1 ms`; MCP then failed for init/gating/TX reasons, not burst settings.

### Check

```powershell
python rs02_channel_bringup.py --bus 2   # CH2 FDCAN
python rs02_channel_bringup.py --bus 4   # CH4 MCP — expect ACT LED on arrow hold
```

---

## 7a. USB feedback rate (CH1–6 × 25) — Jul 2026

Product stress case is **not** MCP alone — it is **all commanded buses sharing one
superloop** with USB CMD/FB. Measured `raw_fb_hz` is how often the MCU finishes a
lap and ships feedback; plant MIT can be slower than host TX.

### Target CFG (factory / timing matrix)

| Bus | Slots | Backend |
|-----|-------|---------|
| CH1 | 8 | FDCAN |
| CH2 | 8 | FDCAN |
| CH3 | 3 | FDCAN |
| CH4–6 | 2 each | MCP2518 SPI-CAN |

RobStride on all enabled slots. Probe skips CFG SET if the table already matches.

### How to measure

```powershell
cd scripts
# Host plant TX 40 Hz (dashboard default)
python bench_load_matrix.py --port COM5 --hz 40 --scenario all

# Stress host TX
python bench_load_matrix.py --port COM5 --hz 200 --scenario all

# GUI (same COM — not concurrent with the probe)
python -m deft_controls_sdk.debug_dashboard --port COM5 --http-port 8766 --hz 40
```

`bench_load_matrix.py` is the durable successor to the retired
`_tmp_mcp_timing_probe.py` — see
[bench-optimize-and-load-matrix-plan.md](bench-optimize-and-load-matrix-plan.md).

Watch: `raw_fb_hz`, `ack_lag_max`, `lap_ms`, `ticks_pending`. CH4–6 ACT LEDs should
strobe under hold (empty bus / no ACK included).

### Acceptance (bench Jul 2026)

| Hold | Host TX | Expect |
|------|---------|--------|
| CH4–6 MCP ×6 | 40 Hz | `fb_hz` ≥ 100 (typically ~600+) , `ack_lag` ≤ 2 |
| all CH1–6 ×25 | 40 Hz | `fb_hz` ≥ 100 (typically ~600–750), pending not pegged |
| CH1–3 FDCAN ×19 | 40 Hz | ~800–1000 Hz FB |
| all ×25 | 200 Hz host | FB still hundreds Hz; lag max may spike (host denser than FB) |

### What raised FB (stacked)

| Step | Change | Effect |
|------|--------|--------|
| Non-blocking plant MCP TX | No `HAL_Delay` in `try_send` / prepare | Stops ~2 Hz USB starvation |
| One-shot TXQ | `RTXAT` + `TXAT=disable` | Empty-bus attempt can finish |
| Reclaim | UINC/ABAT first; rate-limited config FRESET if still full | LEDs keep strobing without pegging laps |
| Coalesce MCP flush | One `prepare_tx` + flush **per rail** per apply | Cuts duplicate SPI |
| Decimate MCP MIT | Every 4 plant ticks (~125 Hz/slot), staggered | MCP apply may be &lt; 500 Hz; host holds last |
| Drop hot-path TEC | Bus-off in `prepare_tx` only | Less SPI per try |
| **Heavy multi-domain** | When FDCAN **and** MCP commanded: FDCAN burst 1 + ÷2 MIT | Shrinks CH1–3 work under all-25 |
| Stagger poll/RX | At most 3 buses serviced per plant tick (RR) | Bounds lap when 6 buses active |
| Global MCP FRESET budget | ≥8 ms between **any** rail starting config reclaim | Stops 3 rails mode-switching together |

MCP flush/reclaim still runs on every apply that enqueues. Heavy mode does **not**
skip MCP TX — it lightens FDCAN and spreads poll/RX. Host desires update every USB
CMD; plant holds last between decimated applies.

### Rough FB history (all-25 hold, host ~40 Hz)

| Era | `fb_hz` | Notes |
|-----|---------|-------|
| Blocking TXQ / bus-off recover | ~2 | Superloop stuck in SPI/`HAL_Delay` |
| FRESET on every busy try | ~26–66 | TX alive; SPI mode-switch tax |
| UINC-only (no kick) | ~500+ | FB OK; TXQ stuck → one LED blink |
| Reclaim + coalesce + MCP decimate | ~289 | LEDs OK; all-25 still SPI-heavy |
| + heavy FDCAN + poll stagger + global FRESET budget | **~600–750** | Product all-25 target met |

MCP-alone can look **slower** than all-25 on `fb_hz`: MCP-only polls all three SPI
rails every tick; all-25 rotates poll/RX across six buses so average MCP SPI/lap drops.

### Code map

| Piece | Where |
|-------|--------|
| TXQ service / FRESET SM | `App/Src/plant/can/mcp2518fd.c` |
| MCP decimate + coalesce flush | `App/Src/plant/plugins/robstride.c` |
| Heavy-load FDCAN + bus poll RR | `robstride.c`, `App/Src/plant/actuator.c` |
| Timing matrix | `scripts/bench_load_matrix.py` |
| Lap timing in thermo PDU | `plant_timing_thermo_fill` bytes 16..21 |

---

## 8. CH4 MCP2518FD + MCP2562 — debug timeline (preserve)

Hardware: MCP2518FD controller + **MCP2562** transceiver on SPI (CS **PB11**, INT **PB10**, ACT **PB14**). Reference path: FDCAN1 CH1. Motor: RS02 `0x70` @ 1 Mbps classic extended.

Full ranked bugs + DoD: [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md) · personal notes: [ch4-mcp2518-bringup-notes.txt](ch4-mcp2518-bringup-notes.txt).

### Timeline (symptoms → breakthrough)

1. No probe HIT, no ACK — could not localize SPI vs MCP vs transceiver vs baud.
2. Driver / packaging rewrite — still no reliable TX/RX.
3. TX diagnostics improved — still no ACK on CH4.
4. CANable: TX visible on CH4, **recessive ACK** + error tail; CH1 OK.
5. Scope: CH4 dominant bit **~1.15 µs** vs CH1 **~1.0 µs**. Trim CiNBTCFG (TSEG1 17→15) → **enable/disable definitive** (0.02↔0.07 A).
6. TXQ wedging on multi-frame wake/disable — UINC drain; **FRESET TXQ only in Config**.
7. Scope showed TX + motor reply; firmware `rx=0` until **FnBP / FIFO SFR** aligned (FIFO ch0=TXQ; first RX = ch1 @ `0x05C`).

### Recessive ACK vs TXQ (how to tell)

| Observation | Likely cause |
|-------------|--------------|
| Full frame, ACK recessive, TEC+8 | Bit timing — motor never decoded |
| Truncated/aborted frame | TXQ/driver — MAC never finished |
| Full frame, ACK dominant, TEC unchanged | Bus OK |
| ACK OK, motor toggles, firmware `rx=0` | RX FIFO/filter bug (not baud) |

**Never** treat TXQEIF (queue empty) as TX done — one-deep TXQ starts empty → false `tx_ok`. Trust ammeter (~0.02 A rest / ~0.07 A enabled) over misleading `mms=rest`.

### Bitrate generations

| Gen | CiNBTCFG | Scope |
|-----|----------|-------|
| `b1b5294` | BRP=2 → ~333 kHz | Invalid |
| `4bdba03f` | TSEG1=17 / 20 TQ “1 Mbps” | **~1.15 µs** |
| `6458890+` | TSEG1=15 / 18 TQ | **~1.0 µs** — motor ACK |

Constants: `App/Inc/plant/can/mcp2518fd.h` (`MCP2518_NBT_*`).

### Bench smoke (SDK)

```powershell
python rs02_channel_bringup.py --bus 4 --motor-id 0x70 --skip-cali
# or: hub.debug.discover_robstride(bus=4); hub.debug.calibrate_robstride(bus=4, motor_id=0x70)
```

---

## 9. CH2 cali after teleop — resolved Jul 2026

After plant teleop on CH2, calibrate showed prep HIT but no shaft spin. Cause: FDCAN path skipped the pre-`0x05` reset that MCP gets (`CALI_SKIP_RESET`). Fix: same **250 ms settle + reset before 0x05** as MCP; teleop exit runs `recovery_on_exit`.

```powershell
python rs02_channel_bringup.py --bus 2
# or: hub.recover(); hub.debug.calibrate_robstride(bus=2, motor_id=0x70)
```

Expect `... cali listen` and shaft spin. Cold cali fail (no prior teleop) → harness/termination, not this path.

---

## 10. Quick checks / mismatches

| Check | Expect |
|-------|--------|
| PC3 blink | Plant alive |
| CAN ACT LEDs | Traffic (MCP needs non-blank desire or probe) |
| `cfg_get_table` / `config show` | 26 slots (layout v3); enabled subset is CFG-defined |
| Damiao discover | FOUND + esc/master |
| CH4 smoke | `tx_ok`, TEC, probe HIT (see §8) |

| Symptom | Likely cause |
|---------|----------------|
| No feedback one bus | Not woken; wrong `--bus` vs schematic |
| Damiao `tx>0` `rx_raw=0` | Motor-end 120 Ω; scan-order flood |
| Mid-chain teleop fault | Un-enabled unit between configured motors |
| Cal **NOISE** | Daisy contention — power-cycle; one motor; `--recovery` first |
| Ctrl+C mid-probe wedges MCU | `--recovery` / USB replug |
| Wrong bus LED | CH2/CH3 Cube swap — use schematic bus in scripts |

---

## 11. Unfinished product work

- Plant-teleop soft-limit stop; `joint home`; batch joint script
- Encoder zeros ↔ MuJoCo (absolute gated)
- Arm2 gains independently bench-verified (mirrored from arm1 today)
- CubeMars: workstream draft only — [lessons.md](lessons.md)
- Host image v2 (672 B) — [decisions.md](decisions.md)

---

## Related

- [lessons.md](lessons.md) — open bugs + closed one-liners
- [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md) — full MCP2562/CH4 story
- [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) — mixed CAN detail
- [architecture.md](architecture.md) · [host-exchange-v3.md](host-exchange-v3.md) · [decisions.md](decisions.md)
- **[docs/peripherals/](peripherals/continuous-ops.md)** — live-verified (2026-07-24, real
  Jetson board) operating manuals per peripheral: arm CH1 Damiao, DXL neck, base
  RobStride (bus5/6), base Damiao (CH6), PDU UART/soft-kill, and continuous-ops (launch/stop,
  stream health, "what good looks like"). Start at
  [continuous-ops.md](peripherals/continuous-ops.md) if you're about to run the live board.
