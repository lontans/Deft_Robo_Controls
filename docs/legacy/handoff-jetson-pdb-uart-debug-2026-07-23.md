# Handoff: Jetson ↔ Controls PDB UART debug (2026-07-23)

**Owner for this pass: Claude (Agent).** Cursor already did firmware baud/clock, ESTOP input, probe experiments, and COM5 proves. Your job is to **find why Jetson `/dev/ttyTHS1` does not decode MCU UART4 TX** and get a live `PDBF`↔`PDBC` prove.

Do **not** expand CubeMars / ZeroErr / LED work unless the user reassigns.

---

## Access

### Jetson (SSH)

| | |
|---|---|
| Host | `192.168.50.48` |
| User | `deft-robotics` |
| Password | `4565` (also via env `JETSON_PASS`) |
| Repo | `/home/deft-robotics/controls_pcb` |
| UART1 (40-pin) | `/dev/ttyTHS1` → DT `serial@3100000` |
| Other UART | `/dev/ttyTHS2` → `serial@3110000` (silent in all tests — wrong port) |

```bash
ssh deft-robotics@192.168.50.48
# password: 4565
cd ~/controls_pcb/scripts
```

From the Windows PC (PowerShell), prefer paramiko helpers over interactive SSH:

```powershell
$env:JETSON_PASS='4565'
$env:PYTHONPATH='scripts'
python scripts/_tmp_jetson_listen_hist.py          # raw THS1/THS2 histogram
python scripts/_tmp_jetson_baud_hunt.py            # baud sweep for UART4_PROBE / PDBC
python scripts/_tmp_jetson_git_pull.py             # git pull on Jetson
```

### Controls board (Windows host)

| | |
|---|---|
| USB CDC | **COM5** — `0483:5740` |
| ST-Link VCP | COM53 (not plant) |
| Soft-DFU | `python -m deft_controls_sdk.bench.soft_dfu …` (from `scripts/`) |
| SWD flash | `STM32_Programmer_CLI -c port=SWD mode=UR -w Debug\DeftRoboticsControlsPCB.elf -v -rst` |

**COM5 ownership:** announce `COM5: Cursor` / `COM5: Claude` / `COM5: free` when taking or releasing the port. Soft-DFU enter can brick CDC visibility if DFU doesn’t enum — prefer ST-Link SWD flash if CDC dies.

---

## Expected wiring

```
Jetson UART1 TX  (BOARD pin 8)  →  Controls PC11 (UART4 RX)
Jetson UART1 RX  (BOARD pin 10) ←  Controls PC10 (UART4 TX)
GND common
ESTOP: Jetson BOARD pin 16 ↔ Controls PB7 (PDU drives; MCU = input only)
```

115200 8N1. PDB frames: 64 B `PDBC` (Controls→PDB) / `PDBF` (PDB→Controls). Contract: `docs/pdb-uart-v1.md`, Python: `scripts/deft_controls_sdk/pdb/`.

---

## Firmware state (already on board as of this handoff)

| Item | Value |
|------|--------|
| `UART4_MODE` | `UART4_MODE_PDB` |
| `UART4_PROBE_STREAM` | **0** (normal PDBC TX @ 50 Hz) |
| `UART4_PIN_LEVEL_INVERT` | **0** (tried 1 — no change on Jetson) |
| UART4 kernel clock | **HSI 16 MHz** (not PCLK/HSE) — forced in `pdb_link_init` + `usart.c` MSP |
| BRR (USB-proven) | **0x008B (139)** → 16000000/115200 ≈ 115108 baud |
| USB diag | `system.reserved0` = clk_src (`1`=HSI); `reserved[0..1]` = BRR LE |
| PB7 | **GPIO input**, PDU-driven ESTOP sense |

Key files:

- `App/Inc/host/uart4_mode.h`
- `App/Src/host/pdb_link.c`
- `Core/Src/usart.c` (HSI override + pull-up / high-speed AF)
- `Core/Src/gpio.c` (PB7 input)

Build: Debug makefile under `Debug/` with ARM GCC from STM32CubeIDE plugins (see recent Cursor shell history), or CubeIDE Release. Flash via SWD if soft-DFU flakes.

---

## What is already proven

1. **MCU USB plant path alive** — HBHF frames on COM5.
2. **UART4 baud math correct** — HSI + BRR=0x8B visible over USB.
3. **MCU is transmitting on a schedule** — Jetson THS1 receives **exactly** `50 Hz × 64 B` when Controls TX is active (`n=9600` in 3 s → 3200 B/s).
4. **Content is wrong at Jetson** — those bytes are **almost all `0x00`**, never `PDBC` / `UART4_PROBE` / heavy `0x55`.
5. **THS2 silent** — not the 40-pin UART1 path.
6. **TX↔RX swap** → silence (likely TX–TX / RX–RX). Swap back → NUL flood returns.
7. **TXINV/RXINV=1** → still NUL flood; reverted to 0.
8. **Continuous blast vs 50 Hz idle gaps** — gaps alone did not yield ASCII; rate still locks to 50×64 NULs.
9. **Hard ESTOP:** PB7 input; Jetson `jetson_estop_drive.py` / `jetson_estop_sense.py` exist. Sense stuck low until PDU/Jetson drive path is solid — separate from UART decode.
10. **PDB Python contract** — `pytest scripts/tests/test_pdb_link_frames.py` (16) green; flags bit0 now packed/parsed.

---

## What failed / red herrings

| Hypothesis | Result |
|------------|--------|
| Missing PDB firmware | False — `UART4_MODE_PDB` linked, TX runs |
| Wrong baud (HSE_VALUE vs crystal) | Mitigated — HSI+BRR proven on COM5; Jetson still NULs |
| Soft-ESTOP / control-loop clamping TX | False — idle voltages were ~3.3 V / 2.7 V loaded; TX cadence present |
| Probe stream required | Off now; same NUL pattern with real PDBC |
| Pin invert | Tried, no help |
| Continuity “short” on meter | Often driver-held-low / clamp; not a PCB copper short when ends probed alone |

User DC notes (useful): jumpers on → MCU TXD ~2.7 V, RXD ~3.5 V; jumpers off → TXD ~3.5 V, RXD ~0.3 V (floating RX).

---

## Success criteria

1. Jetson `python3 jetson_uart_listen.py --ports /dev/ttyTHS1 --seconds 3` shows **HIT** with `PDBC` (or non-zero ASCII), not `all_zero` / `mostly_zero`.
2. `pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20` logs **PDBC seen** (not `no PDBC seen yet`).
3. COM5: `pdb[64]` magic = `PDBF` (`0x46424450`), `system.kill_state == NORMAL` while sim fresh; after sim stop >200 ms → `HARD_ESTOP` / `COMMS_LOSS` again.

---

## Suggested debug plan (Claude)

Work in this order; stop when success criteria hit.

### A. Baseline (5 min)

1. Confirm COM5 free; read USB diag: `clk_src==1`, `BRR==0x8B`.
2. On Jetson: `pkill -f pdb_uart_sim.py`; run histogram listen on THS1/THS2 (`_tmp_jetson_listen_hist.py` or `jetson_uart_listen.py`).
3. Record: `n`, `nz`, `find(PDBC)`, top byte histogram.

### B. Localize the electrical node (scope or meter)

1. With jumpers **on**, measure **Jetson pin 10 (RX)** vs Jetson GND during MCU TX: idle should be ~3.3 V mark; should toggle when Controls transmits.
2. Same for **Controls PC10** vs Controls GND.
3. If PC10 toggles but Jetson pin 10 does not → jumper/header/wrong pin.
4. If both toggle but software sees NULs → Jetson UART config / invert / mux / baud clock.

### C. Jetson-side UART identity

1. Confirm 40-pin UART1 enabled (jetson-io / DT `serial@3100000` status okay).
2. Check nothing else holds THS1 (`fuser -v /dev/ttyTHS1`).
3. Optional: `stty -F /dev/ttyTHS1 115200 raw -echo`; `timeout 3 cat -v /dev/ttyTHS1 | xxd | head`.
4. Baud hunt again after any mux change (`_tmp_jetson_baud_hunt.py`).

### D. Bidirectional smoke

1. Jetson TX known pattern on THS1; confirm Controls MCU RX path (harder — need PDBF from sim or temporary RX diag). Easiest: run `pdb_uart_sim.py` and watch COM5 `pdb[64]` / kill_state.
2. If sim TX (Jetson→MCU) makes COM5 show `PDBF` but MCU→Jetson still NULs → **TX path only** (PC10→pin10).
3. If neither direction works → ground / wrong connector / both pins wrong.

### E. Firmware experiments (only if A–D point at MCU)

Keep changes minimal; prefer `#define` in `uart4_mode.h`:

1. Short `UART4_PROBE_STREAM=1` beacon to distinguish ASCII vs PDBC (remember to set back to 0).
2. `UART4_PIN_LEVEL_INVERT` only if meter shows idle polarity flipped at Jetson RX vs MCU TX.
3. Do **not** re-blame HSE without reading BRR from USB first.

### F. ESTOP (secondary)

- Scripts: `jetson_estop_drive.py`, `jetson_estop_sense.py`, `_tmp_hard_estop_jetson_prove.py`.
- GPIO must stay held open while sampling (`--seconds N`); process exit floats the pin.
- Pass/fail = COM5 `system.estop_sense` tracking pin16, **not** `kill_state` (stale PDB still forces HARD_ESTOP).

---

## Helper scripts (repo)

| Script | Role |
|--------|------|
| `scripts/pdb_uart_sim.py` | Jetson PDB stand-in (PDBF TX / PDBC RX) |
| `scripts/jetson_uart_listen.py` | Multi-port / baud listen |
| `scripts/jetson_estop_drive.py` / `jetson_estop_sense.py` | Hard ESTOP GPIO |
| `scripts/_tmp_jetson_listen_hist.py` | Quick THS1/THS2 histogram via SSH |
| `scripts/_tmp_jetson_baud_hunt.py` | Baud sweep |
| `scripts/_tmp_hard_estop_jetson_prove.py` | Jetson drive + COM5 estop_sense |
| `scripts/_tmp_pdb_led_live_prove.py` | COM5 PDB mirror + LED (use `connection.send_once` not hub double-poll) |

---

## Product constraints (do not violate)

- Equal-rate FB freshness for commanded actuators — no MCP÷, bus RR, priority actuators.
- Soft-kill: PDB must not open contactors without Controls `kill_request=SOFT_KILL_READY`.
- Hard ESTOP wire: **PDU drives**; Controls PB7 input-only.
- Don’t commit secrets; don’t force-push; don’t amend unless user asks.

---

## Report back

When done (or blocked), report:

1. THS1 histogram before/after your change.
2. Whether `PDBC` appeared (hex/ASCII snippet).
3. COM5 `pdb` magic + `kill_state` with sim on/off.
4. One-line root cause (wiring pin / Jetson mux / polarity / other) and what you changed.

---

## Claude pass results (2026-07-23, this handoff)

**No firmware/App changes made** — every check pointed away from firmware, so
Phase E (`#define` experiments) was not attempted; see reasoning below.

### A — Baseline (unchanged from prior handoff, re-confirmed)

- `clk_src=1` (HSI), `BRR=0x008B` (139) — read live off COM5, matches this
  doc's "already proven" table exactly.
- THS1: **9600 B in 3.0 s, nz=0** (`n=9600` = exactly 50 Hz × 64 B × 3 s —
  frame-accurate, not noise), all-zero content, no `PDBC`, no `x55`. THS2:
  silent (`n=0`). Identical to the handoff's existing findings — no change
  from re-running.

### C — Jetson identity (new this pass)

- `fuser -v /dev/ttyTHS1 /dev/ttyTHS2` → nothing holds either port.
- DT status: `bus@0/serial@3100000/status = okay`, `bus@0/serial@3110000/status = okay`
  (both UART nodes enabled — not a disabled-overlay issue).
- `deft-robotics` user is in the `dialout` group (permissions fine).
- **Raw `stty -F /dev/ttyTHS1 115200 raw -echo; cat | xxd`, bypassing pyserial
  entirely** — identical all-zero pattern. Rules out a pyserial-specific
  config bug (parity/flow-control default etc.) — this is a kernel-tty-level
  symptom, not a Python library issue.

### D — Bidirectional smoke test (new this pass, the key result)

Ran `pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20` on the Jetson (confirmed via
its own unbuffered log: opened the port, transmitted **181 real `PDBF`
frames** over ~9 s) concurrently with a COM5 poll (344 samples) on this PC:

- Jetson sim log: `tx_seq` climbing steadily 1→181, **`no PDBC seen yet` the
  entire time** (Controls→Jetson direction still dead, consistent with A).
- COM5 `pdb[64]` mirror: **stayed all-zero magic the entire 8 s window**
  while the Jetson was genuinely transmitting valid `PDBF` frames
  (Jetson→Controls direction is *also* dead).

**Both UART directions fail simultaneously**, each independently confirmed
live and transmitting on its own end.

### F — ESTOP cross-check (new this pass)

Drove Jetson **BOARD pin 16** through 3 full assert(LOW)/release(HIGH) cycles
(`jetson_estop_drive.py pulse`, confirmed via its own log) while polling
Controls `system.estop_sense` on COM5 continuously:

- Jetson log: clean `drive=0` / `drive=1` alternation, 3 cycles, as commanded.
- Controls `estop_sense`: **one single value the entire run — stayed `0`**,
  never moved even during the 3 `drive=1` (HIGH/released) phases.

**The ESTOP wire (a third, physically separate signal from UART TX/RX) fails
in the identical way** — driver-side confirmed toggling, receiver-side never
sees it move.

### One-line root cause

**Three independent signal wires (UART4 TX, UART4 RX, ESTOP) all show the
same "transmitter genuinely active, receiver never sees a real transition"
failure simultaneously** — that's not consistent with three separate
per-wire faults (wrong pin, polarity, baud — all already tried/ruled out per
this doc's earlier sections) and *is* consistent with one shared, common-mode
fault across all of them. The wiring list has exactly one wire common to all
three signal paths: **GND**. A missing, high-impedance, or wrong-pin ground
connection between the two boards would explain every symptom observed
(clean rate-locked framing with dead content on UART, a driven-but-unseen
digital I/O on ESTOP) without requiring three coincidental independent
failures. This is a physical continuity check, not a firmware or software
fix — **recommend a multimeter continuity/resistance check between Jetson
GND (any header GND pin) and Controls board GND before touching firmware
further.** If GND checks out solid, re-open with a scope trace on PC10 vs
Jetson pin 10 per this doc's Phase B (needs hands-on hardware access this
agent doesn't have).

No firmware changes were made — `UART4_PIN_LEVEL_INVERT` was already tried
per this doc with no effect, and the new evidence (identical failure across
three unrelated wires including a plain GPIO with no baud/framing/mux
involved at all) doesn't point at anything a `#define` in `uart4_mode.h`
could fix.

---

## USB common-GND re-prove (2026-07-23, after meter notes)

**Setup change (user):** Controls CDC plugged into Jetson USB (shared return via
USB). Separate GPIO/GND jumpers removed. No external actuator/LED power.
User meter notes before this: Jetson vs Controls GND ~26 Ω; with USB power
path ~17 Ω “load” difference — **not** a solid <1 Ω bond.

**How run:** Cursor synced scripts → Jetson; full prove on Jetson only
(`/dev/ttyACM0` CDC + `/dev/ttyTHS1`). Helper:
`scripts/_tmp_jetson_usb_gnd_reprove.py`.

| Check | Result |
|-------|--------|
| CDC on Jetson | **Yes** — `0483:5740` → `/dev/ttyACM0`, HBHF live (`n≈90`/2 s) |
| UART4 diag | `reserved0`/clk path still looks HSI (`…03050001…` → kill=3 reason=5 estop=0 reserved0=1) |
| THS1 listen 3 s | **Still bad** — `n=9709` (~50×64), `nz=2`, tag `mostly_zero`, hex still zeros; one stray `C` in ASCII dump — not a clean `PDBC` stream |
| THS2 | silent |
| `pdb_uart_sim` TX | Live (`tx_seq` climbing); **`no PDBC seen yet`** |
| CDC `pdb[64]` during sim | **Still magic `0x0`** entire window |
| ESTOP pin16 HI/LO/HI | Jetson drove; Controls `estop_sense` **stuck `[0]`** |

**Verdict:** USB power/GND path did **not** clear the shared fault. All three
success criteria still fail. Next hardware steps (in order):

1. Confirm **signal** jumpers still on (pin8→PC11, pin10←PC10, pin16↔PB7)
   after the GND rewire — USB only commons ground/power, not those nets.
2. Improve GND bond until meter reads **≪1 Ω** Jetson header GND ↔ Controls
   GND (17–26 Ω is still a bad return for UART/GPIO).
3. Scope PC10 vs Jetson pin 10 (Phase B) if (1)+(2) are solid and symptoms remain.

### Retest after user “improved connection” (same day)

Same helper (`_tmp_jetson_usb_gnd_reprove.py`). **No improvement:**

- THS1: `n=9664` / 3 s, `nz=0`, `all_zero` (worse than prior `nz=2` blip)
- sim: TX live, `no PDBC seen yet`; CDC `pdb` magic still `0x0`
- ESTOP: `estop_sense` still stuck `[0]` through HI/LO/HI

CDC on Jetson still healthy. Fault remains on the three header signal paths
(or still-bad return), not USB plant.

### PB7 hard-tied to Jetson 3V3 (sense-path prove)

User tied Controls **PB7** directly to a Jetson **3V3** rail. Live CDC poll on
`/dev/ttyACM0`:

- `estop_sense=1` every sample, `reserved0=1` (HSI), BRR tail `0x008B`
- Firmware GPIO read + USB packing path is **good** — not a stale/stub
  `estop_sense` or wrong byte offset

Full GPIO BOARD sweep earlier (22 pins) never produced `estop_sense=1`; hard
3V3 does. So Jetson GPIO→PB7 drive path was the prior failure, not MCU sense.

Retest with 3V3 still on PB7: UART still mostly-zero / no `PDBC` / no `pdb`
mirror; pin16 assert cannot pull `estop_sense` low against the hard 3V3 tie
(stuck `[1]` as expected).

### Clash notes (DXL vs PDB)

- Dynamixel = **UART5** (`PC12`/`PD2`); PDB = **UART4** (`PC10`/`PC11`) —
  different instances, not a FreeRTOS UART clash with DXL.
- Latent pin conflict: `thermo.h` still claims **PB7** as MAX31855 soft-CS.
  Default `SPI3_ROLE_LED` so thermo is idle now; `gpio.c`/`pdb_link_init` set
  PB7 INPUT. If THERMO role is enabled later, CS toggles would fight ESTOP.

### Pin16 back on ESTOP (post–3V3 prove)

User restored Jetson **BOARD pin 16 (GPIO08)** → PB7. Retest:

- MCU diag still healthy: `clk_src=1`, `BRR=0x008B` (HSI change is MCU-local;
  Jetson `serial@3100000` still `status=okay` — not a Jetson clock corruption)
- pin16 HI/LO/HI: Jetson drives; **`estop_sense` stuck 0** again
- Contrast: hard Jetson **3V3 → PB7** produced `estop_sense=1` reliably
- UART still mostly-zero / no `PDBC` / no `pdb` mirror

Conclusion: MCU sense + USB packing OK; Jetson GPIO08 HIGH is not presenting
like a solid 3V3 on PB7 (contact/level/drive), separate from UART4 clock work.

### Jetson GPIO root cause (BOARD18 / CVM GPIO35) — pinmux, not MCU

Proved electrically by user: hard **3V3 → PB7** → `estop_sense=1`; meter on
Jetson GPIO drive → **0 V**. Treat MCU ESTOP sense as correct.

Software mapping (JP R36.5, Jetson.GPIO 2.1.9):

| Header | CVM name | SoC pad | gpiochip line | linux gpio |
|-------:|----------|---------|--------------:|-----------:|
| 18 | GPIO35 | **PH.00 / SOC_GPIO21_PH0** | gpiochip0 **43** | gpio-391 |
| 16 | GPIO08 | PBB.01 (AON) | gpiochip1 9 | gpio-325 |
| 7 | (ref) | PQ.06 / SOC_GPIO33 | gpiochip0 106 | gpio-454 |

While `gpioset gpiochip0 43=1` (and Jetson.GPIO OUT HIGH):

- `/sys/kernel/debug/gpio`: `PH.00 | gpioset | out hi` ← controller believes HIGH
- `pinmux-pins`: `SOC_GPIO21_PH0: (MUX UNCLAIMED) tegra234-gpio:391` ← **mux not
  selected** (GPIO/`gp` vs `rsvd0`/`i2s7` never claimed)
- MCU `estop_sense` stays 0; user meter 0 V

So this is the classic Orin **pinmux/pad not routed to GPIO** failure:
`Jetson.GPIO` / `gpioset` only poke the GPIO block; they do **not** program
pinmux. Fix on Jetson: configure 40-pin pins as GPIO via
`/opt/nvidia/jetson-io/jetson-io.py` (or pinmux DTB / spreadsheet flash),
reboot, then retest pin18. Not an MCU UART4-HSI side effect.

**Applied 2026-07-23 (pending reboot):** non-interactive jetson-io GPIO config
via `scripts/_tmp_jetson_io_gpio_config.py`:

- DTBO `/boot/jetson-io-hdr40-user-custom.dtbo`
- `extlinux.conf` `DEFAULT JetsonIO` + `OVERLAYS` that dtbo
- GPIO mode: pins **18** (pwm5), **16/32** (dmic3), 7, 15, 29/31, 33/37
- Kept SFIO: **uarta pins 8/10** (PDB UART)

User must reboot/power-cycle; then retest `gpioset gpiochip0 43=1` / pin18.

**Post-reboot prove (2026-07-23 ~17:14):** JetsonIO overlay active.
Pinmux: `SOC_GPIO21_PH0 … function gp … (HOG)`. **ESTOP PASS** — BOARD pin18
HI/LO → MCU `estop_sense` 1/0/1/0. UART/PDB still fail (THS1 mostly-zero,
sim `no PDBC`, CDC `pdb` magic `0x0`).

### UART pinmux fix pending reboot (2026-07-23 ~17:20)

Custom dtbo alone never retained `uarta` (default SFIO pins stripped by
`create_dtbo`). Boot now chains:

`OVERLAYS /boot/tegra234-p3737-0000+p3701-0000-hdr40.dtbo,/boot/jetson-io-hdr40-user-custom.dtbo`

Stock hdr40 carries `hdr40-pin8/10` + `uarta`; custom keeps pin18 GPIO.
MCU reflashed Debug ELF via SWD (`feat/cubemars_full_support`, UART4 PDB/HSI).
**User reboot Jetson**, then re-prove THS1/`pdb_uart_sim`.

FDCAN ACT LED blink during holds = plant loop alive; unrelated.

### End-to-end MCU↔Jetson UART audit (2026-07-23 ~17:30–17:45, Cursor)

Pins restored to nominal (Jetson pin8 TX→PC11, pin10 RX←PC10). Debug ELF
with RX counters + `HAL_UART_ErrorCallback` re-arm flashed via SWD.

#### Jetson node (compatible with MCU 115200 8N1)

- `/dev/ttyTHS1` present; pinmux `UART1_TX_PR2` / `UART1_RX_PR3` = `function uarta`
- Overlays chained: stock `hdr40.dtbo` + `jetson-io-hdr40-user-custom.dtbo`
- termios: `115200 cs8 -parenb -cstopb -crtscts` (raw 8N1) — **matches MCU**
- ESTOP path already proven separately (pin18 ↔ PB7)

#### IOC / MCU UART4 critique (vs Jetson)

| Item | IOC / Cube default | Runtime (PDB) | Jetson | Verdict |
|------|--------------------|---------------|--------|---------|
| Pins | PC10=TX AF5, PC11=RX AF5 async | same | pin8→RX, pin10←TX | pin map OK |
| Baud | 115200 8N1 | 115200 8N1 | 115200 8N1 | format OK |
| Kernel clock | PCLK1 @ 85 MHz (`RCC.UART4Freq_Value`) | **forced HSI 16 MHz**, BRR=`0x8B` (139) | N/A | intentional; live `clk=1` |
| Flow control | none | none | none | OK |
| Invert | none | `TXINV/RXINV=0` (tried both; RXINV→FE) | no DT invert | not a soft invert bug |
| Clash | UART5=DXL on PC12/PD2 | — | — | no UART4 clash |
| RX path | NVIC UART4 prio 6 | `ReceiveToIdle_IT` + ErrorCallback re-arm | — | firmware OK |

**IOC is compatible with how Jetson drives UART.** The Cube PCLK1 default would
be wrong for baud if HSE were off, but firmware already overrides to HSI+BRR.

#### Prove results (CDC overlay on `/dev/ttyACM0`)

1. **UART RX byte counters look alive** (exact 2560 B / 40 events on 0x55 blast)
   but **`last_rx=0xFFFFFFFF`**, never `0x55` / `PDBF`; `valid=0`, `err=0`.
2. TX-off isolation: same 2560/0xFF pattern → **not PC10 self-crosstalk**.
3. TXINV made Jetson RX worse (all-zero); RXINV → sticky FE (`err=4`).
4. **GPIO DC probe on PC11** (`UART4_GPIO_RX_PROBE=1`, UART AF off, no pull):

| Jetson activity | PC11 lows / 10000 | PC11 highs |
|-----------------|-------------------|------------|
| idle (no TX) | **0** | **10000** |
| blast `0x00` | ~1050 | ~8950 |
| blast `0xFF` | ~1050 | ~8950 |
| blast `0x55` | ~1050 | ~8950 |

Duty ≈ **10% low for every pattern** (= start-bit only). Real UART data would
move duty with payload (`0x00` ≫ low, `0xFF` ≪ low, `0x55` ~50%).

**Conclusion:** MCU firmware + Jetson termios/pinmux are fine. Jetson TX is
**not DC-driving PC11** — only brief lows (start-bit / edge coupling). That
makes UART decode as `0xFF` and CRC never pass. MCU→Jetson “mostly `0x00` at
50 Hz×64” is the symmetric symptom on the return path. Next: scope/meter
Jetson pin8 vs Controls PC11 (and pin10 vs PC10) for real 0/3.3 V data swings;
check series caps / wrong connector net / level shifter unpowered — not another
baud/invert firmware knob.

Firmware left at normal PDB (`UART4_PDB_TX_ENABLE=1`, probes/inverts=0) with
bring-up CDC overlays + ErrorCallback retained.

### MCU loopback PASS / Jetson still FAIL (2026-07-23 ~17:46–17:50)

1. **Controls JST TX↔RX short** (Jetson UART unplugged): CDC `last/asc=PDBC`.
   MCU UART4 + PDB TX/RX path + JST nets to PC10/PC11 are **good**.
2. Replugged Jetson: still MCU `last=0xFFFFFFFF` on `0x55`/`PDBF`; Jetson
   THS1 `all_zero` at 50 Hz×64; sim `no PDBC`.
3. Restored clean **HSI** UART4 clock (`clk=1`, `BRR=139`) — same failure
   (rules out “loopback hid wrong PCLK/HSE baud” as the sole cause).
4. Baud/port hunt on THS1/THS2: **no `PDBC` at any tried rate**; 115200 still
   exact-rate zeros.

**Split:** MCU stack proven. Next = Jetson header loopback (short pin8↔pin10,
UART unplugged from Controls) to prove `/dev/ttyTHS1` electrically.

### Jetson header loopback FAIL (2026-07-23 ~17:53)

Pin8↔pin10 shorted, Controls UART unplugged. `/dev/ttyTHS1` @ 115200:
wrote 37 B, read 37 B of **mostly `0x00`** with only trailing scraps
(`ND_00`, `K\\r\\n`). **`JETSON_LOOPBACK_FAIL`.**

Same class of symptom as the MCU↔Jetson link. Controls MCU loopback
passed; Jetson cannot even echo to itself on the header. Root cause is on
the Orin/`ttyTHS1`/hdr40 path (pinmux vs copper, wrong header pins, dead
level-shift enable, or bad short) — not RTS/CTS, not Controls firmware.

### NOT a bad carrier — tegra HSUART bulk TX bug (2026-07-23 ~18:06)

Header pin8↔pin10 short + `/dev/ttyTHS1` (`nvidia,tegra194-hsuart`):

| Write style | Loopback result |
|-------------|-----------------|
| bulk `write(64)` | all `0x00` |
| bulk ≤30 B | first **24** bytes `0x00`, tail OK |
| **paced ≥0.5 ms/byte** | **PASS** (full 64 B `PDBF`) |

GPIO working was a true clue: copper/pinmux fine; **DMA/bulk path on
tegra194-hsuart is what mangled payload**. `pdb_uart_sim.py` now defaults
`--tx-pace-us=500` on `ttyTHS*`. Next: unshort, replug Controls, re-prove.

### Bidirectional PASS with paced TX both ways (2026-07-23 ~18:12)

- MCU: `UART4_TX_PACE_BYTES` — one TX byte per HostTask (~1 ms/byte).
- Jetson: sim `--tx-pace-us=500`.
- Prove: Jetson saw **46× `PDBC`** in 3 s (parsed OK); CDC `pdb=PDBF`,
  `kill=NORMAL`; sim logs `rx_seq` climbing. **Link up both directions.**
