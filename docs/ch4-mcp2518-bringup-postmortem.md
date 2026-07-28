# CH4 MCP2518 SPI-CAN bringup postmortem

Historical record of firmware issues bringing up **CH4** (MCP2518FD + MCP2562 on SPI) vs working **CH1** (STM32 FDCAN1). Motor bench: RobStride RS02 @ CAN ID `0x70`.

## Commits referenced

| Commit | Date (approx) | Summary |
|--------|---------------|---------|
| `b1b5294` | 2026-06-30 | First SPI-CAN implementation — TX sort of working, RX/ACK not |
| `4bdba03f` | 2026-07-01 | Rewrite — scope sees frames, no reliable motor ACK/reply |
| `6458890` | 2026-07-02 | TX path to motor; bit trim; TXQ fixes |
| `725fd8f` | 2026-07-02 | MCP wake/disable reliable |
| `5ce6a93` | 2026-07-02 | RX path (FnBP/FIFO fix); probe HIT; teleop feedback |

## Hardware / software context

| Item | Detail |
|------|--------|
| CH1 (reference) | FDCAN1 PB8/PB9 — probe HIT worked historically |
| CH4 (target) | MCP2518FD + MCP2562: CS **PB11**, INT **PB10**, ACT **PB14** |
| Host | USB CDC COM5 @ 115200, `scripts/rs02_can_scan.py` |
| Motor | RS02 extended CAN 2.0 @ 1 Mbps, ID `0x70` |
| Ground truth | Supply **~0.02 A** rest / **~0.07 A** running; CiTREC **TEC**; PB14 blink |

## Bringup timeline (symptoms)

1. No probe HIT, no ACK — could not localize SPI vs MCP vs transceiver vs baud.
2. Message packaging / driver rewrite — still no reliable TX/RX.
3. TX diagnostics improved — still no ACK on CH4.
4. CANable: TX visible on CH4, recessive ACK + error tail; CH1 OK.
5. Bit time measured **~1.15 µs** → trimmed to **~1.0 µs** (match CH1) → **enable/disable definitive**.
6. TXQ wedging — wake/disable flaky until UINC/FRESET rules fixed.
7. Scope showed TX + motor reply; firmware `rx=0` until **filter FnBP** fixed.

## Recessive ACK vs TXQ — how to tell

Both can produce error tails on scope; mechanisms differ.

| Observation | Likely cause | Firmware hint |
|---------------|--------------|---------------|
| Full frame, **ACK recessive**, error tail | Bit timing mismatch — no node decoded frame | `tx_ok=1`, **TEC +8** |
| Truncated / aborted frame, error tail | TXQ/driver — MAC did not finish valid TX | `tx_ok=0` or lying `tx_ok` (v1) |
| Full frame, **ACK dominant** | At least one node on bus | `tx_ok=1`, **TEC unchanged** |
| ACK OK, motor toggles, **`rx=0`** | RX FIFO/filter bug (not baud) | `tec=0`, probe no HIT |

Retroactively, **recessive ACK at ~1.15 µs** fits bitrate mismatch best. **v1 false `tx_ok`** made it hard to separate driver lies from bus issues until TXQ completion was fixed.

## Bitrate timeline

| Generation | CiNBTCFG | Nominal | Scope (measured) |
|------------|----------|---------|------------------|
| `b1b5294` | BRP=**2**, TSEG1=17, TSEG2=2 | **~333 kHz** | Invalid CAN |
| `4bdba03f` | BRP=0, TSEG1=**17**, TSEG2=2 (20 TQ) | 1 Mbps on paper | **~1.15 µs** |
| `6458890+` | BRP=0, TSEG1=**15**, TSEG2=2 (18 TQ) | ~1.11 Mbps on paper | **~1.0 µs** — motor ACK |

**Breakthrough:** Trimming dominant bit from **1.15 µs → 1.0 µs** (match CH1 FDCAN) was when RS02 **ACK'd dominantly** and **enable/disable** (0.02 ↔ 0.07 A) became reliable.

Authoritative constants: `App/Inc/plant/can/mcp2518fd.h` (`MCP2518_NBT_*`).

---

## Issues in previous code (detailed)

### 1. CiNBTCFG completely wrong (`b1b5294`)

**Bug:** `BRP=2` → bit rate ~333 kHz, not 1 Mbps.

**Symptoms:** No motor ACK; frames not valid 1 Mbps CAN.

**Fix (`4bdba03f`):** BRP=0, 20 TQ.

---

### 2. Bit time ~1.15 µs vs CH1 ~1.0 µs (`4bdba03f` until `6458890`)

**Bug:** TSEG1=17 (20 TQ) programmed; scope **~1.15 µs** per dominant bit (~15% slow vs motor/CH1).

**Symptoms:** Frame-like activity on bus; **ACK recessive**; error frame tail; `TEC +8`; no reliable enable/disable.

**Fix:** TSEG1=**15** (18 TQ) → **~1.0 µs** on scope.

**Impact:** First layer where the **motor participated** (dominant ACK, supply toggle).

---

### 3. TXQ false completion (`b1b5294`; partially `4bdba03f`)

**Bug:** Treated **TXQEIF** (queue empty) as TX done. One-deep TXQ **starts empty** → instant false `tx_ok=1`.

**Symptoms:** `tx_ok=1`, `tec=0`, no PB14 blink, CANable silent.

**Fix:** Wait for **TXATIF** / bus error; `saw_busy`; poll CiTREC + CiBDIAG1.NACKERR.

**Impact:** Trustworthy `tx_ok` and TEC for separating driver vs bus bugs.

---

### 4. TXQ commit, release, FRESET (`4bdba03f` → `6458890` / `725fd8f`)

**Bug:**

- Wrong TXREQ/UINC sequencing.
- **FRESET on TXQ in Normal mode (OPMOD=6)** wedged queue (`txq_sta=0x05`).
- After first frame, frames 2–3 failed (`tx_ok=1 tx_fail=2` on wake).

**Symptoms:** Smoke OK after reinit; wake/disable unreliable.

**Fix:**

- UINC\|TXREQ commit after RAM load.
- **UINC-drain in Normal only**; FRESET only in Config.
- CiTXATIF W1C; `mcp_txq_hard_reset()`; send retry; `plant_diag_mcp_soft_recover()` (no full reinit in normal).

---

### 5. CiTREC read wrong (`b1b5294`)

**Bug:** 32-bit read with TEC at bits 15:8 — wrong layout.

**Fix:** Byte read: byte0=REC, byte1=TEC @ 0x034.

---

### 6. `mcp2518_drain_rx()` no-op (`b1b5294`)

**Bug:** Read FIFOSTA/C1INT only; never UINC or RAM pop.

**Fix (`4bdba03f`):** Loop `mcp_hw_pop_rx()`.

---

### 7. RX FIFO SFR index ≠ filter FnBP (`4bdba03f` → `5ce6a93`) — critical RX regression

**Bug:** v2 rewrite used:

```c
REG_C1FIFOCON(1) = 0x068  /* configured FIFO2 */
FLTCON0 = 0x81            /* FnBP=1 → deliver to FIFO1 @ 0x05C */
```

v1 accidentally matched: `MCP_FIFO_CON(1) = 0x05C` + FnBP=1.

**Symptoms:** `tx_ok=1`, `tec=0`, wake/disable OK; **`rx=0`**, probe no HIT; scope shows TX + reply.

**Fix:**

```c
MCP_RX_FIFO_REG  = 0   /* CiFIFOCON(0) @ 0x05C = hardware FIFO1 */
MCP_RX_FILTER_BP = 1   /* FnBP=1 — NOT 0 (0 = TXQ, cannot RX) */
```

MCP2518 rule: **FIFO channel 0 = TXQ**; first RX FIFO = **channel 1** @ SFR 0x05C = `REG_C1FIFOCON(0)`.

---

### 8. Restrictive RX filter (`4bdba03f`, secondary)

**Bug:** EXIDE + MIDE-only filter; std-ID loopback test failed → misleading `ext_lb=0`.

**Fix:** FLTOBJ=0, MASK=0, FnBP=1.

---

### 9. Host / session fragility (operational)

- Ctrl+C mid-probe wedges `plant_diag` → no USB until replug (~30 s).
- Teleop synced to bogus comm 0x02 p_raw (~−12 rad) → jolt; clamp in `host_teleop_laptop_usb.py`.
- `mms=rest` in logs vs **0.07 A** — trust ammeter.

---

## Ranked impact

1. Bitrate **1.15 µs → 1.0 µs** — motor ACK, enable/disable.
2. TXQ honest completion + multi-frame release — reliable wake/disable; trustworthy TEC.
3. RX FnBP / FIFO SFR alignment — probe HIT, feedback, teleop.
4. v1 wrong BRP (~333 kHz) — nothing worked.
5. drain_rx stub (v1).
6. Wrong TREC read (v1).
7. FIFO index regression in v2 rewrite.

## Definition of done (CH4)

| Layer | Pass test |
|-------|-----------|
| SPI | DEVID readback OK |
| Init | `opmod=6`, `ext_lb=1` |
| Bit timing | Scope ~1000 ns; smoke `tec=0` with motor |
| TX | `tx_ok=1`, PB14 blink; wake/disable 0.02↔0.07 A |
| RX | smoke `rx≥1`; `--probe-id 0x70 --bus 4` HIT comm 0x02 |

## Key files

- `App/Src/plant/can/mcp2518fd.c` — driver
- `App/Inc/plant/can/mcp2518fd.h` — bit timing, diagnostics
- `App/Src/plant/plant_diag.c` — MCP smoke/wake/disable
- `App/Src/plant/can/spi_can_router.c` — RX poll → ring
- `scripts/rs02_can_scan.py` — bench CLI

## Related docs

- [bringup.md](bringup.md) — flash, teleop, calibrate
- [lessons.md](lessons.md) — operational quirks
- [architecture.md](architecture.md) — RS2 PDU vs plant loop

## Copy-paste personal notes

Plain-text version for external notes: [ch4-mcp2518-bringup-notes.txt](legacy/bench/ch4-mcp2518-bringup-notes.txt)
