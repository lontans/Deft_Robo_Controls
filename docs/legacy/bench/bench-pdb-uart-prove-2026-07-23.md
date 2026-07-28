# PDB UART prove-out — 2026-07-23

## Done without UART4 peer (earlier)

- Soft-DFU image on board: Release `-Os` + per-bus RX index.
- USB CDC probe (`COM5`): `pdb[64]` all zeros; `kill_state=HARD_ESTOP(3)`, `kill_reason=COMMS_LOSS(5)`.
- Agent1 unit tests: `python -m pytest scripts/tests/test_pdb_link_frames.py -q` → 16 passed.
- Sim script present: `scripts/pdb_uart_sim.py` (refuses COM5 by design).

## Live Jetson attempt (afternoon — Cursor)

### Flashed

- Applied `docs/patches/led-factory-patterns.patch` → Release rebuild → soft-DFU.
- Factory LED modes 3–7 in `led.c` (green / yellow / red solid + yellow slow blink + **red fast blink** fault).

### Jetson side (SSH `deft-robotics@192.168.50.48`)

- Repo: `/home/deft-robotics/controls_pcb`
- UART devices: `/dev/ttyTHS1`, `/dev/ttyTHS2`
- Sim started successfully:

```bash
cd /home/deft-robotics/controls_pcb/scripts
python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 --gpio-estop 16 --seed 1
```

- Sim log: `kill_state=normal`, `estop_sense=1` (GPIO08/pin16 HIGH = not asserted), **`no PDBC seen yet`** for the entire run.
- Jetson.GPIO 2.1.9 + pyserial OK; user in `dialout` + `gpio`.

### COM5 USB mirror (Controls)

While Jetson sim was TX'ing PDBF @ 20 Hz:

| Field | Observed | Expected if link live |
|-------|----------|------------------------|
| `pdb[0..3]` magic | `0x00000000` | `PDBF` (`0x46424450`) |
| `system.kill_state` | `3` HARD_ESTOP | `0` NORMAL |
| `system.kill_reason` | `5` COMMS_LOSS | `0` none |
| `system.estop_sense` | `0` | live GPIO sense from PDBF |

**Verdict: UART4 link not up.** Fail-safe path is working (stale → HARD_ESTOP/COMMS_LOSS).

### Jetson RX sniff (sim stopped, listen 3 s on `/dev/ttyTHS1`)

- ~9.7 KB received, almost all `0x00`, **no `PDBC`/`PDBF` ASCII magic**.
- Byte volume is roughly consistent with *some* UART activity, but not a valid 115200 PDBC stream.

Most likely: **TX/RX swapped** on the Jetson↔Controls UART1 harness, or UART1 header pins ≠ the pins actually wired.

Correct polarity (from `docs/pdb-uart-v1.md`):

```
Jetson UART1 TX  -> Controls PC11 (UART4 RX)
Jetson UART1 RX  <- Controls PC10 (UART4 TX)
GND              -- common
115200 8N1
```

After a swap fix, re-run sim and expect within ~1 s:

1. Sim log: `PDBC seen` / heartbeat echo advancing  
2. COM5: `pdb` magic `PDBF`, `kill_state==NORMAL`, non-zero rails  
3. Optional: stop sim >200 ms → back to HARD_ESTOP/COMMS_LOSS  

### LED + ESTOP notes

- Factory modes are host-commanded (`led_solid_green` / `led_caution` / `led_fault` → modes 3/6/7). **Firmware does not auto-switch the strip on ESTOP** — prove script commands `BLINK_RED_FAST` when host asserts `McuState.ESTOP`.
- Firmware still **drives** hard-ESTOP GPIO (`PA0` placeholder in `pdb_link.c`, active-low, asserted when link stale). That may disagree with “PDU dominates / Controls does not drive” hardware intent — call out for schematic confirmation vs Jetson pin16 sense net.
- With link down, USB `estop_sense` stays 0 (fail-safe); Jetson GPIO readback of 1 only proves the Jetson pin was HIGH, not that Controls saw it.

### Retry 2026-07-23 14:51 (user confirmed polarity as documented)

User: Jetson pin8 UART1 TX → PC11 RX; Jetson RX ← PC10 TX; GND; 115200.

Retried both `/dev/ttyTHS1` (`serial@3100000` = DT `serial1`) and `/dev/ttyTHS2`
(`serial@3110000` = DT `serial2`):

| Test | Result |
|------|--------|
| COM5 + sim on THS1 | still `pdb` zeros, HARD_ESTOP/COMMS_LOSS; sim `no PDBC seen yet` |
| COM5 + sim on THS2 | same |
| Sniff THS1, sim off, 2.5 s | **8192 B all `0x00`**, no `PDBC` — RX line looks stuck low |
| Sniff THS2, sim off | **0 B** — quiet / not the wired UART |

So: not a “wrong ttyTHS vs THS2 for COM5 mirror” issue; **no valid PDBC/PDBF
crossing either way**. Continuous nulls on THS1 RX (sim off) ≠ a healthy
idle-high UART from Controls PC10. Worth trying a **TX↔RX swap** next (as you
offered), and/or checking continuity PC10→Jetson RX / PC11←Jetson TX and that
the Jetson 40-pin UART1 net is really `ttyTHS1` on this Orin image.

LED colour changes earlier were from **host-commanded** factory modes /
`McuState.ESTOP` + `led_fault` in the prove script — not automatic ESTOP→LED.

### Retry after user TX↔RX swap (14:56)

Still no PDBF/PDBC on THS1 or THS2. Sniff behavior changed (THS1 no longer
floods `0x00` — now quiet), so the harness move did something electrically,
but the USB mirror is still COMMS_LOSS.

### ESTOP pin mapping check (same session)

| Item | Finding |
|------|---------|
| Firmware hard-ESTOP drive | **`PA0` placeholder** in `pdb_link.c` — docs say not schematic-confirmed |
| Jetson sense | BOARD **pin 16 = GPIO08** (correct for `--gpio-estop 16`) |
| Observed pin16 | stuck **HIGH (1)** for 8 s |
| Host `McuState.ESTOP` toggle | **no change** on pin16 (still all 1s) |
| Implication | Jetson pin16 is **not** on the firmware PA0 net (or PA0 isn’t brought out / PDU holds the line). “Whole connector flipped” wouldn’t fix ESTOP until the real Controls ESTOP pad is known. |

Orin 40-pin UART1 is `ttyTHS1` (`uarta@3100000`) — device node choice is OK.

### Resume checklist (wiring fix)

1. Try TX↔RX swap on the harness (user offered).  
2. Confirm common GND + continuity to PC10/PC11.  
3. Restart sim on Jetson (`ttyTHS1`, `--gpio-estop 16`).  
4. COM5: expect `PDBF` + `NORMAL`.  
5. Host ESTOP → Jetson `estop_sense→0` + command `led_fault` (red fast blink).  
6. Host NORMAL → green; stop sim → COMMS_LOSS again.

