# Bench — PDU–Controls SDK contract (Track B)

Date: 2026-07-24

## What shipped

| Area | Change |
|------|--------|
| Wire layout | **v3 / 694 B / 26 actuators** (CH3×4). Supersedes v2 672 B / 25. |
| SDK | `hub.pdb_status()`, `hub.soft_kill_park()`, SI helpers (10 mV / 10 mA placeholder), USB kill parsers |
| FW | On host `ESTOP`/`RECOVERY` park: `pdb_link_set_soft_kill_ready(true)` while peer is `SOFT_KILL_REQ` |
| Docs | `host-exchange-v3.md`, freshness + park in `api.md` / `pdb-uart-v1.md` |

## Jetson UART hygiene

- Host: `192.168.50.48` (`deft-robotics`, `JETSON_PASS`).
- Inventory: paced `pdb_uart_sim.py` present; bringup leftovers (`_tmp_*uart*`, gpio overlays) left in place; **pycache not touched**.
- Synced laptop `scripts/pdb_uart_sim.py` → Jetson (hash differed; local newer).
- Confirmed paced sim on `/dev/ttyTHS1` @ 20 Hz with `tx_pace=500us/byte`; PDBC RX visible.

## Flash

- Soft-DFU enter from COM5 worked, but **0483:DF11 never appeared** to libusb on this Windows host (no WinUSB DFU claim).
- Flashed via ST-Link SWD: `STM32_Programmer_CLI -c port=SWD -w Debug/DeftRoboticsControlsPCB.elf -v -rst`.
- Artifact: `Debug/DeftRoboticsControlsPCB.elf` (layout v3).

## Prove results

### Soft-kill handshake (`--simulate-kill-after`)

`python scripts/_tmp_pdb_softkill_handshake_prove.py --port COM5`

- DXL slot0/1 present sampled; RS02 CH6 `0x70` slot **24** present sampled.
- `SOFT_KILL_REQ` → `hub.soft_kill_park()` → Jetson log: `controls acked SOFT_KILL_READY` → USB `kill_state=2`.
- **PASS**

### LED live prove

`python scripts/_tmp_pdb_led_live_prove.py --port COM5` — all kill/stale/peer-estop phases **PASS** (`failed=0`).

### Plant integ (DXL + RS02 + kill phases)

`python scripts/_tmp_pdb_plant_integ_test.py --port COM5` → `docs/bench-pdb-plant-integ-2026-07-24.md`

| Phase | kill | led | motion | result |
|-------|-----:|----:|--------|--------|
| NORMAL | 0 | 3 | s0 span 1726 / s1 56 | PASS |
| SOFT_KILL_REQ | 1 | 6 | freeze | PASS |
| SOFT_KILL_READY | 2 | 5 | freeze | PASS |
| HARD_ESTOP | 3 | 7 | freeze | PASS |
| NORMAL_RESTORE | 0 | 3 | s0 1209 / s1 55 | PASS |

USB stream ~400 Hz plant FB during phases; `act_lap` ~0.15 ms motion / ~0 freeze. PDB rails via typed SI helpers: 48 V / 19 V / …  

### Bandwidth ×26 (`_tmp_mcp_timing_probe.py`)

Product CFG applied: CH1×8, CH2×8, **CH3×4**, CH4–6×2. Full stage `9_all_CH1-6_x26` with ACTUATOR rx_sim:

| Metric | Value |
|--------|------:|
| raw_fb_hz | ~317 |
| ack_lag max | 0 |
| act_lap mean | 3.2 ms |
| rx_fresh | 26/26 |

Envelope: lag OK, fb_hz ≫ 40 Hz host target, act_lap in prior single-digit ms band.

## Freshness contract (observed)

Stop Jetson sim → USB `kill_state=HARD_ESTOP` / LED blink-red — matches MCU stale fail-safe (`COMMS_LOSS`). Documented in `docs/pdb-uart-v1.md` + `hub.pdb_status().stale_failsafe`.

## Notes / follow-ons

- Soft-DFU on this Windows box needs DFU driver (Zadig WinUSB) or keep using ST-Link for layout bumps.
- Local `estop_sense` (PB7) reads `0` on this bench while peer reports `1` — LEDs correctly follow **peer** PDBF estop for the peer-estop phase.
- Dashboard kill strip left as follow-on (ADR checklist).
- RS02 canonical slot for bus 6 is now **24** (was 23 under v2).
