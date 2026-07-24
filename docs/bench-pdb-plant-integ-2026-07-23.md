# PDU × plant integrated test

Date: 2026-07-23 18:39  
Port: COM5 @ 40 Hz stream · RS02 CH6 / slot 23 / id `0x70` · Jetson `/dev/ttyTHS1` paced PDB sim  
Script: `scripts/_tmp_pdb_plant_integ_test.py`  
Result: **failed=0/5**

## What ran

Jetson PDU peer cycled `NORMAL → SOFT_KILL_REQ → SOFT_KILL_READY → HARD_ESTOP → NORMAL` (`pdb_uart_sim --force-kill-state`).

| Actor | Behavior |
|-------|----------|
| Firmware LEDs | Follow PDU kill (green / yellow blink / solid red / red blink) |
| Host (test) | On kill≠0 → `McuState.ESTOP` (clears DXL session + RS desires → freeze). On NORMAL → stream motion |
| Slot 0 | Full-range bounce @ π/4 rad/s |
| Slot 1 | Init to present, then ±80 ticks (~7°) @ π/16 |
| RS02 | Soft-hold at present (kp=2); expect Δ≈0 |

> Firmware does **not** auto-freeze actuators from `kill_state` alone (LEDs only). This test implements the product host park path: fault kill → ESTOP.

## Results

| Phase | kill | led | host | plant_fb Hz | lag mean/p95/max | act_lap mean | periph_lap | s0 span | s1 span | rs Δ rad | pack_v | result |
|-------|-----:|----:|------|------------:|-----------------:|-------------:|-----------:|--------:|--------:|---------:|--------|--------|
| NORMAL | 0 | 3 | NORMAL | 410 | 0.10 / 1 / 1 | 0.19 ms | 0.18 ms | 1773 | 21 | 0.001 | 48/48/0/0 V | PASS |
| SOFT_KILL_REQ | 1 | 6 | ESTOP | 435 | 0.11 / 1 / 1 | ~0 ms | 0.07 ms | 0 | 0 | 0.000 | 48/48/0/0 V | PASS |
| SOFT_KILL_READY | 2 | 5 | ESTOP | 440 | 0.11 / 1 / 1 | 0 ms | 0.08 ms | 0 | 0 | 0.000 | 48/48/0/0 V | PASS |
| HARD_ESTOP | 3 | 7 | ESTOP | 428 | 0.14 / 1 / 1 | 0 ms | 0.08 ms | 0 | 0 | 0.000 | 48/48/0/0 V | PASS |
| NORMAL_RESTORE | 0 | 3 | NORMAL | 410 | 0.12 / 1 / 2 | 0.07 ms | 0.11 ms | 1208 | 42 | 0.000 | 48/48/0/0 V | PASS |

### PDU rails (sim, all phases)

| | values |
|--|--------|
| pack_v | 48.0, 48.0, 0, 0 V |
| rail_v | 48.0, 19.0, 12.0, 5.0 V |
| pack_i | 1.2, 0.8, 0, 0 A |
| rail_i | 0.5, 0.3, 0.2, 0.1 A |

(`pdb_uart_sim` fixed telemetry; 10 mV / 10 mA counts.)

## Observations

1. **Motion vs freeze:** NORMAL phases show large slot0 travel (1200–1800 ticks) and small slot1 wiggle (21–42 ticks). Fault phases show span 0 on both DXL and RS02.
2. **LEDs track PDU:** 3→6→5→7→3 as kill cycles; host LedDesire left OFF.
3. **Bandwidth:** Stream ~40 Hz cmd; MCU plant FB ~410–440 Hz counted; raw CDC ~480–500 Hz. Cmd lag mean ~0.1 frame, p95=1. `act_lap` ~0.2 ms under motion, ~0 under ESTOP.
4. **RS02:** Held within 0.001 rad across all phases (no travel under fault).
5. **Neck starts:** First run present near table ends (3072 / 2500); restore re-armed at 1513 / 2480 after ESTOP.

## Re-run

```powershell
$env:JETSON_PASS='4565'
python scripts/_tmp_pdb_plant_integ_test.py --port COM5 --hz 40
```
