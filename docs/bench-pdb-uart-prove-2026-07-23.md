# PDB UART prove-out — 2026-07-23

## Done without UART4 peer

- Soft-DFU image on board: Release `-Os` + per-bus RX index.
- USB CDC probe (`COM5`): `pdb[64]` all zeros; `kill_state=HARD_ESTOP(3)`, `kill_reason=COMMS_LOSS(5)`.
- Agent1 unit tests: `python -m pytest scripts/tests/test_pdb_link_frames.py -q` → 16 passed.
- Sim script present: `scripts/pdb_uart_sim.py` (refuses COM5 by design).

## Blocked: live UART4 ↔ Jetson/USB-UART

No physical UART adapter appeared on the host COM enumeration (only COM5 CDC, COM53 ST-Link, and com0com virtual pairs). Cannot inject `PDBF` into PC11 without wire.

### Resume checklist

1. Wire: Jetson/USB-UART TX → PC11, RX ← PC10, GND common, 115200 8N1.
2. Confirm `UART4_MODE_PDB` in `App/Inc/host/uart4_mode.h`.
3. On Jetson (or PC spare UART): `python scripts/pdb_uart_sim.py --port <uart> --hz 20`
4. On PC COM5: stream briefly; expect `pdb[64]` magic `PDBF`, `kill_state==NORMAL`, non-zero rails as set by sim.
5. Optional stale: stop sim >200 ms → back to HARD_ESTOP/COMMS_LOSS (ESTOP GPIO only if safe).
