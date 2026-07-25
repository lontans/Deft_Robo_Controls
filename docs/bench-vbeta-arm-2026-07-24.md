# Bench — one-arm vbeta smoke (post Track B/C)

Date: 2026-07-24  
Owner: Cursor (D1). Layout stays **694 B / v3 / 26 slots** (no 800 B).

## Setup

| Item | Value |
|------|--------|
| Port | COM5 (STM32 CDC `0483:5740`) |
| Flash | ST-Link SWD `Debug/DeftRoboticsControlsPCB.elf` |
| Jetson | `192.168.50.48` paced `pdb_uart_sim.py` @ 20 Hz, `force-kill-state 0` |
| CFG | YAM product via `--apply-cfg` (RAM) |
| Side | **left** only (slots 0–6, CH1 Damiao) — no dual-arm |

## Soft-kill wiring (this D1)

- `ControlsPcbHub.start_streaming` registers `soft_kill_park_if_requested(send=False)` on the plant TX hook.
- `PcbRobotSession.service_soft_kill()` / `send_once()` also call the Track B park API.
- Smoke hold loop ticks `service_soft_kill()` for observability.

## Command

```text
python scripts/vbeta_smoke.py arm --port COM5 --side left --apply-cfg --hold-s 2.0
```

## Results

| Check | Result |
|-------|--------|
| COM exclusive open | PASS |
| YAM CFG apply | PASS — CH1×7, CH2×7, CH3×0, CH4–6×2 |
| PDB fresh NORMAL | PASS — `kill=0`, `stale=False`, `soft_req=False` (pre/post) |
| Soft-kill auto-park path | Wired (no SOFT_KILL_REQ asserted this run) |
| Goal_Position Δj0=+0.05 | **NO MOTION FB** — `Position_Rad` stayed all zeros before/after |
| Script exit | 0 (`arm smoke done`) |

**Verdict:** **Software/session path PASS; Damiao motion NOT proven on this bench** (no non-zero left-arm FB — arms likely unpowered/absent; Track B plant integ used DXL + RS02 instead).

Offline: `pytest tests/test_deft_controls_sdk_vbeta.py …` → 19 passed after soft-kill hook.

## Handoff

**COM5 free** for Claude dashboard live check. Jetson NORMAL paced sim left running so PDU V/I + kill strip can update.
