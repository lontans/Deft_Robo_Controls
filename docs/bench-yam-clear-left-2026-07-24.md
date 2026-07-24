# Bench: left YAM clear ranges (2026-07-24)

**Status:** tooling landed; **supervised sweeps not yet run** (no STM32 CDC when prep was attempted).

## Setup

| Item | Value |
|------|--------|
| Arm | Left / slots 0–6 |
| Bus | CH1 (FDCAN1) |
| Actuators | 7× Damiao (nominal ESC `0x01`…`0x07`) |
| Frame | Motor encoder rad (zeros deferred) |
| Policy | Operator-supervised edges + inset `0.08` rad (or 10% half-span) |

## How to run (operator at keyboard)

Close dashboard / anything holding COM. Power the arm. Then:

```powershell
cd scripts
python yam_arm_clear_range.py --apply-cfg
```

- For each joint: Enter to start PLUS, Enter again **before** contact; same for MINUS.
- Full 7-joint run writes `deft_controls_sdk/vbeta/yam_bench_clear_left.py` (`CLEAR_ACTIVE=True`) and JSON under `.deft_session/`.
- Partial: `--joint N` writes JSON only.

Prove clamps:

```powershell
python vbeta_arm_smoke.py --side left --hold --hold-s 3
python vbeta_arm_smoke.py --side left --jog --joint 0 --delta 0.05
```

## Results (fill after run)

| Joint | Home | Edge lo | Edge hi | Clear lo | Clear hi |
|------:|------|---------|---------|----------|----------|
| J1 | | | | | |
| J2 | | | | | |
| J3 | | | | | |
| J4 | | | | | |
| J5 | | | | | |
| J6 | | | | | |
| J7 | | | | | |

- ESC found:
- Port / serial:
- Notes:
