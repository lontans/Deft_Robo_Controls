# Bench — vbeta live-prove plan executed

Date: 2026-07-24 (evening)  
Owner: Cursonier (execute [`docs/vbeta-live-prove-plan.md`](vbeta-live-prove-plan.md))  
Board: Jetson `192.168.50.48`, `/dev/ttyACM0`, serial `3167376F3435`  
Path: **direct adapters** (`vbeta_product_prove.py` / `PcbRobotSession`) — not `install_pcb_backend` / `YAMAIMobile`

## Exclusive COM

| Check | Result |
|-------|--------|
| `yam_continuous_all` / other vbeta owners | None |
| Debug dashboard process | Present (`python -m deft_controls_sdk.debug_dashboard`) but **not** holding CDC (`fuser` → free) — Claudistic GUI lane only |
| Soft-DFU / DF11 | Idle (CDC only) |
| Post-run CDC | Free |

## Offline gate

```text
python -m pytest scripts/tests/test_deft_controls_sdk_vbeta.py -q
→ 18 passed
```

One pretest failure (`test_arm_goal_position_clamped_by_default`) expected MuJoCo J2 floor `≥ 0` while left `yam_bench_clear_left` motor-frame CLEAR is active (J2 soft ≈ `[-4.59, -2.68]`). Test updated to assert write lands on `clamp_q7(q, "left")` rather than a hardcoded MuJoCo floor. No adapter clamp logic changed.

## Live ladder (`vbeta_product_prove.py`)

```text
PYTHONPATH=scripts python3 -u scripts/vbeta_product_prove.py \
  --port /dev/ttyACM0 --hold-s 30 --jog-joint 0 --status-s 2.0
→ EXIT=0
```

| Step | Result |
|------|--------|
| Product CFG RAM (`yam_product_rows`) | PASS — CH1×7 [0–6], CH2×7 [7–13], CH3×0, CH4–6×2 [14/17, 15/18, 16/19] |
| 1. Left-arm hold / live FB | PASS — `left_live=True`, nonzero `Position_Rad` |
| 2. Left-arm jog joint0 +0.15 rad | PASS — err `0.0011 rad` within 1 s (**CONVERGED**); held 30 s |
| 3. Right arm CH2 discover | HW blocker — sweep empty, `SESSION_BEGIN missed`; CFG'd, not present |
| 4. Base product IDs 0x01/0x02 on CH4–6 | HW blocker — **remap gap** (0/6 found); no silent 22–25 remap |
| 5. Neck | Not on this entrypoint / not wired this session — **blocked, not attempted** |
| Soft-kill tick | No `SOFT_KILL_REQ` trip during window (baseline quiet) |
| Session close | Clean; CDC released |

Summary line from script:

```text
left_live=True left_tracked_motion=True
right_ch2_present=False right_live=False
any_base_found=False base_tracked_motion=False
REMAP GAP: no RobStride answered product IDs 0x01/0x02 …
```

## HW blockers (honest, not adapter failures)

1. **Right arm absent** on CH2 this session.  
2. **Base product-ID remap gap** — physical drives still on bench spare IDs (`0x70`/`0x74`/`0x75`/`0x06`); see prior [`bench-vbeta-product-cfg-2026-07-24.md`](bench-vbeta-product-cfg-2026-07-24.md) and `docs/vbeta-pcb-adapter.md`.  
3. **Neck** not exercised.

## Out of scope (honored)

- Soft-DFU, FW kill math, lift CANopen — untouched  
- Debug dashboard / Claudistic files — untouched (COM left free for GUI work)  
- `YAMAIMobile` / torch / mujoco path — skipped (direct adapters sufficient)

## Handoff

CDC free. Left-arm product-CFG hold+track proven again on Jetson. Remaining product-path gaps are hardware/CFG (right arm, base IDs, neck), not `PcbArmDriver` / session ownership.
