# Bench — vbeta product-CFG prove (Claudacious)

Date: 2026-07-24
Owner: Claudacious (three-agent PDU/vbeta/manual pass).
Board: live Controls PCB on Jetson `192.168.50.48`, `/dev/ttyACM0`, serial `3167376F3435`.
Script: [`scripts/vbeta_product_prove.py`](../scripts/vbeta_product_prove.py) — one exclusive-COM
`PcbRobotSession` running `yam_product_rows()` CFG (arms slots 0–13, base steer/drive slots 14–19
on CH4–6 RobStride IDs `0x01`/`0x02`), not the bench-spare map `yam_continuous_all.py` uses.

## What this proves (vs. earlier work)

[`bench-vbeta-arm-2026-07-24.md`](bench-vbeta-arm-2026-07-24.md) smoke-tested `PcbArmDriver` on a
**different** bench (Windows COM5) where the arm was unpowered — software path passed, no motion
FB. This run is the same adapters against the **live, powered** Jetson board that
[`docs/peripherals/`](peripherals/continuous-ops.md) already proved moves under the bench-spare CFG
— the open question was whether the adapters work correctly under the **product** CFG specifically
(different slot numbers for base, dual-arm CH1+CH2), not whether the board can move at all.

## Results

| Check | Result |
|-------|--------|
| YAM **product** CFG apply (RAM) | PASS — CH1×7 [0-6], CH2×7 [7-13], CH4×2 [14,17], CH5×2 [15,18], CH6×2 [16,19] |
| Left arm (CH1) live FB | PASS — nonzero `Position_Rad`, matches continuous's last resting pose |
| Left arm MIT armed | PASS — `fault=1` on all 7 joints, sustained for the full window every run (10s, 30s, 30s) |
| Left arm `Goal_Position` tracking | PASS — clamped +0.15 rad jog on joint0 converged to 0.0007 rad error within ~1 s, held stable 10 s |
| Right arm (CH2) | CFG'd, discover-only — **no Damiao motor found**, IDs 1–7 swept, `SESSION_BEGIN missed` warning. Documented, no hold attempted (per plan: unpowered → CFG+discover only) |
| Base product IDs (CH4/5/6, steer `0x01` / drive `0x02`) | **FAIL to find — see Known gap below.** 0/6 probes found, 3 consecutive runs |
| `PcbPlatformClient` base creep command | Sent (`send_target_state`, `0.2 rad/s` drive) — no FB motion observed, consistent with no motor answering the commanded IDs |
| Soft-kill service tick | Ticked every loop (`service_soft_kill()`), no false trip |
| Session close / CDC release | Clean on all 3 runs — no leftover process holding `/dev/ttyACM0` afterward |

### One caveat worth flagging

A `PcbArmDriver.read("Position_Rad")` call made **immediately** after `connect()` (before the first
plant FB frame has arrived from the just-started stream) can read back all-zeros rather than the
real present position — seen on the very first line of two of the three runs. This is a one-tick
race in the harness script (`vbeta_product_prove.py` reads once for a status print before priming
the stream), not a `PcbArmDriver` bug — by the next read (a few ticks later, or in the combined
window) FB was live and correct on every run. Anyone using `Position_Rad` right after `connect()`
for a real go/no-go decision should poll briefly rather than trust the first sample.

### Joint-1 jog note (secondary observation, not a failure)

A first attempt jogging **joint 1** (shoulder) by `+0.15 rad` did not converge — `yam_limits`' soft
clamp for that joint matched the CLEAR-envelope upper bound almost exactly where the joint was
already resting (`-2.610` vs. bound `-2.683`), so the requested delta got auto-clamped down to
`~0.07 rad` and the joint didn't visibly move toward it over 30 s (residual error stayed flat, not
decaying). Re-running the same jog logic on **joint 0** (waist, well clear of any soft limit)
converged cleanly in ~1 s (see Results table). Read this as "joint 1 was parked hard against its own
soft wall, unsurprising it didn't push further that direction," not as a driver defect — worth a
follow-up jog test on joint 1 in the *other* direction if someone needs to specifically re-verify
that joint's live tracking.

## Known gap: base product IDs vs. bench wiring (do not silently remap)

`docs/peripherals/base-robstride-mcp.md` and `docs/peripherals/base-damiao-ch6.md` already
documented (2026-07-24, same day) that the physically-wired base motors on this bench answer to
**bench spare IDs** — `0x70`/`0x74` (CH5), `0x75` (CH6 RobStride), `0x06` (CH6 Damiao) — not the
**product** map's `0x01` (steer) / `0x02` (drive) per rail used by `yam_product_rows()`. This run's
direct RobStride probes (`hub.debug.probe_robstride(bus=b, motor_id=0x01|0x02)` for `b in (4,5,6)`)
confirm that from the product-CFG side too: **zero of six probes found a motor**, including on CH4,
which no prior session (bench-spare or product) has ever gotten a hit on at any ID — CH4 may simply
be unpopulated/unwired on this bench, not just wrong-ID.

**This is a hardware/CFG mapping gap, not a software bug in the PCB adapters** — `PcbPlatformClient`
correctly sends product-ID-addressed MIT commands and correctly reports back whatever (zero) FB
comes back. Per the plan's explicit instruction, **this gap is documented, not silently worked
around** — `vbeta_product_prove.py` and the `deft_vbeta` bridge do **not** fall back to slots 22–25
(the bench-spare map) inside the product-CFG path. Closing this gap needs one of:
1. Re-flash/re-configure the physical base RobStride drives' own IDs to `0x01`/`0x02` per rail
   (motor-side, via RobStride's own config tool — out of scope here), or
2. An explicit, separately-tracked ADR choosing to keep bench IDs and remap `yam_product_rows()`
   to match them (listed as a brainstormed backlog item — "Base product `0x01`/`0x02` vs bench
   `0x70`/`0x74` ADR" — in the three-agent plan, owner Claudacious to document / CFG-change only
   with HW access to actually recable or reflash motor IDs).

## deft_vbeta/ (full `YAMAIMobile`) status

A fresh working checkout was cloned to repo-root [`deft_vbeta/`](../deft_vbeta/) (gitignored, 27 MB,
same commit `6cd886f` as the read-only reference at `docs/deft_vbeta_ref/deft_vbeta`). The Option A
monkey-patch bridge from `docs/vbeta-pcb-adapter.md` is implemented as real code at
`deft_vbeta/src/deft_amr/amr/amr/pcb_bridge.py` (`install_pcb_backend(robot)`), ready to drop into
any of the four `YAMAIMobile(config=...)` call sites (`episode_recorder.py`,
`segment_recorder.py`, `yam_episode_recorder.py`, `episode_eval.py`).

**Not yet exercised end-to-end**: `YAMAIMobile.__init__` imports `torch` and `mujoco` at module
level and requires MJCF assets under `~/deft_vbeta/src/py_vr/...`. Checked the Jetson's Python env
directly — **neither `torch` nor `mujoco` is installed**, and no `py_vr` assets are present. Standing
up that environment (large ARM wheel/build for `torch`+`mujoco`) is out of scope for this pass (plan
says: single Jetson board, don't change the bench) and wasn't attempted. The motion-relevant surface
that bridge wires up — `PcbArmDriver` / `PcbPlatformClient` / `PcbRobotSession` — is exactly what
this doc proves directly against the live board, independent of `YAMAIMobile`'s camera/dataset/EE-
kinematics machinery, which none of this pass's hardware prove exercises.

## Reproduce

```bash
# from scripts/ on the Jetson (or synced there via SFTP)
python3 vbeta_product_prove.py --port /dev/ttyACM0 --hold-s 30
# quicker, targeted jog re-check on a specific joint:
python3 vbeta_product_prove.py --port /dev/ttyACM0 --hold-s 10 --jog-joint 0 --jog-delta 0.15 --no-right-arm
```

## Related

- [vbeta-pcb-adapter.md](vbeta-pcb-adapter.md) — adapter contract, Option A/B integration sketch
- [peripherals/base-robstride-mcp.md](peripherals/base-robstride-mcp.md) ·
  [peripherals/base-damiao-ch6.md](peripherals/base-damiao-ch6.md) — bench-spare base ID truth this
  run's remap-gap finding depends on
- [bench-vbeta-arm-2026-07-24.md](bench-vbeta-arm-2026-07-24.md) — earlier single-arm smoke (COM5,
  unpowered bench)
