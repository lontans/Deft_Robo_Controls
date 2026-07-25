# i2rt_cpp YAM arm vs our Controls PCB Damiao path — compare + patch list

Status: **P1/P2/P3 implemented in code on the laptop checkout, not yet
bench-validated on real hardware.** Written per
[`three-track_bringup` plan](../../.cursor/plans/three-track_bringup_e05ed221.plan.md)
Claude-2 track: computer → Controls PCB / Damiao arm comparison, **no CDC
access** (Claude-1 holds `/dev/ttyACM0` for CH5 `0x74`/`0x70` bringup).

Reference (source of truth when they disagree):
[`docs/deft_vbeta_ref/i2rt_cpp/`](deft_vbeta_ref/i2rt_cpp/) — normal YAM
geometry (`robot_models/yam/yam.xml`), MIT Damiao via `dm_driver` /
`motor_chain_robot` (not Ultra).

Our side: [`scripts/deft_controls_sdk/vbeta/slots.py`](../scripts/deft_controls_sdk/vbeta/slots.py),
[`scripts/yam_continuous_all.py`](../scripts/yam_continuous_all.py),
[`scripts/deft_controls_sdk/vbeta/yam_bench_clear_left.py`](../scripts/deft_controls_sdk/vbeta/yam_bench_clear_left.py),
[`App/Src/plant/plugins/damiao.c`](../App/Src/plant/plugins/damiao.c),
[`docs/peripherals/arm-damiao-ch1.md`](peripherals/arm-damiao-ch1.md).

## TL;dr

The wire protocol (MIT 8-byte frame, enable/disable/clear-fault opcodes,
per-motor-type P/V/T ranges) is **already aligned** with i2rt — our firmware
comment in `damiao_apply_cycle` literally says "Matches i2rt: enable with
motion, then sustained MIT." The gaps are all in the **host control layer**:
we run ~12x slower setpoint updates, use a flat `kd` instead of per-joint,
and have **no torque feedforward at all** — gravity is fought entirely with
elevated position gain (`kp`), which the codebase's own history shows
already caused one regression (J2 kp=160 "gravity bump" → vibration,
reverted).

## Compare table

| Axis | i2rt_cpp (reference) | Ours (vbeta + continuous) |
|---|---|---|
| **Control tiers** | 2: unthrottled gravity-comp/gripper thread (`MotorChainRobot::update`, target ~250Hz, self-monitored, warns <100Hz) → mutex latest-value handoff → strictly-timed 250Hz CAN I/O thread (`DMChainCanInterface`) | 3: host Python loop `STREAM_HZ=20Hz` → MCU `control_loop.c` tick 500Hz (TIM6) → per-joint Damiao MIT CAN TX staggered at plant_rate/`DM_MIT_APPLY_DIV`(4) ≈ **125Hz** |
| **Setpoint freshness** | New pos/vel/kp/kd/torque computed & sent to *every* motor every ~4ms cycle (≈250Hz per joint) | New pos/kp/kd/vel target from host only every 50ms (20Hz); firmware resends the same host-set desire at ~125Hz zero-order-hold between host ticks — no interpolation |
| **kp (arm joints)** | `[80, 80, 80, 25, 10, 10]` — flat per motor-type (DM4340 J1-3 = 80, DM4310 J4-6 tapering 25→10) | `(40, 60, 90, 60, 25, 25, 20)` — non-monotonic, J3 spikes to 90; single scalar shape, not grouped by motor type |
| **kd (arm joints)** | `[5, 5, 5, 1.5, 1.5, 1.5]` — per motor-type, proportional to kp | `1.0` flat scalar for all 7 joints (`DEFAULT_ARM_KD`) — same damping on an 80 kp J3 as a 20 kp J7 |
| **Gripper gain** | kp=2–3 / kd=0.5 by gripper type (`GripperType_Helper`) | N/A — no 7th-DOF gripper motor in current CFG (slot 6 = J7 wrist, not a gripper) |
| **Enable sequence** | `motor_on()` loop: 3ms stagger, sequential per motor, retry-until-`ERR==1` (`0xFB` clear ×3 fire-and-forget, then `0xFC` retry loop). Once enabled, immediately commands **full-kp** hold at measured position — no ramp. | Firmware: same `0xFB`/`0xFC` retry pattern (rate-limited 100ms), explicitly modeled on i2rt. Host: **progressive one-joint-at-a-time** latch — kp ramps `0 → 0.35×ARM_KP` over 1.6s, hold 1.2s, check `fault==1`, retry (J4/idx3 gets 3 attempts vs 2) — then a further 2.4s "soft-engage" ramp 0.35×→1.0× after all 7 are green |
| **Gravity compensation** | Live MuJoCo `mj_inverse` gravity torque every cycle from filtered joint position (qd=qdd=0, static-only), scaled by adjustable factor (init uniform **1.45**, online-calibratable per J2-J4 within [0.5,3.0]), sent as `torque` field on every MIT frame | **None.** `torque=0.0` hardcoded on every arm command (`yam_continuous_all.py`). Gravity/static load handled entirely by empirically raising `kp` per joint (e.g. J3=90 "needed to track a jog at all against gravity"). One prior attempt at a kp-based "gravity bump" (J2=160) was reverted — it vibrated under multi-joint CLEAR motion |
| **Zero-torque / compliant mode** | `zero_torque_mode()`: kp=kd=0 for the cycle, but gravity-comp torque **keeps being sent** — arm stays gravity-supported even "limp" | No equivalent mode exists; if kp/kd were ever dropped to 0, the arm has zero gravity support (no torque backstop) |
| **Dither / friction reduction** | `dither_mode()`: alternating-sign torque square wave `[0,5,5,1,0,0,0]` Nm on J2-4, for calibration | Not present |
| **Frame format** | MIT 8-byte: `pos(16b)+vel(12b)+kp(12b)+kd(12b)+torque(12b)`, big-endian packed | Identical bit layout (`damiao_pack_tx`, `App/Src/plant/plugins/damiao.c:219-253`) — **matches** |
| **Enable/disable/clear/zero opcodes** | `0xFC`/`0xFD`/`0xFB`/`0xFE`, data = `0xFF×7 + opcode` in `data[7]` | Same opcodes, same layout (`damiao_pack_cmd`) — **matches**, firmware comment cites i2rt explicitly |
| **Per-motor-type P/V/T ranges** | DM4310: P±12.5(16b) V±30(12b) T±10(12b); DM4340: P±12.5 V±10 T±28 | Same values (`damiao.h`) — **matches**. (Firmware comment notes a past bug where all motors were packed as 4310, clipping J1-3 torque to ~9.5Nm under load; already fixed via `is_4340` selection) |
| **Command-side joint-limit clamp** | Hard clamp to `joint_limits_` (from `yam.xml`, no buffer) on every commanded position, silent clamp | No generic clamp for J1/J3-7 (brace tracks live FB, inherently bounded). J2 bounded by `CLEAR_LO/CLEAR_HI` bench-captured envelope (`INSET_RAD=0.08` already applied) with hysteresis/arrive/stuck reversal logic |
| **Measured-position safety trip** | `check_current_qpos_in_joint_limits(buffer=0.15 rad)` runs **every cycle** on measured position; violation sets `running=false` and throws — kills CAN thread + server thread, arm left holding last torque (no soft-stop) | **No equivalent.** Only safety nets are the Damiao hard-fault byte (`ERR>=8`) and the PDU `COMMS_LOSS` watchdog (stale UART) — neither catches a joint that has physically drifted outside its expected envelope while still reporting a healthy MIT status |
| **CAN dropout handling** | Per-frame retry (15 retries for MIT `set_control`, 200ms timeout) then fatal exception; no infinite retry/reconnect at chain level | Damiao ESC self-faults (`ERR=0xD`) on sustained silence (no host-visible per-motor timeout register write); system-level watchdog is the separate PDU comms-loss detector, not per-Damiao-CAN-frame |

## Patch list (recommended, priority order)

**P1 — Replace flat `DEFAULT_ARM_KD=1.0` with a per-joint array. Implemented.**
A single kd across an 8090 kp range (J3=90 vs J7=20) is very likely
under-damping the stiff joints and over-damping the soft ones. i2rt scales
kd with kp per motor type (`[5,5,5,1.5,1.5,1.5]` alongside
`[80,80,80,25,10,10]`, roughly kd ≈ kp/16-17). `slots.py` `DEFAULT_ARM_KD`
is now `(2.5, 3.75, 5.6, 3.75, 1.5, 1.5, 1.25)` — the same kp/16 ratio,
applied to our `(40,60,90,60,25,25,20)` kp array. Deliberately conservative
(not i2rt's raw ratio) given the J2=160 history of gain changes
destabilizing multi-joint CLEAR motion. Threaded through every consumer:
`arm.py` (`PcbArmDriver`), `yam_continuous_all.py`, `yam_arm_clear_range.py`,
`mission_impossible.py`, `debug_dashboard/teleop.py`, and the two legacy
scripts that still referenced the old scalar. **Not bench-tested — this is
new code, unvalidated on real hardware.**

**P2 — Add real torque feedforward for gravity, stop leaning on kp. Implemented (opt-in, default off).**
The wire format already carried a torque field (`damiao_pack_tx` packs it;
host previously always sent 0.0). New module
[`scripts/deft_controls_sdk/vbeta/gravity_comp.py`](../scripts/deft_controls_sdk/vbeta/gravity_comp.py)
(`GravityComp`) mirrors i2rt's `KDLHelper`/`compute_gravity_compensation`:
loads `docs/deft_vbeta_ref/i2rt_cpp/robot_models/yam/yam.xml` via MuJoCo,
calls `mj_inverse` with qvel=qacc=0 (static gravity only) from the live 6
joint angles, scales by a per-joint factor refined via `calibrate()`
(mirrors i2rt's `calibrate_gravity_comp`, same `[0.0, 3.0]` clamp). Unlike
i2rt's uniform-1.45-then-refine start, `.scale` starts at **0.0 per joint**
— no feedforward torque is sent until someone runs `calibrate()` (or sets
`.scale` directly) from a real bench pass, so nothing changes silently.

Wired in as fully opt-in: `PcbArmDriver(gravity_comp=...)` (default `None`)
and `yam_continuous_all.py --gravity-comp` (default off, lazily constructs
`GravityComp` only if passed) — omitting it is byte-for-byte the old
`torque=0.0` behavior.

**Gotcha found and fixed during this session (would have been a real bug on
hardware):** i2rt's `KDLHelper` ctor disables geom collisions
(`geom_contype`/`geom_conaffinity=0`) and joint limits (`jnt_limited=0`) on
the loaded model before ever calling `mj_inverse` — `GravityComp` didn't do
this initially. Verified directly (see below): at a real bench pose
(`[-0.075, -2.606, 1.65, 0.414, -0.068, -1.409]` from
`docs/peripherals/arm-damiao-ch1.md`), skipping that step produced a J2
"gravity" torque of **-8571 Nm** (a collision/limit-constraint artifact, not
physics) instead of the correct **-5.8 Nm** — in the right ballpark next to
the ~-10 Nm the bench actually measured. Fixed in `GravityComp.__init__`;
this is now load-bearing and commented in the code — do not remove it.

**P3 — Raise host `STREAM_HZ` above 20. Implemented (`STREAM_HZ = 125.0`).**
Firmware already ticks at 500Hz and only gates Damiao TX to ~125Hz per
joint (`DM_MIT_APPLY_DIV=4`) — 125Hz is that ceiling, not an arbitrary
number (i2rt's 250Hz doesn't map directly since their CAN thread *is* the
250Hz clock; ours is downstream of a fixed-rate MCU tick). The loop still
uses a flat `time.sleep(dt_nom)`, so this is a best-effort target: if
per-iteration Python/USB-CDC work exceeds ~8ms the achieved rate is lower,
never unstable/racing. `--stream-hz` still overrides it if 125 turns out to
be too aggressive for the CDC link on a real bench pass.

## Verification done this session (static/offline only, no hardware)

- `python -m py_compile` clean on every edited file.
- Full existing test suite: `pytest scripts/tests/` — **200 passed**, no
  regressions from the kd-array or `_write_mit`/`_desires_for` signature
  changes.
- `GravityComp` sanity-checked standalone (temporary local `pip install
  mujoco`, not part of the shipped change) against real bench poses from
  `docs/peripherals/arm-damiao-ch1.md` / `yam_bench_clear_left.py` — output
  is now single-digit-to-low-double-digit Nm, consistent with measured
  `tau≈-10` in the bench logs, and J1 (yaw about vertical) correctly comes
  out ~0 every time.
- **Not done**, because this track has no CDC: any live run on CH1, latch
  behavior with the new per-joint kd, `--gravity-comp` end-to-end with a
  real `calibrate()` pass, or `--stream-hz 125` link-stability check.

**P4 — Add a continuous measured-position safety trip.**
Mirror i2rt's `check_current_qpos_in_joint_limits(±0.15 rad)`: periodically
check live FB against `CLEAR_LO/CLEAR_HI` (already have the envelope) and
stop the stream on violation. Prefer a **controlled** stop (route through
the existing `hub.recover()` / `stop_can.py` path) rather than i2rt's
uncontrolled thread-kill-by-exception, since our host layer already has a
clean shutdown path i2rt doesn't.

**P5 — Re-examine whether the multi-second progressive latch is load-bearing.**
i2rt enables all 7 motors sequentially too (3ms stagger in `motor_on()`'s
for-loop) but then jumps straight to full-kp hold — no multi-second kp ramp.
Our progressive one-at-a-time ramp (1.6s ramp + 1.2s hold per joint, plus a
2.4s soft-engage after) exists because enabling all 7 simultaneously drops
J4's enable ack on a cold bus. Worth a bench test of "sequential enable,
immediate full-gain hold" (i2rt's pattern) before assuming the elaborate
ramp choreography is required — if the J4 drop is specifically an
all-at-once bus-load issue, a lighter sequential-only fix might suffice.

**P6 — Document (or revisit) why our kp shape is non-monotonic.**
i2rt groups gains by motor type only (flat 80 for all 3 DM4340 joints,
tapering for DM4310). Ours spikes at J3=90 while J1/J2/J4 sit at
40/60/60. This may be a legitimate bench-tuned difference (different link
mass/lever-arm on our arm geometry) — the `slots.py` comment history
(`teleop/defaults.py`) suggests it is empirical. Low priority, but worth a
one-line rationale comment next to `DEFAULT_ARM_KP` so the next person
doesn't assume it's a typo.

## Already aligned — no patch needed

- MIT 8-byte frame bit packing (`damiao_pack_tx` / i2rt `dm_driver.cpp`).
- Enable/disable/clear-fault/set-zero opcodes and `0xFF×7 + opcode` layout.
- Per-motor-type P/V/T ranges (DM4310, DM4340) — both match datasheet values;
  our past all-as-4310 torque-clip bug is already fixed.
- Firmware enable pattern ("clear fault, enable with motion, then sustained
  MIT, not enable flood") — comment in `damiao_apply_cycle` already cites
  i2rt as the model.

## Not covered here

No CDC access on this track, so nothing above was bench-validated against
live hardware — this is a static-analysis compare only. Any of P1-P6 needs
a bench pass on CH1 (left arm, slots 0-6) once the arms lane is free per the
three-track plan's coordination checklist.
