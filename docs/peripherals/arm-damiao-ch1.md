# Arm — Damiao CH1 (left arm, slots 0–6)

Live-verified operating manual for the YAM left-arm Damiao chain on FDCAN1
(PB8/PB9). Source of truth for CFG/gains: `scripts/deft_controls_sdk/vbeta/slots.py`
(`LEFT_ARM_SLOTS`, `DEFAULT_ARM_KP`/`KD`), `scripts/deft_controls_sdk/vbeta/yam_bench_clear_left.py`
(`CLEAR_LO`/`CLEAR_HI`), driver script `scripts/yam_continuous_all.py`.

## AI quickstart

- **Bus / protocol**: FDCAN1, `PROTO_DAMIAO`, ESC IDs `0x01..0x07` → slots `0..6`
  (`LEFT_ARM_SLOTS = range(0,7)`), master RX IDs `0x11..0x17` (`_DAMIAO_MASTER`).
- **Run it**: `python scripts/yam_continuous_all.py --port /dev/ttyACM0` (or `find_cdc_port()`
  auto-detect). `--no-base --no-dxl` isolates the arm only.
- **CFG must be applied before any motion command** — `ensure_yam_left_arm_cfg(hub, force=True)`
  then `hub.debug.cfg_set_slot(slot=i, bus=1, protocol=PROTO_DAMIAO, motor_id=0x01+i, master_id=_DAMIAO_MASTER[i], enabled=True, persist=False)`
  per slot, wrapped in `pause_plant_stream(hub)`.
- **Enable order matters**: latch joints **one at a time** (progressive latch), not all 7 at once —
  see Human deep dive. J4 (index 3) routinely needs an extra retry.
- **fault byte semantics**: `fault == 1` means **MIT-armed/green**, not an error. `fault == 0` means
  not yet latched. `(fault & 0xF) >= 8` is an actual hard fault — stop.
- **J2 (index 1) is the only joint driven with continuous motion** in the bench cruise pattern;
  J1 and J3–J7 are braced at a soft-held FB position (`kp = ARM_KP[i] * LATCH_KP_SCALE` while
  latching, `1.0` once "ready"). Do not read J1/J3–7 non-motion as "stuck" — it's brace-by-design.
- **CLEAR envelope (motor-frame, teleop-verified 2026-07-24)**:
  `CLEAR_LO = (-1.4621, -4.5929, 1.1277, -0.9361, -1.4037, -1.4190, 1.2562)`
  `CLEAR_HI = (1.0345, -2.6826, 3.0601, 1.3488, 1.0597, 0.1792, 2.6456)`
  (7-tuple indexed J1..J7 / slot 0..6). J2's continuous-cruise script only exercises
  `[CLEAR_LO[1], CLEAR_HI[1]] = [-4.593, -2.683]`.
- **Don't**: enable all 7 slots simultaneously on a cold bus and expect green — it will drop joints
  (esp. J4). Don't treat `position≈0` from `latest_feedback()` as "arm is home" — `_read_arm()`
  returns `None`/skips a joint whose `abs(position) < 1e-3` because that's indistinguishable from
  "no live FB yet" on this rig.

## Human deep dive

### Why progressive latch, not all-at-once

`yam_continuous_all.py` arms CH1 joints **one at a time**: add joint `i` to the `armed` set, re-run
`cfg_set_slot` for the whole armed set, hold `mcu_state=NORMAL`, seed `q0[i]` from live FB
(`_seed_hold`), then ramp `kp` from 0→`ARM_KP[i]*LATCH_KP_SCALE` over `LATCH_RAMP_S=1.6s` and hold
`LATCH_HOLD_S=1.2s` while checking `fault==1` on every armed joint (`_latch_armed`). If a joint
doesn't go green in the ramp+hold window, the script calls `_recover_rearm()` (blank desires →
`hub.recover()` → re-CFG the armed set → re-seed) and retries. This exists because bringing up all
7 Damiao ESCs simultaneously on a cold FDCAN1 bus intermittently drops enable acks for one or two
joints (observed most on J4/index 3, hence the extra retry budget `attempts = 3 if i == 3 else 2`).
A final two-pass sweep re-latches any joint still not green before the script calls the arm "ready."

### `fault=1` is not an error code

The Damiao MIT enable handshake reports back through the same byte plant FB uses for fault status.
`fault == 1` on this rig means "MIT control mode active / armed", which is why every status line in
a healthy run prints `faults=[1, 1, 1, 1, 1, 1, 1]` — that's **seven joints healthy**, not seven
faults. Only `(fault & 0xF) >= 8` is treated as a hard fault worth aborting the run over (see the
J2 hard-fault check in the continuous loop).

### Brace vs. drive

Only J2 (slot 1) is actively driven in the bench "continuous" pattern — a hysteresis bounce between
`CLEAR_LO[1]` and `CLEAR_HI[1]` at `CRUISE_UP=0.18 rad/s` / `CRUISE_DOWN=0.12 rad/s`, reversing when
it arrives within `ARRIVE_EPS=0.10 rad` of a bound (with `HYST=0.15 rad` minimum travel before an
arrive-reverse) or gets stuck for `STUCK_S=3.5s` (in which case the aim bound itself shrinks toward
current FB rather than fighting a jam). J1 and J3–J7 are held at a slowly FB-tracking brace position
(`arm_cmd[i] = 0.98*arm_cmd[i] + 0.02*fb_arm[i]` each tick) — this keeps them "live" (drawing torque,
correctable) without commanding travel. **This is the intended bench pattern**, not a partial
implementation — do not "fix" the other 6 joints to also sweep without being asked.

### CLEAR envelope provenance

`CLEAR_LO`/`CLEAR_HI` in `yam_bench_clear_left.py` came from a **teleop min/max capture** on
2026-07-24 with an `INSET_RAD = 0.08` safety margin already applied (see `SOURCE` string in that
file, which also notes "J2 fault on fast drop; matrix kept" — i.e. one capture run faulted on a fast
drop but the resulting envelope was kept as valid). This is a physical joint-limit table from a real
run, not a placeholder.

## Verified

**Date:** 2026-07-24, live board on Jetson (`192.168.50.48`, `/dev/ttyACM0`), via
`python scripts/_tmp_launch_continuous.py` (drives `yam_continuous_all.py --cruise-up 0.18
--cruise-down 0.12 --engage-s 2.4 --base-rate 0.7854 --record --duration 50`).

Discover (all 7 Damiao ESCs present on CH1):
```
Damiao discover summary: 6 motor(s) — 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
Damiao discover on CH1 (PB8/9 FDCAN1)  IDs 2..2
FOUND  probe=0x02  esc_id=0x02  master_rx=0x12  mode=id_sweep  pos=+0.0000  err=0x0
Damiao discover summary: 1 motor(s) — 0x02
```
(0x07/slot 6 confirmed a few lines later in the full log — 7/7 present.)

Progressive latch, all green on first or second try, no joint needed the 3rd J4 retry this run:
```
  armed=[0] faults=[1, 0, 0, 0, 0, 0, 0] ok=True try=1/2
  armed=[0, 1] faults=[1, 1, 0, 0, 0, 0, 0] ok=True try=1/2
  armed=[0, 1, 2] faults=[1, 1, 1, 0, 0, 0, 0] ok=True try=1/2
  armed=[0, 1, 2, 3] faults=[1, 1, 1, 1, 0, 0, 0] ok=True try=1/3
  armed=[0, 1, 2, 3, 4] faults=[1, 1, 1, 1, 1, 0, 0] ok=True try=1/2
  armed=[0, 1, 2, 3, 4, 5] faults=[1, 1, 1, 1, 1, 1, 0] ok=True try=1/2
  armed=[0, 1, 2, 3, 4, 5, 6] faults=[1, 1, 1, 1, 1, 1, 1] ok=True try=1/2
arm home(FB)=[-0.075, -2.606, 1.65, 0.414, -0.068, -1.409, 2.642] faults=[1, 1, 1, 1, 1, 1, 1]
```

Continuous J2 CLEAR bounce, all 7 faults stayed `1` (green) for the full ~26 s cruise sampled below,
reversal at the `-4.593` bound observed and correctly detected as "stuck near bound" (arrived within
hysteresis) then reversed direction:
```
  J2 dir=-1 cmd/fb=-2.917/-2.743 tau=-10.13 faults=[1, 1, 1, 1, 1, 1, 1] | ...
  ...
  J2 dir=-1 cmd/fb=-4.593/-4.412 tau=-10.92 faults=[1, 1, 1, 1, 1, 1, 1] | ...
  J2 reverse stuck fb=-4.412 (wanted -4.593) aim=[-4.412,-2.683]
  J2 dir=+1 cmd/fb=-4.243/-4.106 tau=-9.16 faults=[1, 1, 1, 1, 1, 1, 1] | ...
```
Full 26 s window recorded to NDJSON on the Jetson at
`.deft_session/recordings/record_20260724T214351.ndjson` (~2.7 MB, this run) for anyone who needs
raw per-tick feedback rather than the 2 s status-line samples above.

## Known falsehoods retired

- **"fault≠0 means something is wrong."** False for this MIT chain — `fault=1` is the armed/healthy
  steady state. Old benches that logged raw fault ints without this context read as alarming but
  were describing normal operation.
- **"The arm needs to home to zero before it's usable."** False — the continuous script deliberately
  treats live FB as home (`arm_cmd = fb0.copy()` / `q0` seeded from FB, never a hardcoded zero pose).
  Commanding a hard snap-to-zero is what used to cause the "arm unpowered" morning-note class of
  failures (large kp against a large position error from wherever the arm happened to rest
  overnight); brace-at-FB then soft-engage avoids that.
- **"CFG all 7 joints once, then move on."** Leads to intermittent dropped joints, especially J4.
  The proven pattern is progressive one-at-a-time latch with a recover+re-CFG retry path.
