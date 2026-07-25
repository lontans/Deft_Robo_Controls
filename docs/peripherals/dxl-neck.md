# Neck — Dynamixel (host servo slots 0/1)

Live-verified operating manual for the neck pitch/yaw Dynamixel pair. Source of truth:
`scripts/deft_controls_sdk/vbeta/slots.py` (IDs/slots), `App/Src/plant/plant_config.c`
`servo_table[]` (position clamps, mirrored in `scripts/yam_dxl_clear_teleop.py` `SERVO_CFG`),
driver scripts `scripts/yam_continuous_all.py` and `scripts/yam_dxl_clear_teleop.py`.

## AI quickstart

- **IDs / slots**: pitch = DXL ID `1` → servo slot `0` (`NECK_PITCH_DXL_ID`/`NECK_PITCH_SERVO_SLOT`),
  yaw = DXL ID `2` → servo slot `1` (`NECK_YAW_DXL_ID`/`NECK_YAW_SERVO_SLOT`).
- **Firmware clamp table** (`plant_config.c servo_table[]`, native Dynamixel steps 0–4095):
  pitch `[1024, 3072]`, yaw `[700, 2500]`. Commands outside this range get clamped in firmware
  regardless of what the host sends — `_write_dxl()` in `yam_continuous_all.py` clamps client-side
  to match before sending.
- **Continuous-cruise range** (`yam_continuous_all.py` `DXL_LO`/`DXL_HI`, a 40-step inset off a
  separately-captured clear range): pitch `[2193, 3018]`, yaw `[911, 2582]`.
- **Discover before commanding**: always send `torque_enable=False` first and read back present
  position (`ServoDesire(servo_id=sid, native_step_position=0, torque_enable=False,
  operating_mode=3)`), then seed the goal from that reading before enabling torque. Never command a
  goal position with `torque_enable=True` before you have a real present-position sample — you don't
  know which end of the range the servo is sitting at.
- **`operating_mode=3`** (position control) is required in every `ServoDesire` — this is not a
  default the firmware fills in.
- **Clear (release)**: `session.hub._connection.clear_servos(send=False)`; fallback if that's
  unavailable is `ServoDesire(servo_id=0)` per slot (both are used — see `_clear_dxl()` in
  `yam_continuous_all.py`).
- **Dual-owner warning**: the neck DXL pair is reachable from **either** `yam_continuous_all.py`
  (cruise bounce) **or** the standalone `yam_dxl_clear_teleop.py` (interactive keyboard clear-range
  capture) — but only one process may hold the CDC port (`/dev/ttyACM0` on the live Jetson board) at
  a time. Never launch both against the same board. The debug dashboard must stay in **follow mode**
  (no Connect COM) while either owns the port — see `docs/peripherals/continuous-ops.md`.

## Human deep dive

### Why torque-off discover first

Dynamixel present position is only meaningful once you've actually read it — commanding a position
goal with torque enabled before that first read risks slewing the servo hard from wherever it
happens to be resting toward an arbitrary goal (e.g. 0, or a stale value from a previous session).
`yam_continuous_all.py`'s "DXL PRESENT" phase polls with `torque_enable=False` for up to 2.5 s,
parsing `parse_servo_feedback(fb.raw, slot)` each tick, and only switches to
`torque_enable=True, native_step_position=<last present sample>` once **both** IDs have reported —
i.e. the first torque-enabled command a joint receives is a hold at its own current position, not a
snap.

### Present-position parse quirks

`_read_dxl_fb()` masks the raw feedback position to 12 bits when it comes back above 4095
(`pos &= 0x0FFF`) and only accepts a sample when either `motor_source_id` matches the expected ID
(or is `0`, meaning "not tagged", still accepted) or the position is nonzero. This guards against
treating an all-zero uninitialized feedback slot as a real "present position at zero" reading before
the servo has actually reported once.

### Clear-range provenance

The continuous-cruise `DXL_LO`/`DXL_HI` bounds are **not** the raw firmware clamp table — they are
a 40-step inset off a separately-captured clear range (see `yam_dxl_clear_teleop.py`'s interactive
capture tool, which writes a JSON artifact of observed min/max under keyboard control). Use the
firmware `servo_table[]` values only as the hard safety clamp; use the cruise `DXL_LO`/`DXL_HI` (or
re-run the capture tool) for anything that needs the actual mechanically-clear range.

### Cruise pattern

`yam_continuous_all.py` bounces both IDs independently between `DXL_LO[i]`/`DXL_HI[i]` at
`DXL_CRUISE_TICK_S = 280` ticks/s (native steps/s, not rad/s), reversing direction exactly at each
bound. This runs every tick alongside the arm/base writes in the same `_write_dxl()` call — DXL does
not get its own timing loop.

## Verified

**Date:** 2026-07-24, live board on Jetson, `python scripts/_tmp_launch_continuous.py`
(`yam_continuous_all.py --record --duration 50`, neck DXL enabled by default — no `--no-dxl`).

Present discover, both IDs resolved:
```
== DXL PRESENT ==
DXL present pitch=2597 yaw=1170
```

Continuous cruise, both IDs tracking cmd→fb (goal vs. present position each ~2 s status line,
`dxl=<pitch_cmd>/<pitch_fb>|<yaw_cmd>/<yaw_fb>`):
```
dxl=2906/2680|1191/834
dxl=2360/2591|1737/1415
dxl=2571/2467|2283/2248
dxl=2934/2824|2358/2563
```
Command and feedback track within ~100–300 native steps throughout (normal servo-loop lag at
`DXL_CRUISE_TICK_S=280`, not a fault). Full run recorded to
`.deft_session/recordings/record_20260724T214351.ndjson` on the Jetson.

## Known falsehoods retired

- **"DXL didn't move on this bench."** An earlier note (referenced in `yam_dxl_clear_teleop.py`'s
  docstring: "not moving DXL on this bench") was specific to using `PcbRobotSession`'s stream path
  for DXL commands — that path was confirmed not to move the servos. The fix was switching to
  `ControlsPcbHub` + paced `send_once`, which is what both `yam_dxl_clear_teleop.py` and (via
  `PcbRobotSession.set_servo`) `yam_continuous_all.py` now use successfully — live-verified moving
  both IDs above.
- **"Present position 0 means the servo isn't there."** False in general — only true when
  `motor_source_id` is nonzero *and* doesn't match, per the accept logic in `_read_dxl_fb()`. A
  genuine present-position-zero sample from the right ID is valid data, not an absence signal.
