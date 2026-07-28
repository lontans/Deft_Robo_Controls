# Controls PCB ↔ deft_vbeta adapter

Replace `I2RTArmDriver` and `FeatherPlatformClient` with PCB-backed drivers in
`scripts/deft_controls_sdk/vbeta/`. Cameras / episode packing stay in vbeta.

**Study this stack by testing (not skim):** [study-sdk-damiao-vertical.md](study-sdk-damiao-vertical.md)
— vertical labs from `vbeta_smoke.py arm` / `install_pcb_backend` down to
`damiao_apply_cycle`, with checkpoints and Ask-me prompts.

## Reference checkout (`docs/deft_vbeta_ref/deft_vbeta`)

Declared in [`.gitmodules`](../.gitmodules) as a real submodule
(`gitlab.com/deftrobotics/deft_vbeta`) — the existing `160000` gitlink
already committed to this repo pointed at a valid commit with no
`.gitmodules` entry to explain it, so `git submodule` tooling couldn't see
it. Fixing that is just the gitlink pointer (a few dozen bytes); it does
**not** commit the 36 MB working tree — that stays untracked inside the
submodule's own `.git`, per the existing bloat-avoidance call in
[`act-lap-bloat-deepdive-2026-07-23.md`](legacy/act-lap-bloat-deepdive-2026-07-23.md).
Read-only reference for contract-matching; never edit files under it from
this repo.

## Live prove (2026-07-24, product CFG)

`PcbArmDriver` (left, CH1) + `PcbPlatformClient` (base) were run against the **live** Jetson board
under the actual `yam_product_rows()` CFG (not the bench-spare map) via
[`scripts/vbeta_product_prove.py`](../scripts/vbeta_product_prove.py) — full results, the base
ID-remap gap below, and `deft_vbeta/`'s current blocker in
[`bench-vbeta-product-cfg-2026-07-24.md`](legacy/bench/bench-vbeta-product-cfg-2026-07-24.md). Left arm: live,
MIT-armed, `Goal_Position` tracking confirmed. Right arm (CH2): CFG'd, not physically present this
session. Base (CH4–6, product IDs `0x01`/`0x02`): **not found** — see remap gap.

### Base ID remap gap (product `0x01`/`0x02` vs. bench `0x70`/`0x74`/`0x75`/`0x06`)

The physically-wired base RobStride/Damiao drives on this bench answer to bench-spare IDs
(`docs/peripherals/base-robstride-mcp.md`, `docs/peripherals/base-damiao-ch6.md`), not the product
map's `0x01` (steer) / `0x02` (drive) per CH4–6 rail. `PcbPlatformClient` is CFG- and protocol-correct
against the product map as specified — this is a hardware/CFG mismatch on the current bench, not an
adapter bug. **Do not** silently point `PcbPlatformClient`/`yam_product_rows()` at the bench-spare
IDs to paper over this — track it as its own ADR if/when the base needs to actually drive under the
product stack (see the three-agent plan's backlog: "Base product `0x01`/`0x02` vs bench `0x70`/`0x74`
ADR").

## Parity status (vs `docs/deft_vbeta_ref/deft_vbeta` @ `6cd886f`)

Checked method-for-method against the reference `I2RTArmDriver`,
`FeatherPlatformClient`, and `YAMAIMobile` call sites (arms, platform
non-lift, neck, `yam_product_rows`/CFG). Everything actually called by
`YAMAIMobile` on these surfaces is matched; `motor_models`/`motor_indices`
(I2RT properties unused for the YAM/I2RT arm path — only feetech/dynamixel
calibration scripts touch them) were not ported since nothing calls them.

**Fixed:** `PcbPlatformClient` was double-applying `neck_pitch_offset_deg` —
`YAMAIMobile.convert_vr_head_angles` already bakes the offset into pitch
before enqueueing `"neck_cmd"`, and the reference `FeatherPlatformClient`
control loop forwards pitch/yaw raw (`neck.go_to(pitch, yaw)`, no second
offset). `PcbPlatformClient._apply_neck` re-added it, which would have
pointed the physical neck wrong under real teleop. Removed the constructor's
`neck_pitch_offset_deg` param; `neck_cmd` now forwards pitch/yaw unmodified,
matching the reference. Regression-guarded by
`test_platform_neck_cmd_no_double_offset` in
[`scripts/tests/test_deft_controls_sdk_vbeta.py`](../scripts/tests/test_deft_controls_sdk_vbeta.py).

Known, intentional divergences (not bugs — see Method maps below):
`base_cmd` is limited vs. Feather's `nav_planner` go_to/rotate; lift is a
stub; `PcbPlatformClient` is in-process (no subprocess), so `is_alive()`
means "connected" rather than "child process alive".

## USB COM ownership

Exactly **one** process owns CDC. Hot loop = soft-DFU + streaming + smoke/matrix.

| Role | Owns COM? | Board? |
|------|-----------|--------|
| API + unit tests | No | No (fake hub) |
| Cursor HW smoke | Yes (exclusive) | Yes |
| Claude firmware edit/build | No | No |
| Claude soft-DFU + load matrix | Yes (exclusive) | Yes |
| Dashboard | Close before smoke/matrix | — |

`PcbRobotSession` is the sole hub owner when live.

## Slot map (YAM product CFG)

`ACTUATOR_COUNT = 26` (host exchange layout v3, [`docs/host-exchange-v3.md`](host-exchange-v3.md) —
bumped from v2/25 by Track B's PDU-SDK contract work; image 694 B, servos @ 616,
LED @ 628, PDB/DEBUG mailbox @ 630). Neck = `servo[]`. LEDs = SK9822 word.

| Slots | Name | Bus | Protocol |
|------:|------|-----|----------|
| 0–6 | left arm J1–J7 | CH1 | Damiao |
| 7–13 | right arm J1–J7 | CH2 | Damiao |
| 14–16 | BwC, BwR, BwL | CH4–6 MCP | RobStride |
| 17–19 | BpC, BpR, BpL | CH4–6 MCP | RobStride |
| 20 | lift **reserved** | CH3 | **disabled** until bringup |
| 21–25 | spare | — | disabled |

`yam_product_rows()` (the YAM-specific CFG this adapter applies) keeps all of
20–25 disabled/spare — distinct from the plant's generic factory-default NVM
layout (`CH1×8, CH2×8, CH3×4, CH4–6×2`) and from the all-RobStride
timing-matrix CFG used by load benches, both of which are Track B's bench
territory, not the YAM product mapping.

## Lift (stub)

`lift_cmd` / `lift_velocity` / `lift_height` exist for Feather API compatibility but
are **no-ops** (no plant desire). Slot 20 stays CFG-disabled. Bring up later via
FeatherSDK — **confirmed CANopen**, see
[`feathersdk-lift-teardown.md`](feathersdk-lift-teardown.md) for the full API
surface, what's reusable from `zeroerr.c`/`canopen.c`, and the ordered
de-stub plan (bench discovery is the blocking first step, not more code).

## Method maps

### Arms — `PcbArmDriver` ↔ `I2RTArmDriver`

|I2RT|PCB|
|------|-----|
|`connect()`|session recover + stream; optional go_to home|
|`read("Position_Rad")`|FB positions slots 0–6 or 7–13|
|`read_all()`|joint_pos[6], gripper_pos, vel, torque|
|`write("Goal_Position", q7)`|7× `ActuatorDesire` MIT (rad)|
|`write("Zero_Torque", bool)`|idle desires (`kp=0`)|
|`go_to(pos, dt)`|host-side linear interp|
|`disconnect()`|sleep pose + clear|

Default MIT gains (bringup): kp ≈ `(40,60,90,60,25,25,20)`, kd ≈ `1.0`.

### Soft limits (`yam_limits`)

Host soft stops live in [`scripts/deft_controls_sdk/vbeta/yam_limits.py`](../scripts/deft_controls_sdk/vbeta/yam_limits.py)
(port of legacy `yam_limits.py`): J1–J6 from `yam.xml`, J7 provisional motor-frame,
left/right mirrored. API: `load_yam_limits`, `soft_limits_q7`, `clamp_q7`,
`plan_hold_q7` / `plan_jog_q7`.

`PcbArmDriver` clamps `Goal_Position` / `go_to` by default (`clamp_goals=True`).
**Caveat:** XML = model frame; Damiao FB = motor encoder until zeros — clamps are
host soft stops for relative hold/jog, not a substitute for calibration. Absolute
teleop still needs zeros (see [bringup.md](bringup.md)).

One-arm smoke (clamped): `vbeta_smoke.py arm --hold` / `--jog` — Jetson CLI
documented in `vbeta_smoke_lib.py` docstring; HW prove deferred until rig ready.

### Platform — `PcbPlatformClient` ↔ `FeatherPlatformClient`

|Cmd|PCB|
|-----|-----|
|`base_target_cmd` / `send_target_state`|**Preferred.** Steer pos + drive vel → slots 14–19|
|`base_cmd`|**Limited.** `(turn_deg, linear_m_s, angular_deg_s)`: all-zero → stop; otherwise hold common steer angle and **zero drive** (no Feather `nav_planner`). Teleop that needs go_to/rotate must stay host-side or use `base_target_cmd`|
|`lift_cmd`|stub|
|`neck_cmd`|servo 0/1, pitch/yaw forwarded raw (no offset — caller already applied it)|
|`heartbeat`|refresh watchdog clock|
|`enable/disable_drive_current`|re-apply / blank **drive** slots only|
|`get_state()`|angles/vels from FB; lift zeros; `lift_unimplemented=1`|

Watchdog ≈ 1 s without heartbeat/cmd → zero drive + lift stub 0 (steer held).

### LEDs

`set_led(mode, brightness, count=0)` / `led_off()` → `hub.set_led(LedDesire(...))`.
Modes: 0=OFF, 1=TEST, 2=FLASH, 3=SOLID_GREEN, 4=SOLID_YELLOW, 5=SOLID_RED,
6=BLINK_YELLOW_SLOW, 7=BLINK_RED_FAST, 8=IDLE_CORNFLOWER (`led_idle()`,
`#6495ED`, flat 500 ms on/off — Track B's default for PDB `NORMAL`+fresh,
replacing solid green). Not part of the reference `deft_vbeta` surface —
PCB-only, no I2RT/Feather equivalent to match.

## Units

|Domain|Units|
|--------|-------|
|Arm|rad, rad/s|
|Base steer / drive|rad / rad/s|
|Lift (API only)|mm / mm/s|
|Neck|deg → DXL native steps|

## YAMAIMobile integration sketch (patch in deft_vbeta, not this repo)

**Landed as real code**: `deft_vbeta/src/deft_amr/amr/amr/pcb_bridge.py`
(`install_pcb_backend(robot)`) in the repo-root `deft_vbeta/` working checkout (gitignored fresh
clone, not the read-only `docs/deft_vbeta_ref/deft_vbeta` reference). Not yet exercised through a
live `YAMAIMobile` instance — the Jetson's Python env has neither `torch` nor `mujoco`, both
required at `YAMAIMobile.__init__` import time. See
[`bench-vbeta-product-cfg-2026-07-24.md`](legacy/bench/bench-vbeta-product-cfg-2026-07-24.md) for the direct
(non-`YAMAIMobile`) adapter prove that doesn't depend on that environment.

Live call sites construct `YAMAIMobile` directly; arms come from
`make_motors_buses_from_configs` (`i2rt_driver`); platform from
`FeatherPlatformClient` when `use_feather_platform=True`.

### Option A — minimal monkey-patch at recorder entry (fastest)

```python
# In episode_recorder / eval, before YAMAIMobile(...):
from deft_controls_sdk.vbeta import (
    PcbRobotSession, PcbArmDriver, PcbPlatformClient, ensure_yam_product_cfg
)

session = PcbRobotSession.connect(apply_yam_cfg=True)  # exclusive COM
robot = YAMAIMobile(config)  # still constructs I2RT + Feather

# Replace arms (I2RT-compatible surface)
robot.follower_arms = {
    "left": PcbArmDriver(session, side="left"),
    "right": PcbArmDriver(session, side="right"),
}
# Replace platform client; keep use_feather_platform=True so YAM still
# calls send_command / get_state / heartbeat on robot.feather_client.
robot.feather_client = PcbPlatformClient(
    session, use_neck=robot.use_feather_neck,
)
# Do NOT pass neck_pitch_offset_deg here: YAMAIMobile.convert_vr_head_angles
# already bakes it into pitch before "neck_cmd" is enqueued. PcbPlatformClient
# forwards pitch/yaw raw (matches reference FeatherPlatformClient control loop) —
# applying the offset again here would double it.
# Skip Feather subprocess connect path: PcbPlatformClient.connect is cheap.
# Ensure robot.connect() still calls feather_client.connect() — it does today.
```

On exit / `robot.disconnect()`, also `session.close()` (stops stream, blanks desires).

### Option B — factory branch (cleaner)

1. Add `PcbArmDriverConfig` / register `type="pcb_arm"` in motors configs +
   `make_motors_buses_from_configs` → construct `PcbArmDriver` with a shared session.
2. Add config `use_pcb_platform: bool` (or `platform_backend="pcb"|"feather"`).
3. In `_init_feather_platform`: if PCB backend, `self.feather_client = PcbPlatformClient(...)`.
4. Own one `PcbRobotSession` on the robot (`self._pcb_session`); connect/disconnect with arms.

Keep `send_command` / `get_state` / `send_target_state` / `heartbeat` names so
`_enqueue_feather_cmd`, teleop_base_lift, and `send_action` base/lift slices need
**no** call-site edits. Lift remains stub until slot 20 is wired.

### Teleop `base_cmd` vs policy `base_target_cmd`

|YAM path|Feather today|PCB first cut|
|--------|-------------|-------------|
|`teleop_base_lift` → `base_cmd`|nav_planner go_to / rotate|stop or steer-hold only — **prefer converting teleop to `send_target_state`**|
|`send_action` → `send_target_state`|manual targets|full support|

## Rig integration (single Damiao arm + optional pieces)

Offline scaffold for composing the bench rig on **one** `PcbRobotSession` as
pieces come online — see
[`scripts/deft_controls_sdk/vbeta/rig.py`](../scripts/deft_controls_sdk/vbeta/rig.py).
Nothing here has been proven on hardware; it's additive to the standalone
`PcbArmDriver` path already smoke-tested (see
[`bench-vbeta-arm-2026-07-24.md`](legacy/bench/bench-vbeta-arm-2026-07-24.md)).

**Bring-up order** (each step keeps everything before it working):

1. Hold baseline — one `PcbArmDriver` alone (already smoke-tested)
2. Add RobStride soft-hold — `robstride_soft_hold()` on the rig's canonical
   RS02 bus-6 slot (`RIG_RS02_BUS6_SLOT = 24`, `id=0x70`, per
   [`bench-pdb-plant-integ-2026-07-23.md`](legacy/bench/bench-pdb-plant-integ-2026-07-23.md)).
   This slot is CFG-disabled/spare in `yam_product_rows()` — needs a bench CFG
   override to actually drive it, not the YAM product CFG.
3. Neck DXL hold — `neck_hold_present()` re-issues the present pitch/yaw so
   the neck doesn't relax rather than actively moving it
4. LED idle — `led_idle()` (mode 8, already the PDB-`NORMAL` default)
5. PDU strip — `pdb_poll()` (`hub.pdb_status()`, Track B API)

`RigComponents` batches whichever of 2–4 are attached behind one `tick()`
call per loop iteration — construct with only the pieces the bench actually
has wired (all default off). It services soft-kill first
(`PcbRobotSession.service_soft_kill()`) and skips every other component on
that tick if parked, same park-first order `send_once()` already uses. Every
write is `send=False` (held-desire update only) — the caller's existing
`send_once()` / streaming loop does the actual TX, and `PcbRobotSession`
stays the sole COM owner throughout; `RigComponents` never opens or shares a
second connection.

Fake-hub tests: `scripts/tests/test_deft_controls_sdk_vbeta.py`
(`test_robstride_soft_hold_*`, `test_neck_hold_present_*`, `test_pdb_poll_*`,
`test_rig_components_tick_*`). No COM5 — HW prove is deferred until the rig
is ready (single-arm only; no dual-arm, no CubeMars CFG flip in this pass).

## Soft-DFU (user-facing)

```powershell
python scripts/soft_dfu_flash.py
# or: python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
```

Do not teach enter/leave/tag/`DFU!` for normal flash; those are internals.
