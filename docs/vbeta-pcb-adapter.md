# Controls PCB ↔ deft_vbeta adapter

Replace `I2RTArmDriver` and `FeatherPlatformClient` with PCB-backed drivers in
`scripts/deft_controls_sdk/vbeta/`. Cameras / episode packing stay in vbeta.

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

`ACTUATOR_COUNT = 25`. Neck = `servo[]`. LEDs = SK9822 word.

| Slots | Name | Bus | Protocol |
|------:|------|-----|----------|
| 0–6 | left arm J1–J7 | CH1 | Damiao |
| 7–13 | right arm J1–J7 | CH2 | Damiao |
| 14–16 | BwC, BwR, BwL | CH4–6 MCP | RobStride |
| 17–19 | BpC, BpR, BpL | CH4–6 MCP | RobStride |
| 20 | lift **reserved** | CH3 | **disabled** until bringup |
| 21–24 | spare | — | disabled |

Distinct from the all-RobStride timing-matrix CFG used by load benches.

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

### Platform — `PcbPlatformClient` ↔ `FeatherPlatformClient`

|Cmd|PCB|
|-----|-----|
|`base_target_cmd` / `send_target_state`|**Preferred.** Steer pos + drive vel → slots 14–19|
|`base_cmd`|**Limited.** `(turn_deg, linear_m_s, angular_deg_s)`: all-zero → stop; otherwise hold common steer angle and **zero drive** (no Feather `nav_planner`). Teleop that needs go_to/rotate must stay host-side or use `base_target_cmd`|
|`lift_cmd`|stub|
|`neck_cmd`|servo 0/1|
|`heartbeat`|refresh watchdog clock|
|`enable/disable_drive_current`|re-apply / blank **drive** slots only|
|`get_state()`|angles/vels from FB; lift zeros; `lift_unimplemented=1`|

Watchdog ≈ 1 s without heartbeat/cmd → zero drive + lift stub 0 (steer held).

### LEDs

`set_led(mode, brightness, count=0)` / `led_off()` → `hub.set_led(LedDesire(...))`.
Modes: 0=OFF, 1=TEST, 2=FLASH.

## Units

|Domain|Units|
|--------|-------|
|Arm|rad, rad/s|
|Base steer / drive|rad / rad/s|
|Lift (API only)|mm / mm/s|
|Neck|deg → DXL native steps|

## YAMAIMobile integration sketch (patch in deft_vbeta, not this repo)

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
    neck_pitch_offset_deg=robot.neck_pitch_offset_deg,
)
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

## Soft-DFU (user-facing)

```powershell
python scripts/soft_dfu_flash.py
# or: python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
```

Do not teach enter/leave/tag/`DFU!` for normal flash; those are internals.
