# deft_controls_sdk

Host SDK for the Deft Robotics controls PCB (USB CDC).

Runs on **Windows** and **Ubuntu/Jetson**. Pass a port (`COM5`, `/dev/ttyACM0`) or omit it and let discovery pick STM32 CDC `0483:5740`. Install host deps from [`../requirements.txt`](../requirements.txt) (`pyserial`, `libusb1`, …); Linux Soft-DFU setup is in [`../README.md`](../README.md).

## Mental model

```text
connect  → open COM + stream          (choose mode)
arm      → plant_apply ON             (motors track)
command  → mount → apply → clear
disarm   → plant_apply OFF
close    → release COM
```

| `mode=` | Plant motion | CFG / discover / inventory / cal |
|---------|--------------|----------------------------------|
| `"bandwidth"` | yes | **no** (`hub.debug` blocked) |
| `"debug"` | yes | **yes** — use for bringup |

Change mode only by disconnect + reconnect.

## Shape

| Layer | Role |
|-------|------|
| `HostProxy` | Session + section demux (`set_section`) into held 694B CMDH |
| `proxy.actions` | Lab/notebook batch: `mount` / `apply` / `clear` (not product path) |
| `actions/` | `ActuatorAction` / `ServoAction` / `LedAction` / `TeleopEngine` |
| `config/` | Stock assemblies (`product` sections / `bench`) + YAM CFG preset |
| `debug/` | `hub.debug.*`, `run_inventory`, `collect_cfg`, Soft-DFU |
| `ros/` | Optional ROS node → `set_section` (MIT `5*n` commands) |
| `ControlsPcbHub` | Wire / USB |

Product YAM drivers live in parent **deft_vbeta** (not this package).

Lab CLI: [`../pcb_lab/`](../pcb_lab/) — flash/USB via `python -m pcb_lab`; interactive prove via `python -m pcb_lab.debug`.

## Quick start (notebook)

```python
from deft_controls_sdk import HostProxy
from deft_controls_sdk.actions import make_teleop_engine, neck_cruise
from deft_controls_sdk.debug import as_hex, run_inventory

# Bringup session — disarmed until you say so
# Port: "COM5" / "/dev/ttyACM0" / omit for auto-discover
with HostProxy.connect(mode="debug", armed=False) as proxy:
    hub = proxy.hub
    assert proxy.mode == "debug"
    assert proxy.armed is False

    # Who's on the bus? (debug mode only)
    run_inventory(
        proxy,
        buses=(5, 6),
        ranges={"robstride": (0x70, 0x72)},  # explicit IDs — or preset="bench"
        include_servos=True,
        include_pdu=False,
    )
    print(as_hex(hub.debug.discover_robstride_by_bus(buses=[5], start=0x70, end=0x72)))
    print(proxy.cfg_snapshot()["enabled_count"])  # debug mode only

    # connect → arm → mount/apply → clear → disarm
    proxy.arm_plant()
    a = proxy.actions
    wheels = a.actuator(slots=(22, 23))
    neck = a.servo()

    a.mount(wheels.hold(kp=20.0, kd=1.0))   # sample FB → stay put (no move)
    a.mount(neck.neck_center())
    print(a.pending)                   # inspect mounted patches
    a.apply()                          # commit once

    a.mount(wheels.nudge(index=0, delta=0.05, kp=20.0, kd=1.0))
    a.apply()
    print("FB", wheels.positions())

    a.clear()                          # blank touched groups + empty pending
    # Neck cruise uses TeleopEngine (timed slew — separate from mount/apply)
    engine = make_teleop_engine(lambda: hub)
    neck_cruise(engine, pitch=2048, yaw=2048, cruise=200)
    # ... later: engine.stop()

    proxy.disarm_plant()
```

### Product sections (deft_vbeta)

Default connect uses `yam_product_assembly()` — fixed demux names:

`left_arm` · `right_arm` · `base_wheel_1|2|3` · `torso` (+ servo `neck`)

```python
from deft_controls_sdk.link import ActuatorDesire

with HostProxy.connect("COM5", mode="bandwidth", armed=True, listen_pdu=True) as proxy:
    # Product authors full MIT fields; HostProxy only demuxes onto slots.
    proxy.set_section(
        "left_arm",
        [ActuatorDesire(position=0.0, kp=30.0, kd=1.0) for _ in range(7)],
    )
```

### Lab named groups (`actions`)

```python
from deft_controls_sdk.config import assembly_from_name

asm = assembly_from_name("bench")  # spare-slot wheels 22–25
with HostProxy.connect("COM5", mode="debug", armed=False, assembly=asm) as proxy:
    proxy.arm_plant()
    a = proxy.actions
    a.mount(a.actuator(name="base").hold(kp=20.0, kd=1.0))  # stay put
    a.apply()
    a.clear()
    proxy.disarm_plant()
```

See [docs/integration.md](../../docs/integration.md) for the product demux contract.

### CFG identity (RAM vs NVM)

`ActuatorProfile` holds **slots + CFG identity** (bus / protocol / motor_id). Not gains.

```python
from deft_controls_sdk.config import single_profile

# Build host-side identity for plant slot 22
prof = single_profile(22, protocol="robstride", motor_id=0x70, bus=5)

# Check live RAM table
table = hub.debug.cfg_get_table()          # 26 rows (or None)
print(proxy.cfg_snapshot()["enabled_count"])

# Write RAM only (lost on reboot unless SAVE)
for row in prof.as_cfg_rows():
    hub.debug.cfg_set_slot(**row, persist=False)

# Persist RAM → flash (NVM)
hub.debug.cfg_save_nvm()                   # or cfg_set_slot(..., persist=True)

# Reload flash → RAM (e.g. after reboot / to restore NVM into live table)
hub.debug.cfg_load_nvm()
```

Discover → map hits onto slots → `single_profile` / `as_cfg_rows` → `cfg_set_slot` is the bringup path. `zip(SLOT_POOL, hits)` is just assigning plant slot indices to discovered `(bus, protocol, id)` tuples.

### `mount` / `apply` / `clear`

- Helpers (`hold`, `nudge`, `neck_center`, …) return a `MountedAction` patch.
- `a.mount(...)` appends to the pending list (inspect via `a.pending`).
- `a.apply()` merges pending into the hub held map and `commit()`s once.
- `a.clear()` blanks groups touched by prior applies, then empties pending.
- `send=True` on helpers is a legacy shortcut (immediate TX / mount+apply).
- Pass `kp`/`kd` on hold — there is no joint/wheel `kind` on profiles or actions.

## CFG: RAM vs flash

```text
GET / SET     → live RAM (lost on reboot unless SAVE)
SAVE          → RAM → flash
LOAD          → flash → RAM
```

**USB flash:** `python -m pcb_lab flash`  
**Dashboard:** `python -m deft_controls_sdk.debug_dashboard`  
**Tests:** `pytest pcb_lab/tests`
