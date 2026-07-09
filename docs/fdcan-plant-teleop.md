# FDCAN RS02 plant teleop

RS02 runtime control on **FDCAN CH1–CH2** uses the **plant path** only: zero PDU tag, `actuator_commands[]` at host rate, MCU applies MIT comm 0x01 at **500 Hz**.

Bench (`R/S/2` PDU → `plant_diag`) is for **calibrate / discover / probe** only. Do not mix bench and teleop without a handoff.

## Slots and buses

| Slot | Bus | Hardware | Plant teleop |
|------|-----|----------|--------------|
| 0 | CH1 | FDCAN1 | RS02 runtime OK |
| 1 | CH2 | FDCAN3 | RS02 runtime OK |
| 2 | CH3 | FDCAN2 | Damiao (separate teleop) |
| 3–5 | CH4–6 | MCP2518 | RS02 plant teleop OK (slot 3 default) |

## Workflow

```powershell
# After calibrate or any bench command:
python scripts/control_hub.py --port COM5 recover --bus 2
python scripts/control_hub.py --port COM5 link-test

# Runtime teleop (slot 1 = CH2 0x70):
python scripts/control_hub.py --port COM5 teleop --slot 1

# After cal, skip homing if already at zero:
python scripts/control_hub.py --port COM5 teleop --slot 1 --skip-home
```

Teleop calls `ensure_plant_runtime()` first: heal USB, burst neutral plant frames, require `plant_block=none`.

During motion, live line should show `block=none` and `ack` tracking `tx` (mod 256).

## Tuning (laptop teleop UX only)

Defaults live in `scripts/control_hub/teleop/defaults.py`. Override per session on the CLI:

| Flag | Default | Effect |
|------|---------|--------|
| `--arrow-vel` | 3.5 | Peak speed while **holding** arrow (rad/s) |
| `--ramp-up` | 0.12 | Velocity build-up on press (s) — lower = snappier hold |
| `--ramp-down` | 0.35 | Coast-down on release (s) |
| `--kp` | slot table | Max stiffness while moving (ramps with speed) |
| `--kd` | 0.45 | D gain while moving |
| `--home-kp` | 6.0 | Homing stiffness |
| `--home-slew` | 0.18 | Homing slew rate (rad/s) |
| `--hz` | 40 | Host command rate |
| `--skip-home` | off | Skip slew to 0 rad at start |

Examples:

```powershell
# Gentler jog on a small supply:
python scripts/control_hub.py --port COM5 teleop --slot 1 --arrow-vel 2 --kp 5 --ramp-down 1.0

# Snappier hold response:
python scripts/control_hub.py --port COM5 teleop --slot 1 --ramp-up 0.08 --arrow-vel 4

# Slower homing after cal:
python scripts/control_hub.py --port COM5 teleop --slot 1 --skip-home --home-slew 0.15
```

Teleop is **hold-to-cruise**: press ramps to a fixed `arrow_vel`, hold keeps that speed (position integrates), release coasts down. Speed does not build the longer you hold.

**Software tests** should not use these ramps — send `(position, velocity, kp, kd, torque)` directly via `PcbSession.send_plant()` (PlantSession API planned).

## Keys

- Arrows: velocity-mode jog (integrates `cmd_position` while moving; coasts down on release)
- `0`–`6`: limit motion to one bus (`0` = all)
- `r`: re-sync `cmd_position` from feedback
- `q`: quit (neutral hold, then exit)

## plant_block values

| Name | Meaning |
|------|---------|
| `none` | 500 Hz apply running — good for teleop |
| `bench_session` | RS2/DM bench active — run `recover` |
| `probe_busy` | Blocking probe — wait or `recover` |
| `quiet_period` | Post-session cooldown — wait ~3 s or `recover` |
| `host_stale` | Normal when idle; should clear while streaming teleop |

## MCP CH4–6 (after firmware reflash)

Same plant path as FDCAN — slot maps to bus in `plant_config.c` / host mirror:

```powershell
python scripts/control_hub.py --port COM5 recover --bus 4
python scripts/control_hub.py --port COM5 teleop --slot 3   # CH4 MCP, default 0x70
```

Bench cal still uses `calibrate --bus 4`. Run `recover` before teleop. Watch PC14 (CH4 ACT) for CAN traffic.

If motion is choppy, SPI queue may be saturated at 500 Hz — try one MCP slot at a time first.


- [architecture.md](architecture.md) — dual host paths
- [bringup.md](bringup.md) — flash and motor map
