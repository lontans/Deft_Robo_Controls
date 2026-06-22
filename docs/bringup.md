# Bring-up

## 1. Select host transport (firmware)

Edit `App/Inc/host/host_transport.h` before building:

```c
#define HOST_TRANSPORT_UART 1   // dev board: UART4
// #define HOST_TRANSPORT_UART 0   // controls PCB: USB CDC
```

| Board | `HOST_TRANSPORT_UART` | Physical link |
|-------|----------------------|---------------|
| Dev / UART wiring | `1` | UART4 PC10/11 @ 115200 8N1 |
| Controls PCB | `0` | USB FS CDC |

Rebuild and flash from STM32CubeIDE (Debug).

## 2. Motor and CAN (bench)

Current `plant_config.c` stub:

- One actuator, `PROTO_ROBSTRIDE`, `CAN_BUS_CH1`, motor ID `0x7F`, enabled
- FDCAN1 @ 1 Mbit/s, extended IDs accepted (mask filter)

On boot, `control_loop_init()` sends RobStride **enable** (comm type 3) once, then starts TIM6 @ 500 Hz.

## 3. Jetson / host teleop

```bash
cd /path/to/DeftRoboticsControlsPCB
pip3 install -r scripts/requirements.txt
python3 scripts/host_teleop.py
```

When prompted:

- **1** — USB CDC → `/dev/ttyACM*` (ignore baud; pyserial still sets 115200)
- **0** — UART → `/dev/ttyUSB*` or `/dev/ttyTHS*` @ 115200 8N1

Skip prompt: `python3 scripts/host_teleop.py --transport usb` or `--transport uart`.

Optional: `--port /dev/ttyACM0`, `--hz 20`, `--kp 50`, `--kd 1`.

### Teleop keys

| Key | Action |
|-----|--------|
| Left / Right | Move slot-0 position command |
| r | Zero command position |
| q | Quit |

Default rates: **30 Hz** USB, **10 Hz** UART (see [host-exchange-v1.md](host-exchange-v1.md)).

## 4. What success looks like

| Check | Expected |
|-------|----------|
| Heartbeat LED (PC3) | Toggles ~2 Hz |
| Teleop status | `ok`, feedback position updates |
| `ack_seq` | Tracks low 8 bits of command seq after key press |
| `tick` | Increments over time (12-bit plant counter) |
| CAN | RS-02 responds after enable; parse updates `position` / `velocity` |

## 5. Common mismatches

| Symptom | Likely cause |
|---------|----------------|
| No feedback | Wrong transport toggle vs cable (`ACM` vs `USB`) |
| Garbage / no apply | Baud mismatch (UART), or RX desync — see [known-issues.md](known-issues.md) |
| Motor silent | Enable failed, wrong CAN ID, or drive not on bus |
| Stale tick on UART | Blocking TX; feedback not rebuilt until prior send completes |

## 6. Size check (optional)

After build:

```bash
arm-none-eabi-size Debug/DeftRoboticsControlsPCB.elf
```

Expect on the order of **~53 KiB flash**, **~21 KiB static RAM** (see conversation notes / future README addendum).
