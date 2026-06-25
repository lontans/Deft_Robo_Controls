# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: fixed-rate plant loop (500 Hz), cyclic host command/feedback exchange (562-byte binary images), and plugin-based RobStride motor protocols over three FDCAN branches.

**Bench status:** four actuators validated on USB CDC (laptop) — RS02 + three RS01 drives on CH1 (daisy chain) and CH2/CH3. Multi-motor **plant teleop** (`--plant-teleop`) is the primary runtime path; RS2 PDU tools handle per-bus calibrate, discover, and single-motor bench work.

## Quick links

| Doc | Contents |
|-----|----------|
| [docs/architecture.md](docs/architecture.md) | Threads, buffers, dual host paths, module map |
| [docs/host-exchange-v1.md](docs/host-exchange-v1.md) | Wire layout v1 (562 B), magics, PDU RS2 backdoor |
| [docs/bringup.md](docs/bringup.md) | Flash, motor map, laptop USB teleop, calibrate |
| [docs/known-issues.md](docs/known-issues.md) | Open gaps, calibration NOISE on daisy bus |

## Hardware

- **MCU:** STM32G474RE (512 KiB flash, 128 KiB RAM)
- **Plant I/O:** FDCAN1/2/3 @ 1 Mbit/s, SPI1/SPI3, UART4/UART5
- **Host link:** USB FS CDC **or** UART4 (PC10 TX / PC11 RX @ 115200 8N1)
- **Plant tick:** TIM6 @ 500 Hz (2 ms); heartbeat LED PC3 @ ~2 Hz
- **CAN activity LEDs:** PC7 = CH1, PC6 = CH2, PB15 = CH3

### Actuator map (current `plant_config.c`)

| Slot | Schematic bus | Pins | Motor ID | Type |
|------|---------------|------|----------|------|
| 0 | CH1 | PB8 / PB9 | `0x70` | RS02 |
| 1 | CH1 (daisy) | PB8 / PB9 | `0x74` | RS01 |
| 2 | CH2 | PA8 / PA15 | `0x73` | RS01 |
| 3 | CH3 | PB12 / PB13 | `0x75` | RS01 |

**Note:** Schematic CH2/CH3 are wired to Cube **FDCAN3 / FDCAN2** respectively (`can_router.c` `bus_handle` swap). Host scripts use schematic bus numbers `1` / `2` / `3` in `--bus` and `pdu.data[11]`.

## Build

1. Open the project in **STM32CubeIDE** (Debug configuration).
2. Set host transport in `App/Inc/host/host_transport.h`:

   ```c
   #define HOST_TRANSPORT_UART 0   // 0 = USB CDC (controls PCB laptop bench)
   // #define HOST_TRANSPORT_UART 1   // 1 = UART4 (dev board / Jetson)
   ```

3. Build and flash `Debug/DeftRoboticsControlsPCB.elf`.

Cube-generated sources live under `Core/`, `USB_Device/`, and `Drivers/`. Application logic is under `App/`.

## Host tools (laptop USB — primary bench)

```powershell
pip install -r scripts/requirements.txt
python scripts/host_teleop_laptop_usb.py --list-ports
python scripts/host_teleop_laptop_usb.py --port COM9 --plant-teleop
```

Per-bus calibrate / discover:

```powershell
python scripts/rs02_can_scan.py --port COM9 --bench-cmds --bus 1 --target 0x70
python scripts/host_teleop_laptop_usb.py --port COM9 --calibrate --bus 2 --motor-id 0x73
```

Jetson / UART path: see [docs/bringup.md](docs/bringup.md) (`scripts/host_teleop.py`).

## Repository layout

```
App/           Application: host link, actuators, plugins, control loop, plant_diag
Core/          Cube HAL init (main, FDCAN, UART, TIM6, …)
USB_Device/    Cube USB CDC device stack + transport hooks
scripts/       host_teleop_laptop_usb.py, rs02_can_scan.py, host_teleop.py
docs/          Architecture and contracts
External_Documentation/   Vendor PDFs (not linked into build)
```

## Git

Firmware builds from tracked Cube + App sources. `Debug/` and `Release/` are ignored (see `.gitignore`).
