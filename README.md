# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: fixed-rate plant loop (500 Hz), cyclic host command/feedback exchange (562-byte binary images), and plugin-based motor protocols over CAN.

**Bench scope today:** one RobStride RS-02 on FDCAN1, host link over USB CDC (controls PCB) or UART4 (dev board).

## Quick links

| Doc | Contents |
|-----|----------|
| [docs/architecture.md](docs/architecture.md) | Threads, buffers, module map, invariants |
| [docs/host-exchange-v1.md](docs/host-exchange-v1.md) | Wire layout v1 (562 B), magics, Python parity |
| [docs/bringup.md](docs/bringup.md) | Flash, transport toggle, Jetson teleop |
| [docs/known-issues.md](docs/known-issues.md) | As-built gaps and upgrade backlog |

## Hardware

- **MCU:** STM32G474RE (512 KiB flash, 128 KiB RAM)
- **Plant I/O:** FDCAN1/2/3 (CH1 used for RS-02), SPI1/SPI3, UART4/UART5
- **Host link:** USB FS CDC **or** UART4 (PC10 TX / PC11 RX @ 115200 8N1)
- **Plant tick:** TIM6 @ 500 Hz (2 ms); heartbeat LED PC3 @ ~2 Hz

## Build

1. Open the project in **STM32CubeIDE** (Debug configuration).
2. Set host transport in `App/Inc/host_transport.h`:

   ```c
   #define HOST_TRANSPORT_UART 1   // 1 = UART4, 0 = USB CDC
   ```

3. Build and flash `Debug/DeftRoboticsControlsPCB.elf`.

Cube-generated sources live under `Core/`, `USB_Device/`, and `Drivers/` (via `.cproject`). Application logic is under `App/`.

## Host teleop (Jetson / dev PC)

```bash
sudo apt install python3-serial
pip3 install -r scripts/requirements.txt
python3 scripts/host_teleop.py
```

At startup: press **1** = USB (`/dev/ttyACM*`), **0** = UART (`/dev/ttyUSB*` or `/dev/ttyTHS*`). See [docs/bringup.md](docs/bringup.md).

## Repository layout

```
App/           Application: host link, actuators, plugins, control loop
Core/          Cube HAL init (main, FDCAN, UART, TIM6, …)
USB_Device/    Cube USB CDC device stack + transport hooks
scripts/       Host-side teleop and tools
docs/          Architecture and contracts (this documentation set)
External_Documentation/   Vendor PDFs and reference code (not linked into build)
```

## Git

Firmware builds from tracked Cube + App sources. `Debug/` and `Release/` are ignored (see `.gitignore`).
