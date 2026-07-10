# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: fixed-rate plant loop (500 Hz), cyclic host command/feedback exchange (562-byte binary images), and plugin-based motor protocols over FDCAN (RobStride extended CAN, Damiao standard CAN, and **mixed** std+ext on CH1/CH3) plus MCP2518 SPI-CAN on CH4–6.

**Bench status (Jul 2026):**

| Area | Status |
|------|--------|
| **RobStride CH1–CH3 (FDCAN)** | Plant teleop, calibrate, discover — validated on USB CDC |
| **RobStride CH4–CH6 (MCP)** | Plant teleop and calibrate — validated (see MCP postmortem doc) |
| **Mixed Damiao + RS (CH1 / CH3)** | Firmware `FDCAN_RX_STD_AND_EXT`; multi-slot plant teleop verified when all configured motors are enabled |
| **Damiao CH1 daisy chain** | **In progress** — **DM-J4310** discoverable (`--discover --host-only`); **DM-J4340** not found yet; **homing OK** on configured 4310 slots; **teleop not yet** — 4310s fault when sandwiched between un-enabled 4340s on the harness |
| **Damiao CH3 (single motor)** | Earlier bench — USB/DM0 OK; `rx_raw=0` until motor-end **120 Ω** (see [bringup.md](docs/bringup.md)) |

## Quick links

| Doc | Contents |
|-----|----------|
| [docs/architecture.md](docs/architecture.md) | Threads, buffers, dual host paths, module map |
| [docs/host-exchange-v1.md](docs/host-exchange-v1.md) | Wire layout v1 (562 B), RS2 + DM0 PDU backdoors |
| [docs/bringup.md](docs/bringup.md) | Flash, motor map, plant teleop, Damiao CH1 daisy chain, mixed bus |
| [docs/plan-damiao-4340-bringup.md](docs/plan-damiao-4340-bringup.md) | Agent plan: DM-J4340 + YAM slots via NVM (minimal stack changes) |
| [docs/plan-yam-joint-commands.md](docs/plan-yam-joint-commands.md) | Agent plan: YAM joint limits + user/AI command options |
| [docs/fdcan-dual-id-mixed-bus.md](docs/fdcan-dual-id-mixed-bus.md) | Mixed 11-bit + 29-bit classic CAN — **§0 as-built** (FIFOs, filter masks, demux) |
| [docs/known-issues.md](docs/known-issues.md) | Open gaps — Damiao 4340 discover, cal NOISE, etc. |
| [docs/ch4-mcp2518-bringup-postmortem.md](docs/ch4-mcp2518-bringup-postmortem.md) | CH4 MCP2518 SPI-CAN bringup bugs and fixes |
| [docs/free_rtos-bringup.md](docs/free_rtos-bringup.md) | FreeRTOS migration, regressions, verification |

Extended Damiao bench notes (local, gitignored): `docs/damiao-bringup.md`.

## Hardware

- **MCU:** STM32G474RE (512 KiB flash, 128 KiB RAM)
- **Plant I/O:** FDCAN1/2/3 @ 1 Mbit/s, SPI1/SPI3, UART4/UART5
- **Host link:** USB FS CDC **or** UART4 (PC10 TX / PC11 RX @ 115200 8N1)
- **Plant tick:** TIM6 @ 500 Hz (2 ms); heartbeat LED PC3 @ ~2 Hz
- **CAN activity LEDs:** PC7 = CH1, PC6 = CH2, PB15 = CH3

### Actuator map (factory defaults — `plant_config_nvm.c`)

| Slot | Schematic bus | Pins | Motor ID | Protocol |
|------|---------------|------|----------|----------|
| 0 | CH1 | PB8 / PB9 | `0x06` | Damiao DM-J4310 |
| 1 | CH2 | PA8 / PA15 | `0x70` | RobStride RS02 |
| 2 | CH3 | PB12 / PB13 | `0x75` | RobStride RS02 |
| 3 | CH4 (MCP) | SPI | `0x73` | RobStride |
| 4 | CH5 (MCP) | SPI | `0x73` | RobStride |
| 5 | CH6 (MCP) | SPI | `0x75` | RobStride |

Runtime layout is configurable over USB (`config set --bus` / `--channel`, `--persist` for flash NVM). See [docs/bringup.md](docs/bringup.md).

**CAN framing:** Schematic CH2 uses **extended** 29-bit CAN (RobStride). CH1 and CH3 support **mixed** standard (Damiao) + extended (RobStride) on one branch when slots are assigned accordingly. Schematic CH2/CH3 map to Cube **FDCAN3 / FDCAN2** (`can_router.c` `bus_handle` swap). Host scripts use schematic bus numbers `1`–`6`.

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

Preferred entrypoint:

```powershell
pip install -r scripts/requirements.txt
python scripts/control_hub.py --list-ports
python scripts/control_hub.py --port COM5 recover --bus 2
python scripts/control_hub.py --port COM5 teleop --slot 1
python scripts/control_hub.py --port COM5 --plant-teleop --plant-slots 0,1,2,3
```

RobStride discover / calibrate:

```powershell
python scripts/control_hub.py discover --port COM5 --protocol robstride --bus 1
python scripts/control_hub.py calibrate --port COM5 --bus 2 --id 0x70
```

Damiao discover on CH1 (register scan — motor can stay disabled):

```powershell
python scripts/control_hub.py discover --port COM5 --protocol damiao --bus 1   # first hit only
python scripts/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 1 --end 16
```

Legacy aliases: `scripts/host_teleop_laptop_usb.py`, `scripts/rs02_can_scan.py`. Expert flags: `python scripts/control_hub.py expert damiao -- --help`.

See [docs/bringup.md](docs/bringup.md) and [docs/known-issues.md](docs/known-issues.md) for current bench status, mixed-bus setup, and termination notes.

Jetson / UART path: see [docs/bringup.md](docs/bringup.md) (`scripts/host_teleop.py`).

## Repository layout

```
App/           Application: host link, actuators, plugins, control_loop, plant_diag
Core/          Cube HAL init (main, FDCAN, UART, TIM6, …)
USB_Device/    Cube USB CDC device stack + transport hooks
scripts/       control_hub.py, host_teleop_laptop_usb.py, damiao_scan.py, …
docs/          Architecture and contracts
External_Documentation/   Vendor PDFs (not linked into build)
```

## Git

Firmware builds from tracked Cube + App sources. `Debug/` and `Release/` are ignored (see `.gitignore`). `docs/damiao-bringup.md` is gitignored (local bench handoff).
