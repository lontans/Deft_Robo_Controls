# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: fixed-rate plant loop (500 Hz), cyclic host command/feedback exchange (562-byte binary images), and plugin-based motor protocols over FDCAN (RobStride extended CAN) and Damiao standard CAN.

**Bench status (Jul 2026):**

| Area | Status |
|------|--------|
| **RobStride CH1** | Slots 0–1 validated on USB CDC — RS02 + RS01 daisy chain |
| **Damiao CH3** | **In progress** — USB + DM0 probe path OK; CAN **TX** OK (`tx>0`); **no motor RX** yet (`rx_raw=0` on all ESC_IDs). Suspected missing **120 Ω at motor end** of bus (PCB has one terminator; J4310 has no software termination). |
| **CH4–CH6 MCP** | SPI-CAN bench path documented separately |
| **Plant teleop** | RobStride slots; Damiao teleop blocked until CH3 discovery succeeds |

## Quick links

| Doc | Contents |
|-----|----------|
| [docs/architecture.md](docs/architecture.md) | Threads, buffers, dual host paths, module map |
| [docs/host-exchange-v1.md](docs/host-exchange-v1.md) | Wire layout v1 (562 B), RS2 + DM0 PDU backdoors |
| [docs/bringup.md](docs/bringup.md) | Flash, motor map, laptop USB teleop, Damiao CH3 |
| [docs/known-issues.md](docs/known-issues.md) | Open gaps — **Damiao CAN RX**, cal NOISE, etc. |
| [docs/ch4-mcp2518-bringup-postmortem.md](docs/ch4-mcp2518-bringup-postmortem.md) | CH4 MCP2518 SPI-CAN bringup bugs and fixes |

Extended Damiao bench notes (local, gitignored): `docs/damiao-bringup.md`.

## Hardware

- **MCU:** STM32G474RE (512 KiB flash, 128 KiB RAM)
- **Plant I/O:** FDCAN1/2/3 @ 1 Mbit/s, SPI1/SPI3, UART4/UART5
- **Host link:** USB FS CDC **or** UART4 (PC10 TX / PC11 RX @ 115200 8N1)
- **Plant tick:** TIM6 @ 500 Hz (2 ms); heartbeat LED PC3 @ ~2 Hz
- **CAN activity LEDs:** PC7 = CH1, PC6 = CH2, PB15 = CH3

### Actuator map (current `plant_config.c`)

| Slot | Schematic bus | Pins | Motor ID | Protocol |
|------|---------------|------|----------|----------|
| 0 | CH1 | PB8 / PB9 | `0x76` | RobStride RS02 |
| 1 | CH1 (daisy) | PB8 / PB9 | `0x74` | RobStride RS01 |
| 2 | **CH3** | PB12 / PB13 | `0x01` (placeholder) | **Damiao** DM-J4310 |
| 3 | CH4 (MCP) | SPI | `0x70` | RobStride |
| 4 | CH5 (MCP) | SPI | `0x70` | RobStride |
| 5 | CH6 (MCP) | SPI | `0x70` | RobStride |

**Note:** Schematic CH2/CH3 map to Cube **FDCAN3 / FDCAN2** (`can_router.c` `bus_handle` swap). Host scripts use schematic bus numbers `1`–`6` in `--bus` and `pdu.data[11]`. CH3 uses **standard** 11-bit CAN (Damiao); CH1/CH2 use **extended** CAN (RobStride).

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

RobStride per-bus calibrate / discover:

```powershell
python scripts/rs02_can_scan.py --port COM9 --bench-cmds --bus 1 --target 0x76
```

Damiao CH3 discover (reg scan over DM0 PDU — motor can stay disabled):

```powershell
python scripts/damiao_scan.py --port COM5 --link-test
python scripts/damiao_scan.py --port COM5 --discover --bus 3 --start 1 --end 16
python scripts/damiao_scan.py --port COM5 --probe-id 6 --bus 3
```

See [docs/bringup.md](docs/bringup.md) and [docs/known-issues.md](docs/known-issues.md) for current Damiao debug status and termination notes.

Jetson / UART path: see [docs/bringup.md](docs/bringup.md) (`scripts/host_teleop.py`).

## Repository layout

```
App/           Application: host link, actuators, plugins, control_loop, plant_diag
Core/          Cube HAL init (main, FDCAN, UART, TIM6, …)
USB_Device/    Cube USB CDC device stack + transport hooks
scripts/       host_teleop_laptop_usb.py, rs02_can_scan.py, damiao_scan.py, …
docs/          Architecture and contracts
External_Documentation/   Vendor PDFs (not linked into build)
```

## Git

Firmware builds from tracked Cube + App sources. `Debug/` and `Release/` are ignored (see `.gitignore`). `docs/damiao-bringup.md` is gitignored (local bench handoff).
