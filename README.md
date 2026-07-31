# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: **~500 Hz** plant loop, **694 B** USB host exchange (layout v3), plugins over FDCAN (RobStride / Damiao / ZeroErr / CubeMars) and MCP2518 SPI-CAN on CH4–6, Dynamixel neck UART, SK9822 LEDs, PDB UART kill.

## Docs (start here)

| Doc | Contents |
|-----|----------|
| [docs/README.md](docs/README.md) | Doc map |
| [docs/architecture.md](docs/architecture.md) | Runtime, modes, plant hot path, platform north star |
| [docs/host-contract.md](docs/host-contract.md) | 694 B plant/DEBUG wire, Soft-DFU, SDK call surface |
| [docs/bringup.md](docs/bringup.md) | Flash, plant map, how to run |
| [docs/plant.md](docs/plant.md) | Buses, protocols, PDB kill |
| [docs/integration.md](docs/integration.md) | SDK / vbeta / i2rt stacks, HostProxy / pcb_lab |
| [docs/decisions.md](docs/decisions.md) | ADRs |
| [docs/vendor.md](docs/vendor.md) | Vendor PDF/EDS cheat-sheet (PCB-relevant only) |

Host tools: [scripts/README.md](scripts/README.md) (SDK + `pcb_lab/`). Local vendor files: [External_Documentation/README.md](External_Documentation/README.md).

## Host software

Windows and Ubuntu/Jetson (USB CDC). Details: [scripts/README.md](scripts/README.md).

```bash
cd scripts
pip install -r requirements.txt

python soft_dfu_flash.py          # Linux: ./soft_dfu_flash.sh
python -m pcb_lab
python -m deft_controls_sdk.debug_dashboard
```

Canonical API: [docs/host-contract.md](docs/host-contract.md). Lab: [scripts/pcb_lab/](scripts/pcb_lab/).

## Hardware (short)

- **MCU:** STM32G474RE  
- **Plant:** FDCAN1/2/3 @ 1 Mbps, SPI-CAN CH4–6, UART4/5, TIM6 plant tick  
- **Host:** USB CDC (laptop / Jetson); UART4 Jetson host conflicts with PDB mode  
- **LEDs:** SK9822 + per-channel ACT  

## Repository layout

```
App/           host link, actuators, plugins, control_loop
Core/          Cube HAL
USB_Device/    USB CDC
scripts/       deft_controls_sdk/, pcb_lab/, soft_dfu_flash.py
docs/          living set (see docs/README.md)
External_Documentation/   local PDFs (ignored) + stub README
```

Build with STM32CubeIDE / `Debug/makefile`. Routine flash via Soft-DFU, not ST-Link.
