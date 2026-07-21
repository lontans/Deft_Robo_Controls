# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: 500 Hz plant loop, cyclic USB host exchange (562 B v1), plugins over FDCAN (RobStride / Damiao, mixed std+ext) and MCP2518 SPI-CAN on CH4–6.

**Bench status (Jul 2026):** Dual YAM Damiao arms (14 slots: arm1 CH1, arm2 CH2) teleop validated. RobStride FDCAN + MCP CH4–6 previously validated. CubeMars = workstream draft only (not in live build).

## Docs (start here)

| Doc | Contents |
|-----|----------|
| [docs/bringup.md](docs/bringup.md) | **How to run** + bench stories (Damiao, dual-arm, plant cadence, MCP2562/CH4) |
| [docs/ch4-mcp2518-bringup-postmortem.md](docs/ch4-mcp2518-bringup-postmortem.md) | Full CH4 MCP2518FD + MCP2562 debug timeline |
| [docs/lessons.md](docs/lessons.md) | Open bugs + durable one-liner lessons |
| [docs/architecture.md](docs/architecture.md) | Runtime, host API modes, wire current vs target |
| [docs/decisions.md](docs/decisions.md) | ADR — target 672 B host image + PDB UART 64 B |
| [docs/host-exchange-v1.md](docs/host-exchange-v1.md) | Current 562 B layout |
| [docs/fdcan-dual-id-mixed-bus.md](docs/fdcan-dual-id-mixed-bus.md) | Mixed std/ext FDCAN detail |

## Host software

```powershell
cd scripts
pip install -r requirements.txt
python -m deft_controls_sdk.debug_dashboard --port COM5
# or: from deft_controls_sdk import ControlsPcbHub
```

Frozen legacy CLIs: [scripts/legacy/](scripts/legacy/README.md).

## Hardware (short)

- **MCU:** STM32G474RE  
- **Plant:** FDCAN1/2/3 @ 1 Mbps, SPI-CAN CH4–6, UART4/5, TIM6 @ 500 Hz  
- **Host:** USB CDC (laptop) or UART4 (Jetson) — see bringup  
- **LEDs:** PC3 heartbeat; CH1–3 activity PC7/PC6/PB15  

## Repository layout

```
App/           Application: host link, actuators, plugins, control_loop
Core/          Cube HAL
USB_Device/    USB CDC
scripts/       deft_controls_sdk/ (preferred), legacy/, requirements.txt
docs/          bringup, lessons, architecture, decisions, wire v1
External_Documentation/   Vendor PDFs
2026-07-10 workstreams/   Unmerged CubeMars + thermo draft
```

Firmware builds from tracked Cube + App sources. `Debug/` / `Release/` gitignored.
