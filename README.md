# Deft Robotics Controls PCB

STM32G474 firmware for the Deft controls board: ~500 Hz plant loop, **672 B**
USB host exchange (layout v2), plugins over FDCAN (RobStride / Damiao / ZeroErr)
and MCP2518 SPI-CAN on CH4–6, Dynamixel neck UART, SK9822 LEDs.

**Bench (Jul 2026):** Dual YAM Damiao arms + MCP CH4–6 + DXL/LED load matrix on
`main`. Equal-rate freshness path is the product trunk; MCP-decouple experiments
parked on `feat/mcp-decoupled-optimzation`.

## Docs (start here)

| Doc | Contents |
|-----|----------|
| [docs/bringup.md](docs/bringup.md) | How to run + bench stories |
| [docs/api.md](docs/api.md) | Host SDK call surface |
| [docs/host-exchange-v2.md](docs/host-exchange-v2.md) | **672 B** wire layout |
| [docs/vbeta-pcb-adapter.md](docs/vbeta-pcb-adapter.md) | deft_vbeta / YAM slot map + adapters |
| [docs/architecture.md](docs/architecture.md) | Runtime / tasks |
| [docs/decisions.md](docs/decisions.md) | ADR — 672 B + PDB UART |
| [docs/scripts-hygiene.md](docs/scripts-hygiene.md) | `_tmp_*` / legacy retirement |
| [docs/lessons.md](docs/lessons.md) | Open bugs + lessons |

## Host software

```powershell
cd scripts
pip install -r requirements.txt

# Flash (no ST-Link) — preferred one-liner
python soft_dfu_flash.py
# python soft_dfu_flash.py --image ../Debug/DeftRoboticsControlsPCB.elf

# Dashboard (owns COM)
python -m deft_controls_sdk.debug_dashboard

# Plant / vbeta adapters
python -c "from deft_controls_sdk import ControlsPcbHub; ..."
python vbeta_neck_led_smoke.py
```

Canonical API: [docs/api.md](docs/api.md). Frozen legacy tree: [scripts/legacy/](scripts/legacy/) (do not extend; see hygiene doc).

## Hardware (short)

- **MCU:** STM32G474RE  
- **Plant:** FDCAN1/2/3 @ 1 Mbps, SPI-CAN CH4–6, UART4/5, TIM6 plant tick  
- **Host:** USB CDC (laptop) or UART4 (Jetson)  
- **LEDs:** SK9822 chain + per-channel ACT LEDs  

## Repository layout

```
App/           Application: host link, actuators, plugins, control_loop
Core/          Cube HAL
USB_Device/    USB CDC
scripts/       deft_controls_sdk/, vbeta smokes, soft_dfu_flash.py, legacy/
docs/          bringup, api, architecture, decisions, wire v2
External_Documentation/   Vendor PDFs
```

Build with STM32CubeIDE / `Debug/makefile`. Flash via soft-DFU above (not ST-Link for routine work).
