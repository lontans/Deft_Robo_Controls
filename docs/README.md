# Docs

Living documentation for the Controls PCB (STM32G474 plant platform). Keep this tree thin — detail that aged out lives on `archive/pre-plant-platform`.

## Start here

| Doc | Contents |
|-----|----------|
| [architecture.md](architecture.md) | Runtime, modes, hot path, HostProxy north star |
| [host-contract.md](host-contract.md) | 694 B plant/DEBUG wire, Soft-DFU, SDK surface |
| [bringup.md](bringup.md) | Flash, plant map, how to run living tools |
| [plant.md](plant.md) | Buses, protocols, PDB kill |
| [integration.md](integration.md) | SDK / parent deft_vbeta / pcb_lab |
| [controls-map.md](controls-map.md) | Lab vs product paths, commands, dashboard keys, Jetson |
| [thermo-spi3.md](thermo-spi3.md) | SPI3 LED vs MAX31855 exclusive role + NVM flags |
| [decisions.md](decisions.md) | ADRs |
| [vendor.md](vendor.md) | Vendor PDF/EDS cheat-sheet (PCB-relevant only) |

Repo entry: [`../README.md`](../README.md). Host package + CLIs: [`../scripts/README.md`](../scripts/README.md).

## What belongs where

- **These docs** — contracts and mental models you need to own the board.
- **`scripts/deft_controls_sdk/`** — Hub + HostProxy + config/actions/debug + dashboard.
- **`scripts/pcb_lab/`** — lab app + tests; optional local `legacy/` (gitignored).
- **Parent `deft_vbeta`** — product drivers (`set_section`); not nested in this repo.
- **`External_Documentation/`** — local vendor PDFs (gitignored); summarized in [vendor.md](vendor.md).
