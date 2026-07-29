# Docs

Living documentation for the Controls PCB (STM32G474 plant platform). Keep this tree thin — detail that aged out lives on `archive/pre-plant-platform`.

## Start here

| Doc | Contents |
|-----|----------|
| [architecture.md](architecture.md) | Runtime, modes, hot path, PlantProxy north star |
| [host-contract.md](host-contract.md) | 694 B plant/DEBUG wire, Soft-DFU, SDK surface |
| [bringup.md](bringup.md) | Flash, plant map, how to run living tools |
| [plant.md](plant.md) | Buses, protocols, PDB kill |
| [integration.md](integration.md) | SDK / vbeta / i2rt stacks |
| [decisions.md](decisions.md) | ADRs |
| [vendor.md](vendor.md) | Vendor PDF/EDS cheat-sheet (PCB-relevant only) |

Repo entry: [`../README.md`](../README.md). Host package + CLIs: [`../scripts/README.md`](../scripts/README.md).

## What belongs where

- **These docs** — contracts and mental models you need to own the board.
- **`scripts/deft_controls_sdk/`** — preferred host API (living code).
- **`scripts/legacy/`** — working-but-deprecated CLIs and old packages; do not extend; replace with PlantProxy / `pcb_lab`.
- **`External_Documentation/`** — local vendor PDFs (gitignored); summarized in [vendor.md](vendor.md).
