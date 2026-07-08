# Structure thinning log

Mechanical cleanup only — **no changes** to debugged timing, probe blocking order, gate policy, or host-staleness thresholds.

## Firmware (`App/`)

| Removed / consolidated | Notes |
|------------------------|--------|
| `App/Inc/plant/plant_runtime.h` | Shim deleted; `plant_runtime_*` lives in `plant/diag/diag.h` |
| `#include "plant/plant_runtime.h"` in `actuator.c`, `host_link.c` | Single include: `plant/plant_diag.h` |
| `DIAG_ACTUATOR_STALE_MS` in `diag_internal.h` | Unused macro (actuator stale stays in `diag_gates.c` only) |

**Kept as-is (debugged behavior):** `plant_diag_service()` order in `app.c`, `diag_flush_usb()`, DM sync probes, quiet periods, `plant_block` feedback, MCP PDU packing.

**Layout (prior work, unchanged here):** bench code under `App/Src/plant/diag/` + `App/Inc/plant/diag/`; `plant_diag.h` remains a one-line shim.

## Python scripts

| Change | Lines (approx.) |
|--------|-----------------|
| `host_teleop_laptop_usb.py` — `--plant-teleop` / `--damiao-teleop` delegate to `control_hub.teleop.plant` | **−403** (2560 → 2157) |
| Removed unused `DM_DEFAULT_*`, `PLANT_TELEOP_BUS_KEYS` from host teleop | Defaults live in `control_hub/teleop/defaults.py` |
| Removed duplicate plant teleop: `run_plant_teleop`, homing/shutdown/wake helpers | Canonical copy in `control_hub/teleop/plant.py` |
| **Kept** in host teleop: `run_plant_launch_seq`, `PlantSlotTeleop`, `plant_send_slots` (launch demo only) | |
| `damiao_scan.py` — thin CLI (109 lines) | **−1236** (1345 → 109) |
| New `controls_pcb_host/plugins/damiao_expert.py` — discover / fw-sweep / probe / link-test / ack-debug | +862 (moved from scan, uses shared `commands`, `feedback`, `transport`) |

**Daily entry points (unchanged):**

```bash
python scripts/control_hub.py teleop --slot 1
python scripts/control_hub.py discover --protocol damiao --bus 3
python scripts/damiao_scan.py --port COM5 --fw-sweep --bus 3   # expert only
python scripts/host_teleop_laptop_usb.py --port COM5 --plant-teleop  # now calls control_hub
```

## Flash size (reference)

Compared to commit `7470755`, working tree is ~**+750 B text** (~0.77%) — split `diag/` + `plant_block` + RS02 cal fix; not a flash win, a structure win.

## Not done (intentionally)

- Async / deferred bench probes
- Collapsing `diag_state` globals
- Unifying 500 ms vs 8 s staleness
- Trimming `rs02_can_scan.py` or RS2 PDU teleop in host teleop
- Merging `damiao_expert` into `plugins/damiao.py` (could be a follow-up)
