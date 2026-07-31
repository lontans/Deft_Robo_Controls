# Bring-up

How to flash and talk to a board. Contract: [host-contract.md](host-contract.md). Buses: [plant.md](plant.md). Scripts map: [`../scripts/README.md`](../scripts/README.md).

## Transport

Edit `App/Inc/host/host_transport.h` before build:

| Board | `HOST_TRANSPORT_UART` | Link |
|-------|----------------------|------|
| Laptop / Jetson over USB | `0` | USB FS CDC → `COM*` or `/dev/ttyACM*` |
| Jetson UART host (no PDB on UART4) | `1` | UART4 PC10/11 @ 115200 |

**Conflict:** Jetson host UART and PDB UART both want UART4 — boards with a populated PDB connector must use USB host (`HOST_TRANSPORT_UART=0`) + `UART4_MODE_PDB` (see [plant.md](plant.md)#pdb-kill). That matches the usual “build on PC, flash from Jetson over USB” workflow.

Flash (preferred):

```bash
cd scripts
python soft_dfu_flash.py          # Windows or Linux
# Ubuntu/Jetson helper (sudo + dfu-util):
#   ./soft_dfu_flash.sh
```

PC3 ≈ 2 Hz heartbeat when plant is alive.

## Plant map (dual YAM Damiao)

| Arm | Slots | Joints | Bus |
|-----|-------|--------|-----|
| Arm1 | 0–6 | J1–J7 | CH1 (FDCAN1) |
| Arm2 | 7–13 | J8–J14 | CH2 (FDCAN3) |

Nominal Damiao ESC `0x01`…`0x07`, Master `0x11`…`0x17` — confirm with discover. Soft limits: SDK `yam_limits.py`. Schematic: CH2 → `hfdcan3`, CH3 → `hfdcan2`. Product CFG also maps base RobStride on CH4–6 (slots 14–19); bench spare IDs often differ from product `0x01`/`0x02` — do not silently remap in adapters.

## Quick start (living)

```bash
cd scripts
pip install -r requirements.txt   # pyserial, numpy, libusb1, …

python soft_dfu_flash.py
python -m pcb_lab status          # or --port COM5 / --port /dev/ttyACM0
python -m pcb_lab.debug show --pcb
python -m deft_controls_sdk.debug_dashboard
pytest pcb_lab/tests
```

Linux once-off packages + udev: see [`../scripts/README.md`](../scripts/README.md#ubuntu--jetson-once).

## Deprecated CLIs (`scripts/pcb_lab/legacy/` — gitignored)

Old bringup / continuous / PDB / vbeta smoke scripts may still exist **locally** under `scripts/pcb_lab/legacy/` (not tracked). Prefer HostProxy + `python -m pcb_lab` / vbeta. Restore from `archive/pre-plant-platform` if needed.

## Hygiene

Already gitignored: `scripts/pcb_lab/legacy/`, `scripts/_tmp_*`, `.deft_session/`, `__pycache__/`, `.pytest_cache/`, `*.egg-info/`. Wipe local dumps anytime:

```bash
rm -rf scripts/.deft_session scripts/__pycache__ scripts/.pytest_cache
```

Vendor PDFs: local under `External_Documentation/` (gitignored); notes in [vendor.md](vendor.md).
