# Controls map (self-handoff)

How to drive the Controls PCB without mixing lab and product paths.

## Stack

```text
Firmware (App/plant + App/host)  — 500 Hz plant, CFG/NVM, Soft-DFU
        ↑ USB CDC 694 B
deft_controls_sdk                — Hub + HostProxy + config/actions/debug
        ↑
pcb_lab CLI                      — scan / status / flash / debug suite (lab only)
deft_vbeta (parent repo)         — product teleop via HostProxy.set_section
```

Pin the PCB submodule to **GitHub `main`** (cleaned tip). Do not `git pull` GitLab `main` into a cleaned checkout — histories diverged; trees may match while SHAs differ.

## Two APIs (do not conflate)

| Path | Who | How |
|------|-----|-----|
| **Lab / notebooks** | You at the desk | `HostProxy` + `proxy.actions` (`mount` → `apply` → `clear`) or `python -m pcb_lab.debug` |
| **Product** | Parent `deft_vbeta` | Author `ActuatorDesire` → `HostProxy.set_section(...)` — **not** `proxy.actions` |

One process owns the CDC port. Dashboard, `pcb_lab.debug`, and vbeta must not Connect at the same time.

## Modes

| `HostProxy` / hub mode | Plant motion | CFG / discover / inventory |
|------------------------|--------------|----------------------------|
| `"debug"` | yes (when armed) | **yes** |
| `"bandwidth"` | yes (when armed) | **no** |

Reconnect to change mode. `arm_plant()` / `disarm_plant()` (dashboard: Enable control / Observe) gate apply.

Dashboard UI: Connect always starts **observe** (`armed=False`). Enable control before teleop.

## Everyday commands

```bash
cd scripts
pip install -r requirements.txt   # pyserial, numpy, libusb1, …

# Board
python -m pcb_lab scan
python -m pcb_lab status          # or --port COM5 / /dev/ttyACM0
python -m pcb_lab flash           # Soft-DFU; add --require-usb-dfu for USB-only
python -m pcb_lab --port COM4 flash --prove 3                 # N-cycle prove (SWD ok)
python -m pcb_lab --port COM4 flash --prove 3 --require-usb-dfu  # USB-only Soft-DFU
# (--port is top-level; USB-only needs WinUSB on 0483:DF11)

# Lab prove CLI
python -m pcb_lab.debug show --pcb
python -m pcb_lab.debug test              # board → peripherals
python -m pcb_lab.debug test --actuators

# Dashboard (sliders + keyboard teleop when control enabled)
python -m deft_controls_sdk.debug_dashboard
# http://127.0.0.1:8766
```

Firmware compile remains **STM32CubeIDE**; commit the ELF for Jetson pull → flash.

## Jetson (USB host + PDB on UART4)

1. Flash an ELF built with `HOST_TRANSPORT_UART=0` (USB CDC host).
2. Once: `dfu-util`, `libusb`, udev [`scripts/udev/99-stm32-dfu.rules`](../scripts/udev/99-stm32-dfu.rules), user in `dialout`.
3. Clone parent `deft_vbeta` with this repo as submodule @ GitHub SHA; venv + `pip install -r controls_pcb/scripts/requirements.txt`.
4. Product path: vbeta session → `set_section` (see parent repo). Lab path: same `pcb_lab` commands with `/dev/ttyACM0`.
5. Talk-through scripts: [`scripts/demo_scripts/`](../scripts/demo_scripts/) (`01`…`04` — CFG + product idle + bench base).

## Soft-DFU

Preferred flash: USB soft-DFU (no ST-Link). Use `--require-usb-dfu` when proving USB-only. ST-Link SWD is fallback/recovery only.

## Keyboard teleop (dashboard)

With Connect + **Enable control**, focus not in an input:

| Keys | Action |
|------|--------|
| `Space` | Stop all teleop (hold pose; not idle/slack) |
| `←` `→` | Neck yaw nudge |
| `↑` `↓` | Neck pitch nudge |
| `1`–`7` | Select left-arm joint (verified slots only) |
| `[` `]` | Nudge selected arm joint toward lo/hi |
| `z` `x` | Jog first bench base slot − / + |
| `c` `v` | Jog second bench base slot − / + |

## Where to read next

- [integration.md](integration.md) — SDK / vbeta / pcb_lab ownership  
- [host-contract.md](host-contract.md) — wire + Soft-DFU  
- [bringup.md](bringup.md) — flash + transport  
- [architecture.md](architecture.md) — plant hot path  
