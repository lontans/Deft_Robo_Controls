# Soft-DFU (USB-only firmware flash)

Flash the Controls PCB over the device USB cable — no ST-Link required for a
successful cycle. ST-Link SWD is **recovery only**.

Entrypoint:

```bash
python scripts/soft_dfu_flash.py
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
python scripts/soft_dfu_flash.py --serial 3167375E3435 --require-usb-dfu
python scripts/soft_dfu_flash.py scan
```

Linux / Jetson: same Python entry, or `./scripts/soft_dfu_flash.sh` (sudo +
`dfu-util` + udev).

## Success criteria

A cycle **passes** only when the script prints:

```text
flash ok — CDC at …
```

**without** `(SWD)`. Use `--require-usb-dfu` in prove loops so any SWD fallback
is a hard failure.

## How it works

1. Host sends plant DEBUG PDU tag `DFU!` over app CDC (`0483:5740`).
2. Firmware drops USB D+, programs option byte **nBOOT0=0** (nSWBOOT0 stays 0),
   and resets into the STM32 ROM bootloader → host sees **`0483:DF11`**.
3. Host programs the ELF/BIN (Windows: STM32CubeProgrammer USB DFU; Linux:
   `dfu-util`).
4. Host AN3156 Leave jumps to the leave trampoline at `0x0803F800`, which
   restores **nBOOT0=1** and resets → app CDC re-enumerates.

Soft MEMRMP jumps into system memory are **not** reliable on this board (CDC
drops, DF11 never appears). Option-byte boot is the supported path.

## Windows setup

### App CDC (`0483:5740`) → COM port (`usbser`)

Must show up as a **Ports** device (e.g. `USB Serial Device (COMx)`), not
WinUSB. If Zadig was pointed at the wrong PID, Device Manager shows
`STM32 Virtual ComPort` under USB devices with service `WinUSB` and
`soft_dfu_flash.py scan` lists **no** `[CDC]` row even though PnP sees
`VID_0483&PID_5740`.

Fix (elevated PowerShell) — remove the libwdi/Zadig package for 5740 and let
Windows bind `usbser`:

```powershell
# Find the oemXX.inf that is libwdi / WinUSB for 0483:5740, then:
pnputil /delete-driver oemXX.inf /uninstall /force
pnputil /remove-device "USB\VID_0483&PID_5740\<serial>"
pnputil /scan-devices
```

Do **not** install WinUSB on `0483:5740`. That PID needs a COM port for soft-enter.

### DFU (`0483:DF11`) → WinUSB / ST DFU

CubeProgrammer and libusb need a WinUSB (or ST DFU) claim on **DF11 only**.
If DF11 appears in Device Manager but CubeProg `-l usb` is empty, use Zadig on
`STM32 BOOTLOADER` / `0483:DF11` — never on the CDC PID.

### Inventory

```bash
python scripts/soft_dfu_flash.py scan
```

Expect something like:

```text
[CDC] COM4  sn=3167375E3435  vid=1155 pid=22336  …
DFU 0483:DF11: no          # idle app
ST-Link SWD: yes           # optional recovery probe
```

## Recovery with ST-Link (not a success)

If a soft-enter leaves the board without CDC and without DF11, or Leave fails:

```bash
# Default flash allows SWD fallback when CubeProg can open SWD:
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
```

Or force SWD via CubeProg CLI (`port=SWD`). After recovery, re-run with
`--require-usb-dfu` to confirm USB-only again.

If the board is stuck in ROM DFU with `nBOOT0=0` and Leave cannot run, restore
flash boot over SWD:

```text
STM32_Programmer_CLI -c port=SWD mode=UR -ob nBOOT0=1 -rst
```

## Jetson / Linux

1. Install udev rules (once):

   ```bash
   sudo cp scripts/udev/99-stm32-dfu.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger --subsystem-match=usb --action=add
   ```

2. Prefer `./scripts/soft_dfu_flash.sh` or `sudo -E python scripts/soft_dfu_flash.py …`.
3. **Always pin serial** when more than one Controls PCB is on the Jetson:

   ```bash
   python scripts/soft_dfu_flash.py --serial <new-board-serial> --require-usb-dfu \
     --image Debug/DeftRoboticsControlsPCB.elf
   ```

4. No ST-Link on Jetson — if DFU sticks, power-cycle; do not Soft-DFU the live
   product board used by teleop.

## Dual-board serials (Jetson)

When both Controls PCBs are on the Jetson, **always** pin Soft-DFU with `--serial`:

| Role | USB serial | Typical ACM |
|------|------------|-------------|
| Soft-DFU / Cursor board | `3167375E3435` | `/dev/ttyACM1` (may move) |
| Live teleop (Claude 2) | `3167376F3435` | `/dev/ttyACM0` (may move) |

Never Soft-DFU the live teleop serial.

## Bench prove

### Windows (2026-07-24)

| Item | Value |
|------|--------|
| Board CDC/DFU serial | `3167375E3435` |
| Host CDC | COM4 (`usbser`) |
| Flasher | STM32CubeProgrammer USB DFU |
| Result | **5/5** USB-only, alternating Debug/Release, `--require-usb-dfu` |
| Wall time | ~6–9 s/cycle |
| SWD used | no (seed flash only before the prove) |

### Jetson `192.168.50.48` (2026-07-24)

| Item | Value |
|------|--------|
| Target serial | `3167375E3435` only (`dfu-util -S …`) |
| Sibling left alone | `3167376F3435` still on CDC after prove |
| Flasher | `dfu-util` via `sudo -E` (udev rules present) |
| Result | **5/5** USB-only, alternating Debug/Release, `--require-usb-dfu` |
| Wall time | ~15–26 s/cycle |
| ST-Link | none |

### Jetson caveat (2026-07-24 evening)

When only **`3167376F3435`** is plugged (the former sibling), soft-enter can drop
CDC without `0483:DF11` enumerating on the Jetson USB tree. Treat that as
**USB Soft-DFU blocked on this host+serial combo** until re-proven: recover with
ST-Link SWD (`nBOOT0=1` + flash ELF), then re-run
`--require-usb-dfu --serial <scan>` once DF11 is visible again. Prefer the
previously proven Soft-DFU serial (`3167375E3435`) when both boards are available.
Always `scan` first and pin `--serial`.

## Related

- Firmware: `App/Src/host/soft_dfu.c`, `App/Inc/host/soft_dfu.h`
- Host: `scripts/soft_dfu_flash.py` → `scripts/deft_controls_sdk/bench/soft_dfu.py`
- API summary: `docs/api.md` (Soft-DFU section)
