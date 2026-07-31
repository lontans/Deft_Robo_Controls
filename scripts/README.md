# scripts/

Thin living host surface for the Controls PCB (Windows + Ubuntu/Jetson).

## Living

| Path | Role |
|------|------|
| [`deft_controls_sdk/`](deft_controls_sdk/README.md) | Hub + HostProxy + vbeta + **debug** + dashboard |
| [`pcb_lab/`](pcb_lab/README.md) | Lab app + **tests** (+ optional local `legacy/`, gitignored) |
| [`soft_dfu_flash.py`](soft_dfu_flash.py) / [`.sh`](soft_dfu_flash.sh) | Soft-DFU entry |
| [`udev/`](udev/) | Linux udev rules |
| [`requirements.txt`](requirements.txt) / [`requirements-dev.txt`](requirements-dev.txt) | Deps (`pyserial`, `numpy`, `libusb1`, …) |

```bash
cd scripts
pip install -r requirements.txt

# Flash (Windows: CubeProg or dfu-util; Linux: dfu-util)
python soft_dfu_flash.py
# or on Ubuntu/Jetson:
#   ./soft_dfu_flash.sh

python -m pcb_lab
python -m pcb_lab scan
python -m pcb_lab.debug test              # omit --port to auto-discover CDC
pytest pcb_lab/tests
```

### Ubuntu / Jetson (once)

```bash
sudo apt install dfu-util libusb-1.0-0 binutils
sudo cp udev/99-stm32-dfu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER"   # re-login
```

Typical workflow: build ELF on a CubeIDE machine → commit/push → `git pull` on Jetson → `python -m pcb_lab flash` (or `./soft_dfu_flash.sh`). Port is usually `/dev/ttyACM0`.

## pcb_lab layout

```text
pcb_lab/
  lab.py          # board CLI + thin LabRobot
  board.py        # scan / status / flash / interactive menu
  debug/          # alias → sdk.debug.suite (show|set|test)
  tests/          # offline SDK + lab tests
  legacy/         # gitignored local CLIs (not in repo)
```

Docs: [`../docs/README.md`](../docs/README.md).
