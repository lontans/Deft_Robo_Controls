#!/usr/bin/env bash
# One-shot USB soft-DFU flash (Jetson / Linux / macOS).
#
#   ./scripts/soft_dfu_flash.sh
#   ./scripts/soft_dfu_flash.sh --image Debug/DeftRoboticsControlsPCB.elf
#   ./scripts/soft_dfu_flash.sh --serial 3167376F3435
#
# Needs: python3, pyserial, libusb1, dfu-util, objcopy (binutils).
# Does NOT use dfu-util :leave — Leave goes through the reset trampoline.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS="${ROOT}/scripts"
cd "${SCRIPTS}"

if [[ -x "${SCRIPTS}/.venv/bin/python" ]]; then
  PYTHON="${SCRIPTS}/.venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON=python3
else
  PYTHON=python
fi

exec "${PYTHON}" "${SCRIPTS}/soft_dfu_flash.py" "$@"
