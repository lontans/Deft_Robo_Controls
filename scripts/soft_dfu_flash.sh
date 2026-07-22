#!/usr/bin/env bash
# Linux/Jetson one-shot USB soft-DFU flash.
#
#   ./scripts/soft_dfu_flash.sh
#   ./scripts/soft_dfu_flash.sh --serial 3167376F3435
#
# Uses sudo so dfu-util can open 0483:df11 without custom udev rules.
# (Windows: use soft_dfu_flash.py — CubeProg, no sudo.)
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

# Root can open DFU; ttyACM enter also works as root.
exec sudo -E "${PYTHON}" "${SCRIPTS}/soft_dfu_flash.py" "$@"
