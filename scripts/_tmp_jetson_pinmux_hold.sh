#!/bin/bash
PASS="${JETSON_PASS:-4565}"
sudok() { echo "$PASS" | sudo -S -p '' "$@"; }

echo "=== Hold gpiochip0 line43 PH.00 / BOARD18 / CVM GPIO35 for 15s ==="
echo "Meter header pin 18 now."
gpioset -m time -s 15 gpiochip0 43=1 &
GP=$!
sleep 1
echo "gpioinfo:"; gpioinfo | grep 'PH.00'
echo "pinmux-pins:"; sudok grep -n 'SOC_GPIO21_PH0' /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins
echo "pinconf:"; sudok grep -n -A3 'SOC_GPIO21_PH0' /sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-pins | head -20
echo "debug-gpio:"; sudok grep -n 'PH.00' /sys/kernel/debug/gpio
# MCU sense
python3 - <<'PY'
import glob, sys, time
sys.path.insert(0, "/home/deft-robotics/controls_pcb/scripts")
from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.link.exchange.wire_layout import SYSTEM_FB_OFF
cdc = sorted(glob.glob("/dev/ttyACM*"))
if cdc:
    with ControlsPcbHub.connect(cdc[0], persist_telemetry=False) as hub:
        hub.recover()
        for _ in range(3):
            hub._connection.send_once()
            fb = hub._connection.poll_feedback()
            print("estop_sense", None if fb is None else fb.raw[SYSTEM_FB_OFF + 16])
            time.sleep(0.4)
PY
wait "$GP"
echo "hold done"

echo
echo "=== Control hold PQ.06 BOARD7 8s + pinmux ==="
gpioset -m time -s 8 gpiochip0 106=1 &
GP=$!
sleep 1
gpioinfo | grep 'PQ.06'
sudok grep -n 'SOC_GPIO33' /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins || sudok grep -ni 'pq.06\|gpio33' /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins | head
wait "$GP"

echo
echo "=== pinmux function list for soc_gpio21 ==="
sudok grep -n 'soc_gpio21\|SOC_GPIO21' /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-functions | head -20

echo DONE
