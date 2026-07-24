#!/bin/bash
# Diagnose why AGX Orin BOARD pin18 (GPIO35 / PH.00) stays 0V when "driven HIGH".
set -u
PASS="${JETSON_PASS:-4565}"
sudok() { echo "$PASS" | sudo -S -p '' "$@"; }

echo "=== sudo ==="
if echo "$PASS" | sudo -S -p '' true 2>/dev/null; then echo sudo_ok; else echo sudo_fail; fi

echo
echo "=== pin18 mapping (from Jetson.GPIO) ==="
python3 - <<'PY'
import Jetson.GPIO as GPIO
info = GPIO.gpio_pin_data.get_data()[2]["BOARD"][18]
print("BOARD18", {a: getattr(info, a) for a in ("gpio_chip","gpio_name","line_offset","pwm_chip_dir","pwm_id")})
info16 = GPIO.gpio_pin_data.get_data()[2]["BOARD"][16]
print("BOARD16", {a: getattr(info16, a) for a in ("gpio_chip","gpio_name","line_offset","pwm_chip_dir","pwm_id")})
info7 = GPIO.gpio_pin_data.get_data()[2]["BOARD"][7]
print("BOARD7 ", {a: getattr(info7, a) for a in ("gpio_chip","gpio_name","line_offset","pwm_chip_dir","pwm_id")})
print("CVM GPIO35 == BOARD18 PH.00")
PY

echo
echo "=== PWM sysfs (pin18 has pwmchip2) ==="
ls -la /sys/class/pwm/ || true
for chip in /sys/class/pwm/pwmchip*; do
  echo "-- $chip"
  ls "$chip" 2>/dev/null || true
  for p in "$chip"/pwm*; do
    [ -d "$p" ] || continue
    echo " exported $p"
    for f in enable period duty_cycle polarity; do
      [ -f "$p/$f" ] && echo "  $f=$(cat "$p/$f")"
    done
  done
done

echo
echo "=== gpioinfo before ==="
gpioinfo | grep -E 'PH\.00|PQ\.06|PBB\.01' || true

echo
echo "=== pinctrl debug (needs root) ==="
sudok ls /sys/kernel/debug/pinctrl/ 2>&1 | head -20
sudok bash -c 'grep -n "PH.00\|PQ.06\|PBB.01\|soc_gpio35" /sys/kernel/debug/pinctrl/*/pinmux-pins 2>/dev/null | head -50' || true
sudok bash -c 'grep -n "PH.00\|PQ.06\|PBB.01" /sys/kernel/debug/pinctrl/*/pins 2>/dev/null | head -50' || true

echo
echo "=== Hold PH.00 (pin18) HIGH 10s via gpioset — meter pin18 now ==="
pkill -f 'gpioset|Jetson.GPIO|jetson_estop' 2>/dev/null || true
sleep 0.3
# gpiochip0 line 43 = PH.00
gpioset -m time -s 10 gpiochip0 43=1 &
GP=$!
sleep 1
echo "during hold:"
gpioinfo | grep 'PH.00' || true
# also poll MCU
python3 - <<'PY'
import glob, sys, time
sys.path.insert(0, "/home/deft-robotics/controls_pcb/scripts")
from deft_controls_sdk import ControlsPcbHub
from deft_controls_sdk.link.exchange.wire_layout import SYSTEM_FB_OFF
acms = sorted(glob.glob("/dev/ttyACM*"))
if not acms:
    print("no CDC")
else:
    with ControlsPcbHub.connect(acms[0], persist_telemetry=False) as hub:
        hub.recover()
        for i in range(5):
            hub._connection.send_once()
            fb = hub._connection.poll_feedback()
            es = None if fb is None else fb.raw[SYSTEM_FB_OFF + 16]
            print(f"mcu estop_sense={es}")
            time.sleep(0.5)
PY
wait $GP 2>/dev/null || true
echo "pin18 hold done"

echo
echo "=== Control: Hold PQ.06 (BOARD pin7, no PWM) HIGH 8s — meter pin7 if curious ==="
gpioset -m time -s 8 gpiochip0 106=1 &
GP=$!
sleep 1
gpioinfo | grep 'PQ.06' || true
wait $GP 2>/dev/null || true

echo
echo "=== DT snippets mentioning ph / gpio35 ==="
python3 - <<'PY'
import os
root = "/proc/device-tree"
for dirpath, _, files in os.walk(root):
    for f in files:
        p = os.path.join(dirpath, f)
        try:
            raw = open(p, "rb").read()
        except Exception:
            continue
        low = raw.lower()
        if b"ph.00" in low or b"soc_gpio35" in low or b"gpio35" in low:
            print(p, "=>", raw[:100].replace(b"\0", b" "))
PY

echo DONE
