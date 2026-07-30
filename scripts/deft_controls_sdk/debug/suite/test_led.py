"""LED prove — ``mode=debug``; plant LED via ``actions.LedAction`` + config presets."""
from __future__ import annotations

import argparse
import json
import sys
import time

from deft_controls_sdk.config import LED_PRESETS
from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link import LedDesire
from deft_controls_sdk.link.api_types import led_mode_name

from .presets import apply_led_preset


def run_led_test(args: argparse.Namespace) -> int:
    name = str(getattr(args, "preset", "idle") or "idle")
    preset = LED_PRESETS.get(name)
    if preset is None:
        print(
            f"unknown LED preset {name!r}; known: {', '.join(sorted(LED_PRESETS))}",
            file=sys.stderr,
        )
        return 2

    hold = max(0.0, float(getattr(args, "hold_s", 2.0)))
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz

    print(
        f"test --led  mode=debug  preset={preset.name}  "
        f"policy={preset.policy}  hold={hold:g}s  (actions.LedAction)"
    )

    with HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        idle_first=True,
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        mode="debug",
    ) as proxy:
        try:
            periph = dict(proxy.hub.debug.cfg_get_periph())
        except Exception as exc:  # noqa: BLE001
            print(f"cfg_get_periph failed: {exc}", file=sys.stderr)
            periph = {"led": {}, "flags": 0, "servos": []}

        # NVM block update + actions.LedAction plant desire
        apply_led_preset(proxy, preset, periph)
        if hold:
            time.sleep(hold)

        report = proxy.doctor()
        led = report.get("led") or {}
        desire = getattr(proxy.hub, "led_desire", None)
        if not isinstance(desire, LedDesire):
            desire = None

        out = {
            "ok": True,
            "preset": preset.name,
            "policy": preset.policy,
            "pattern": int(preset.pattern),
            "brightness": int(preset.brightness),
            "held_s": hold,
            "doctor_led": led,
            "host_desire": (
                {
                    "mode": desire.mode,
                    "pattern": int(desire.pattern),
                    "wire_mode": int(desire.wire_mode),
                    "wire_mode_name": led_mode_name(desire.wire_mode),
                    "master_brightness": int(desire.master_brightness),
                }
                if desire is not None
                else None
            ),
            "note": "functional observe only — timing floors not applied",
        }
        # Pass if host desire matches requested policy (doctor may refine effective).
        if desire is not None and desire.mode != preset.policy:
            out["ok"] = False
            out["error"] = (
                f"host LedDesire.mode={desire.mode!r} != preset policy={preset.policy!r}"
            )

        print(json.dumps(out, indent=2, default=str))
        return 0 if out["ok"] else 1
