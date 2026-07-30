"""Servo / neck prove — ``mode=debug`` via suite ``pcb_tui`` helpers (no vbeta.rig)."""
from __future__ import annotations

import argparse
import json
import sys

from deft_controls_sdk.host_proxy import HostProxy

from .pcb_tui import sample_servo_fb, servo_connection_summary
from .show import collect_cfg


def run_servo_test(args: argparse.Namespace) -> int:
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz

    print("test --servo  mode=debug  (functional observe — no timing gates)")

    with HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        idle_first=True,
        listen_pdu=bool(getattr(args, "listen_pdu", False)),
        mode="debug",
    ) as proxy:
        try:
            cfg = collect_cfg(proxy)
        except Exception as exc:  # noqa: BLE001
            print(f"collect_cfg failed: {exc}", file=sys.stderr)
            return 1

        if not cfg.get("periph_ok", True):
            print(
                json.dumps(
                    {
                        "ok": False,
                        "error": cfg.get("periph_error") or "periph GET failed",
                        "note": "functional observe only",
                    },
                    indent=2,
                )
            )
            return 1

        servo_fb = sample_servo_fb(proxy, cfg)
        summary = servo_connection_summary(cfg, servo_fb)

        rows = []
        for s in cfg.get("servos") or []:
            slot = int(s.get("slot", 0))
            fb = servo_fb.get(slot)
            rows.append(
                {
                    "slot": slot,
                    "enabled": bool(s.get("enabled")),
                    "id": int(s.get("id", 0)) & 0xFF,
                    "model": int(s.get("model", 0)),
                    "connected": (
                        fb is not None
                        and int(fb.get("motor_source_id", 0) or 0)
                        == (int(s.get("id", 0)) & 0xFF)
                        if s.get("enabled")
                        else None
                    ),
                    "present_position": (
                        None if fb is None else fb.get("present_position")
                    ),
                    "motor_source_id": (
                        None if fb is None else fb.get("motor_source_id")
                    ),
                }
            )

        # Pass when CFG has no enabled servos (nothing to prove) OR at least one live.
        enabled = int(summary.get("enabled", 0))
        ok = enabled == 0 or bool(summary.get("connected"))
        out = {
            "ok": ok,
            "summary": summary,
            "servos": rows,
            "listen_pdu": bool(cfg.get("listen_pdu", False)),
            "note": (
                "no enabled NVM servo rows — CFG empty is not a fail"
                if enabled == 0
                else "functional observe only — timing floors not applied"
            ),
        }
        if not ok:
            out["error"] = "enabled servos present but none connected (FB id mismatch)"

        print(json.dumps(out, indent=2, default=str))
        return 0 if ok else 1
