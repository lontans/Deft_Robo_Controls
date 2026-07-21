"""COM5 probe: MCP ACT LED mark/toggle + try_send ok/busy from thermo PDU.

Requires firmware with plant_timing_thermo_fill LED debug (bytes 22..31).

  cd scripts
  python _tmp_mcp_led_debug_probe.py --port COM5

Interpretation (Δ over hold):
  try_ok~1, try_busy climbing, toggle~1..2  → TXQ stuck after first load
  try_ok climbing, toggle climbing          → TX+LED path OK (GPIO/visual?)
  try_ok climbing, toggle flat              → mark/TX OK but LED poll broken
  try_ok flat, try_busy flat                → plant not calling try_send
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import parse_feedback_header


LED_KEYS = (
    "mark_ch4",
    "mark_ch5",
    "mark_ch6",
    "toggle_ch4",
    "toggle_ch5",
    "toggle_ch6",
    "try_ok_ch4",
    "try_ok_ch5",
    "try_ok_ch6",
    "try_busy_ch4",
)


def _conn(hub: ControlsPcbHub):
    return hub._connection  # noqa: SLF001


def _drain_latest(hub: ControlsPcbHub):
    raw = None
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        raw = frame
    return raw


def _u8_delta(a: int, b: int) -> int:
    return (b - a) & 0xFF


def _sample_led(hub: ControlsPcbHub, *, settle_s: float = 0.15) -> Optional[dict]:
    time.sleep(settle_s)
    raw = _drain_latest(hub)
    if raw is None:
        return None
    hdr = parse_feedback_header(raw)
    if not hdr:
        return None
    led = hdr.get("led_debug")
    if not isinstance(led, dict):
        return None
    return led


def measure(
    hub: ControlsPcbHub,
    label: str,
    desires: Dict[int, ActuatorDesire],
    *,
    seconds: float = 3.0,
    hz: float = 40.0,
) -> None:
    blank = {slot: ActuatorDesire() for slot in range(ACTUATOR_COUNT)}
    blank.update(desires)
    _conn(hub).set_actuators(blank, send=False)

    before = _sample_led(hub)
    dt = 1.0 / hz
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    reader = _conn(hub).reader
    tf0 = reader.total_frames
    tags: Dict[str, int] = {}
    last_led: Optional[dict] = None

    while time.perf_counter() < t_end:
        raw = _drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr:
                tag = str(hdr.get("pdu_tag", "?"))
                tags[tag] = tags.get(tag, 0) + 1
                led = hdr.get("led_debug")
                if isinstance(led, dict):
                    last_led = led
        _conn(hub).send_once()
        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    after = last_led or _sample_led(hub)
    raw_fb = reader.total_frames - tf0
    fb_hz = raw_fb / seconds if seconds > 0 else 0.0

    print(f"\n=== {label} ===")
    print(f"  desires slots={list(desires.keys())}  fb_hz~{fb_hz:.1f}  pdu_tags={tags}")
    if before is None or after is None:
        print("  led_debug: MISSING (need 't' thermo PDU — SPI3_ROLE_THERMO?)")
        print(f"  before={before} after={after}")
        return

    print("  key            d   before -> after")
    for k in LED_KEYS:
        d = _u8_delta(int(before[k]), int(after[k]))
        print(f"  {k:14s} {d:3d}   {before[k]:3d} -> {after[k]:3d}")


def find_slots_by_bus(hub: ControlsPcbHub) -> Dict[int, List[int]]:
    """bus (1..6) -> slot indices."""
    out: Dict[int, List[int]] = {b: [] for b in range(1, 7)}
    try:
        table = hub.debug.cfg_get_table()
    except Exception as exc:
        print(f"CFG get failed ({exc}); using slots 0..5 = buses 1..6")
        for slot in range(6):
            out[slot + 1].append(slot)
        return out

    for slot, row in enumerate(table):
        if isinstance(row, dict):
            enabled = row.get("enabled", True)
            bus = int(row.get("bus", 0))
        else:
            enabled = getattr(row, "enabled", True)
            bus = int(getattr(row, "bus", 0))
        if enabled and 1 <= bus <= 6:
            out[bus].append(slot)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=40.0)
    args = ap.parse_args()

    hold = ActuatorDesire(position=0.01, velocity=0.0, kp=2.0, kd=0.5, torque=0.0)

    with ControlsPcbHub.connect(args.port, persist_telemetry=False) as hub:
        by_bus = find_slots_by_bus(hub)
        print("slots by bus:", {b: by_bus[b] for b in range(1, 7) if by_bus[b]})

        measure(hub, "blank", {}, seconds=args.seconds, hz=args.hz)

        fdcan_slots = [s for b in (1, 2, 3) for s in by_bus[b]]
        mcp_slots = [s for b in (4, 5, 6) for s in by_bus[b]]

        if fdcan_slots:
            measure(
                hub,
                "hold FDCAN CH1-3",
                {s: hold for s in fdcan_slots},
                seconds=args.seconds,
                hz=args.hz,
            )
        if mcp_slots:
            measure(
                hub,
                "hold MCP CH4-6",
                {s: hold for s in mcp_slots},
                seconds=args.seconds,
                hz=args.hz,
            )
            for bus in (4, 5, 6):
                slots = by_bus[bus]
                if not slots:
                    continue
                measure(
                    hub,
                    f"hold MCP CH{bus} alone (slot {slots[0]})",
                    {slots[0]: hold},
                    seconds=args.seconds,
                    hz=args.hz,
                )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
