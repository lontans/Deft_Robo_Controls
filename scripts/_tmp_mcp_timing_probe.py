"""Plant hold matrix: fb_hz, ack_lag, lap_ms across bus groups.

Product CFG (25 slots): CH1x8, CH2x8, CH3x3, CH4-6x2 each.

By default enables plant RX-sim children (actuator CAN + soft servo + LED SPI)
so held phases exercise TX+RX plus concurrent bench load without motors.

  cd scripts
  python _tmp_mcp_timing_probe.py --port COM5
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, ".")

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from deft_controls_sdk.link.api_types import McuState
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import (
    parse_actuator_feedback,
    parse_feedback_header,
)

PROTO_ROBSTRIDE = 1

# (schematic_bus, count) — matches plant_config_load_factory_defaults().
PRODUCT_LAYOUT: Tuple[Tuple[int, int], ...] = (
    (1, 8),
    (2, 8),
    (3, 3),
    (4, 2),
    (5, 2),
    (6, 2),
)

# CAN ACT LEDs return to solid idle after CAN_LED_ACTIVE_MS (250).
_LED_IDLE_SETTLE_S = 0.35


def _pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


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


def _row_bus_en_proto_mid(row) -> Tuple[int, bool, int, int]:
    if isinstance(row, dict):
        return (
            int(row.get("bus", 0)),
            bool(row.get("enabled", True)),
            int(row.get("protocol", 0)),
            int(row.get("motor_id", 0)),
        )
    return (
        int(getattr(row, "bus", 0)),
        bool(getattr(row, "enabled", True)),
        int(getattr(row, "protocol", 0)),
        int(getattr(row, "motor_id", 0)),
    )


def _expected_product_rows() -> List[Tuple[int, bool, int, int]]:
    """(bus, enabled, protocol, motor_id) per slot."""
    rows: List[Tuple[int, bool, int, int]] = []
    for bus, count in PRODUCT_LAYOUT:
        for n in range(count):
            rows.append((bus, True, PROTO_ROBSTRIDE, 0x01 + n))
    while len(rows) < ACTUATOR_COUNT:
        rows.append((1, False, PROTO_ROBSTRIDE, 1))
    return rows


def table_by_bus(table: List) -> Dict[int, List[int]]:
    by_bus: Dict[int, List[int]] = {b: [] for b in range(1, 7)}
    for slot, row in enumerate(table):
        bus, enabled, _proto, _mid = _row_bus_en_proto_mid(row)
        if enabled and 1 <= bus <= 6:
            by_bus[bus].append(slot)
    return by_bus


def ensure_product_cfg(hub: ControlsPcbHub, *, force: bool = False) -> Dict[int, List[int]]:
    """Use flash/RAM table if it already matches product layout; else CFG SET."""
    table = hub.debug.cfg_get_table()
    expect = _expected_product_rows()
    matches = (
        len(table) >= ACTUATOR_COUNT
        and all(
            _row_bus_en_proto_mid(table[i]) == expect[i] for i in range(ACTUATOR_COUNT)
        )
    )
    if matches and not force:
        print(f"CFG already matches product layout ({ACTUATOR_COUNT} slots) — skip SET")
    else:
        if len(table) < ACTUATOR_COUNT:
            print(
                f"CFG has {len(table)} slots (need {ACTUATOR_COUNT}) — applying product layout"
            )
        else:
            print("CFG differs from product layout — applying RAM SET")
        for slot, (bus, enabled, proto, mid) in enumerate(expect):
            hub.debug.cfg_set_slot(
                slot=slot,
                bus=bus,
                protocol=proto,
                motor_id=mid,
                enabled=enabled,
                persist=False,
            )
        table = hub.debug.cfg_get_table()

    by_bus = table_by_bus(table)
    print(f"CFG total_slots={len(table)} (expect {ACTUATOR_COUNT})")
    for b in range(1, 7):
        print(f"  CH{b}: {len(by_bus[b])} slots -> {by_bus[b]}")
    return by_bus


def leave_idle(hub: ControlsPcbHub) -> None:
    """Blank desires, disable RX-sim (full SK9822 blackout), NORMAL."""
    hub.set_rx_sim(False)
    for slot in range(ACTUATOR_COUNT):
        hub.set_actuator(slot, ActuatorDesire(), send=False)
    hub.set_mcu_state(McuState.NORMAL, send=False)
    hub.send_once()
    time.sleep(0.05)
    # Second clear: firmware led_force_all_off runs again (full chain ×2 TX).
    hub.set_rx_sim(False)
    hub.send_once()
    time.sleep(_LED_IDLE_SETTLE_S)
    hub.send_once()
    print(f"leave_idle: blank + rx_sim off + LED all-off (settled {_LED_IDLE_SETTLE_S:.2f}s)")


def measure(
    hub: ControlsPcbHub,
    label: str,
    desires: Dict[int, ActuatorDesire],
    *,
    seconds: float = 4.0,
    hz: float = 40.0,
    rx_sim: bool = True,
) -> dict:
    blank = {slot: ActuatorDesire() for slot in range(ACTUATOR_COUNT)}
    blank.update(desires)
    hub.set_rx_sim(rx_sim and bool(desires))
    _conn(hub).set_actuators(blank, send=False)

    dt = 1.0 / hz
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    reader = _conn(hub).reader
    tf0 = reader.total_frames

    ack_lags: List[int] = []
    img_lags: List[int] = []
    applied_lags: List[int] = []
    stage_vs_applied: List[int] = []
    lap_ms: List[int] = []
    lap_max: List[int] = []
    ticks_pend: List[int] = []
    ticks_svc: List[int] = []
    last_sent: Optional[int] = None
    last_sent_full: Optional[int] = None
    pdu_tags: Dict[str, int] = {}
    held = sorted(desires.keys())
    rx_fresh_samples: List[int] = []
    img_id_samples: List[Tuple[float, int, int]] = []  # t, staged, applied

    while time.perf_counter() < t_end:
        raw = _drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr:
                tag = str(hdr.get("pdu_tag", "?"))
                pdu_tags[tag] = pdu_tags.get(tag, 0) + 1
                ack = int(hdr["last_cmd_seq"]) & 0xFF
                if last_sent is not None:
                    lag = (last_sent - ack) & 0xFF
                    if lag <= 128:
                        ack_lags.append(lag)
                img_id = hdr.get("last_image_id")
                app_id = hdr.get("last_applied_image_id")
                if img_id is not None and app_id is not None:
                    img_id_samples.append(
                        (time.perf_counter(), int(img_id), int(app_id))
                    )
                    if last_sent_full is not None:
                        img_lags.append(int(last_sent_full) - int(img_id))
                        applied_lags.append(int(last_sent_full) - int(app_id))
                    stage_vs_applied.append(int(img_id) - int(app_id))
                if hdr.get("lap_ms") is not None:
                    lap_ms.append(int(hdr["lap_ms"]))
                if hdr.get("lap_max_ms") is not None:
                    lap_max.append(int(hdr["lap_max_ms"]))
                if hdr.get("ticks_pending") is not None:
                    ticks_pend.append(int(hdr["ticks_pending"]))
                if hdr.get("ticks_svc") is not None:
                    ticks_svc.append(int(hdr["ticks_svc"]))
                if held and rx_sim:
                    n_fresh = 0
                    for slot in held:
                        act = parse_actuator_feedback(raw, slot)
                        if act is None:
                            continue
                        # Synthetic FB echoes commanded position (~0.01 hold).
                        if abs(float(act["position"]) - float(desires[slot].position)) < 0.05:
                            n_fresh += 1
                    rx_fresh_samples.append(n_fresh)

        _conn(hub).send_once()
        sent = _conn(hub)._last_sent_seq  # noqa: SLF001
        last_sent_full = int(sent) if sent is not None else None
        last_sent = (sent & 0xFF) if sent is not None else None

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    time.sleep(0.05)
    while _drain_latest(hub) is not None:
        pass

    raw_fb = reader.total_frames - tf0
    raw_fb_hz = raw_fb / seconds if seconds > 0 else None
    rx_fresh_max = max(rx_fresh_samples) if rx_fresh_samples else None
    rx_fresh_mean = statistics.mean(rx_fresh_samples) if rx_fresh_samples else None

    staged_hz = None
    applied_hz = None
    if len(img_id_samples) >= 2:
        t0, s0, a0 = img_id_samples[0]
        t1, s1, a1 = img_id_samples[-1]
        dt_s = t1 - t0
        if dt_s > 0:
            staged_hz = (s1 - s0) / dt_s
            applied_hz = (a1 - a0) / dt_s

    def istat(xs: List[int]) -> str:
        if not xs:
            return "n/a"
        return (
            f"n={len(xs)} mean={statistics.mean(xs):.1f} "
            f"p95={_pct([float(x) for x in xs], 95):.0f} max={max(xs)}"
        )

    print(f"\n=== {label} ===")
    print(f"  held_slots={len(desires)}  ids={held}  rx_sim={rx_sim and bool(desires)}")
    print(
        f"  raw_fb_hz~{raw_fb_hz:.1f}" if raw_fb_hz is not None else "  raw_fb_hz=n/a",
        f"  frames={raw_fb}",
    )
    print(f"  ack_lag(8b): {istat(ack_lags)}")
    print(f"  img_lag(u32 staged):   {istat(img_lags)}")
    print(f"  app_lag(u32 applied):  {istat(applied_lags)}")
    print(f"  stage-applied gap:     {istat(stage_vs_applied)}")
    if staged_hz is not None and applied_hz is not None:
        print(
            f"  image_id advance: staged_hz~{staged_hz:.1f}  "
            f"applied_hz~{applied_hz:.1f}  (host target {hz:.0f})"
        )
    print(f"  lap_ms:  {istat(lap_ms)}")
    print(f"  lap_max: {istat(lap_max)}")
    print(f"  ticks_pending: {istat(ticks_pend)}")
    print(f"  ticks_svc:     {istat(ticks_svc)}")
    if held and rx_sim:
        print(
            f"  rx_fresh_slots: mean={rx_fresh_mean:.1f} max={rx_fresh_max} "
            f"(of {len(held)})" if rx_fresh_mean is not None else "  rx_fresh_slots: n/a"
        )
    print(f"  pdu_tags: {pdu_tags}")

    ok_lag = (not ack_lags) or (max(ack_lags) <= 2)
    ack_p95 = _pct([float(x) for x in ack_lags], 95) if ack_lags else None
    ok_lag_p95 = (ack_p95 is None) or (ack_p95 <= 2)
    fb_ok = raw_fb_hz is not None and raw_fb_hz >= 20.0
    ok_rx = (not held) or (not rx_sim) or (
        rx_fresh_max is not None and rx_fresh_max >= len(held)
    )
    ok_img = (not img_lags) or (max(img_lags) <= 2)
    print(
        f"  PASS_ack_lag<=2: {ok_lag}  PASS_ack_lag_p95<=2: {ok_lag_p95}  "
        f"PASS_fb_hz>=20: {fb_ok}  PASS_rx_fresh: {ok_rx}  "
        f"PASS_img_lag<=2: {ok_img}"
    )
    return {
        "label": label,
        "n_slots": len(desires),
        "raw_fb_hz": raw_fb_hz,
        "ack_lag_mean": statistics.mean(ack_lags) if ack_lags else None,
        "ack_lag_max": max(ack_lags) if ack_lags else None,
        "ack_lag_p95": int(ack_p95) if ack_p95 is not None else None,
        "img_lag_mean": statistics.mean(img_lags) if img_lags else None,
        "img_lag_max": max(img_lags) if img_lags else None,
        "app_lag_mean": statistics.mean(applied_lags) if applied_lags else None,
        "app_lag_max": max(applied_lags) if applied_lags else None,
        "stage_app_gap_max": max(stage_vs_applied) if stage_vs_applied else None,
        "staged_hz": staged_hz,
        "applied_hz": applied_hz,
        "lap_mean": statistics.mean(lap_ms) if lap_ms else None,
        "lap_max_max": max(lap_max) if lap_max else None,
        "pend_max": max(ticks_pend) if ticks_pend else None,
        "svc_mean": statistics.mean(ticks_svc) if ticks_svc else None,
        "rx_fresh_max": rx_fresh_max,
        "ok_lag": ok_lag,
        "ok_lag_p95": ok_lag_p95,
        "ok_fb": fb_ok,
        "ok_rx": ok_rx,
        "ok_img": ok_img,
    }


def slots_for(by_bus: Dict[int, List[int]], buses: Sequence[int]) -> List[int]:
    out: List[int] = []
    for b in buses:
        out.extend(by_bus[b])
    return out


def main() -> int:
    from deft_controls_sdk import find_cdc_port

    ap = argparse.ArgumentParser(
        description="Plant hold matrix probe (local tool — prefer deft_controls_sdk for apps)"
    )
    ap.add_argument(
        "--port",
        default=None,
        help="Serial port (default: auto-pick STM32 USB CDC 0483:5740)",
    )
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument(
        "--force-cfg",
        action="store_true",
        help="Always CFG SET product layout (default: skip if table already matches)",
    )
    ap.add_argument(
        "--no-rx-sim",
        action="store_true",
        help="TX-only hold (legacy); default injects synthetic RS02 RX per held slot",
    )
    args = ap.parse_args()
    port = args.port or find_cdc_port()
    rx_sim = not args.no_rx_sim

    hold = ActuatorDesire(position=0.01, velocity=0.0, kp=2.0, kd=0.5, torque=0.0)
    results: List[dict] = []

    print(f"Opening {port} ... ACTUATOR_COUNT={ACTUATOR_COUNT}  rx_sim={rx_sim}")
    with ControlsPcbHub.connect(port, persist_telemetry=False) as hub:
        try:
            by_bus = ensure_product_cfg(hub, force=args.force_cfg)

            def desire_map(*buses: int) -> Dict[int, ActuatorDesire]:
                return {s: hold for s in slots_for(by_bus, buses)}

            phases = [
                ("0_blank", {}),
                ("1_CH1_x8", desire_map(1)),
                ("2_CH2_x8", desire_map(2)),
                ("3_CH3_x3", desire_map(3)),
                ("4_CH4_x2", desire_map(4)),
                ("5_CH5_x2", desire_map(5)),
                ("6_CH6_x2", desire_map(6)),
                ("7_CH1-3_fdcan", desire_map(1, 2, 3)),
                ("8_CH4-6_mcp", desire_map(4, 5, 6)),
                ("9_all_CH1-6_x25", desire_map(1, 2, 3, 4, 5, 6)),
                ("10_blank_after", {}),
            ]

            for label, desires in phases:
                sec = 2.0 if "blank" in label else args.seconds
                results.append(
                    measure(
                        hub,
                        label,
                        desires,
                        seconds=sec,
                        hz=args.hz,
                        rx_sim=rx_sim,
                    )
                )
        finally:
            leave_idle(hub)

    print("\n=== SUMMARY ===")
    all_lag_ok = True
    all_rx_ok = True
    for r in results:
        fb = r["raw_fb_hz"]
        fb_s = f"{fb:.0f}" if fb is not None else "n/a"
        lag = r["ack_lag_max"]
        lag_s = str(lag) if lag is not None else "n/a"
        rx = r.get("rx_fresh_max")
        rx_s = str(rx) if rx is not None else "-"
        app_hz = r.get("applied_hz")
        app_s = f"{app_hz:.0f}" if app_hz is not None else "-"
        app_lag = r.get("app_lag_max")
        al_s = str(app_lag) if app_lag is not None else "-"
        print(
            f"  {r['label']:18s}  n={r['n_slots']:2d}  fb_hz={fb_s:>5s}  "
            f"ack_lag_max={lag_s:>3s}  "
            f"app_lag_max={al_s:>3s}  "
            f"applied_hz={app_s:>4s}  "
            f"rx_fresh={rx_s:>3s}  "
            f"lag_ok={r['ok_lag']}  fb_ok={r['ok_fb']}  rx_ok={r.get('ok_rx')}"
        )
        if not r["ok_lag"]:
            all_lag_ok = False
        if not r.get("ok_rx", True):
            all_rx_ok = False

    print(f"\nOVERALL ack_lag OK: {all_lag_ok}  rx_fresh OK: {all_rx_ok}")
    return 0 if (all_lag_ok and all_rx_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
