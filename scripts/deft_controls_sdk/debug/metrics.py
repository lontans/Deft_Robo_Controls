"""Plant-stream hold metrics: fb_hz, ack_lag, lap timing, stm32_mode.

The Controls teleop path is the normal 694 B plant image (CMDH/HBHF), not DEBUG
frames. These metrics exercise that same transaction at a host ``hz``.
"""
from __future__ import annotations

import statistics
import time
from typing import TYPE_CHECKING, Dict, List, Optional, Union

from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import parse_feedback_header
from deft_controls_sdk.link.exchange.wire_layout import (
    LINK_MODE_ALIASES,
    STM32_MODE_BANDWIDTH,
    STM32_MODE_NAMES,
    STM32_MODE_SOFT_DFU,
)

if TYPE_CHECKING:
    from deft_controls_sdk import ControlsPcbHub


def _pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


def _conn(hub: "ControlsPcbHub"):
    return hub._connection  # noqa: SLF001


def drain_latest(hub: "ControlsPcbHub"):
    raw = None
    while True:
        frame = _conn(hub).reader.pop()
        if frame is None:
            break
        raw = frame
    return raw


def _resolve_stm32_mode(expected: Union[int, str, None]) -> Optional[int]:
    if expected is None:
        return None
    if isinstance(expected, int):
        return int(expected) & 0x3
    key = str(expected).strip().lower()
    if key in LINK_MODE_ALIASES:
        return LINK_MODE_ALIASES[key]
    raise ValueError(
        f"unknown expected stm32_mode {expected!r}; "
        f"use {sorted(LINK_MODE_ALIASES)} or 0..2"
    )


def settle_stm32_mode(
    hub: "ControlsPcbHub",
    want_mode: int,
    *,
    hz: float = 200.0,
    timeout_s: float = 1.5,
    confirm: int = 3,
) -> dict:
    """TX until plant FB echoes ``want_mode`` for ``confirm`` consecutive samples.

    Drains USB-IN leftovers from a prior session. Soft-DFU echo is a hard fail
    (board needs ``pcb_lab leave`` / Soft-DFU recovery — not a startup glitch).
    """
    want = int(want_mode) & 0x3
    want_name = STM32_MODE_NAMES.get(want, f"mode_{want}")
    dt = 1.0 / max(1.0, float(hz))
    deadline = time.perf_counter() + float(timeout_s)
    matched = 0
    last_mode: Optional[int] = None
    saw_soft_dfu = False

    # Drop anything already queued before we TX the new mode.
    while drain_latest(hub) is not None:
        pass

    while time.perf_counter() < deadline:
        _conn(hub).send_once()
        time.sleep(dt)
        raw = drain_latest(hub)
        if raw is None:
            continue
        hdr = parse_feedback_header(raw)
        if hdr is None or hdr.get("is_debug"):
            continue
        mode = int(hdr.get("stm32_mode", -1)) & 0x3
        last_mode = mode
        if mode == STM32_MODE_SOFT_DFU:
            saw_soft_dfu = True
            matched = 0
            break
        if mode == want:
            matched += 1
            if matched >= confirm:
                while drain_latest(hub) is not None:
                    pass
                return {
                    "ok": True,
                    "saw_soft_dfu": False,
                    "last_mode": mode,
                    "last_mode_name": STM32_MODE_NAMES.get(mode, f"mode_{mode}"),
                    "want_mode": want,
                    "want_mode_name": want_name,
                }
        else:
            matched = 0

    return {
        "ok": False,
        "saw_soft_dfu": saw_soft_dfu,
        "last_mode": last_mode,
        "last_mode_name": (
            STM32_MODE_NAMES.get(last_mode, f"mode_{last_mode}")
            if last_mode is not None
            else None
        ),
        "want_mode": want,
        "want_mode_name": want_name,
    }


def measure_hold(
    hub: "ControlsPcbHub",
    label: str,
    desires: Dict[int, ActuatorDesire],
    *,
    seconds: float = 3.0,
    hz: float = 40.0,
    min_fb_hz: Optional[float] = None,
    expected_stm32_mode: Union[int, str, None] = STM32_MODE_BANDWIDTH,
    settle_timeout_s: float = 1.5,
    print_report: bool = True,
) -> dict:
    """Hold desires at ``hz`` for ``seconds``; collect FB health stats.

    Pass gates (also returned as ok_lag / ok_fb / ok_mode):
      ack_lag p95 <= 2,
      fb_hz >= ``min_fb_hz`` (default: 80% of host ``hz``),
      after settle, every plant FB sample echoes ``expected_stm32_mode``
      (default bandwidth). Soft-DFU echo fails hard (needs leave/recover).
      Pass ``expected_stm32_mode=None`` to skip the mode gate.
    """
    want_mode = _resolve_stm32_mode(expected_stm32_mode)
    blank = {slot: ActuatorDesire() for slot in range(ACTUATOR_COUNT)}
    blank.update(desires)
    _conn(hub).set_actuators(blank, send=False)

    host_mode = int(getattr(hub, "stm32_mode", STM32_MODE_BANDWIDTH)) & 0x3
    host_mode_name = STM32_MODE_NAMES.get(host_mode, f"mode_{host_mode}")
    want_name = (
        STM32_MODE_NAMES.get(want_mode, str(want_mode)) if want_mode is not None else "any"
    )

    settle: Optional[dict] = None
    if want_mode is not None:
        settle = settle_stm32_mode(
            hub,
            want_mode,
            hz=float(hz),
            timeout_s=float(settle_timeout_s),
        )
        if print_report:
            if settle.get("saw_soft_dfu"):
                print(
                    "stm32_mode settle: FB echoes soft_dfu — board needs Soft-DFU "
                    "leave / recovery (python -m pcb_lab leave), not a startup race"
                )
            elif not settle.get("ok"):
                print(
                    f"stm32_mode settle FAILED: want={want_name} "
                    f"last_fb={settle.get('last_mode_name')!r} "
                    f"(host={host_mode_name}). Rebuild/flash if echo stays sticky."
                )
            else:
                print(
                    f"stm32_mode settle ok: fb={settle.get('last_mode_name')} "
                    f"(host={host_mode_name})"
                )

    dt = 1.0 / hz
    t_end = time.perf_counter() + seconds
    next_t = time.perf_counter()
    reader = _conn(hub).reader
    tf0 = reader.total_frames

    ack_lags: List[int] = []
    lap_ms: List[int] = []
    lap_max_sticky: List[int] = []
    periph_lap_ms: List[int] = []
    periph_lap_max_sticky: List[int] = []
    ticks_pend: List[int] = []
    ticks_svc: List[int] = []
    last_sent: Optional[int] = None
    mode_counts: Dict[int, int] = {}

    while time.perf_counter() < t_end:
        raw = drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr and not hdr.get("is_debug"):
                mode = hdr.get("stm32_mode")
                if mode is not None:
                    mode_i = int(mode) & 0x3
                    mode_counts[mode_i] = mode_counts.get(mode_i, 0) + 1
                ack = int(hdr["last_cmd_seq"]) & 0xFF
                if last_sent is not None:
                    lag = (last_sent - ack) & 0xFF
                    if lag <= 128:
                        ack_lags.append(lag)
                if hdr.get("lap_ms") is not None:
                    lap_ms.append(int(hdr["lap_ms"]))
                if hdr.get("lap_max_ms") is not None:
                    lap_max_sticky.append(int(hdr["lap_max_ms"]))
                if hdr.get("periph_lap_ms") is not None:
                    periph_lap_ms.append(int(hdr["periph_lap_ms"]))
                if hdr.get("periph_lap_max_ms") is not None:
                    periph_lap_max_sticky.append(int(hdr["periph_lap_max_ms"]))
                if hdr.get("ticks_pending") is not None:
                    ticks_pend.append(int(hdr["ticks_pending"]))
                if hdr.get("ticks_svc") is not None:
                    ticks_svc.append(int(hdr["ticks_svc"]))

        _conn(hub).send_once()
        sent = _conn(hub)._last_sent_seq  # noqa: SLF001
        last_sent = (sent & 0xFF) if sent is not None else None

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    time.sleep(0.05)
    while drain_latest(hub) is not None:
        pass

    raw_fb = reader.total_frames - tf0
    raw_fb_hz = raw_fb / seconds if seconds > 0 else None
    fb_floor = float(min_fb_hz) if min_fb_hz is not None else max(20.0, 0.8 * float(hz))

    def istat(xs: List[int]) -> str:
        if not xs:
            return "n/a"
        return (
            f"n={len(xs)} mean={statistics.mean(xs):.1f} "
            f"p95={_pct([float(x) for x in xs], 95):.0f} max={max(xs)}"
        )

    ok_lag = (not ack_lags) or (max(ack_lags) <= 2)
    if len(ack_lags) >= 20:
        p95 = _pct([float(x) for x in ack_lags], 95)
        ok_lag = p95 is not None and p95 <= 2.0
    ok_fb = raw_fb_hz is not None and raw_fb_hz >= fb_floor
    fb_mode = max(mode_counts, key=mode_counts.get) if mode_counts else None
    fb_mode_name = (
        STM32_MODE_NAMES.get(fb_mode, f"mode_{fb_mode}") if fb_mode is not None else None
    )
    saw_soft_dfu = STM32_MODE_SOFT_DFU in mode_counts or bool(
        settle and settle.get("saw_soft_dfu")
    )
    if want_mode is None:
        ok_mode = True
    else:
        # Strict: settle must succeed, then every measured sample must match.
        # Soft-DFU in hist is always a fail (recovery required).
        ok_mode = (
            host_mode == want_mode
            and settle is not None
            and bool(settle.get("ok"))
            and not saw_soft_dfu
            and len(mode_counts) == 1
            and want_mode in mode_counts
        )
    lap_window_max = max(lap_ms) if lap_ms else None
    lap_sticky_max = max(lap_max_sticky) if lap_max_sticky else None
    periph_window_max = max(periph_lap_ms) if periph_lap_ms else None
    periph_sticky_max = max(periph_lap_max_sticky) if periph_lap_max_sticky else None

    mode_hist = {
        STM32_MODE_NAMES.get(k, f"mode_{k}"): v for k, v in sorted(mode_counts.items())
    }

    if print_report:
        print(f"\n=== {label} ===")
        print(f"  held_slots={len(desires)}  ids={sorted(desires.keys())}")
        print(
            f"  raw_fb_hz~{raw_fb_hz:.1f}" if raw_fb_hz is not None else "  raw_fb_hz=n/a",
            f"  frames={raw_fb}  host_hz={hz:.0f}  fb_floor={fb_floor:.0f}",
        )
        print(
            f"  stm32_mode host={host_mode_name}  "
            f"fb={fb_mode_name or 'n/a'}  hist={mode_hist}  expect={want_name}"
        )
        print(f"  ack_lag:       {istat(ack_lags)}")
        print(f"  act_lap_ms:    {istat(lap_ms)}")
        print(
            f"  act_lap_max:   window={lap_window_max}  sticky_fw={lap_sticky_max}"
        )
        print(f"  periph_lap_ms: {istat(periph_lap_ms)}")
        print(
            f"  periph_lap_max: window={periph_window_max}  sticky_fw={periph_sticky_max}"
        )
        print(f"  ticks_pending: {istat(ticks_pend)}")
        print(f"  ticks_svc:     {istat(ticks_svc)}")
        print(
            f"  PASS_ack_lag_p95<=2: {ok_lag}  "
            f"PASS_fb_hz>={fb_floor:.0f}: {ok_fb}  "
            f"PASS_stm32_mode={want_name}: {ok_mode}"
        )
        if saw_soft_dfu:
            print("  note: soft_dfu echo seen — run: python -m pcb_lab leave")

    return {
        "label": label,
        "n_slots": len(desires),
        "raw_fb_hz": raw_fb_hz,
        "fb_floor": fb_floor,
        "host_hz": float(hz),
        "host_stm32_mode": host_mode,
        "host_stm32_mode_name": host_mode_name,
        "fb_stm32_mode": fb_mode,
        "fb_stm32_mode_name": fb_mode_name,
        "stm32_mode_hist": mode_hist,
        "expected_stm32_mode": want_mode,
        "settle": settle,
        "saw_soft_dfu": saw_soft_dfu,
        "ack_lag_max": max(ack_lags) if ack_lags else None,
        "ack_lag_mean": statistics.mean(ack_lags) if ack_lags else None,
        "lap_ms_mean": statistics.mean(lap_ms) if lap_ms else None,
        "lap_max_ms": lap_window_max,
        "lap_max_sticky_ms": lap_sticky_max,
        "periph_lap_ms_mean": statistics.mean(periph_lap_ms) if periph_lap_ms else None,
        "periph_lap_max_ms": periph_window_max,
        "periph_lap_max_sticky_ms": periph_sticky_max,
        "ok_lag": ok_lag,
        "ok_fb": ok_fb,
        "ok_mode": ok_mode,
        "ok": ok_lag and ok_fb and ok_mode,
    }
