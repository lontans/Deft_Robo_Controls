"""Plant-stream hold metrics: fb_hz, ack_lag, lap timing, plant PDU tags.

The Controls teleop path is the normal 672 B plant image (CMDH/HBHF), not DEBUG
frames. These metrics exercise that same transaction at a host ``hz``.
"""
from __future__ import annotations

import statistics
import time
from typing import TYPE_CHECKING, Dict, List, Optional

from deft_controls_sdk.link import ActuatorDesire
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.link.exchange.parse import parse_feedback_header

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


def measure_hold(
    hub: "ControlsPcbHub",
    label: str,
    desires: Dict[int, ActuatorDesire],
    *,
    seconds: float = 3.0,
    hz: float = 40.0,
    print_report: bool = True,
) -> dict:
    """Hold desires at ``hz`` for ``seconds``; collect FB health stats.

    Pass gates (also returned as ok_lag / ok_fb / ok_plant_tag):
      ack_lag max <= 2, fb_hz >= 20, plant ``pdu`` tag is plain plant
      (0x00 or live PDBF ``P…``) — not DEBUG/bench mailbox tags.
    """
    blank = {slot: ActuatorDesire() for slot in range(ACTUATOR_COUNT)}
    blank.update(desires)
    _conn(hub).set_actuators(blank, send=False)

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
    pdu_tags: Dict[str, int] = {}

    while time.perf_counter() < t_end:
        raw = drain_latest(hub)
        if raw is not None:
            hdr = parse_feedback_header(raw)
            if hdr and not hdr.get("is_debug"):
                tag = str(hdr.get("pdu_tag", "?"))
                pdu_tags[tag] = pdu_tags.get(tag, 0) + 1
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

    def istat(xs: List[int]) -> str:
        if not xs:
            return "n/a"
        return (
            f"n={len(xs)} mean={statistics.mean(xs):.1f} "
            f"p95={_pct([float(x) for x in xs], 95):.0f} max={max(xs)}"
        )

    ok_lag = (not ack_lags) or (max(ack_lags) <= 2)
    ok_fb = raw_fb_hz is not None and raw_fb_hz >= 20.0
    # Plant mailbox may be zeros (legacy) or live PDBF ('P' from magic PDBF).
    # Fail only when DEBUG/bench tags appear on the plant stream.
    _plant_ok_tags = {"0x00", "0", "?", "P"}
    ok_plant_tag = all(k in _plant_ok_tags for k in pdu_tags)
    lap_window_max = max(lap_ms) if lap_ms else None
    lap_sticky_max = max(lap_max_sticky) if lap_max_sticky else None
    periph_window_max = max(periph_lap_ms) if periph_lap_ms else None
    periph_sticky_max = max(periph_lap_max_sticky) if periph_lap_max_sticky else None

    if print_report:
        print(f"\n=== {label} ===")
        print(f"  held_slots={len(desires)}  ids={sorted(desires.keys())}")
        print(
            f"  raw_fb_hz~{raw_fb_hz:.1f}" if raw_fb_hz is not None else "  raw_fb_hz=n/a",
            f"  frames={raw_fb}",
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
        print(f"  pdu_tags: {pdu_tags}")
        print(
            f"  PASS_ack_lag<=2: {ok_lag}  PASS_fb_hz>=20: {ok_fb}  "
            f"PASS_plant_tag_0: {ok_plant_tag}"
        )

    return {
        "label": label,
        "n_slots": len(desires),
        "raw_fb_hz": raw_fb_hz,
        "ack_lag_max": max(ack_lags) if ack_lags else None,
        "ack_lag_mean": statistics.mean(ack_lags) if ack_lags else None,
        "lap_ms_mean": statistics.mean(lap_ms) if lap_ms else None,
        "lap_max_ms": lap_window_max,
        "lap_max_sticky_ms": lap_sticky_max,
        "periph_lap_ms_mean": statistics.mean(periph_lap_ms) if periph_lap_ms else None,
        "periph_lap_max_ms": periph_window_max,
        "periph_lap_max_sticky_ms": periph_sticky_max,
        "pdu_tags": pdu_tags,
        "ok_lag": ok_lag,
        "ok_fb": ok_fb,
        "ok_plant_tag": ok_plant_tag,
        "ok_pdb": ok_plant_tag,  # back-compat alias
        "ok": ok_lag and ok_fb and ok_plant_tag,
    }
