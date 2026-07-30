"""Collectors for ``pcb_lab.debug show``."""
from __future__ import annotations

import shutil
from typing import Any, Dict, List, Optional

from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link.api_types import led_mode_name
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import SERVO_MODEL_NAMES

from pcb_lab.debug.proto import protocol_name


def terminal_cols(*, fallback: int = 80, floor: int = 40) -> int:
    """Current TTY width (Windows PowerShell / Linux / Ubuntu)."""
    try:
        cols = int(shutil.get_terminal_size(fallback=(fallback, 24)).columns)
    except Exception:  # noqa: BLE001
        cols = fallback
    return max(floor, cols)


def _wrap_line(ln: str, width: int) -> List[str]:
    """Wrap one line to ``width`` without collapsing table spacing."""
    if width < 1:
        return [ln]
    if not ln:
        return [""]
    out: List[str] = []
    rest = ln
    while rest:
        if len(rest) <= width:
            out.append(rest)
            break
        chunk = rest[:width]
        br = chunk.rfind(" ")
        # Prefer a late space break; otherwise hard-wrap (wide table rows).
        if br >= max(8, width // 4):
            out.append(rest[:br])
            rest = rest[br + 1 :].lstrip(" ")
        else:
            out.append(chunk)
            rest = rest[width:]
    return out or [""]


def format_banner(banner: str = "", *, width: Optional[int] = None) -> str:
    """Clip banner lines to the terminal so a narrow window doesn't wrap messily."""
    cols = terminal_cols() if width is None else max(20, int(width))
    src = banner if banner else DEFT_BANNER
    return "\n".join(ln[:cols].rstrip() for ln in src.splitlines())


def _normalize_table(table: List[dict]) -> List[dict]:
    out: List[dict] = []
    for i, row in enumerate(table):
        if row is None:
            continue
        slot = int(row.get("slot", i))
        proto = int(row.get("protocol", 0))
        out.append(
            {
                "slot": slot,
                "enabled": bool(row.get("enabled", False)),
                "bus": int(row.get("bus", 0)),
                "protocol": proto,
                "protocol_name": protocol_name(proto),
                "motor_id": int(row.get("motor_id", 0)),
                "motor_id_hex": f"0x{int(row.get('motor_id', 0)) & 0xFF:02X}",
            }
        )
    return out


def collect_cfg(proxy: HostProxy) -> Dict[str, Any]:
    """Full NVM v2 view: actuator slots + periph (neck/LED/listen_pdu)."""
    with pause_plant_stream(proxy.hub):
        table = proxy.hub.debug.cfg_get_table()
        try:
            periph = dict(proxy.hub.debug.cfg_get_periph())
            periph_ok = True
            periph_error = None
        except Exception as exc:  # noqa: BLE001
            periph = {}
            periph_ok = False
            periph_error = str(exc)
    rows = _normalize_table(table)
    enabled = [r for r in rows if r["enabled"]]
    out: Dict[str, Any] = {
        "slots": rows,
        "enabled_count": len(enabled),
        "total": len(rows),
        "periph_ok": periph_ok,
    }
    if periph_ok:
        out["listen_pdu"] = bool(periph.get("listen_pdu", False))
        out["flags"] = int(periph.get("flags", 0))
        out["servos"] = list(periph.get("servos") or [])
        out["led"] = dict(periph.get("led") or {})
    else:
        out["periph_error"] = periph_error
    return out


def collect_bandwidth(proxy: HostProxy) -> Dict[str, Any]:
    """Plant / periph lap timing from latest FB + host stream rates."""
    hub = proxy.hub
    conn = hub._connection  # noqa: SLF001
    hot = getattr(conn, "_hot_stats", None) or {}
    telem = getattr(hub, "telemetry", None)
    snap = None
    if telem is not None:
        try:
            snap = telem.snapshot()
        except Exception:  # noqa: BLE001
            snap = None

    def _from_snap(name: str) -> Optional[Any]:
        if snap is None:
            return None
        return getattr(snap, name, None)

    return {
        "stream_hz": hub.stream_hz,
        "telemetry_hz": hub.telemetry_hz,
        "act_lap_ms": _from_snap("lap_ms") if _from_snap("lap_ms") is not None else hot.get("lap_ms"),
        "act_lap_peak_ms": (
            _from_snap("lap_max_ms")
            if _from_snap("lap_max_ms") is not None
            else hot.get("lap_max_ms")
        ),
        "periph_lap_ms": (
            _from_snap("periph_lap_ms")
            if _from_snap("periph_lap_ms") is not None
            else hot.get("periph_lap_ms")
        ),
        "periph_lap_peak_ms": (
            _from_snap("periph_lap_max_ms")
            if _from_snap("periph_lap_max_ms") is not None
            else hot.get("periph_lap_max_ms")
        ),
        "ticks_pending": (
            _from_snap("ticks_pending")
            if _from_snap("ticks_pending") is not None
            else hot.get("ticks_pending")
        ),
        "note": "lap_* from plant FB / telemetry; stream_hz is host TX rate",
    }


def collect_status(proxy: HostProxy) -> Dict[str, Any]:
    return proxy.doctor()


# figlet-style "Deft Robotics Controls" (keep trailing spaces for alignment)
DEFT_BANNER = r"""   _____            _             _       _____   _____ ____   _____             __ _       
  / ____|          | |           | |     |  __ \ / ____|  _ \ / ____|           / _(_)      
 | |     ___  _ __ | |_ _ __ ___ | |___  | |__) | |    | |_) | |     ___  _ __ | |_ _  __ _ 
 | |    / _ \| '_ \| __| '__/ _ \| / __| |  ___/| |    |  _ <| |    / _ \| '_ \|  _| |/ _` |
 | |___| (_) | | | | |_| | | (_) | \__ \ | |    | |____| |_) | |___| (_) | | | | | | | (_| |
  \_____\___/|_| |_|\__|_|  \___/|_|___/ |_|     \_____|____/ \_____\___/|_| |_|_| |_|\__, |
                                     ______               ______                       __/ |
                                    |______|             |______|                     |___/ """.strip(
    "\n"
)


def _box_lines(
    body_lines: List[str],
    *,
    pad: int = 1,
    width: Optional[int] = None,
    fill: bool = True,
) -> List[str]:
    """ASCII box around ``body_lines`` (Windows PowerShell + Linux/Ubuntu).

    Long lines wrap to the terminal width. With ``fill=True`` (default) the box
    spans the window; otherwise it shrinks to the widest wrapped line.
    """
    # Leave one column free — some hosts wrap when the cursor hits the last col.
    cols = terminal_cols() if width is None else max(20, int(width))
    outer = max(20, cols - 1)
    inner = outer - 2
    content_w = max(8, inner - 2 * pad)

    wrapped: List[str] = []
    for raw in body_lines:
        wrapped.extend(_wrap_line(raw.rstrip("\n"), content_w))
    if not wrapped:
        wrapped = [""]

    if fill:
        text_w = content_w
        box_inner = inner
    else:
        text_w = min(content_w, max(len(ln) for ln in wrapped))
        box_inner = text_w + 2 * pad

    top = "+" + "-" * box_inner + "+"
    bot = "+" + "-" * box_inner + "+"
    out = [top]
    for ln in wrapped:
        out.append("|" + (" " * pad) + ln.ljust(text_w)[:text_w] + (" " * pad) + "|")
    out.append(bot)
    return out


def format_cfg_table(
    cfg: Dict[str, Any],
    *,
    only_enabled: bool = False,
    banner: bool = False,
    box: bool = False,
) -> str:
    rows = cfg.get("slots") or []
    lines = [
        f"CFG NVM v2  actuators enabled={cfg.get('enabled_count')}/{cfg.get('total')}",
        f"{'slot':>4}  {'en':>3}  {'bus':>3}  {'protocol':<10}  {'motor_id':>8}",
        "-" * 40,
    ]
    for r in rows:
        if only_enabled and not r["enabled"]:
            continue
        lines.append(
            f"{r['slot']:4d}  {'Y' if r['enabled'] else '.':>3}  "
            f"{r['bus']:3d}  {r['protocol_name']:<10}  {r['motor_id_hex']:>8}"
        )

    lines.append("")
    if not cfg.get("periph_ok", True) and "periph_error" in cfg:
        lines.append(f"periph: GET_PERIPH failed ({cfg['periph_error']})")
    elif "listen_pdu" in cfg or "servos" in cfg or "led" in cfg:
        listen = bool(cfg.get("listen_pdu", False))
        lines.append(f"listen_pdu: {'ON' if listen else 'off'}  (NVM flag bit0)")
        lines.append("neck servos:")
        lines.append(
            f"  {'slot':>4}  {'en':>3}  {'model':<14}  {'id':>4}  "
            f"{'pos_min':>7}  {'pos_max':>7}"
        )
        for s in cfg.get("servos") or []:
            mid = int(s.get("model", 0))
            mname = SERVO_MODEL_NAMES.get(mid, str(mid))
            lines.append(
                f"  {int(s.get('slot', 0)):4d}  "
                f"{'Y' if s.get('enabled') else '.':>3}  "
                f"{mname:<14}  "
                f"0x{int(s.get('id', 0)) & 0xFF:02X}  "
                f"{int(s.get('pos_min', 0)):7d}  "
                f"{int(s.get('pos_max', 0)):7d}"
            )
        led = cfg.get("led") or {}
        mode = int(led.get("default_mode", 0))
        lines.append(
            f"led defaults: count={int(led.get('default_count', 0))}  "
            f"mode={mode} ({led_mode_name(mode)})  "
            f"brightness={int(led.get('default_brightness', 0))}"
        )

    # Drop trailing blank lines before boxing.
    while lines and lines[-1] == "":
        lines.pop()

    if box:
        lines = _box_lines(lines)

    if banner:
        return format_banner() + "\n\n" + "\n".join(lines)
    return "\n".join(lines)
