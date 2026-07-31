"""Formatters + thin re-exports for the plant debug suite ``show`` command.

Programmatic snapshots live in ``deft_controls_sdk.debug.snapshot`` /
``HostProxy.cfg_snapshot()`` — import those from notebooks, not this module.
"""
from __future__ import annotations

import shutil
from typing import Any, Dict, List, Optional

from deft_controls_sdk.debug.snapshot import (  # noqa: F401 — suite re-export
    collect_bandwidth,
    collect_cfg,
)
from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link.api_types import led_mode_name
from deft_controls_sdk.config import SERVO_MODEL_NAMES


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


def collect_status(proxy: HostProxy) -> Dict[str, Any]:
    """Alias of ``proxy.doctor()`` (kept for suite CLI / older imports)."""
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
