"""Terminal PCB dashboard for ``show --pcb``.

Shows CFG connections (bus/protocol/id per slot) overlaid with the live
TelemetryCache actuator strip, cache age, PDB, neck servos, and periph —
refreshed until Ctrl+C (or once with ``once=True``).

Plant note: true blank (p=0, kp=0) on an uncommanded bus is skipped for both
FDCAN and MCP (shared blank-bus policy). This TUI idle-anchors enabled slots
(p=1e-6, kp=0) and holds NORMAL so CH4–6 stay in the plant path with CH1–3.

Neck: brief ``torque_enable=False`` host session (read-only after FW honors that
bit — no torque-on / no goal write), then clear so actuator apply is not
SERVO_SESSION-blocked.
"""
from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict, List, Optional

from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link import ActuatorDesire, McuState, ServoDesire
from deft_controls_sdk.link.api_types import led_mode_name
from deft_controls_sdk.link.exchange import parse_servo_feedback
from deft_controls_sdk.vbeta.slots import SERVO_MODEL_NAMES

from .show import _box_lines, collect_cfg, format_banner

_MCU_STATE_NAMES = {0: "NORMAL", 1: "RECOVERY", 2: "DIAG_ONLY(legacy)", 3: "ESTOP"}
# Firmware treats idle + position==0 as blank; uncommanded blank buses are
# skipped (FDCAN and MCP share the policy in actuator.c). Idle-anchor uses eps.
HOME_POS_EPS = 1e-6
SERVO_COUNT = 2
SERVO_SAMPLE_S = 0.35


def _clear() -> None:
    if os.name == "nt":
        os.system("cls")
    else:
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()


def _fmt(v: Any, w: int, nd: int = 3) -> str:
    if v is None:
        return f"{'—':>{w}}"
    if isinstance(v, float):
        return f"{v:{w}.{nd}f}"
    return f"{v!s:>{w}}"


def _fault_yn(fault: Any) -> str:
    if fault is None:
        return f"{'—':>5}"
    try:
        return f"{'yes' if int(fault) != 0 else 'no':>5}"
    except (TypeError, ValueError):
        return f"{'yes' if fault else 'no':>5}"


def _yn(ok: Optional[bool], *, w: int = 5) -> str:
    if ok is None:
        return f"{'—':>{w}}"
    return f"{'yes' if ok else 'no':>{w}}"


def slot_connected(row: Dict[str, Any], fb: Optional[Dict[str, Any]]) -> bool:
    """True when plant FB shows a drive reply for this CFG motor_id.

    Plugins reject RX when the frame id ≠ CFG motor_id, so wrong / placeholder
    ``0x00`` ids leave the slot zeroed — same as discover not finding anyone.
    """
    if not row.get("enabled"):
        return False
    mid = int(row.get("motor_id", 0)) & 0xFF
    if mid == 0:
        return False
    if fb is None:
        return False
    try:
        if float(fb.get("temperature") or 0.0) != 0.0:
            return True
    except (TypeError, ValueError):
        pass
    try:
        if int(fb.get("fault") or 0) != 0:
            return True
    except (TypeError, ValueError):
        pass
    for key in ("position", "velocity", "torque"):
        try:
            if abs(float(fb.get(key) or 0.0)) > 1e-4:
                return True
        except (TypeError, ValueError):
            continue
    return False


def connection_summary(cfg: Dict[str, Any], snap) -> Dict[str, Any]:
    enabled = 0
    live = 0
    for r in cfg.get("slots") or []:
        if not r.get("enabled"):
            continue
        enabled += 1
        if slot_connected(r, _fb_for_slot(snap, int(r["slot"]))):
            live += 1
    return {
        "enabled": enabled,
        "live": live,
        "connected": live > 0,
    }


def servo_connected(row: Dict[str, Any], fb: Optional[Dict[str, Any]]) -> bool:
    """True when plant servo FB reports ``motor_source_id`` matching CFG id.

    MCU only sets ``motor_source_id`` after a successful DXL present-position
    read for that id — wrong / placeholder ``0x00`` stays disconnected.
    """
    if not row.get("enabled"):
        return False
    sid = int(row.get("id", 0)) & 0xFF
    if sid == 0:
        return False
    if fb is None:
        return False
    src = int(fb.get("motor_source_id", 0) or 0)
    return src == sid


def servo_connection_summary(
    cfg: Dict[str, Any], servo_fb: Dict[int, Optional[Dict[str, Any]]]
) -> Dict[str, Any]:
    enabled = 0
    live = 0
    for s in cfg.get("servos") or []:
        if not s.get("enabled"):
            continue
        enabled += 1
        slot = int(s.get("slot", 0))
        if servo_connected(s, servo_fb.get(slot)):
            live += 1
    return {
        "enabled": enabled,
        "live": live,
        "connected": live > 0,
    }


def _latest_raw(proxy: HostProxy) -> Optional[bytes]:
    raw = getattr(proxy.hub._connection, "_latest_fb_raw", None)  # noqa: SLF001
    return raw if isinstance(raw, (bytes, bytearray)) else None


def _read_servo_fb(proxy: HostProxy) -> Dict[int, Optional[Dict[str, Any]]]:
    raw = _latest_raw(proxy)
    out: Dict[int, Optional[Dict[str, Any]]] = {}
    for slot in range(SERVO_COUNT):
        if raw is None:
            out[slot] = None
            continue
        try:
            out[slot] = parse_servo_feedback(bytes(raw), slot)
        except Exception:  # noqa: BLE001
            out[slot] = None
    return out


def _age_line(snap) -> str:
    age = snap.age_s
    updated = getattr(snap, "updated_at", 0.0) or 0.0
    wall_age = (time.time() - updated) if updated else None
    parts = [
        f"grade={snap.grade}",
        f"summary={snap.summary}",
        f"fb_age={age:.3f}s" if age is not None else "fb_age=?",
        f"cache_wall_age={wall_age:.3f}s" if wall_age is not None else "cache_wall_age=?",
        f"fb_hz={snap.fb_hz:.1f}" if snap.fb_hz is not None else "fb_hz=?",
        f"tx_hz={snap.stream_tx_hz:.1f}" if snap.stream_tx_hz is not None else "tx_hz=?",
    ]
    return "  ".join(parts)


def _mcu_line(snap) -> str:
    mcu = snap.mcu_state
    mcu_s = _MCU_STATE_NAMES.get(int(mcu), str(mcu)) if mcu is not None else "?"
    return (
        f"mcu={mcu_s}  block={snap.plant_block_name or snap.plant_block}  "
        f"tick={snap.tick}  ack={snap.ack_seq}  ack_lag={snap.stream_ack_lag}  "
        f"act_lap={snap.lap_ms}ms  periph_lap={snap.periph_lap_ms}ms"
    )


def _pdb_line(snap) -> str:
    pdb = snap.pdb_status or {}
    if not pdb:
        return "pdb: (no kill bytes in cache yet)"
    return (
        f"pdb: kill={pdb.get('kill_state_name', pdb.get('kill_state'))}  "
        f"reason={pdb.get('kill_reason_name', pdb.get('kill_reason'))}  "
        f"estop_sense={pdb.get('estop_sense')}  "
        f"stale_failsafe={pdb.get('stale_failsafe')}"
    )


def _fb_for_slot(snap, slot: int) -> Optional[Dict[str, Any]]:
    acts = list(snap.actuators or [])
    if 0 <= slot < len(acts) and isinstance(acts[slot], dict):
        return acts[slot]
    for a in acts:
        if isinstance(a, dict) and a.get("slot") == slot:
            return a
    return None


def _rail(bus: int) -> str:
    if 1 <= bus <= 3:
        return "fdcan"
    if 4 <= bus <= 6:
        return "mcp"
    return "?"


def _slot_rows(cfg: Dict[str, Any], snap) -> List[str]:
    lines = [
        f"{'slot':>4} {'en':>3} {'bus':>3} {'rail':<5} {'protocol':<10} {'id':>6}  "
        f"{'conn':>5} {'pos':>8} {'vel':>8} {'tau':>8} {'T':>4} {'fault':>5}",
        "-" * 86,
    ]
    shown = 0
    for r in cfg.get("slots") or []:
        slot = int(r["slot"])
        fb = _fb_for_slot(snap, slot)
        if not r.get("enabled") and not fb:
            continue
        shown += 1
        bus = int(r.get("bus", 0))
        conn = slot_connected(r, fb) if r.get("enabled") else None
        lines.append(
            f"{slot:4d} {'Y' if r.get('enabled') else '.':>3} "
            f"{bus:3d} {_rail(bus):<5} "
            f"{str(r.get('protocol_name', '?')):<10} "
            f"{str(r.get('motor_id_hex', '—')):>6}  "
            f"{_yn(conn)} "
            f"{_fmt(None if fb is None else fb.get('position'), 8)} "
            f"{_fmt(None if fb is None else fb.get('velocity'), 8)} "
            f"{_fmt(None if fb is None else fb.get('torque'), 8)} "
            f"{_fmt(None if fb is None else fb.get('temperature'), 4, 0)} "
            f"{_fault_yn(None if fb is None else fb.get('fault'))}"
        )
    if shown == 0:
        lines.append("(no enabled CFG slots / no actuator FB yet)")
    return lines


def _periph_lines(
    cfg: Dict[str, Any],
    *,
    servo_fb: Optional[Dict[int, Optional[Dict[str, Any]]]] = None,
) -> List[str]:
    if not cfg.get("periph_ok", True):
        return [f"periph: GET failed ({cfg.get('periph_error')})"]
    fb_map = servo_fb or {}
    lines = [f"listen_pdu: {'ON' if cfg.get('listen_pdu') else 'off'}"]
    lines.append(
        f"{'slot':>4} {'en':>3} {'model':<14} {'id':>6}  "
        f"{'conn':>5} {'pos':>6} {'src':>4}  limits"
    )
    lines.append("-" * 64)
    shown = 0
    for s in cfg.get("servos") or []:
        shown += 1
        slot = int(s.get("slot", 0))
        mid = int(s.get("model", 0))
        mname = SERVO_MODEL_NAMES.get(mid, str(mid))
        sid = int(s.get("id", 0)) & 0xFF
        fb = fb_map.get(slot)
        conn = servo_connected(s, fb) if s.get("enabled") else None
        pos = None if fb is None else fb.get("present_position")
        src = None if fb is None else fb.get("motor_source_id")
        lines.append(
            f"{slot:4d} {'Y' if s.get('enabled') else '.':>3} "
            f"{mname:<14} "
            f"{f'0x{sid:02X}':>6}  "
            f"{_yn(conn)} "
            f"{_fmt(pos, 6, 0)} "
            f"{('—' if src is None else f'0x{int(src) & 0xFF:02X}'):>4}  "
            f"[{int(s.get('pos_min', 0))}..{int(s.get('pos_max', 0))}]"
        )
    if shown == 0:
        lines.append("(no NVM servo rows)")
    led = cfg.get("led") or {}
    mode = int(led.get("default_mode", 0))
    lines.append(
        f"led NVM: count={led.get('default_count')} "
        f"mode={mode} ({led_mode_name(mode)}) "
        f"bright={led.get('default_brightness')}"
    )
    return lines


def seed_mcp_observe(proxy: HostProxy, cfg: Dict[str, Any]) -> int:
    """Idle-anchor enabled MCP slots + NORMAL so CH4–6 get pararead CAN TX.

    Returns number of MCP slots anchored.
    """
    desires: Dict[int, ActuatorDesire] = {}
    for r in cfg.get("slots") or []:
        if not r.get("enabled"):
            continue
        if int(r.get("bus", 0)) < 4:
            continue
        desires[int(r["slot"])] = ActuatorDesire(position=HOME_POS_EPS)
    if desires:
        proxy.set_actuators(desires, send=False)
    # plant_apply=0 blocks actuator_apply — need apply armed for plant CAN.
    proxy.hub.set_mcu_state(McuState.NORMAL, send=False)
    proxy.hub.set_plant_apply(True, send=True)
    return len(desires)


def seed_servo_observe(proxy: HostProxy, cfg: Dict[str, Any]) -> int:
    """Read-only DXL host session (``torque_enable=False``) for enabled NVM ids.

    After FW fix, this must not torque-on or write goals. Returns armed count.
    """
    armed = 0
    for slot in range(SERVO_COUNT):
        row = None
        for s in cfg.get("servos") or []:
            if int(s.get("slot", -1)) == slot:
                row = s
                break
        sid = 0
        if row is not None and row.get("enabled"):
            sid = int(row.get("id", 0)) & 0xFF
        if sid:
            armed += 1
            proxy.hub.set_servo(
                slot,
                ServoDesire(
                    servo_id=sid,
                    native_step_position=0,
                    torque_enable=False,
                    operating_mode=3,
                ),
                send=False,
            )
        else:
            proxy.hub.set_servo(slot, ServoDesire(servo_id=0), send=False)
    return armed


def sample_servo_fb(
    proxy: HostProxy,
    cfg: Dict[str, Any],
    *,
    settle_s: float = SERVO_SAMPLE_S,
) -> Dict[int, Optional[Dict[str, Any]]]:
    """Brief torque_enable=0 session, snapshot servo FB, clear session."""
    if seed_servo_observe(proxy, cfg) <= 0:
        proxy.hub.clear_servos(send=False)
        return {slot: None for slot in range(SERVO_COUNT)}
    try:
        proxy.hub.send_once()
    except Exception:  # noqa: BLE001
        pass
    time.sleep(max(0.1, float(settle_s)))
    fb = _read_servo_fb(proxy)
    proxy.hub.clear_servos(send=False)
    try:
        proxy.hub.send_once()
    except Exception:  # noqa: BLE001
        pass
    return fb


def render_frame(
    proxy: HostProxy,
    *,
    cfg: Optional[Dict[str, Any]] = None,
    mcp_anchored: int = 0,
    servo_fb: Optional[Dict[int, Optional[Dict[str, Any]]]] = None,
    servo_armed: int = 0,
) -> str:
    hub = proxy.hub
    telem = hub.telemetry
    snap = telem.snapshot()
    if cfg is None:
        try:
            cfg = collect_cfg(proxy)
        except Exception as exc:  # noqa: BLE001
            cfg = {
                "slots": [],
                "enabled_count": 0,
                "total": 0,
                "periph_ok": False,
                "periph_error": str(exc),
            }

    if servo_fb is None:
        servo_fb = _read_servo_fb(proxy)

    persist = getattr(telem, "_persist", None)
    link = connection_summary(cfg, snap)
    neck = servo_connection_summary(cfg, servo_fb)
    body: List[str] = [
        "PCB live dashboard  (Ctrl+C quit)",
        f"port={getattr(hub, 'port', None)}  com=yes  "
        f"state.json={telem.state_path}  persist_telemetry={persist}",
        (
            f"actuators connected={'true' if link['connected'] else 'false'}  "
            f"live={link['live']}/{link['enabled']}  "
            "(conn=yes when plant FB replies for CFG motor_id; 0x00/wrong id → no)"
        ),
        (
            f"servos connected={'true' if neck['connected'] else 'false'}  "
            f"live={neck['live']}/{neck['enabled']}  "
            f"(torque_enable=0 sample ~{SERVO_SAMPLE_S:g}s; "
            "motor_source_id must match CFG id — needs FW that honors torque_enable)"
        ),
        _age_line(snap),
        _mcu_line(snap),
        _pdb_line(snap),
        (
            f"observe: NORMAL + idle-anchor MCP (p={HOME_POS_EPS:g}, kp=0) on "
            f"{mcp_anchored} slot(s); servo read-only sample arms {servo_armed} id(s)"
        ),
        "",
        "connections (CFG) + live FB",
    ]
    body.extend(_slot_rows(cfg, snap))
    body.append("")
    body.append("periph (NVM v2 RAM) + neck FB")
    body.extend(_periph_lines(cfg, servo_fb=servo_fb))
    if snap.context:
        body.append("")
        body.append("context: " + " | ".join(str(c) for c in snap.context[:4]))

    boxed = _box_lines(body)  # wraps + sizes to TTY width (Win/Linux)
    return format_banner() + "\n\n" + "\n".join(boxed) + "\n"


def run_pcb_dashboard(
    proxy: HostProxy,
    *,
    once: bool = False,
    refresh_hz: float = 2.0,
    cfg_refresh_s: float = 2.0,
) -> int:
    """Live TUI. Returns process exit code."""
    period = 1.0 / max(0.2, float(refresh_hz))
    cfg: Optional[Dict[str, Any]] = None
    cfg_next = 0.0
    mcp_anchored = 0
    servo_armed = 0
    servo_fb: Dict[int, Optional[Dict[str, Any]]] = {
        slot: None for slot in range(SERVO_COUNT)
    }
    try:
        try:
            cfg = collect_cfg(proxy)
            servo_fb = sample_servo_fb(proxy, cfg)
            servo_armed = sum(
                1
                for s in (cfg.get("servos") or [])
                if s.get("enabled") and (int(s.get("id", 0)) & 0xFF)
            )
            mcp_anchored = seed_mcp_observe(proxy, cfg)
            time.sleep(0.15)
        except Exception as exc:  # noqa: BLE001
            cfg = {
                "slots": [],
                "enabled_count": 0,
                "total": 0,
                "periph_ok": False,
                "periph_error": str(exc),
            }
        cfg_next = time.monotonic() + float(cfg_refresh_s)

        while True:
            now = time.monotonic()
            if now >= cfg_next:
                try:
                    cfg = collect_cfg(proxy)
                    servo_fb = sample_servo_fb(proxy, cfg)
                    servo_armed = sum(
                        1
                        for s in (cfg.get("servos") or [])
                        if s.get("enabled") and (int(s.get("id", 0)) & 0xFF)
                    )
                    mcp_anchored = seed_mcp_observe(proxy, cfg)
                except Exception as exc:  # noqa: BLE001
                    cfg = {
                        "slots": [],
                        "enabled_count": 0,
                        "total": 0,
                        "periph_ok": False,
                        "periph_error": str(exc),
                    }
                cfg_next = now + float(cfg_refresh_s)
            frame = render_frame(
                proxy,
                cfg=cfg,
                mcp_anchored=mcp_anchored,
                servo_fb=servo_fb,
                servo_armed=servo_armed,
            )
            if once:
                print(frame)
                return 0
            _clear()
            print(frame, end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n(pcb dashboard stopped)", flush=True)
        return 0
