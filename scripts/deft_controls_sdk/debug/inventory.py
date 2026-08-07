"""Board hardware inventory smoke — actuators + servos + PDU.

``mode=debug`` only. Actuator discovery uses DEBUG lanes (not plant CMDH).
**ID ranges are required** for actuator probes — wide defaults are too slow
(RobStride enable+promisc per ID). Use a preset, explicit ``ranges=``, or the TUI.

Automation (AI / scripts)::

    hub.debug.inventory(preset="bench", buses=(5, 6))
    hub.debug.inventory(ranges={"robstride": (0x70, 0x75), "damiao": (1, 8)})
    hub.debug.inventory(include_actuators=False)  # servos + PDU only
"""
from __future__ import annotations

import json
import time
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .discover import (
    DEFAULT_ID_RANGE,
    discover_queued,
    parse_bus_list,
    parse_protocol_queue,
    summarize_queued,
)

if TYPE_CHECKING:
    from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
    from deft_controls_sdk.host_proxy import HostProxy
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

# Named presets — never silently use discover's wide DEFAULT_ID_RANGE.
# CubeMars/ZeroErr follow the same bench/product/full shape as Damiao (no
# established bench-vs-product ID convention of their own yet) — this just
# means --preset alone now covers all 4 protocols instead of only RS/DM;
# pass --cm-range/--ze-range (or ranges=…) to override per protocol.
RANGE_PRESETS: Dict[str, Dict[str, Tuple[int, int]]] = {
    "bench": {
        "robstride": (0x70, 0x75),
        "damiao": (1, 8),
        "cubemars": (1, 8),
        "zeroerr": (1, 8),
    },
    "product": {
        "robstride": (0x01, 0x02),
        "damiao": (1, 7),
        "cubemars": (1, 7),
        "zeroerr": (1, 7),
    },
    "yam": {  # alias of product
        "robstride": (0x01, 0x02),
        "damiao": (1, 7),
        "cubemars": (1, 7),
        "zeroerr": (1, 7),
    },
    "full": {
        "robstride": (0x01, 0x7F),
        "damiao": (1, 16),
        "cubemars": (1, 16),
        "zeroerr": (1, 16),
    },
}

PRESET_BLURBS: Dict[str, str] = {
    "bench": "RS 0x70..0x75 + DM/CM/ZE 1..8 (bench continuous IDs)",
    "product": "RS 0x01..0x02 + DM/CM/ZE 1..7 (YAM product map)",
    "yam": "alias of product",
    "full": "RS 0x01..0x7F + DM/CM/ZE 1..16 — SLOW; opt-in only",
}


def parse_id_range(text: str, *, protocol: str = "robstride") -> Tuple[int, int]:
    """Parse ``0x70-0x75`` / ``0x70:0x75`` / ``112 117`` / ``1-8``."""
    raw = (text or "").strip().lower().replace(":", "-").replace(",", " ")
    if not raw:
        raise ValueError("empty id range")
    if "-" in raw:
        left, right = raw.split("-", 1)
        start, end = int(left.strip(), 0), int(right.strip(), 0)
    else:
        toks = raw.split()
        if len(toks) != 2:
            raise ValueError(f"id range needs start-end, got {text!r}")
        start, end = int(toks[0], 0), int(toks[1], 0)
    if start > end:
        start, end = end, start
    if protocol == "robstride":
        start = max(0, min(0x7F, start))
        end = max(0, min(0x7F, end))
    else:
        start = max(1, min(0xFF, start))
        end = max(1, min(0xFF, end))
    return start, end


def resolve_ranges(
    *,
    ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
    preset: Optional[str] = None,
    protocols: Sequence[str] = ("robstride", "damiao"),
) -> Dict[str, Tuple[int, int]]:
    """Build per-protocol ranges. Raises if a requested protocol has no range."""
    out: Dict[str, Tuple[int, int]] = {}
    if preset:
        key = preset.strip().lower()
        if key not in RANGE_PRESETS:
            known = ", ".join(sorted(RANGE_PRESETS))
            raise ValueError(f"unknown inventory preset {preset!r}; known: {known}")
        out.update(RANGE_PRESETS[key])
    if ranges:
        out.update({k: (int(v[0]), int(v[1])) for k, v in ranges.items()})
    missing = [p for p in protocols if p not in out]
    if missing:
        raise ValueError(
            "actuator inventory needs ID ranges for: "
            + ", ".join(missing)
            + ". Pass ranges=…, preset='bench'|'product'|'full', or use the TUI "
            "(python -m pcb_lab.debug test --inventory). Wide defaults are intentionally not used."
        )
    return {p: out[p] for p in protocols}


def _fmt_id(protocol: str, n: int) -> str:
    """RobStride IDs as 0xNN; Damiao as decimal (bench habit) with 0x in paren."""
    v = int(n) & 0xFF
    if protocol == "robstride":
        return f"0x{v:02X}"
    return f"{v} (0x{v:02X})"


def format_ranges_for_display(
    ranges: Mapping[str, Tuple[int, int]],
) -> Dict[str, str]:
    """Human/JSON-friendly range map, e.g. ``{\"robstride\": \"0x70-0x75\"}``."""
    out: Dict[str, str] = {}
    for proto, (start, end) in ranges.items():
        a, b = _fmt_id(proto, start), _fmt_id(proto, end)
        # Damiao already embeds 0x; keep compact start-end
        if proto == "robstride":
            out[proto] = f"{a}-{b}"
        else:
            out[proto] = f"{int(start)}-{int(end)} (0x{int(start) & 0xFF:02X}-0x{int(end) & 0xFF:02X})"
    return out


def _collect_servos(proxy: "HostProxy") -> dict:
    from deft_controls_sdk.debug.suite.pcb_tui import (
        sample_servo_fb,
        servo_connected,
        servo_connection_summary,
    )
    from deft_controls_sdk.debug.snapshot import collect_cfg

    try:
        cfg = collect_cfg(proxy)
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": str(exc), "rows": []}

    if not cfg.get("periph_ok", True):
        return {
            "ok": False,
            "error": cfg.get("periph_error") or "periph GET failed",
            "rows": [],
        }

    servo_fb = sample_servo_fb(proxy, cfg)
    summary = servo_connection_summary(cfg, servo_fb)
    rows = []
    for s in cfg.get("servos") or []:
        slot = int(s.get("slot", 0))
        fb = servo_fb.get(slot)
        enabled = bool(s.get("enabled"))
        sid = int(s.get("id", 0)) & 0xFF
        rows.append(
            {
                "slot": slot,
                "enabled": enabled,
                "id": sid,
                "model": int(s.get("model", 0)),
                "connected": (
                    servo_connected(s, fb) if enabled else None
                ),
                "present_position": None if fb is None else fb.get("present_position"),
                "motor_source_id": None if fb is None else fb.get("motor_source_id"),
            }
        )
    return {
        "ok": True,
        "enabled": int(summary.get("enabled", 0)),
        "live": int(summary.get("live", 0)),
        "connected": bool(summary.get("connected")),
        "rows": rows,
    }


def _collect_pdu(
    hub: "ControlsPcbHub",
    *,
    require_peer: bool = False,
) -> dict:
    time.sleep(0.35)
    nvm_listen: Optional[bool] = None
    nvm_error: Optional[str] = None
    try:
        periph = dict(hub.debug.cfg_get_periph())
        nvm_listen = bool(periph.get("listen_pdu", False))
    except Exception as exc:  # noqa: BLE001
        nvm_error = str(exc)

    wire: Optional[Dict[str, Any]] = None
    wire_error: Optional[str] = None
    try:
        pdb = hub.pdb_status()
        if pdb is not None:
            wire = {
                "ok": True,
                "kill_state": int(pdb.kill_state),
                "kill_state_name": pdb.kill_state_name,
                "kill_reason": int(pdb.kill_reason),
                "kill_reason_name": pdb.kill_reason_name,
                "estop_sense": int(pdb.estop_sense),
                "stale_failsafe": bool(pdb.stale_failsafe),
            }
    except Exception as exc:  # noqa: BLE001
        wire_error = str(exc)

    errors: List[str] = []
    ok = True
    if nvm_error:
        ok = False
        errors.append(f"cfg_get_periph: {nvm_error}")
    if wire_error:
        ok = False
        errors.append(f"pdb_status: {wire_error}")
    peer_ok: Optional[bool] = None
    if require_peer:
        if wire is None:
            ok = False
            peer_ok = False
            errors.append("no pdb wire bytes yet (--peer requires peer)")
        elif wire.get("stale_failsafe"):
            ok = False
            peer_ok = False
            errors.append("pdb wire stale_failsafe=True")
        else:
            peer_ok = True
    elif wire is not None:
        peer_ok = not bool(wire.get("stale_failsafe"))

    return {
        "ok": ok,
        "host_listen_pdu": bool(hub.listen_pdu),
        "nvm_listen_pdu": nvm_listen,
        "wire": wire,
        "peer_ok": peer_ok,
        "require_peer": bool(require_peer),
        "errors": errors,
        "note": (
            "PDU presence via plant kill-byte mirror (pdb_status); "
            "not a separate handshake opcode"
        ),
    }


def run_inventory(
    proxy: "HostProxy",
    *,
    buses: Sequence[int] = (1, 2, 3, 4, 5, 6),
    protocols: Sequence[str] = ("robstride", "damiao"),
    ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
    preset: Optional[str] = None,
    include_actuators: bool = True,
    include_servos: bool = True,
    include_pdu: bool = True,
    require_pdu_peer: bool = False,
    listen_ms: int = 40,
    print_report: bool = True,
) -> dict:
    """Full board inventory. Prefer this for automation over the TUI.

    Raises ``ValueError`` if actuators are included without ranges/preset.
    """
    hub = proxy.hub
    hub._connection.require_debug_mode("inventory")  # noqa: SLF001

    out: Dict[str, Any] = {
        "ok": True,
        "mode": "debug",
        "actuators": None,
        "servos": None,
        "pdu": None,
    }

    if include_actuators:
        resolved = resolve_ranges(ranges=ranges, preset=preset, protocols=protocols)
        if preset and str(preset).strip().lower() == "full" and print_report:
            print(
                "WARNING: preset=full sweeps a wide ID space — expect minutes on MCP.",
                flush=True,
            )
        ranges_disp = format_ranges_for_display(resolved)
        if print_report:
            print(
                f"inventory actuators  buses={list(buses)}  "
                f"protocols={list(protocols)}  ranges={ranges_disp}  "
                f"(DEBUG lanes)",
                flush=True,
            )
        results = discover_queued(
            hub._connection,  # noqa: SLF001
            hub.telemetry,
            buses=buses,
            protocols=protocols,
            ranges=resolved,
            listen_ms=listen_ms,
        )
        actuators = summarize_queued(results)
        actuators["ranges"] = {k: [v[0], v[1]] for k, v in resolved.items()}
        actuators["ranges_display"] = ranges_disp
        actuators["preset"] = preset
        if print_report:
            for row in results:
                bus = row.get("bus")
                proto = row.get("protocol")
                if not row.get("ok"):
                    print(f"  FAIL  bus={bus}  {proto}: {row.get('error')}")
                    continue
                ids = row.get("ids") or []
                if ids:
                    print(f"  HIT   bus={bus}  {proto}: {', '.join(ids)}")
                else:
                    print(f"  miss  bus={bus}  {proto}")
        out["actuators"] = actuators
        if not all(r.get("ok") for r in results):
            out["ok"] = False

    if include_servos:
        if print_report:
            print("inventory servos  (CFG periph + plant FB sample)", flush=True)
        servos = _collect_servos(proxy)
        out["servos"] = servos
        if not servos.get("ok"):
            out["ok"] = False
        elif print_report:
            print(
                f"  servos enabled={servos.get('enabled')}  "
                f"live={servos.get('live')}  connected={servos.get('connected')}"
            )

    if include_pdu:
        if print_report:
            print(
                f"inventory pdu  host_listen_pdu={hub.listen_pdu}  "
                f"require_peer={require_pdu_peer}",
                flush=True,
            )
        pdu = _collect_pdu(hub, require_peer=require_pdu_peer)
        out["pdu"] = pdu
        if not pdu.get("ok"):
            out["ok"] = False
        elif print_report:
            wire = pdu.get("wire")
            if wire:
                print(
                    f"  pdu wire  kill={wire.get('kill_state_name')}  "
                    f"stale={wire.get('stale_failsafe')}  peer_ok={pdu.get('peer_ok')}"
                )
            else:
                print("  pdu wire  (no bytes yet)")

    out["note"] = (
        "inventory = actuators (DEBUG discover lanes) + servos (FB) + PDU (pdb_status)"
    )
    if print_report:
        print(json.dumps(out, indent=2, default=str))
    return out


def inventory_smoke(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    buses: Sequence[int] = (1, 2, 3, 4, 5, 6),
    protocols: Sequence[str] = ("robstride", "damiao"),
    ranges: Optional[Mapping[str, Tuple[int, int]]] = None,
    preset: Optional[str] = None,
    listen_ms: int = 40,
    print_report: bool = True,
) -> dict:
    """Actuators-only inventory (Connection API). Prefer :func:`run_inventory`.

    Requires ``ranges`` or ``preset`` — no silent wide sweep.
    """
    resolved = resolve_ranges(ranges=ranges, preset=preset, protocols=protocols)
    ranges_disp = format_ranges_for_display(resolved)
    if print_report:
        print(
            f"inventory smoke  buses={list(buses)}  protocols={list(protocols)}  "
            f"ranges={ranges_disp}  (DEBUG lanes / mode=debug)"
        )
    results = discover_queued(
        connection,
        telemetry,
        buses=buses,
        protocols=protocols,
        ranges=resolved,
        listen_ms=listen_ms,
    )
    summary = summarize_queued(results)
    summary["ranges"] = {k: [v[0], v[1]] for k, v in resolved.items()}
    summary["ranges_display"] = ranges_disp
    summary["preset"] = preset
    summary["note"] = (
        "actuators-only; use hub.debug.inventory / run_inventory for servos+PDU"
    )
    if print_report:
        for row in results:
            bus = row.get("bus")
            proto = row.get("protocol")
            if not row.get("ok"):
                print(f"  FAIL  bus={bus}  {proto}: {row.get('error')}")
                continue
            ids = row.get("ids") or []
            if ids:
                print(f"  HIT   bus={bus}  {proto}: {', '.join(ids)}")
            else:
                print(f"  miss  bus={bus}  {proto}")
        print(json.dumps(summary, indent=2))
    return summary


# Re-export parsers for CLI / AI helpers
__all__ = [
    "RANGE_PRESETS",
    "PRESET_BLURBS",
    "DEFAULT_ID_RANGE",
    "parse_id_range",
    "parse_bus_list",
    "parse_protocol_queue",
    "resolve_ranges",
    "format_ranges_for_display",
    "run_inventory",
    "inventory_smoke",
]
