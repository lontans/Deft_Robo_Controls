"""Interactive CFG editor (stdlib only — no rich/textual dependency).

Commands (at the prompt)::

    <n>           edit actuator slot n
    t <n>         toggle enabled on slot n
    neck / led / listen   edit NVM v2 periph fields
    a             apply dirty slots + dirty periph to RAM (no flash)
    p / persist   apply dirty + CFG SAVE to NVM v2
    r             reload table + periph from board
    s / se        show table (+ enabled-only)
    q             quit (warns if dirty)

``preset`` — arm/torso/base/neck/led catalogue with custom slot maps.
"""
from __future__ import annotations

import sys
from typing import Any, Dict, List, Set

from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER, led_mode_name
from deft_controls_sdk.link.exchange import CFG_FLAG_LISTEN_PDU
from deft_controls_sdk.vbeta.cfg import pause_plant_stream
from deft_controls_sdk.vbeta.slots import (
    PROTO_DAMIAO,
    SERVO_MODEL_NAMES,
    SERVO_MODEL_XL330_M288,
    SERVO_MODEL_XL430,
)

from .presets import (
    apply_gen2_all,
    run_component_preset,
    run_preset_dialogue,
)
from .proto import parse_protocol, protocol_name
from .show import format_cfg_table


def _row_key(row: Dict[str, Any]) -> tuple:
    return (
        bool(row.get("enabled")),
        int(row.get("bus", 0)),
        int(row.get("protocol", 0)),
        int(row.get("motor_id", 0)),
        int(row.get("master_id", 0)),
    )


def _load_rows(proxy: HostProxy) -> List[Dict[str, Any]]:
    with pause_plant_stream(proxy.hub):
        table = proxy.hub.debug.cfg_get_table()
    rows: List[Dict[str, Any]] = []
    for i, raw in enumerate(table):
        if raw is None:
            continue
        slot = int(raw.get("slot", i))
        proto = int(raw.get("protocol", 0))
        rows.append(
            {
                "slot": slot,
                "enabled": bool(raw.get("enabled", False)),
                "bus": int(raw.get("bus", 0)),
                "protocol": proto,
                "motor_id": int(raw.get("motor_id", 0)),
                "master_id": int(raw.get("master_id", 0xFF if proto == PROTO_DAMIAO else 0)),
            }
        )
    rows.sort(key=lambda r: r["slot"])
    return rows


def _load_periph(proxy: HostProxy) -> Dict[str, Any]:
    with pause_plant_stream(proxy.hub):
        return dict(proxy.hub.debug.cfg_get_periph())


def _cfg_view(
    rows: List[Dict[str, Any]],
    dirty: Set[int],
    periph: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "slots": [
            {
                **r,
                "protocol_name": protocol_name(r["protocol"])
                + ("*" if r["slot"] in dirty else ""),
                "motor_id_hex": f"0x{r['motor_id'] & 0xFF:02X}",
            }
            for r in rows
        ],
        "enabled_count": sum(1 for r in rows if r["enabled"]),
        "total": len(rows),
        "periph_ok": True,
        "listen_pdu": bool(periph.get("listen_pdu", False)),
        "flags": int(periph.get("flags", 0)),
        "servos": list(periph.get("servos") or []),
        "led": dict(periph.get("led") or {}),
    }


def _print_table(
    rows: List[Dict[str, Any]],
    dirty: Set[int],
    periph: Dict[str, Any],
    *,
    periph_dirty: bool = False,
    only_enabled: bool = False,
) -> None:
    print(
        format_cfg_table(
            _cfg_view(rows, dirty, periph),
            only_enabled=only_enabled,
            banner=True,
            box=True,
        )
    )
    if dirty:
        print(f"dirty slots (RAM not applied): {sorted(dirty)}")
    if periph_dirty:
        print("dirty periph (neck/LED/listen_pdu — RAM not applied)")
    print(
        "cmds: <n> edit | t <n> toggle | arm | torso | base | neck | led | "
        "listen | preset | gen_2 | a apply | p persist | r reload | s/se | q"
    )


def _prompt(msg: str, default: str) -> str:
    raw = input(f"{msg} [{default}]: ").strip()
    return default if raw == "" else raw


def _edit_slot(row: Dict[str, Any]) -> Dict[str, Any]:
    print(
        f"\nEditing slot {row['slot']}  "
        f"en={row['enabled']} bus={row['bus']} "
        f"{protocol_name(row['protocol'])} id=0x{row['motor_id'] & 0xFF:02X}"
    )
    en_s = _prompt("enabled (y/n)", "y" if row["enabled"] else "n").lower()
    row["enabled"] = en_s in ("y", "yes", "1", "true", "on")
    row["bus"] = int(_prompt("bus (1-6)", str(row["bus"])), 0)
    proto_s = _prompt(
        "protocol (none|robstride|cubemars|damiao|zeroerr|int)",
        protocol_name(row["protocol"]),
    )
    row["protocol"] = parse_protocol(proto_s)
    mid_s = _prompt("motor_id (hex ok)", f"0x{row['motor_id'] & 0xFF:02X}")
    row["motor_id"] = int(mid_s, 0) & 0xFF
    if row["protocol"] == PROTO_DAMIAO:
        m_s = _prompt("master_id (0xFF=auto)", f"0x{row['master_id'] & 0xFF:02X}")
        row["master_id"] = int(m_s, 0) & 0xFF
    else:
        row["master_id"] = 0
    return row


def _ensure_servos(periph: Dict[str, Any]) -> List[dict]:
    servos = list(periph.get("servos") or [])
    while len(servos) < 2:
        servos.append(
            {
                "slot": len(servos),
                "model": 0,
                "id": 0,
                "enabled": False,
                "pos_min": 0,
                "pos_max": 4095,
            }
        )
    periph["servos"] = servos
    return servos


def _parse_servo_model(raw: str, default: int) -> int:
    key = (raw or "").strip().lower().replace(" ", "")
    if not key:
        return int(default) & 0xFF
    aliases = {
        "0": SERVO_MODEL_XL430,
        "xl430": SERVO_MODEL_XL430,
        "1": SERVO_MODEL_XL330_M288,
        "xl330": SERVO_MODEL_XL330_M288,
        "xl330-m288-t": SERVO_MODEL_XL330_M288,
        "xl330_m288": SERVO_MODEL_XL330_M288,
        "xl330m288t": SERVO_MODEL_XL330_M288,
    }
    if key in aliases:
        return aliases[key]
    return int(key, 0) & 0xFF


def _edit_neck(periph: Dict[str, Any]) -> None:
    servos = _ensure_servos(periph)
    for i, label in enumerate(("pitch", "yaw")):
        s = servos[i]
        mid = int(s.get("model", 0))
        mname = SERVO_MODEL_NAMES.get(mid, str(mid))
        print(
            f"\nNeck {label} (servo slot {i}): "
            f"model={mname} id=0x{int(s.get('id', 0)) & 0xFF:02X} "
            f"en={bool(s.get('enabled'))} "
            f"[{int(s.get('pos_min', 0))}..{int(s.get('pos_max', 0))}]"
        )
        en_s = _prompt("enabled (y/n)", "y" if s.get("enabled") else "n").lower()
        s["enabled"] = en_s in ("y", "yes", "1", "true", "on")
        s["model"] = _parse_servo_model(
            _prompt("model (XL430|XL330-M288-T|int)", mname), mid
        )
        s["id"] = int(_prompt("id (hex ok)", f"0x{int(s.get('id', 0)) & 0xFF:02X}"), 0) & 0xFF
        s["pos_min"] = int(_prompt("pos_min", str(int(s.get("pos_min", 0)))), 0) & 0xFFFF
        s["pos_max"] = int(_prompt("pos_max", str(int(s.get("pos_max", 0)))), 0) & 0xFFFF
        s["slot"] = i
        servos[i] = s
    periph["servos"] = servos


def _edit_led(periph: Dict[str, Any]) -> None:
    led = dict(periph.get("led") or {})
    mode = int(led.get("default_mode", LED_MODE_IDLE_CORNFLOWER))
    print(
        f"\nLED defaults: count={led.get('default_count', 300)} "
        f"mode={mode} ({led_mode_name(mode)}) "
        f"brightness={led.get('default_brightness', 8)}"
    )
    led["default_count"] = int(
        _prompt("default_count", str(int(led.get("default_count", 300)))), 0
    ) & 0xFFFF
    led["default_mode"] = int(
        _prompt("default_mode (0..8)", str(mode)), 0
    ) & 0x1F
    led["default_brightness"] = int(
        _prompt("default_brightness (0..31)", str(int(led.get("default_brightness", 8)))),
        0,
    ) & 0x1F
    periph["led"] = led


def _edit_listen(proxy: HostProxy, periph: Dict[str, Any]) -> None:
    from deft_controls_sdk.link import LedDesire

    cur = bool(periph.get("listen_pdu", False))
    print(
        "\nlisten_pdu NVM bit — MCU LED traffic-light when host LedDesire is "
        "follow/pdu (wire mode 0). ON + no PDU peer => blink-red estop."
    )
    ans = _prompt("listen_pdu (y/n)", "y" if cur else "n").lower()
    on = ans in ("y", "yes", "1", "true", "on")
    periph["listen_pdu"] = on
    flags = int(periph.get("flags", 0)) & 0xFF
    if on:
        flags |= CFG_FLAG_LISTEN_PDU
    else:
        flags &= ~CFG_FLAG_LISTEN_PDU
    periph["flags"] = flags
    # Drop any host debug pattern so MCU policy can win immediately.
    proxy.listen_pdu = on
    proxy.set_led(LedDesire(mode="follow", master_brightness=8), send=True)
    print(f"  host LedDesire -> follow; listen_pdu={on}")


def _apply_slots(
    proxy: HostProxy,
    rows: List[Dict[str, Any]],
    slots: Set[int],
) -> None:
    by_slot = {r["slot"]: r for r in rows}
    with pause_plant_stream(proxy.hub):
        for slot in sorted(slots):
            r = by_slot[slot]
            proxy.hub.debug.cfg_set_slot(
                slot=int(r["slot"]),
                bus=int(r["bus"]),
                protocol=int(r["protocol"]),
                motor_id=int(r["motor_id"]),
                master_id=int(r["master_id"]),
                enabled=bool(r["enabled"]),
                persist=False,
            )
            print(
                f"  RAM set slot {slot}: bus={r['bus']} "
                f"{protocol_name(r['protocol'])} "
                f"id=0x{r['motor_id'] & 0xFF:02X} en={r['enabled']}"
            )


def _apply_periph(proxy: HostProxy, periph: Dict[str, Any]) -> None:
    from deft_controls_sdk.link import LedDesire

    with pause_plant_stream(proxy.hub):
        proxy.hub.debug.cfg_set_periph(periph, persist=False)
    listen = bool(periph.get("listen_pdu", False))
    proxy.listen_pdu = listen
    # Ensure stream is not pinning a debug pattern over PDU policy.
    proxy.set_led(LedDesire(mode="follow", master_brightness=8), send=True)
    led = periph.get("led") or {}
    print(
        f"  RAM set periph: listen_pdu={listen} "
        f"led mode={led.get('default_mode')} bright={led.get('default_brightness')} "
        f"(host LedDesire follow)"
    )


def run_cfg_editor(proxy: HostProxy) -> int:
    """Blocking interactive editor. Returns process exit code."""
    if not sys.stdin.isatty():
        print(
            "set --cfg interactive needs a TTY.\n"
            "Use non-interactive:\n"
            "  python -m deft_controls_sdk.debug.suite set --cfg --slot N --bus B "
            "--protocol NAME --motor-id 0x.. [--persist]\n"
            "  python -m deft_controls_sdk.debug.suite set --stream 200\n"
            "  python -m deft_controls_sdk.debug.suite set --led-mode 8",
            file=sys.stderr,
        )
        return 2

    rows = _load_rows(proxy)
    baseline = {r["slot"]: _row_key(r) for r in rows}
    dirty: Set[int] = set()
    try:
        periph = _load_periph(proxy)
    except Exception as exc:  # noqa: BLE001
        print(f"warn: CFG GET_PERIPH failed ({exc}); neck/LED need FW NVM v2")
        periph = {
            "flags": 0,
            "listen_pdu": False,
            "servos": [
                {"slot": 0, "model": 0, "id": 1, "enabled": True, "pos_min": 1024, "pos_max": 3072},
                {"slot": 1, "model": 0, "id": 2, "enabled": True, "pos_min": 700, "pos_max": 2500},
            ],
            "led": {"default_count": 300, "default_mode": 8, "default_brightness": 8},
        }
    periph_dirty = [False]

    print("CFG editor — edits are local until apply (a) or persist (p).")
    print("NVM v2 SAVE writes actuators + neck + LED + listen_pdu together.")
    _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])

    while True:
        try:
            line = input("cfg> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            line = "q"

        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            if dirty or periph_dirty[0]:
                n = len(dirty) + (1 if periph_dirty[0] else 0)
                ans = input(f"{n} dirty item(s) not applied — quit anyway? [y/N]: ")
                if ans.strip().lower() not in ("y", "yes"):
                    continue
            return 0

        if cmd in ("s", "show"):
            _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            continue
        if cmd == "se":
            _print_table(
                rows, dirty, periph, periph_dirty=periph_dirty[0], only_enabled=True
            )
            continue

        if cmd in ("r", "reload"):
            rows = _load_rows(proxy)
            baseline = {r["slot"]: _row_key(r) for r in rows}
            dirty.clear()
            try:
                periph = _load_periph(proxy)
                periph_dirty[0] = False
            except Exception as exc:  # noqa: BLE001
                print(f"reload periph failed: {exc}", file=sys.stderr)
            print("reloaded from board")
            _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            continue

        if cmd in ("a", "apply"):
            if not dirty and not periph_dirty[0]:
                print("nothing dirty")
                continue
            try:
                if dirty:
                    _apply_slots(proxy, rows, set(dirty))
                    for s in dirty:
                        baseline[s] = _row_key(next(r for r in rows if r["slot"] == s))
                    dirty.clear()
                if periph_dirty[0]:
                    _apply_periph(proxy, periph)
                    periph_dirty[0] = False
            except Exception as exc:  # noqa: BLE001
                print(f"apply failed: {exc}", file=sys.stderr)
            continue

        if cmd in ("p", "persist", "save"):
            try:
                if dirty:
                    _apply_slots(proxy, rows, set(dirty))
                    for s in dirty:
                        baseline[s] = _row_key(next(r for r in rows if r["slot"] == s))
                    dirty.clear()
                if periph_dirty[0]:
                    _apply_periph(proxy, periph)
                    periph_dirty[0] = False
                with pause_plant_stream(proxy.hub):
                    proxy.hub.debug.cfg_save_nvm()
                print("NVM SAVE ok (full v2 image)")
            except Exception as exc:  # noqa: BLE001
                print(f"persist failed: {exc}", file=sys.stderr)
            continue

        if cmd in ("t", "toggle") and len(parts) >= 2:
            try:
                slot = int(parts[1], 0)
            except ValueError:
                print("usage: t <slot>")
                continue
            for r in rows:
                if r["slot"] == slot:
                    r["enabled"] = not r["enabled"]
                    if _row_key(r) != baseline.get(slot):
                        dirty.add(slot)
                    else:
                        dirty.discard(slot)
                    print(f"slot {slot} enabled={r['enabled']}")
                    break
            else:
                print(f"no slot {slot}")
            continue

        if cmd in ("arm", "torso", "base"):
            msg = run_component_preset(
                proxy, rows, dirty, periph, periph_dirty, cmd
            )
            if msg:
                print(msg)
                _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            continue

        if cmd == "neck":
            # bare ``neck`` -> edit fields; ``neck`` preset via ``preset`` / name
            if len(parts) >= 2 and parts[1].lower() in ("preset", "p"):
                msg = run_component_preset(
                    proxy, rows, dirty, periph, periph_dirty, "neck"
                )
                if msg:
                    print(msg)
                    _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            else:
                _edit_neck(periph)
                periph_dirty[0] = True
                _print_table(rows, dirty, periph, periph_dirty=True)
            continue

        if cmd == "led":
            if len(parts) >= 2 and parts[1].lower() in ("preset", "p"):
                msg = run_component_preset(
                    proxy, rows, dirty, periph, periph_dirty, "led"
                )
                if msg:
                    print(msg)
                    _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            else:
                _edit_led(periph)
                periph_dirty[0] = True
                _print_table(rows, dirty, periph, periph_dirty=True)
            continue

        if cmd in ("listen", "listen_pdu", "pdu"):
            _edit_listen(proxy, periph)
            periph_dirty[0] = True
            _print_table(rows, dirty, periph, periph_dirty=True)
            continue

        if cmd == "preset":
            msg = run_preset_dialogue(proxy, rows, dirty, periph, periph_dirty)
            if msg:
                print(msg)
                _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            continue

        if cmd in ("gen_2", "gen2"):
            msg = apply_gen2_all(proxy, rows, dirty, periph, periph_dirty)
            print(msg)
            _print_table(rows, dirty, periph, periph_dirty=periph_dirty[0])
            continue

        try:
            slot = int(cmd, 0)
        except ValueError:
            print(f"unknown cmd {line!r}")
            continue

        for r in rows:
            if r["slot"] == slot:
                before = _row_key(r)
                _edit_slot(r)
                if _row_key(r) != baseline.get(slot):
                    dirty.add(slot)
                else:
                    dirty.discard(slot)
                if _row_key(r) != before:
                    pass
                break
        else:
            print(f"no slot {slot}")
