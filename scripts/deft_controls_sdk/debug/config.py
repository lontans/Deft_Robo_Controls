"""CFG bench PDU — actuator table get/set/save/load (RAM vs flash).

Semantics (no dirty bit on the wire)::

    GET / GET_PERIPH  → live RAM (not “is persistent?”)
    SET / SET_PERIPH  → RAM only (lost on reboot unless SAVE)
    SAVE              → RAM → flash
    LOAD              → flash → RAM

Ported from scripts/legacy/controls_pcb_host/plugins/plant_config.py. CFG does
not need an RS2/DM session bracket — plant_config_nvm.c answers CFG PDU
requests directly in the main-loop dispatch, so this talks straight to the
Connection (no lease()). Telemetry mode is still published as "cfg" so the
dashboard shows *why* nothing else is happening while a save is retrying.

Flash SAVE is the one part of this that is NOT reliable in practice (legacy's
own retry loop). Requires firmware with the G4 BKER erase fix (otherwise SAVE
returns flash_err).
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence

from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    CFG_OP_GET,
    CFG_OP_GET_PERIPH,
    CFG_OP_LOAD,
    CFG_OP_SAVE,
    CFG_OP_SET,
    CFG_OP_SET_PERIPH,
    CFG_STATUS_OK,
    IMAGE_BYTES,
    PDU_OFF,
    build_cfg_command,
    parse_cfg_feedback,
)

# PROTO_NONE — keep debug.config free of config.actuator import cycles
_PROTO_NONE = 0

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

_DEFAULT_TIMEOUT_S = 1.5


def _parse_cfg_frame(frame: bytes) -> Optional[dict]:
    if len(frame) != IMAGE_BYTES:
        return None
    from deft_controls_sdk.link.exchange.debug_lanes import extract_cfg_mailbox

    pdu = extract_cfg_mailbox(frame)
    parsed = parse_cfg_feedback(pdu)
    if parsed is not None:
        return parsed
    return parse_cfg_feedback(frame[PDU_OFF : PDU_OFF + 32])


def exchange_cfg(
    connection: "Connection",
    op: int,
    *,
    slot: int = 0,
    bus: int = 1,
    protocol: int = 0,
    motor_id: int = 0,
    master_id: int = 0,
    enabled: bool = True,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> dict:
    connection.reader.drain()
    frame = build_cfg_command(
        op,
        connection.next_seq(),
        slot=slot,
        bus=bus,
        protocol=protocol,
        motor_id=motor_id,
        master_id=master_id,
        enabled=enabled,
    )
    resp = connection.exchange_raw(
        frame, _parse_cfg_frame, timeout_s=timeout_s, predicate=lambda p: p["op"] == op
    )
    if resp is None:
        raise TimeoutError("no CFG response from MCU within timeout")
    if resp["status"] != CFG_STATUS_OK:
        raise RuntimeError(f"CFG op={op} failed: {resp['status_name']}")
    return resp


def fetch_table(connection: "Connection", *, timeout_s: float = _DEFAULT_TIMEOUT_S) -> List[dict]:
    """CFG GET, paging through the whole table (dual-arm ACTUATOR_COUNT=14 exceeds
    one PDU's worth of slots per response — see plant_config_feedback_fill())."""
    collected: List[dict] = []
    start = 0
    for _ in range(32):  # safety cap; each page carries a few slots
        resp = exchange_cfg(connection, CFG_OP_GET, slot=start, timeout_s=timeout_s)
        page_slots = resp["slots"]
        if not page_slots:
            break
        collected.extend(page_slots)
        total = resp.get("total_count", len(collected))
        if len(collected) >= total:
            break
        start += len(page_slots)
    return collected


def apply_slot(
    connection: "Connection",
    *,
    slot: int,
    bus: int,
    protocol: int,
    motor_id: int,
    master_id: int = 0,
    enabled: bool = True,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> dict:
    """RAM apply only (survives until reboot, not a flash write). Call save_nvm()
    separately to persist — the two are intentionally not conflated."""
    return exchange_cfg(
        connection,
        CFG_OP_SET,
        slot=slot,
        bus=bus,
        protocol=protocol,
        motor_id=motor_id,
        master_id=master_id,
        enabled=enabled,
        timeout_s=timeout_s,
    )


def save_nvm(connection: "Connection", *, timeout_s: float = 8.0, retries: int = 3) -> dict:
    """Flash NVM save (last flash page @ 0x0807F800). Retries on timeout.

    A raised exception means RAM apply (if any) already succeeded but the
    change will NOT survive reboot. Pre-BKER firmware always failed verify
    with flash_err; current G4 erase path should return status ok."""
    last_err: Optional[Exception] = None
    for attempt in range(retries):
        try:
            return exchange_cfg(connection, CFG_OP_SAVE, timeout_s=timeout_s)
        except (RuntimeError, TimeoutError) as exc:
            last_err = exc
            if attempt + 1 < retries:
                time.sleep(0.25)
    assert last_err is not None
    raise last_err


def load_nvm(connection: "Connection", *, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    """Reload flash NVM into RAM (CFG_OP_LOAD). Wipes unsaved RAM edits.

    GET after this reflects flash (or fails if flash CRC invalid). There is
    still no dirty bit — LOAD is how you force RAM = flash without reboot.
    """
    return exchange_cfg(connection, CFG_OP_LOAD, timeout_s=timeout_s)


def _row_by_slot(
    rows: Sequence[Optional[Mapping[str, Any]]],
) -> Dict[int, Mapping[str, Any]]:
    out: Dict[int, Mapping[str, Any]] = {}
    for i, row in enumerate(rows):
        if row is None:
            continue
        out[int(row.get("slot", i))] = row
    return out


def apply_table(
    connection: "Connection",
    rows: Sequence[Optional[Mapping[str, Any]]],
    *,
    disable_missing: bool = True,
    slot_count: int = ACTUATOR_COUNT,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> List[dict]:
    """RAM-apply a full actuator table description (no flash SAVE).

    For each slot in ``0 .. slot_count-1``: SET from ``rows`` when present;
    if ``disable_missing`` and the slot has no row, SET disabled / PROTO_NONE.
    Use this for blank-then-pack (pass only the slots you want enabled).
    """
    by_slot = _row_by_slot(rows)
    responses: List[dict] = []
    for slot in range(int(slot_count)):
        row = by_slot.get(slot)
        if row is not None:
            responses.append(
                apply_slot(
                    connection,
                    slot=slot,
                    bus=int(row.get("bus", 0)),
                    protocol=int(row.get("protocol", _PROTO_NONE)),
                    motor_id=int(row.get("motor_id", 0)) & 0xFF,
                    master_id=int(row.get("master_id", 0)) & 0xFF,
                    enabled=bool(row.get("enabled", True)),
                    timeout_s=timeout_s,
                )
            )
            continue
        if disable_missing:
            responses.append(
                apply_slot(
                    connection,
                    slot=slot,
                    bus=0,
                    protocol=_PROTO_NONE,
                    motor_id=0,
                    master_id=0,
                    enabled=False,
                    timeout_s=timeout_s,
                )
            )
    return responses


def fetch_bundle(
    connection: "Connection", *, timeout_s: float = _DEFAULT_TIMEOUT_S
) -> Dict[str, Any]:
    """Snapshot live RAM CFG: actuators table + periph block.

    Host-side memory only — no file I/O. Does not indicate flash persistence.
    """
    return {
        "actuators": fetch_table(connection, timeout_s=timeout_s),
        "periph": get_periph(connection, timeout_s=timeout_s),
    }


def apply_bundle(
    connection: "Connection",
    bundle: Mapping[str, Any],
    *,
    telemetry: Optional["TelemetryCache"] = None,
    disable_missing: bool = True,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> Dict[str, Any]:
    """RAM-apply an actuators+periph bundle (no flash SAVE)."""
    actuators = bundle.get("actuators") or []
    slot_resps = apply_table(
        connection,
        actuators,
        disable_missing=disable_missing,
        timeout_s=timeout_s,
    )
    periph = bundle.get("periph")
    periph_resp = None
    if periph is not None:
        periph_resp = set_periph(
            connection,
            telemetry,
            dict(periph),
            persist=False,
            timeout_s=timeout_s,
        )
    return {"actuators": slot_resps, "periph": periph_resp}


def get_periph(connection: "Connection", *, timeout_s: float = _DEFAULT_TIMEOUT_S) -> dict:
    """CFG GET_PERIPH — listen_pdu flags + neck servos + LED defaults."""
    return exchange_cfg(connection, CFG_OP_GET_PERIPH, timeout_s=timeout_s)


def set_periph(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    periph: dict,
    *,
    persist: bool = False,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> dict:
    """CFG SET_PERIPH (RAM); optional flash SAVE of full NVM v2 image."""
    if telemetry is not None:
        telemetry.set_connected(True, mode="cfg")
    try:
        connection.reader.drain()
        frame = build_cfg_command(
            CFG_OP_SET_PERIPH,
            connection.next_seq(),
            periph=periph,
        )
        resp = connection.exchange_raw(
            frame,
            _parse_cfg_frame,
            timeout_s=timeout_s,
            predicate=lambda p: p["op"] == CFG_OP_SET_PERIPH,
        )
        if resp is None:
            raise TimeoutError("no CFG SET_PERIPH response from MCU")
        if resp["status"] != CFG_STATUS_OK:
            raise RuntimeError(f"CFG SET_PERIPH failed: {resp['status_name']}")
        if persist:
            time.sleep(0.05)
            save_nvm(connection)
        return resp
    finally:
        if telemetry is not None:
            telemetry.set_connected(
                True, mode="plant_stream" if connection.is_streaming else "idle"
            )


def set_slot(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    slot: int,
    bus: int,
    protocol: int,
    motor_id: int,
    master_id: int = 0,
    enabled: bool = True,
    persist: bool = False,
    timeout_s: float = _DEFAULT_TIMEOUT_S,
) -> dict:
    """Apply a slot (RAM), optionally persist to flash. Publishes mode="cfg" while
    in flight. Returns the RAM-apply response; raises if persist=True and the
    flash save fails after retries (RAM apply still took effect — see save_nvm)."""
    if telemetry is not None:
        telemetry.set_connected(True, mode="cfg")
    try:
        resp = apply_slot(
            connection,
            slot=slot,
            bus=bus,
            protocol=protocol,
            motor_id=motor_id,
            master_id=master_id,
            enabled=enabled,
            timeout_s=timeout_s,
        )
        if persist:
            time.sleep(0.05)
            save_nvm(connection)
        return resp
    finally:
        if telemetry is not None:
            telemetry.set_connected(True, mode="plant_stream" if connection.is_streaming else "idle")
