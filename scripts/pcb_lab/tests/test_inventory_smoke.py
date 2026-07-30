"""Offline tests for inventory ranges + actuators smoke (DEBUG lanes)."""
from __future__ import annotations

import os
import struct
import sys

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.debug import DebugAPI
from deft_controls_sdk.debug.inventory import (
    parse_id_range,
    resolve_ranges,
    run_inventory,
)
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import (
    DM_PROBE_ID_SWEEP,
    DM_RESP_TAG,
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    PROBE_ENABLE_ONLY,
    PROBE_PROMISC,
    RS2_RESP_TAG,
    SESSION_BEGIN,
    SESSION_END,
    STM32_MODE_DEBUG,
    build_rs2_scan_command,
    debug_lanes_header_present,
    extract_dm_mailbox,
    extract_rs2_mailbox,
)


def _fake_connection(responder) -> Connection:
    conn = Connection("TESTPORT", stm32_mode=STM32_MODE_DEBUG)
    conn._ser = object()

    def _write_raw(frame: bytes, *, drain: bool = False) -> None:
        reply = responder(frame)
        if reply is not None:
            conn._reader.feed(reply)

    conn.write_raw = _write_raw  # type: ignore[method-assign]
    return conn


def _blank_feedback() -> bytearray:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_DEBUG_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    return buf


def test_resolve_ranges_requires_explicit_or_preset() -> None:
    with pytest.raises(ValueError, match="needs ID ranges"):
        resolve_ranges(protocols=("robstride",))
    r = resolve_ranges(preset="bench", protocols=("robstride", "damiao"))
    assert r["robstride"] == (0x70, 0x75)
    assert r["damiao"] == (1, 8)


def test_parse_id_range() -> None:
    assert parse_id_range("0x70-0x75") == (0x70, 0x75)
    assert parse_id_range("1-8", protocol="damiao") == (1, 8)


def test_format_ranges_shows_hex_for_robstride() -> None:
    from deft_controls_sdk.debug.inventory import format_ranges_for_display

    disp = format_ranges_for_display({"robstride": (0x70, 0x75), "damiao": (1, 8)})
    assert disp["robstride"] == "0x70-0x75"
    assert "0x01" in disp["damiao"] and "0x08" in disp["damiao"]


def test_inventory_requires_ranges() -> None:
    conn = _fake_connection(lambda _f: None)
    debug = DebugAPI(conn, None)
    with pytest.raises(ValueError, match="needs ID ranges"):
        debug.inventory(buses=(1,), print_report=False)


def test_inventory_finds_rs_and_dm_on_single_bus() -> None:
    """Single-bus path — RS + DM on CH1 with tight ranges."""
    rs_id = 0x70
    dm_id = 0x03

    def responder(frame: bytes):
        rs = extract_rs2_mailbox(frame)
        dm = extract_dm_mailbox(frame)
        resp = _blank_feedback()
        rpdu = bytearray(32)

        if rs[:3] == b"RS2":
            motor_id, kind = rs[3], rs[4]
            rpdu[0] = RS2_RESP_TAG
            rpdu[1] = motor_id
            rpdu[25] = kind
            if kind in (SESSION_BEGIN, SESSION_END):
                pass
            elif kind in (PROBE_ENABLE_ONLY, PROBE_PROMISC) and motor_id == rs_id:
                rpdu[2] = 1
                rpdu[24] = motor_id
                rpdu[27] = 1
            resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
            return bytes(resp)

        if dm[:3] == b"DM0":
            start_id, kind, end_id = dm[3], dm[4], dm[8]
            rpdu[0] = DM_RESP_TAG
            rpdu[1] = start_id
            rpdu[3] = kind
            if kind == DM_PROBE_ID_SWEEP and start_id <= dm_id <= end_id:
                rpdu[2] = 1
                rpdu[1] = dm_id
                rpdu[24] = dm_id
            resp[PDU_OFF : PDU_OFF + 32] = bytes(rpdu)
            return bytes(resp)

        return None

    conn = _fake_connection(responder)
    debug = DebugAPI(conn, None)
    summary = debug.inventory(
        buses=(1,),
        protocols=("robstride", "damiao"),
        preset="bench",
        ranges={"robstride": (0x70, 0x70), "damiao": (1, 8)},
        print_report=False,
    )
    assert summary["hit_count"] == 2
    by = {(r["bus"], r["protocol"]): r["ids"] for r in summary["results"] if r.get("ok")}
    assert by[(1, "robstride")] == ["0x70"]
    assert by[(1, "damiao")] == ["0x03"]
    assert DebugAPI.smoke is DebugAPI.inventory


def test_builders_default_to_debug_lanes() -> None:
    frame = build_rs2_scan_command(0x70, SESSION_BEGIN, seq=1, bus=1)
    assert debug_lanes_header_present(frame)


def test_inventory_cli_parser_has_preset() -> None:
    from pcb_lab.lab import _build_parser

    args = _build_parser().parse_args(
        ["inventory", "--preset", "bench", "--buses", "5,6", "--no-tui"]
    )
    assert args._cmd == "inventory"
    assert args.preset == "bench"
    assert args.buses == "5,6"
    assert args.no_tui is True


def test_run_inventory_importable() -> None:
    assert callable(run_inventory)
