"""Offline tests for plant debug suite (pcb_lab.debug alias / SDK suite)."""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from pcb_lab.debug.proto import parse_protocol, protocol_name
from pcb_lab.debug.show import format_cfg_table


def test_parse_protocol_names():
    assert parse_protocol("robstride") == 1
    assert parse_protocol("dm") == 3
    assert parse_protocol("0x4") == 4
    with pytest.raises(ValueError):
        parse_protocol("nope")


def test_format_cfg_table_enabled_only():
    cfg = {
        "enabled_count": 1,
        "total": 2,
        "slots": [
            {
                "slot": 0,
                "enabled": True,
                "bus": 1,
                "protocol_name": "damiao",
                "motor_id_hex": "0x01",
            },
            {
                "slot": 1,
                "enabled": False,
                "bus": 1,
                "protocol_name": "none",
                "motor_id_hex": "0x00",
            },
        ],
    }
    text = format_cfg_table(cfg, only_enabled=True)
    assert "damiao" in text
    assert "none" not in text
    assert protocol_name(1) == "robstride"


def test_debug_parser_scan():
    from pcb_lab.debug.cli import _build_parser

    args = _build_parser().parse_args(["scan"])
    assert args._cmd == "scan"


def test_debug_parser_show_flags():
    from pcb_lab.debug.cli import _build_parser

    args = _build_parser().parse_args(
        ["--port", "COM5", "show", "--cfg", "--bandwidth", "--json"]
    )
    assert args.port == "COM5"
    assert args.cfg and args.bandwidth and args.json
    assert not hasattr(args, "status") or getattr(args, "status", None) in (None, False)


def test_debug_parser_show_rejects_status():
    from pcb_lab.debug.cli import _build_parser

    with pytest.raises(SystemExit):
        _build_parser().parse_args(["show", "--status"])


def test_debug_parser_set_oneshot():
    from pcb_lab.debug.cli import _build_parser

    args = _build_parser().parse_args(
        [
            "set",
            "--cfg",
            "--slot",
            "22",
            "--bus",
            "5",
            "--protocol",
            "robstride",
            "--motor-id",
            "0x70",
            "--persist",
        ]
    )
    assert args.slot == 22
    assert args.persist is True
    assert args.enabled is None


def test_debug_parser_test_bandwidth_knobs():
    from pcb_lab.debug.cli import _build_parser

    args = _build_parser().parse_args(
        [
            "test",
            "--bandwidth",
            "--hz",
            "100",
            "--seconds",
            "1.5",
            "--rx-sim",
            "--slots",
            "0,1,2",
        ]
    )
    assert args._cmd == "test"
    assert args.bandwidth is True
    assert args.hz == 100.0
    assert args.seconds == 1.5
    assert args.rx_sim is True
    assert args.slots == "0,1,2"

    m = _build_parser().parse_args(
        [
            "test",
            "--bandwidth",
            "--matrix",
            "--hz-list",
            "40,200",
            "--trials",
            "2",
            "--scenario",
            "mcp",
        ]
    )
    assert m.matrix is True
    assert m.hz_list == "40,200"
    assert m.trials == 2
    assert m.scenario == "mcp"

    v = _build_parser().parse_args(
        ["test", "--bandwidth", "--virtual", "--matrix"]
    )
    assert v.virtual is True
    assert v.hardware is False
    h = _build_parser().parse_args(
        ["test", "--bandwidth", "--hardware", "--scenario", "mcp"]
    )
    assert h.hardware is True
    assert h.scenario == "mcp"


def test_debug_parser_test_domains_mutex():
    from pcb_lab.debug.cli import _build_parser

    p = _build_parser()
    for flag in ("--actuators", "--led", "--servo", "--pdu-link"):
        args = p.parse_args(["test", flag])
        assert args._cmd == "test"
    with pytest.raises(SystemExit):
        p.parse_args(["test", "--bandwidth", "--actuators"])


def test_debug_parser_test_led_preset():
    from pcb_lab.debug.cli import _build_parser

    args = _build_parser().parse_args(
        ["test", "--led", "--preset", "pdu", "--hold-s", "0.5"]
    )
    assert args.led is True
    assert args.preset == "pdu"
    assert args.hold_s == 0.5


def test_parse_slot_list_optional():
    from deft_controls_sdk.debug.suite.test_cmd import parse_slot_list_optional

    assert parse_slot_list_optional(None) is None
    assert parse_slot_list_optional("") is None
    assert list(parse_slot_list_optional("0,1,2")) == [0, 1, 2]
    assert list(parse_slot_list_optional("3 4 5")) == [3, 4, 5]
    with pytest.raises(ValueError):
        parse_slot_list_optional("0,0")
    with pytest.raises(ValueError):
        parse_slot_list_optional("99")


def test_plant_apply_wire_bit():
    from deft_controls_sdk.link.api_types import CommandImage, McuState
    from deft_controls_sdk.link.exchange.pack import patch_system_plant_apply
    from deft_controls_sdk.link.exchange.wire_layout import (
        PLANT_APPLY_SHIFT,
        SYSTEM_CMD_OFF,
    )
    import struct

    img = CommandImage(seq=1, mcu_state=McuState.NORMAL, plant_apply=True)
    word = struct.unpack_from("<I", img.to_bytes(), SYSTEM_CMD_OFF)[0]
    assert (word >> PLANT_APPLY_SHIFT) & 1 == 1
    img.set_plant_apply(False)
    word = struct.unpack_from("<I", img.to_bytes(), SYSTEM_CMD_OFF)[0]
    assert (word >> PLANT_APPLY_SHIFT) & 1 == 0
    buf = bytearray(img.to_bytes())
    patch_system_plant_apply(buf, True)
    word = struct.unpack_from("<I", buf, SYSTEM_CMD_OFF)[0]
    assert (word >> PLANT_APPLY_SHIFT) & 1 == 1


def test_actuators_hex_ids():
    from deft_controls_sdk.debug.suite.test_actuators import _hex_id, _hex_ids

    assert _hex_id(112) == "0x70"
    assert _hex_id(116) == "0x74"
    assert _hex_ids([0x70, 0x74]) == ["0x70", "0x74"]


def test_actuators_parse_bus_and_protocol_queue():
    from deft_controls_sdk.debug.discover import parse_bus_list, parse_protocol_queue

    assert parse_bus_list("1,5") == [1, 5]
    assert parse_bus_list("all") == [1, 2, 3, 4, 5, 6]
    assert parse_bus_list("5 5 1") == [5, 1]
    with pytest.raises(ValueError):
        parse_bus_list("7")

    assert parse_protocol_queue("robstride") == ["robstride"]
    assert parse_protocol_queue("damiao,robstride") == ["damiao", "robstride"]
    assert parse_protocol_queue("all") == ["robstride", "damiao"]
    assert parse_protocol_queue("rs,dm") == ["robstride", "damiao"]
    with pytest.raises(ValueError):
        parse_protocol_queue("zeroerr")


def test_discover_queued_order_and_summary(monkeypatch):
    """RobStride multi-bus collapses to one by_bus call; Damiao stays per-bus."""
    from deft_controls_sdk.debug import discover as discover_mod
    from deft_controls_sdk.debug import robstride as robstride_mod

    rs_calls: list[tuple] = []
    dm_calls: list[tuple] = []

    def fake_rs_by_bus(connection, telemetry, *, buses, start, end):
        rs_calls.append((tuple(buses), start, end))
        return {5: [0x70], 1: []}

    def fake_one(connection, telemetry, *, protocol, bus, start, end, listen_ms):
        dm_calls.append((protocol, bus, start, end))
        return []

    monkeypatch.setattr(robstride_mod, "discover_all_by_bus", fake_rs_by_bus)
    monkeypatch.setattr(discover_mod, "_robstride", robstride_mod)
    monkeypatch.setattr(discover_mod, "_discover_one", fake_one)
    results = discover_mod.discover_queued(
        object(),  # type: ignore[arg-type]
        None,
        buses=[5, 1],
        protocols=["robstride", "damiao"],
        ranges={"robstride": (0x70, 0x80), "damiao": (1, 8)},
    )
    # discover_all_by_bus sorts buses; call is still one multi-bus lease.
    assert len(rs_calls) == 1
    assert set(rs_calls[0][0]) == {1, 5}
    assert rs_calls[0][1:] == (0x70, 0x80)
    assert [(c[0], c[1]) for c in dm_calls] == [("damiao", 5), ("damiao", 1)]
    summary = discover_mod.summarize_queued(results)
    assert summary["hit_count"] == 1
    assert any(r.get("ids") == ["0x70"] for r in summary["results"])


def test_suite_test_modules_avoid_vbeta_imports():
    """Guard: suite test_* must not pull product vbeta helpers."""
    import ast
    from pathlib import Path

    suite = Path(_SCRIPTS) / "deft_controls_sdk" / "debug" / "suite"
    offenders = []
    for path in sorted(suite.glob("test_*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name.startswith("deft_controls_sdk.vbeta") or alias.name == "vbeta":
                        offenders.append(f"{path.name}: import {alias.name}")
            elif isinstance(node, ast.ImportFrom):
                mod = node.module or ""
                if mod.startswith("deft_controls_sdk.vbeta") or mod.startswith("vbeta"):
                    offenders.append(f"{path.name}: from {mod}")
    assert offenders == []
