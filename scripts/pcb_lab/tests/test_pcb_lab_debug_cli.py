"""Offline tests for pcb_lab.debug (no COM)."""
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
        ["--port", "COM5", "show", "--cfg", "--bandwidth", "--status", "--json"]
    )
    assert args.port == "COM5"
    assert args.cfg and args.bandwidth and args.status and args.json


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
