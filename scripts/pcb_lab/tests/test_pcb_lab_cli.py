"""Offline tests for pcb_lab CLI / board helpers / link-mode defaults."""
from __future__ import annotations

import io
from contextlib import redirect_stdout

import pytest

from deft_controls_sdk.link.exchange.wire_layout import (
    LINK_MODE_ALIASES,
    STM32_MODE_BANDWIDTH,
    STM32_MODE_DEBUG,
    STM32_MODE_SOFT_DFU,
)
from pcb_lab.board import list_firmware_images, print_factory_defaults, repo_root
from pcb_lab.lab import _build_parser, main


def test_link_mode_aliases_prefer_bandwidth() -> None:
    assert "plant" not in LINK_MODE_ALIASES
    assert LINK_MODE_ALIASES["bandwidth"] == STM32_MODE_BANDWIDTH
    assert LINK_MODE_ALIASES["debug"] == STM32_MODE_DEBUG
    assert LINK_MODE_ALIASES["soft_dfu"] == STM32_MODE_SOFT_DFU


def test_parser_board_subcommands() -> None:
    p = _build_parser()
    for name in (
        "scan",
        "status",
        "leave",
        "flash",
        "images",
        "build",
        "debug",
    ):
        args = p.parse_args([name] if name != "debug" else ["debug", "--", "show"])
        assert args._cmd == name
    args = p.parse_args(["show", "defaults"])
    assert args._cmd == "show"
    assert args.show_what == "defaults"
    args = p.parse_args(["--port", "COM5"])
    assert args.cmd is None
    assert args.port == "COM5"


def test_parser_rejects_peripheral_cmds() -> None:
    p = _build_parser()
    for name in ("inventory", "doctor", "continuous", "hold", "step", "blank", "demux"):
        with pytest.raises(SystemExit):
            p.parse_args([name])


def test_help_exits_zero() -> None:
    try:
        main(["-h"])
    except SystemExit as exc:
        assert exc.code == 0
    else:
        raise AssertionError("expected SystemExit from -h")


def test_show_defaults_prints_scaffold() -> None:
    buf = io.StringIO()
    with redirect_stdout(buf):
        assert print_factory_defaults() == 0
    text = buf.getvalue()
    assert "CH1" in text
    assert "listen_pdu" in text


def test_list_images_shape() -> None:
    rows = list_firmware_images(repo_root())
    assert {r["config"] for r in rows} == {"Release", "Debug"}
    assert all("elf" in r for r in rows)
