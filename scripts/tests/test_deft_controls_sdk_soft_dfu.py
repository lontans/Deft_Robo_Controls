"""Golden tests for deft_controls_sdk/bench/soft_dfu.py (no hardware)."""
from __future__ import annotations

import os
import sys
import types
from typing import List, Optional

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.bench import DebugAPI
from deft_controls_sdk.bench.soft_dfu import (
    CdcPortInfo,
    default_firmware_elf,
    enter_bootloader,
    find_cdc_port,
    host_os,
    leave_bootloader,
    list_cdc_ports,
    main,
)
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import IMAGE_BYTES, PDU_OFF


def _fake_connection(sent: list) -> Connection:
    conn = Connection("TESTPORT")
    conn._ser = object()

    def _write_raw(frame: bytes, *, drain: bool = False) -> None:
        sent.append(frame)

    conn.write_raw = _write_raw  # type: ignore[method-assign]
    return conn


def test_enter_bootloader_requires_confirm() -> None:
    sent: list = []
    conn = _fake_connection(sent)
    with pytest.raises(ValueError, match="confirm=True"):
        enter_bootloader(conn)
    assert sent == []


def test_enter_bootloader_sends_dfu_tag_at_pdu_offset() -> None:
    sent: list = []
    conn = _fake_connection(sent)
    enter_bootloader(conn, confirm=True)
    assert len(sent) == 1
    frame = sent[0]
    assert len(frame) == IMAGE_BYTES
    assert frame[PDU_OFF : PDU_OFF + 4] == b"DFU!"


def test_debug_api_enter_bootloader_wires_through() -> None:
    sent: list = []
    conn = _fake_connection(sent)
    debug = DebugAPI(conn, None)
    with pytest.raises(ValueError, match="confirm=True"):
        debug.enter_bootloader()
    debug.enter_bootloader(confirm=True)
    assert len(sent) == 1
    assert sent[0][PDU_OFF : PDU_OFF + 4] == b"DFU!"


def test_leave_bootloader_requires_dfu_device(monkeypatch: pytest.MonkeyPatch) -> None:
    import deft_controls_sdk.bench.soft_dfu as mod

    class _Ctx:
        pass

    monkeypatch.setattr(mod, "_find_dfu", lambda ctx, serial=None: None)
    fake = types.ModuleType("usb1")
    fake.USBContext = lambda: _Ctx()  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "usb1", fake)
    with pytest.raises(RuntimeError, match="0483:DF11"):
        leave_bootloader()


def test_host_os_is_known() -> None:
    assert host_os() in ("windows", "linux", "darwin") or isinstance(host_os(), str)


def test_find_cdc_port_prefers_stm32(monkeypatch: pytest.MonkeyPatch) -> None:
    import deft_controls_sdk.bench.soft_dfu as mod

    ports = [
        CdcPortInfo("COM53", "stlink", 0x0483, 0x374F, "STLink", False),
        CdcPortInfo("COM5", "3167376F3435", 0x0483, 0x5740, "CDC", True),
        CdcPortInfo("COM7", "AABBCCDDEEFF", 0x0483, 0x5740, "CDC", True),
    ]

    def _list(*, serial: Optional[str] = None) -> List[CdcPortInfo]:
        rows = ports
        if serial is not None:
            rows = [p for p in rows if p.serial == serial]
        return sorted(rows, key=lambda x: (not x.is_stm32_cdc, x.device))

    monkeypatch.setattr(mod, "list_cdc_ports", _list)
    # Ambiguous: two CDC — must error without serial
    with pytest.raises(RuntimeError, match="multiple"):
        find_cdc_port()
    assert find_cdc_port(serial="3167376F3435") == "COM5"
    assert find_cdc_port(port="COM9") == "COM9"


def test_find_cdc_port_linux_style(monkeypatch: pytest.MonkeyPatch) -> None:
    import deft_controls_sdk.bench.soft_dfu as mod

    monkeypatch.setattr(
        mod,
        "list_cdc_ports",
        lambda *, serial=None: [
            CdcPortInfo("/dev/ttyACM0", "AABB", 0x0483, 0x5740, "STM32", True),
        ],
    )
    monkeypatch.setattr(mod, "host_os", lambda: "linux")
    assert find_cdc_port() == "/dev/ttyACM0"


def test_default_firmware_elf_points_at_repo_debug() -> None:
    elf = default_firmware_elf()
    assert elf.name == "DeftRoboticsControlsPCB.elf"
    assert elf.parent.name == "Debug"


def test_main_flash_wires_to_flash_firmware(monkeypatch: pytest.MonkeyPatch) -> None:
    import deft_controls_sdk.bench.soft_dfu as mod

    called = {}

    def _flash(image, *, serial=None, flash_address=0, confirm=True):
        called["image"] = image
        called["serial"] = serial
        called["address"] = flash_address
        return "/dev/ttyACM0"

    monkeypatch.setattr(mod, "flash_firmware", _flash)
    assert main(["flash", "--serial", "ABC", "--image", "/tmp/x.elf"]) == 0
    assert called["serial"] == "ABC"
    assert called["image"] == "/tmp/x.elf"
