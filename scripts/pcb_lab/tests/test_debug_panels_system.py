"""Tests for deft_controls_sdk.debug.panels.system.system_panel — no hardware.

Every panel function is exercised against small duck-typed fakes (no real
COM port, no real ControlsPcbHub), following the fake-hub pattern already
used in test_deft_controls_sdk_dashboard.py.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import deft_controls_sdk.debug.panels.system.system_panel as system_panel
from deft_controls_sdk.debug.panels.system.system_panel import (
    host_link_eval_panel,
    run_bandwidth_panel,
    run_inventory_panel,
    save_cfg_nvm_panel,
    set_cfg_periph_panel,
    set_cfg_slot_panel,
    show_cfg_panel,
)


# -- fakes ------------------------------------------------------------------


@dataclass
class _FakePdb:
    kill_state: int = 0
    kill_state_name: str = "none"
    kill_reason: int = 0
    kill_reason_name: str = "none"
    estop_sense: int = 0
    stale_failsafe: bool = False


class _FakeConnection:
    def __init__(self, *, debug_ok: bool = True) -> None:
        self.debug_ok = debug_ok
        self._stream_hz = 40.0

    def require_debug_mode(self, op: str) -> None:
        if not self.debug_ok:
            raise RuntimeError(f"{op} needs mode='debug'; this session is mode='bandwidth'")


class _FakeDebugAPI:
    def __init__(self) -> None:
        self.table: List[dict] = [
            {"slot": 0, "enabled": True, "bus": 1, "protocol": 1, "motor_id": 0x70},
        ]
        self.periph: Dict[str, Any] = {
            "listen_pdu": False,
            "flags": 0,
            "servos": [],
            "led": {},
        }
        self.calls: List[tuple] = []

    def cfg_get_table(self, *, timeout_s: float = 1.5) -> List[dict]:
        return list(self.table)

    def cfg_get_periph(self, *, timeout_s: float = 1.5) -> dict:
        return dict(self.periph)

    def cfg_set_slot(self, **kwargs: Any) -> dict:
        self.calls.append(("cfg_set_slot", kwargs))
        return {"ok": True, **kwargs}

    def cfg_set_periph(self, periph: dict, *, persist: bool = False, timeout_s: float = 1.5) -> dict:
        self.calls.append(("cfg_set_periph", dict(periph), persist))
        return {"ok": True, "persist": persist}

    def cfg_save_nvm(self, *, timeout_s: float = 8.0) -> dict:
        self.calls.append(("cfg_save_nvm",))
        return {"ok": True, "saved": True}


class _FakeHub:
    def __init__(self, *, debug_ok: bool = True, pdb: Optional[_FakePdb] = None) -> None:
        self._connection = _FakeConnection(debug_ok=debug_ok)
        self.debug = _FakeDebugAPI()
        self.is_streaming = False
        self.listen_pdu = False
        self._pdb = pdb

    def stop_streaming(self) -> None:
        self.is_streaming = False

    def start_streaming(self, hz: float = 40.0) -> None:
        self.is_streaming = True

    def pdb_status(self):
        return self._pdb


class _FakeProxy:
    def __init__(self, hub: _FakeHub) -> None:
        self.hub = hub


# -- run_inventory_panel ------------------------------------------------------


def test_run_inventory_panel_happy() -> None:
    hub = _FakeHub(pdb=_FakePdb(kill_state=0, kill_state_name="none"))
    proxy = _FakeProxy(hub)
    out = run_inventory_panel(
        proxy,
        include_actuators=False,
        include_servos=False,
        include_pdu=True,
        print_report=False,
    )
    assert out["ok"] is True
    assert out["pdu"]["ok"] is True
    assert out["actuators"] is None
    assert out["servos"] is None


def test_run_inventory_panel_error_when_not_debug_mode() -> None:
    hub = _FakeHub(debug_ok=False)
    proxy = _FakeProxy(hub)
    with pytest.raises(RuntimeError, match="debug"):
        run_inventory_panel(
            proxy,
            include_actuators=False,
            include_servos=False,
            include_pdu=True,
            print_report=False,
        )


# -- run_bandwidth_panel -------------------------------------------------------


class _FakeBandwidthHub:
    def __init__(self) -> None:
        self.plant_apply: Optional[bool] = None
        self.rx_sim: Optional[bool] = None

    def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
        self.plant_apply = bool(enable)

    def set_rx_sim(self, enable: bool) -> None:
        self.rx_sim = bool(enable)


class _FakeBandwidthProxy:
    def __init__(self) -> None:
        self.hub = _FakeBandwidthHub()
        self.closed = False

    def __enter__(self) -> "_FakeBandwidthProxy":
        return self

    def __exit__(self, *exc: object) -> None:
        self.closed = True


def test_run_bandwidth_panel_happy(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_proxy = _FakeBandwidthProxy()
    calls: Dict[str, Any] = {}

    def _fake_connect(cls, port, **kwargs):
        calls["port"] = port
        calls["connect_kwargs"] = kwargs
        return fake_proxy

    def _fake_measure_hold(hub, label, desires, **kwargs):
        calls["hub"] = hub
        calls["label"] = label
        calls["desires"] = desires
        calls["measure_kwargs"] = kwargs
        return {"ok": True, "raw_fb_hz": 40.0, "label": label}

    monkeypatch.setattr(system_panel.HostProxy, "connect", classmethod(_fake_connect))
    monkeypatch.setattr(system_panel, "measure_hold", _fake_measure_hold)

    out = run_bandwidth_panel("COM5", hz=40.0, seconds=0.01, slots=[2, 3], print_report=False)

    assert out["ok"] is True
    assert out["slots"] == [2, 3]
    assert out["port"] == "COM5"
    assert out["hz"] == 40.0
    assert calls["port"] == "COM5"
    assert calls["connect_kwargs"]["mode"] == "bandwidth"
    assert set(calls["desires"].keys()) == {2, 3}
    assert fake_proxy.closed is True
    assert fake_proxy.hub.plant_apply is True


def test_run_bandwidth_panel_default_slots_when_none_given(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_proxy = _FakeBandwidthProxy()

    def _fake_connect(cls, port, **kwargs):
        return fake_proxy

    def _fake_measure_hold(hub, label, desires, **kwargs):
        return {"ok": True, "raw_fb_hz": 40.0}

    monkeypatch.setattr(system_panel.HostProxy, "connect", classmethod(_fake_connect))
    monkeypatch.setattr(system_panel, "measure_hold", _fake_measure_hold)

    out = run_bandwidth_panel("COM5", print_report=False)
    assert out["slots"] == [0, 1]


def test_run_bandwidth_panel_connect_error_propagates(monkeypatch: pytest.MonkeyPatch) -> None:
    def _boom_connect(cls, port, **kwargs):
        raise RuntimeError("COM5 already owned")

    monkeypatch.setattr(system_panel.HostProxy, "connect", classmethod(_boom_connect))

    with pytest.raises(RuntimeError, match="already owned"):
        run_bandwidth_panel("COM5", print_report=False)


# -- show_cfg_panel -------------------------------------------------------------


def test_show_cfg_panel_happy() -> None:
    hub = _FakeHub()
    proxy = _FakeProxy(hub)
    out = show_cfg_panel(proxy)
    assert out["periph_ok"] is True
    assert out["total"] == 1
    assert out["enabled_count"] == 1


def test_show_cfg_panel_error_propagates() -> None:
    hub = _FakeHub()

    def _boom(*, timeout_s: float = 1.5):
        raise RuntimeError("cfg get_table timeout")

    hub.debug.cfg_get_table = _boom  # type: ignore[assignment]
    proxy = _FakeProxy(hub)
    with pytest.raises(RuntimeError, match="timeout"):
        show_cfg_panel(proxy)


# -- set_cfg_slot_panel ---------------------------------------------------------


def test_set_cfg_slot_panel_happy() -> None:
    hub = _FakeHub()
    proxy = _FakeProxy(hub)
    out = set_cfg_slot_panel(proxy, slot=2, bus=1, protocol=1, motor_id=0x70)
    assert out["ok"] is True
    assert hub.debug.calls[0][0] == "cfg_set_slot"
    assert hub.debug.calls[0][1]["slot"] == 2
    assert hub.debug.calls[0][1]["persist"] is False


def test_set_cfg_slot_panel_error_propagates() -> None:
    hub = _FakeHub()

    def _boom(**kwargs: Any):
        raise ValueError("bad slot index")

    hub.debug.cfg_set_slot = _boom  # type: ignore[assignment]
    proxy = _FakeProxy(hub)
    with pytest.raises(ValueError, match="bad slot"):
        set_cfg_slot_panel(proxy, slot=99, bus=1, protocol=1, motor_id=1)


# -- set_cfg_periph_panel -------------------------------------------------------


def test_set_cfg_periph_panel_happy() -> None:
    hub = _FakeHub()
    proxy = _FakeProxy(hub)
    out = set_cfg_periph_panel(proxy, {"listen_pdu": True}, persist=True)
    assert out["ok"] is True
    assert out["persist"] is True
    assert hub.debug.calls[0] == ("cfg_set_periph", {"listen_pdu": True}, True)


def test_set_cfg_periph_panel_error_propagates() -> None:
    hub = _FakeHub()

    def _boom(periph, *, persist=False, timeout_s=1.5):
        raise RuntimeError("periph write failed")

    hub.debug.cfg_set_periph = _boom  # type: ignore[assignment]
    proxy = _FakeProxy(hub)
    with pytest.raises(RuntimeError, match="periph write failed"):
        set_cfg_periph_panel(proxy, {})


# -- save_cfg_nvm_panel ----------------------------------------------------------


def test_save_cfg_nvm_panel_happy() -> None:
    hub = _FakeHub()
    proxy = _FakeProxy(hub)
    out = save_cfg_nvm_panel(proxy)
    assert out["ok"] is True
    assert out["saved"] is True
    assert hub.debug.calls[0] == ("cfg_save_nvm",)


def test_save_cfg_nvm_panel_error_propagates() -> None:
    hub = _FakeHub()

    def _boom(*, timeout_s: float = 8.0):
        raise RuntimeError("flash erase failed")

    hub.debug.cfg_save_nvm = _boom  # type: ignore[assignment]
    proxy = _FakeProxy(hub)
    with pytest.raises(RuntimeError, match="flash erase failed"):
        save_cfg_nvm_panel(proxy)


# -- host_link_eval_panel --------------------------------------------------------


def test_host_link_eval_panel_happy() -> None:
    hub = _FakeHub(pdb=_FakePdb(kill_state=1, kill_state_name="soft_kill_req"))
    proxy = _FakeProxy(hub)
    out = host_link_eval_panel(proxy)
    assert out["ok"] is True
    assert out["wire"]["kill_state_name"] == "soft_kill_req"
    assert out["nvm_listen_pdu"] is False


def test_host_link_eval_panel_require_peer_without_wire_reports_not_ok() -> None:
    hub = _FakeHub(pdb=None)
    proxy = _FakeProxy(hub)
    out = host_link_eval_panel(proxy, require_peer=True)
    assert out["ok"] is False
    assert out["peer_ok"] is False
    assert any("no pdb wire bytes yet" in e for e in out["errors"])
