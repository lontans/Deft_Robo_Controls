"""Offline tests: CFG apply_table / load_nvm / profile_in_table."""
from __future__ import annotations

import os
import sys
import warnings

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.actions.cfg_identity import (  # noqa: E402
    profile_in_nvm,
    profile_in_table,
)
from deft_controls_sdk.config import PROTO_ROBSTRIDE, single_profile  # noqa: E402
from deft_controls_sdk.debug import config as cfg_mod  # noqa: E402
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, CFG_OP_LOAD  # noqa: E402


def test_cfg_op_load_constant():
    assert CFG_OP_LOAD == 4


def test_load_nvm_calls_exchange(monkeypatch):
    calls = []

    def fake_exchange(connection, op, **kwargs):
        calls.append(op)
        return {"op": op, "status": 0, "status_name": "ok"}

    monkeypatch.setattr(cfg_mod, "exchange_cfg", fake_exchange)
    out = cfg_mod.load_nvm(object())
    assert calls == [CFG_OP_LOAD]
    assert out["op"] == CFG_OP_LOAD


def test_apply_table_sets_rows_and_disables_missing(monkeypatch):
    applied = []

    def fake_apply_slot(connection, **kwargs):
        applied.append(dict(kwargs))
        return {"ok": True, "slot": kwargs["slot"]}

    monkeypatch.setattr(cfg_mod, "apply_slot", fake_apply_slot)
    rows = [
        {
            "slot": 0,
            "bus": 5,
            "protocol": PROTO_ROBSTRIDE,
            "motor_id": 0x70,
            "master_id": 0,
            "enabled": True,
        },
        {
            "slot": 2,
            "bus": 6,
            "protocol": PROTO_ROBSTRIDE,
            "motor_id": 0x71,
            "enabled": True,
        },
    ]
    resps = cfg_mod.apply_table(object(), rows, disable_missing=True, slot_count=4)
    assert len(resps) == 4
    by_slot = {a["slot"]: a for a in applied}
    assert by_slot[0]["motor_id"] == 0x70 and by_slot[0]["enabled"] is True
    assert by_slot[2]["motor_id"] == 0x71
    assert by_slot[1]["enabled"] is False and by_slot[1]["protocol"] == 0
    assert by_slot[3]["enabled"] is False


def test_apply_table_skip_missing(monkeypatch):
    applied = []

    def fake_apply_slot(connection, **kwargs):
        applied.append(kwargs["slot"])
        return {}

    monkeypatch.setattr(cfg_mod, "apply_slot", fake_apply_slot)
    cfg_mod.apply_table(
        object(),
        [{"slot": 1, "bus": 1, "protocol": 1, "motor_id": 1, "enabled": True}],
        disable_missing=False,
        slot_count=3,
    )
    assert applied == [1]


def test_fetch_and_apply_bundle(monkeypatch):
    monkeypatch.setattr(
        cfg_mod,
        "fetch_table",
        lambda c, **k: [{"slot": 0, "bus": 5, "protocol": 1, "motor_id": 0x70, "enabled": True}],
    )
    monkeypatch.setattr(
        cfg_mod,
        "get_periph",
        lambda c, **k: {"listen_pdu": False, "servos": [], "led": {}},
    )
    bundle = cfg_mod.fetch_bundle(object())
    assert "actuators" in bundle and "periph" in bundle

    applied = {"table": False, "periph": False}

    def fake_table(connection, rows, **kwargs):
        applied["table"] = True
        return []

    def fake_periph(connection, telemetry, periph, **kwargs):
        applied["periph"] = True
        assert kwargs.get("persist") is False
        return {"ok": True}

    monkeypatch.setattr(cfg_mod, "apply_table", fake_table)
    monkeypatch.setattr(cfg_mod, "set_periph", fake_periph)
    cfg_mod.apply_bundle(object(), bundle)
    assert applied["table"] and applied["periph"]


def test_profile_in_table_and_deprecated_alias():
    prof = single_profile(22, protocol="robstride", motor_id=0x70, bus=5)
    table = [None] * 26
    table[22] = {
        "slot": 22,
        "bus": 5,
        "protocol": PROTO_ROBSTRIDE,
        "motor_id": 0x70,
        "master_id": 0,
        "enabled": True,
    }
    assert profile_in_table(prof, table) is True
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        assert profile_in_nvm(prof, table) is True
        assert any(issubclass(x.category, DeprecationWarning) for x in w)


def test_actuator_count_matches_apply_default():
    assert ACTUATOR_COUNT == 26
