"""Offline tests for plant debug suite (pcb_lab.debug alias / SDK suite)."""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug.suite.proto import parse_protocol, protocol_name
from deft_controls_sdk.debug.suite.show import format_cfg_table


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


def test_debug_parser_rejects_scan():
    """Board USB scan lives on ``python -m pcb_lab scan``, not debug suite."""
    from deft_controls_sdk.debug.suite.cli import _build_parser

    with pytest.raises(SystemExit):
        _build_parser().parse_args(["scan"])


def test_debug_parser_show_flags():
    from deft_controls_sdk.debug.suite.cli import _build_parser

    args = _build_parser().parse_args(
        ["--port", "COM5", "show", "--cfg", "--bandwidth", "--json"]
    )
    assert args.port == "COM5"
    assert args.cfg and args.bandwidth and args.json
    assert not hasattr(args, "status") or getattr(args, "status", None) in (None, False)


def test_debug_parser_show_rejects_status():
    from deft_controls_sdk.debug.suite.cli import _build_parser

    with pytest.raises(SystemExit):
        _build_parser().parse_args(["show", "--status"])


def test_debug_parser_set_oneshot():
    from deft_controls_sdk.debug.suite.cli import _build_parser

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
    from deft_controls_sdk.debug.suite.cli import _build_parser

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
    from deft_controls_sdk.debug.suite.cli import _build_parser

    p = _build_parser()
    for flag in (
        "--inventory",
        "--actuators",
        "--led",
        "--servos",
        "--servo",
        "--pdu-link",
    ):
        args = p.parse_args(["test", flag])
        assert args._cmd == "test"
    with pytest.raises(SystemExit):
        p.parse_args(["test", "--bandwidth", "--actuators"])
    with pytest.raises(SystemExit):
        p.parse_args(["test", "--inventory", "--actuators"])


def test_bare_test_and_peripheral_scopes(monkeypatch):
    """Bare ``test`` → board; flags open filtered peripheral menus."""
    from deft_controls_sdk.debug.suite import test_cmd as tc

    called = {}

    def _fake_board(args):
        called["board"] = True
        return 0

    def _fake_act(args):
        called["actuators"] = True
        return 0

    def _fake_servos(args):
        called["servos"] = True
        return 0

    def _fake_led(args):
        called["led"] = True
        return 0

    monkeypatch.setattr(tc, "_run_board_verify", _fake_board)
    monkeypatch.setattr(tc, "_run_actuators", _fake_act)
    monkeypatch.setattr(tc, "_run_servos", _fake_servos)
    monkeypatch.setattr(tc, "_run_led", _fake_led)

    from deft_controls_sdk.debug.suite.cli import _build_parser

    bare = _build_parser().parse_args(["test", "--assembly", "bench"])
    assert tc._domain_from_args(bare) is None
    assert tc.run_test(bare) == 0
    assert called.get("board") is True

    act = _build_parser().parse_args(["test", "--actuators"])
    assert tc._domain_from_args(act) == "actuators"
    assert tc.run_test(act) == 0
    assert called.get("actuators") is True

    srv = _build_parser().parse_args(["test", "--servos"])
    assert tc._domain_from_args(srv) == "servos"
    assert tc.run_test(srv) == 0
    assert called.get("servos") is True

    # --servo is an alias of --servos
    srv2 = _build_parser().parse_args(["test", "--servo"])
    assert tc._domain_from_args(srv2) == "servos"

    led = _build_parser().parse_args(["test", "--led"])
    assert tc._domain_from_args(led) == "led"
    assert tc.run_test(led) == 0
    assert called.get("led") is True


def test_debug_parser_test_led_preset():
    from deft_controls_sdk.debug.suite.cli import _build_parser

    args = _build_parser().parse_args(
        ["test", "--led", "--led-preset", "pdu", "--hold-s", "0.5"]
    )
    assert args.led is True
    assert args.led_preset == "pdu"
    assert args.hold_s == 0.5


def test_debug_parser_test_inventory():
    from deft_controls_sdk.debug.suite.cli import _build_parser

    args = _build_parser().parse_args(
        ["test", "--inventory", "--preset", "bench", "--buses", "5,6", "--no-tui"]
    )
    assert args.inventory is True
    assert args.preset == "bench"
    assert args.buses == "5,6"
    assert args.no_tui is True


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


def test_actuators_resolve_motion_target_and_hold():
    """Plant motion helpers use ActuatorAction kinds without needing COM."""
    from types import SimpleNamespace

    from deft_controls_sdk.config import (
        DEFAULT_WHEEL_KP,
        LEFT_ARM_SLOTS,
        PROTO_ROBSTRIDE,
        yam_product_assembly,
        yam_product_profile,
    )
    from deft_controls_sdk.debug.suite import test_actuators as ta
    from deft_controls_sdk.link import ActuatorDesire

    class _Conn:
        def __init__(self) -> None:
            self.actuators = {}

        def set_actuators(self, desires, *, send=False):
            self.actuators.update(desires)

        def poll_feedback(self):
            return None

    class _Hub:
        def __init__(self) -> None:
            from deft_controls_sdk.link import McuState

            self._connection = _Conn()
            self._plant_apply = False
            self._mcu_state = McuState.NORMAL

        def set_plant_apply(self, enable, *, send=False):
            self._plant_apply = bool(enable)

        def set_mcu_state(self, state, *, send=False):
            self._mcu_state = state

        def send_once(self):
            pass

        def set_actuators(self, desires, *, send=False):
            self._connection.set_actuators(desires, send=send)

        def latest_feedback(self):
            return None

    hub = _Hub()
    asm = yam_product_assembly()
    proxy = SimpleNamespace(
        profile=yam_product_profile(),
        hub=hub,
        set_actuators=hub.set_actuators,
        latest_feedback=hub.latest_feedback,
        send_once=hub.send_once,
    )

    arm = ta.resolve_motion_target(proxy, "left_arm", assembly=asm)  # type: ignore[arg-type]
    assert arm.slots == LEFT_ARM_SLOTS
    assert arm.actuator_profile is not None

    wheel = ta.resolve_motion_target(proxy, "22")  # type: ignore[arg-type]
    assert wheel.slots == (22,)

    single = ta.resolve_motion_target(
        proxy, "single:22:robstride:0x70:5"
    )  # type: ignore[arg-type]
    assert single.actuator_profile is not None
    assert single.actuator_profile.as_cfg_row()["protocol"] == PROTO_ROBSTRIDE

    ta.apply_hold(proxy, wheel, [0.3], hold_s=0.0, kp=DEFAULT_WHEEL_KP)  # type: ignore[arg-type]
    assert hub._plant_apply is True
    assert hub._connection.actuators[22].position == pytest.approx(0.3)
    assert hub._connection.actuators[22].kp == pytest.approx(DEFAULT_WHEEL_KP)

    ta.apply_blank(proxy, wheel)  # type: ignore[arg-type]
    assert hub._connection.actuators[22].kp == pytest.approx(0.0)
    assert isinstance(hub._connection.actuators[22], ActuatorDesire)


def test_actuators_parse_bus_and_protocol_queue():
    from deft_controls_sdk.debug.discover import parse_bus_list, parse_protocol_queue

    assert parse_bus_list("1,5") == [1, 5]
    assert parse_bus_list("all") == [1, 2, 3, 4, 5, 6]
    assert parse_bus_list("5 5 1") == [5, 1]
    with pytest.raises(ValueError):
        parse_bus_list("7")

    assert parse_protocol_queue("robstride") == ["robstride"]
    assert parse_protocol_queue("damiao,robstride") == ["damiao", "robstride"]
    assert parse_protocol_queue("all") == ["robstride", "damiao", "cubemars", "zeroerr"]
    assert parse_protocol_queue("rs,dm") == ["robstride", "damiao"]
    assert parse_protocol_queue("zeroerr") == ["zeroerr"]
    assert parse_protocol_queue("cm") == ["cubemars"]
    with pytest.raises(ValueError):
        parse_protocol_queue("nope")


def test_as_hex_formats_discover_ids():
    from deft_controls_sdk.debug import as_hex, hex_id

    assert hex_id(112) == "0x70"
    assert as_hex([112, 116, 117]) == ["0x70", "0x74", "0x75"]
    assert as_hex({5: [112, 116], 6: [117]}) == {
        5: ["0x70", "0x74"],
        6: ["0x75"],
    }


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


def test_suite_modules_avoid_vbeta_imports():
    """Guard: debug.suite must not import removed nested vbeta package."""
    import ast
    from pathlib import Path

    suite = Path(_SCRIPTS) / "deft_controls_sdk" / "debug" / "suite"
    offenders = []
    for path in sorted(suite.glob("*.py")):
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
