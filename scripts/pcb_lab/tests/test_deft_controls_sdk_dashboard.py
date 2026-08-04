"""Golden tests for the localhost controller dashboard (no hardware).

Runs the real ThreadingHTTPServer on an OS-assigned free port and drives it
with plain urllib — this exercises the actual HTTP routing/JSON contract,
not just AppState in isolation. No serial port is ever opened; every
"connected" scenario uses AppState with no hub, which is exactly what the
disconnected-error-handling paths need to prove.
"""
from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.debug_dashboard.app import AppState, serve
from deft_controls_sdk.host_proxy import HostProxy
from deft_controls_sdk.link import McuState


@pytest.fixture()
def server(tmp_path):
    state = AppState(session_dir=str(tmp_path), stream_hz=50.0)
    httpd = serve(state, http_port=0)
    _, port = httpd.server_address
    time.sleep(0.05)
    try:
        yield state, f"http://127.0.0.1:{port}"
    finally:
        httpd.shutdown()


def _get(base: str, path: str):
    with urllib.request.urlopen(base + path) as r:
        return r.status, json.loads(r.read())


def _post(base: str, path: str, body=None):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(base + path, data=data, method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _install_fake_proxy_connect(monkeypatch, fake):
    """Patch AppState's HostProxy.connect to wrap ``fake`` (no real COM).

    Mimics ``HostProxy.connect(..., armed=False)`` side effects the dashboard
    and these tests assert on (stream + plant_apply off).
    """
    import deft_controls_sdk.debug_dashboard.app as app_module

    if not hasattr(fake, "send_once"):
        fake.send_once = lambda: None  # type: ignore[attr-defined]
    if not hasattr(fake, "set_led"):
        fake.set_led = lambda *a, **k: None  # type: ignore[attr-defined]
    if not hasattr(fake, "stop_streaming"):
        fake.stop_streaming = lambda: None  # type: ignore[attr-defined]
    if not hasattr(fake, "plant_apply"):
        fake.plant_apply = False  # type: ignore[attr-defined]

    def _connect(cls, port, **kwargs):
        if hasattr(fake, "start_streaming"):
            fake.start_streaming(
                hz=float(kwargs.get("stream_hz", 50.0)),
                telemetry_hz=float(kwargs.get("telemetry_hz", 10.0)),
                auto_soft_kill=False,
            )
        if hasattr(fake, "set_mcu_state"):
            fake.set_mcu_state(McuState.NORMAL, send=False)
        if hasattr(fake, "set_plant_apply"):
            fake.set_plant_apply(False, send=True)
            fake.plant_apply = False
        proxy = HostProxy(
            fake,
            owns_hub=True,
            listen_pdu=False,
            telemetry_hz=float(kwargs.get("telemetry_hz", 10.0)),
        )
        proxy._stream_hz = float(kwargs.get("stream_hz", 50.0))
        return proxy

    monkeypatch.setattr(app_module.HostProxy, "connect", classmethod(_connect))
    return fake


def test_index_serves_html(server) -> None:
    _state, base = server
    with urllib.request.urlopen(base + "/") as r:
        assert r.status == 200
        body = r.read().decode("utf-8")
    assert "<title>" in body
    assert "portSelect" in body  # connection card present
    assert "Plant control" in body  # control card present


def test_state_before_any_connect_is_well_formed(server) -> None:
    _state, base = server
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["connected"] is False
    assert len(data["actuators"]) == 26  # wire slot count even with no hub yet
    assert all(a is None for a in data["actuators"])
    # Plant control clarity fields (held-desire / streaming) must also be
    # well-formed with no hub yet, not just the feedback side.
    assert data["streaming"] is False
    assert data["held"] == [None] * 26
    # Idle must not look like a board fault (was grade=red / "disconnected").
    assert data["grade"] == "idle"
    assert data["control_mode"] == "idle"
    assert "not connected" in (data.get("summary") or "").lower()


def test_held_state_reflects_active_vs_idle_commands(server, monkeypatch) -> None:
    """/api/state must distinguish an actively-held non-zero command from an
    idle-hold zero desire from a never-commanded slot — that distinction is
    the whole point of the "active plant control state" clarity ask."""
    from deft_controls_sdk.link import ActuatorDesire

    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def __init__(self) -> None:
            self._held = {0: ActuatorDesire(position=1.0, kp=8.0, kd=0.4), 3: ActuatorDesire()}
            self.auto_soft_kill = None
            self.mcu_states = []
            self.plant_apply_flags = []

        def held_desire(self, slot: int):
            return self._held.get(slot)

        def held_desires(self):
            return dict(self._held)

        def start_streaming(
            self, hz: float = 50.0, *, telemetry_hz: float = 10.0, auto_soft_kill: bool = True
        ) -> None:
            self.auto_soft_kill = auto_soft_kill

        def set_auto_soft_kill(self, enabled: bool) -> None:
            self.auto_soft_kill = enabled

        def set_mcu_state(self, state, *, send: bool = True) -> None:
            self.mcu_states.append(int(state))

        def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
            self.plant_apply_flags.append(bool(enable))

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            self._held[slot] = desire

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)

    state, base = server
    state.connect("COM5")  # observe default
    assert fake.auto_soft_kill is False
    assert fake.mcu_states and int(fake.mcu_states[-1]) == 0  # NORMAL
    assert fake.plant_apply_flags and fake.plant_apply_flags[-1] is False
    assert state.control_mode == "observe"
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["streaming"] is True
    assert data["control_mode"] == "observe"
    assert data["held"][0] == {
        "position": 1.0,
        "velocity": 0.0,
        "kp": 8.0,
        "kd": 0.4,
        "torque": 0.0,
        "active": True,
    }
    assert data["held"][3]["active"] is False  # all-zero desire — idle-hold, not "active"
    assert data["held"][1] is None  # never commanded, distinct from idle-hold


def test_state_exposes_pdb_status_from_telemetry(server) -> None:
    """/api/state must surface TelemetryCache's pdb_status verbatim — this is
    the PDU telemetry strip's only data source (see debug_dashboard/app.py
    tick() -> s.pdb_status)."""
    state, base = server
    state.telemetry.set_connected(True, port="COM5")
    state.telemetry.update_from_feedback(
        tick=1, ack_seq=0, mcu_state=3, plant_block=0, plant_block_name="none",
        pdu_tag="S", lap_ms=1, lap_max_ms=1, ticks_pending=0, svd_present=True,
        actuators=[],
        pdb_status={
            "kill_state": 1,
            "kill_state_name": "soft_kill_req",
            "estop_sense": 1,
            "pdb": {"kill_state": 1, "estop_sense": 1, "contactor_state": 3},
        },
    )
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["pdb_status"]["kill_state_name"] == "soft_kill_req"
    assert data["pdb_status"]["pdb"]["contactor_state"] == 3
    assert data["mcu_state"] == 3  # dashboard derives "host requested ESTOP" from this


def test_soft_kill_park_without_connection_writes_peer_request(server) -> None:
    """Follow mode: Soft-kill Park signals the CDC owner via a flag file."""
    state, base = server
    flag = state.soft_kill_request_path()
    if flag.is_file():
        flag.unlink()
    status, data = _post(base, "/api/pdb/soft_kill_park")
    assert status == 200
    assert data.get("ok") is True
    assert data.get("mode") == "peer_request"
    assert flag.is_file()
    flag.unlink(missing_ok=True)


def test_soft_kill_park_calls_hub_when_connected(server, monkeypatch) -> None:
    class _FakeHub:
        port = "COM5"
        is_streaming = True
        parked = False

        def held_desires(self):
            return {}

        def start_streaming(
            self, hz: float = 50.0, *, telemetry_hz: float = 10.0, auto_soft_kill: bool = True
        ) -> None:
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def set_mcu_state(self, state, *, send: bool = True) -> None:
            pass

        def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
            pass

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def soft_kill_park(self) -> None:
            self.parked = True

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)

    state, base = server
    state.connect("COM5")
    status, data = _post(base, "/api/pdb/soft_kill_park")
    assert status == 200
    assert data.get("ok") is True
    assert data.get("mode") == "direct"
    assert fake.parked is True


def test_observe_blocks_plant_commands_until_enable_control(server, monkeypatch) -> None:
    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def __init__(self) -> None:
            self.auto_soft_kill = True
            self.mcu = None

        def held_desires(self):
            return {}

        def start_streaming(
            self, hz: float = 50.0, *, telemetry_hz: float = 10.0, auto_soft_kill: bool = True
        ) -> None:
            self.auto_soft_kill = auto_soft_kill

        def set_auto_soft_kill(self, enabled: bool) -> None:
            self.auto_soft_kill = enabled

        def set_mcu_state(self, state, *, send: bool = True) -> None:
            self.mcu = state

        def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
            self.plant_apply = bool(enable)

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state, base = server
    state.connect("COM5", mode="observe")
    status, data = _post(base, "/api/mcu_state", {"state": 0})
    assert status == 400
    assert "Enable control" in data["error"]
    status, data = _post(base, "/api/control_mode", {"mode": "control"})
    assert status == 200
    assert state.control_mode == "control"
    assert fake.auto_soft_kill is True
    assert fake.mcu == McuState.NORMAL


def test_state_exposes_session_dir_for_peer_alignment(server) -> None:
    """Operator needs to see which session dir this dashboard is watching so
    a DEFT_SESSION_DIR mismatch with a peer script (continuous/vbeta) is
    visible instead of silently reading nothing in follow mode."""
    state, base = server
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["session_dir"] == str(state.telemetry.session_dir)


def test_soft_kill_park_response_includes_path_for_follow_mode_ui(server) -> None:
    """The UI shows the actual flag path/mode after Park — not just ok/error —
    so the operator can see 'flag written' vs 'parked direct' immediately."""
    state, base = server
    flag = state.soft_kill_request_path()
    flag.unlink(missing_ok=True)
    status, data = _post(base, "/api/pdb/soft_kill_park")
    assert status == 200
    assert data["mode"] == "peer_request"
    assert data["path"] == str(flag)
    flag.unlink(missing_ok=True)


def test_ports_endpoint_returns_a_list(server) -> None:
    _state, base = server
    status, data = _get(base, "/api/ports")
    assert status == 200
    assert isinstance(data["ports"], list)


def test_actuator_command_without_connection_fails_cleanly(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/actuator/0", {"position": 1.0, "kp": 8.0, "kd": 0.4})
    assert status == 400
    assert "not connected" in data["error"]


def test_idle_without_connection_fails_cleanly(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/actuator/0/idle")
    assert status == 400


def test_mcu_state_without_connection_fails_cleanly(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/mcu_state", {"state": 1})
    assert status == 400


def test_recover_without_connection_fails_cleanly(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/recover")
    assert status == 400


def test_connect_to_nonexistent_port_fails_cleanly_not_500(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/connect", {"port": "COM_DOES_NOT_EXIST_999"})
    assert status == 400
    assert "error" in data


def test_connect_requires_port_in_body(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/connect", {})
    assert status == 400


def test_disconnect_when_never_connected_is_a_noop_not_an_error(server) -> None:
    _state, base = server
    status, _data = _post(base, "/api/disconnect")
    assert status == 200


def test_record_start_stop_via_http(server) -> None:
    """Control POSTs return {"ok": True} only (not the full snapshot) — that
    was itself a fix for lock contention (snapshot_dict() takes the same lock
    the stream thread needs). Recording status is read back via GET /api/state,
    same as the dashboard JS does after every action."""
    state, base = server
    status, data = _post(base, "/api/record/start")
    assert status == 200 and data == {"ok": True}
    assert state.telemetry.snapshot().recording is True
    status, data = _post(base, "/api/record/stop")
    assert status == 200 and data == {"ok": True}
    assert state.telemetry.snapshot().recording is False


def test_unknown_route_is_404(server) -> None:
    state, base = server
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        urllib.request.urlopen(base + "/api/nonsense")
    assert exc_info.value.code == 404


# -- AppState in isolation (double-connect guard, locking) -----------------------------


def test_connect_refuses_while_peer_owns_com(server, monkeypatch) -> None:
    """Dashboard must not HostProxy.connect over a live CDC owner — follow
    state.json instead of colliding on the COM port."""
    state, base = server
    # Fresh peer publish in the shared session dir.
    state.telemetry.set_connected(True, port="COM5")
    state.telemetry.flush(timeout_s=1.0)
    assert state.peer_com_owner() is not None

    called = {"n": 0}

    def _boom(cls, port, **kwargs):
        called["n"] += 1
        raise AssertionError("HostProxy.connect must not run while peer owns COM")

    import deft_controls_sdk.debug_dashboard.app as app_module

    monkeypatch.setattr(app_module.HostProxy, "connect", classmethod(_boom))
    status, data = _post(base, "/api/connect", {"port": "COM5"})
    assert status == 400
    assert "already owned" in data["error"]
    assert called["n"] == 0
    assert state.connected is False


def test_connect_allowed_when_peer_state_is_stale(server, monkeypatch) -> None:
    """Abandoned state.json (crashed peer) must not block Connect forever."""
    import deft_controls_sdk.debug_dashboard.app as app_module

    state, base = server
    sp = state.telemetry.state_path
    sp.parent.mkdir(parents=True, exist_ok=True)
    sp.write_text(
        json.dumps(
            {
                "connected": True,
                "port": "COM5",
                "updated_at": time.time() - (app_module.PEER_OWNER_MAX_AGE_S + 1.0),
            }
        ),
        encoding="utf-8",
    )
    assert state.peer_com_owner() is None
    fake = _TeleopFakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    status, data = _post(base, "/api/connect", {"port": "COM5"})
    assert status == 200
    assert data == {"ok": True}
    assert state.connected is True


def test_app_state_rejects_double_connect_via_lock(monkeypatch) -> None:
    """Two rapid Connect clicks must not open two Connections — the second
    caller must see 'already connected', not silently replace the first."""
    import deft_controls_sdk.debug_dashboard.app as app_module

    class _FakeHub:
        is_streaming = True

        def __init__(self, port):
            self.port = port

        def start_streaming(self, hz=50.0, *, telemetry_hz=10.0, auto_soft_kill=True):
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def set_mcu_state(self, state, *, send: bool = True) -> None:
            pass

        def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
            self.plant_apply = bool(enable)

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def close(self):
            pass

    def _connect_fresh(cls, port, **kwargs):
        fake = _FakeHub(port)
        fake.send_once = lambda: None
        fake.set_led = lambda *a, **k: None
        fake.stop_streaming = lambda: None
        fake.plant_apply = False
        fake.start_streaming(auto_soft_kill=False)
        fake.set_mcu_state(McuState.NORMAL, send=False)
        fake.set_plant_apply(False, send=True)
        return HostProxy(fake, owns_hub=True, listen_pdu=False)

    monkeypatch.setattr(app_module.HostProxy, "connect", classmethod(_connect_fresh))

    state = app_module.AppState(persist_telemetry=False)
    state.connect("COM5")
    assert state.connected is True
    with pytest.raises(RuntimeError, match="already connected"):
        state.connect("COM6")
    state.disconnect()
    assert state.connected is False


# -- Teleop (per-slot target+cruise slew) — no hardware, everything through a fake hub -----


class _TeleopFakeHub:
    """Records set_actuator/set_servo calls and answers held_desire/held_servo
    from whatever was last written — close enough to the real hub's held-state
    contract for the slew engine's seed-then-ramp logic to exercise for real."""

    port = "COM5"
    is_streaming = True

    def __init__(self) -> None:
        from deft_controls_sdk.link import ActuatorDesire

        self._acts = {
            0: ActuatorDesire(position=0.0),
            22: ActuatorDesire(position=0.0),
            25: ActuatorDesire(position=1.0),
        }
        self._servos: dict = {}
        self.mcu_states: list = []
        self.auto_soft_kill = None
        self.set_actuator_calls: list = []
        self.set_servo_calls: list = []

    def held_desires(self):
        return dict(self._acts)

    def held_desire(self, slot):
        return self._acts.get(slot)

    def held_servo(self, slot):
        return self._servos.get(slot)

    def start_streaming(self, hz=50.0, *, telemetry_hz=10.0, auto_soft_kill=True):
        self.auto_soft_kill = auto_soft_kill

    def set_auto_soft_kill(self, enabled: bool) -> None:
        self.auto_soft_kill = enabled

    def set_mcu_state(self, state_, *, send: bool = True) -> None:
        self.mcu_states.append(int(state_))

    def set_plant_apply(self, enable: bool, *, send: bool = True) -> None:
        pass

    def set_actuator(self, slot, desire, *, send: bool = True) -> None:
        self._acts[slot] = desire
        self.set_actuator_calls.append((slot, desire))

    def set_actuators(self, desires, *, send: bool = True) -> None:
        # Left-arm brace path writes the full batch via set_actuators.
        for slot, desire in desires.items():
            self.set_actuator(int(slot), desire, send=send)

    def set_servo(self, slot, desire, *, send: bool = True) -> None:
        self._servos[slot] = desire
        self.set_servo_calls.append((slot, desire))

    def clear_servos(self, *, send: bool = True) -> None:
        self._servos.clear()

    def close(self) -> None:
        pass


def _connect_fake_teleop_hub(monkeypatch, state, base):
    fake = _TeleopFakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state.connect("COM5", mode="control")
    return fake


def test_teleop_groups_endpoint_marks_unverified_slots(server) -> None:
    """Right arm (7-13) and the product-map base rows have no live-verified
    range on this bench yet (docs/peripherals/*.md) — the UI must be able to
    tell those apart from left arm / bench base so it can gate target-teleop
    without inventing a range."""
    _state, base = server
    status, data = _get(base, "/api/teleop/groups")
    assert status == 200
    assert data["cfg_map"] == "bench"
    assert data["actuators"]["0"]["group"] == "arm_left"
    assert data["actuators"]["0"]["verified"] is True
    assert data["actuators"]["7"]["group"] == "arm_right"
    assert data["actuators"]["7"]["verified"] is False
    assert data["actuators"]["22"]["group"] == "base"
    assert data["actuators"]["22"]["verified"] is True
    assert data["servos"]["0"]["label"] == "pitch"


def test_cfg_map_switch_relabels_base_without_touching_board(server) -> None:
    state, base = server
    status, _ = _post(base, "/api/cfg_map", {"map": "product"})
    assert status == 200
    assert state.cfg_map == "product"
    status, data = _get(base, "/api/teleop/groups")
    assert data["actuators"]["14"]["group"] == "base"
    assert data["actuators"]["14"]["verified"] is False  # not wired on this bench today
    assert "22" not in data["actuators"]  # bench-only slot gone once relabeled to product


def test_teleop_actuator_target_requires_control_mode(server, monkeypatch) -> None:
    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)
    state.set_control_mode("observe")
    status, data = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 400
    assert "Enable control" in data["error"]


def test_teleop_actuator_rejects_unverified_right_arm(server, monkeypatch) -> None:
    """Right arm has no live-verified CLEAR envelope yet — refuse the target
    instead of guessing a range, per arm-damiao-ch1.md provenance notes."""
    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)
    status, data = _post(base, "/api/teleop/actuator/7", {"target": 0.5, "cruise": 0.3})
    assert status == 400
    assert "live-verified" in data["error"]


def test_teleop_actuator_target_engages_and_slews(server, monkeypatch) -> None:
    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    status, data = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 200
    time.sleep(0.2)
    snap = state.teleop.snapshot()
    assert snap["actuators"][0]["target"] == 0.5
    assert 0.0 < snap["actuators"][0]["pos"] <= 0.5  # slewing toward target, not snapped
    assert fake.set_actuator_calls  # engine actually wrote through the hub
    status, _ = _post(base, "/api/teleop/actuator/0/stop")
    assert status == 200
    frozen = state.teleop.snapshot()["actuators"][0]
    assert frozen["target"] == frozen["pos"]  # froze in place, did not blank


def test_teleop_stop_all_freezes_without_blanking(server, monkeypatch) -> None:
    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 200
    time.sleep(0.15)
    status, data = _post(base, "/api/teleop/stop_all")
    assert status == 200
    assert data.get("ok") is True
    frozen = state.teleop.snapshot()["actuators"][0]
    assert frozen["target"] == frozen["pos"]


def test_teleop_stop_all_requires_control_mode(server, monkeypatch) -> None:
    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/control_mode", {"mode": "observe"})
    assert status == 200
    status, data = _post(base, "/api/teleop/stop_all")
    assert status == 400
    assert "control" in data["error"].lower()


def test_teleop_actuator_jog_clamps_to_verified_rail(server, monkeypatch) -> None:
    """Base RobStride slots (bench map) use the documented MIT rail with the
    RS_MARGIN inset — see base-robstride-mcp.md — not an unbounded jog."""
    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/actuator/22/jog", {"direction": 1, "cruise": 0.3})
    assert status == 200
    snap = state.teleop.snapshot()["actuators"][22]
    assert snap["target"] == pytest.approx(12.22, abs=1e-6)


def test_teleop_actuator_jog_damiao_base_is_seed_relative(server, monkeypatch) -> None:
    """Slot 25 (Damiao CH6) has no hardware MIT rail — its window is
    +/-2*pi from wherever it was seeded, per base-damiao-ch6.md."""
    import math

    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)  # fake hub seeds slot 25 at position=1.0
    status, _ = _post(base, "/api/teleop/actuator/25/jog", {"direction": 1, "cruise": 0.3})
    assert status == 200
    snap = state.teleop.snapshot()["actuators"][25]
    assert snap["target"] == pytest.approx(1.0 + 2 * math.pi, abs=1e-6)


def test_idle_group_blanks_only_that_group(server, monkeypatch) -> None:
    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 200
    status, _ = _post(base, "/api/idle_group/arm_left")
    assert status == 200
    assert 0 not in state.teleop.snapshot()["actuators"]  # disengaged, not just frozen
    blanked = fake._acts[0]
    assert blanked.kp == 0.0 and blanked.position == 0.0  # idle desire
    assert 25 in fake._acts and fake._acts[25].position == 1.0  # base slot untouched


def test_idle_group_neck_clears_servos(server, monkeypatch) -> None:
    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    fake._servos[0] = object()  # pretend something was held
    status, _ = _post(base, "/api/idle_group/neck")
    assert status == 200
    assert fake._servos == {}


def test_teleop_servo_idle_releases_only_that_slot(server, monkeypatch) -> None:
    """Single-joint neck release — the counterpart to per-slot arm Idle, so
    the operator isn't forced to use the whole-group neck Idle for one servo."""
    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/servo/0", {"target": 2500, "cruise": 100})
    assert status == 200
    status, _ = _post(base, "/api/teleop/servo/0/idle")
    assert status == 200
    assert 0 not in state.teleop.snapshot()["servos"]
    assert fake._servos[0].servo_id == 0  # released, not left at a stale goal
    assert 1 not in fake._servos  # yaw untouched


def test_continuous_launch_and_stop_use_injected_callables(server) -> None:
    """Real SSH/paramiko must never run in tests — AppState.continuous_launcher
    / continuous_stopper are swappable exactly so this contract is provable
    offline (see remote_continuous.py's module docstring)."""
    state, base = server
    calls = []

    def fake_launcher(*, duration_s=0.0, extra_args=""):
        calls.append(("launch", duration_s))
        return {"ok": True, "host": "fake"}

    def fake_stopper():
        calls.append(("stop",))
        return {"ok": True}

    state.continuous_launcher = fake_launcher
    state.continuous_stopper = fake_stopper

    status, data = _post(base, "/api/continuous/launch", {"duration_s": 12.0})
    assert status == 200 and data["state"] == "launching"
    for _ in range(50):
        if state.continuous_status()["state"] != "launching":
            break
        time.sleep(0.02)
    assert state.continuous_status() == {"state": "launched", "detail": {"ok": True, "host": "fake"}}
    assert ("launch", 12.0) in calls

    status, data = _post(base, "/api/continuous/stop")
    assert status == 200 and data["state"] == "stopping"
    for _ in range(50):
        if state.continuous_status()["state"] != "stopping":
            break
        time.sleep(0.02)
    assert state.continuous_status() == {"state": "stopped", "detail": {"ok": True}}
    assert ("stop",) in calls


def test_teleop_hold_boosts_damping_and_flags_without_moving_target(server, monkeypatch) -> None:
    """Field report: releasing J3 let J2 sag, which cascaded into dropping the
    whole arm. The settled-hold logic must react to a real disturbance by
    damping harder (kd), never by quietly letting the commanded target drift
    toward wherever gravity dragged the joint — see teleop.py's HOLD_* constants
    and their module docstring."""
    from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 200
    time.sleep(0.3)  # let it arrive (0.3 rad/s * 0.3s = 0.09 < 0.5 target... give it longer)
    time.sleep(1.5)
    assert state.teleop.snapshot()["actuators"][0]["target"] == 0.5

    # No disturbance yet — nominal hold, default kd, not flagged.
    nominal_kd = fake.set_actuator_calls[-1][1].kd
    assert state.teleop.snapshot()["actuators"][0]["flagged"] is False

    # Simulate live feedback showing the joint sagging away from its held target.
    actuators = [None] * ACTUATOR_COUNT
    actuators[0] = {"position": 0.2, "velocity": -0.4, "torque": 0.0, "temperature": 30.0, "fault": 1}
    state.telemetry.update_from_feedback(
        tick=1, ack_seq=0, mcu_state=0, plant_block=0, plant_block_name="none",
        pdu_tag=None, lap_ms=1, lap_max_ms=1, ticks_pending=0, svd_present=True,
        actuators=actuators,
    )
    # Teleop tick is ~60 Hz; wait until a hold tick consumes the sag FB.
    snap = None
    for _ in range(40):
        time.sleep(0.025)
        snap = state.teleop.snapshot()["actuators"][0]
        if snap.get("flagged"):
            break
    assert snap is not None
    assert snap["target"] == 0.5  # commanded target never moved toward the sag
    assert snap["flagged"] is True
    last_desire = fake.set_actuator_calls[-1][1]
    assert last_desire.position == 0.5  # still commanding the real target, not fb
    assert last_desire.kd > nominal_kd  # damping reacted to the sag


def test_continuous_launch_error_surfaces_without_crashing(server) -> None:
    state, base = server
    state.continuous_launcher = lambda **kw: (_ for _ in ()).throw(RuntimeError("no route to host"))
    status, data = _post(base, "/api/continuous/launch")
    assert status == 200 and data["state"] == "launching"
    for _ in range(50):
        if state.continuous_status()["state"] != "launching":
            break
        time.sleep(0.02)
    assert state.continuous_status()["state"] == "error"
    assert "no route to host" in state.continuous_status()["detail"]
