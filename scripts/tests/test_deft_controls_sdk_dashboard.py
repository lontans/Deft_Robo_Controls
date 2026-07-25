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

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.debug_dashboard.app import AppState, serve


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
    import deft_controls_sdk.debug_dashboard.app as app_module
    from deft_controls_sdk.link import ActuatorDesire

    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def __init__(self) -> None:
            self._held = {0: ActuatorDesire(position=1.0, kp=8.0, kd=0.4), 3: ActuatorDesire()}
            self.auto_soft_kill = None
            self.mcu_states = []

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

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            self._held[slot] = desire

        def close(self) -> None:
            pass

    fake = _FakeHub()
    monkeypatch.setattr(
        app_module.ControlsPcbHub, "connect", staticmethod(lambda port, **kw: fake)
    )

    state, base = server
    state.connect("COM5")  # observe default
    assert fake.auto_soft_kill is False
    assert fake.mcu_states and int(fake.mcu_states[-1]) == 2  # DIAG_ONLY
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
    import deft_controls_sdk.debug_dashboard.app as app_module

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

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def soft_kill_park(self) -> None:
            self.parked = True

        def close(self) -> None:
            pass

    fake = _FakeHub()
    monkeypatch.setattr(app_module.ControlsPcbHub, "connect", staticmethod(lambda port, **kw: fake))

    state, base = server
    state.connect("COM5")
    status, data = _post(base, "/api/pdb/soft_kill_park")
    assert status == 200
    assert data.get("ok") is True
    assert data.get("mode") == "direct"
    assert fake.parked is True


def test_observe_blocks_plant_commands_until_enable_control(server, monkeypatch) -> None:
    import deft_controls_sdk.debug_dashboard.app as app_module
    from deft_controls_sdk.link import McuState

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

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    monkeypatch.setattr(
        app_module.ControlsPcbHub, "connect", staticmethod(lambda port, **kw: fake)
    )
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


def test_app_state_rejects_double_connect_via_lock(monkeypatch) -> None:
    """Two rapid Connect clicks must not open two Connections — the second
    caller must see 'already connected', not silently replace the first."""
    import deft_controls_sdk.debug_dashboard.app as app_module

    class _FakeHub:
        def __init__(self, port):
            self.port = port

        def start_streaming(self, hz=50.0, *, telemetry_hz=10.0, auto_soft_kill=True):
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def set_mcu_state(self, state, *, send: bool = True) -> None:
            pass

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def close(self):
            pass

    def fake_connect(port, *, baud=None, telemetry=None):
        return _FakeHub(port)

    monkeypatch.setattr(app_module.ControlsPcbHub, "connect", staticmethod(fake_connect))

    state = app_module.AppState(persist_telemetry=False)
    state.connect("COM5")
    assert state.connected is True
    with pytest.raises(RuntimeError, match="already connected"):
        state.connect("COM6")
    state.disconnect()
    assert state.connected is False
