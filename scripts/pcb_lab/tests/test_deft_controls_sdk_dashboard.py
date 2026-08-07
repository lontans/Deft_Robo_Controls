"""Golden tests for the localhost controller dashboard (no hardware).

Runs the real ThreadingHTTPServer on an OS-assigned free port and drives it
with plain urllib — this exercises the actual HTTP routing/JSON contract,
not just AppState in isolation. No serial port is ever opened; every
"connected" scenario uses AppState with no hub, which is exactly what the
disconnected-error-handling paths need to prove.

Follow/Active x soft_kill model (see debug_dashboard/state.py): Connect
always enters Active *frozen* (soft_kill=True) — motion-issuing calls
(Apply / teleop target / jog) are rejected until ``/api/soft_kill/release``.
Idle / Stop / Hard-ESTOP-park are never gated by soft_kill.
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

import deft_controls_sdk.debug_dashboard.routes as routes_module
import deft_controls_sdk.debug_dashboard.state as state_module
from deft_controls_sdk.debug_dashboard.app import AppState, serve
from deft_controls_sdk.debug_dashboard.registry import ACTION_REGISTRY
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
    """Patch state.py's ``HostProxy.connect`` to wrap ``fake`` (no real COM).

    Mimics the real connect path's side effects (Active, frozen: NORMAL +
    plant_apply=1, soft_kill=True) that the dashboard and these tests assert
    on. Records the kwargs the dashboard actually requested (e.g. ``mode``)
    on ``fake.last_connect_kwargs`` so tests can assert on it directly.
    """
    if not hasattr(fake, "send_once"):
        fake.send_once = lambda: None  # type: ignore[attr-defined]
    if not hasattr(fake, "set_led"):
        fake.set_led = lambda *a, **k: None  # type: ignore[attr-defined]
    if not hasattr(fake, "stop_streaming"):
        fake.stop_streaming = lambda: None  # type: ignore[attr-defined]
    if not hasattr(fake, "plant_apply"):
        fake.plant_apply = False  # type: ignore[attr-defined]
    if not hasattr(fake, "set_actuators"):
        def _set_actuators(desires, *, send=True):
            for slot, d in desires.items():
                fake.set_actuator(int(slot), d, send=send)
        fake.set_actuators = _set_actuators  # type: ignore[attr-defined]
    if not hasattr(fake, "soft_kill_freeze"):
        def _freeze(*, send=True):
            fake.soft_kill = True
        fake.soft_kill_freeze = _freeze  # type: ignore[attr-defined]
    if not hasattr(fake, "soft_kill_unfreeze"):
        def _unfreeze():
            fake.soft_kill = False
        fake.soft_kill_unfreeze = _unfreeze  # type: ignore[attr-defined]
    if not hasattr(fake, "soft_kill"):
        fake.soft_kill = False  # type: ignore[attr-defined]

    def _connect(cls, port, **kwargs):
        fake.last_connect_kwargs = dict(kwargs)  # type: ignore[attr-defined]
        if hasattr(fake, "start_streaming"):
            fake.start_streaming(
                hz=float(kwargs.get("stream_hz", 50.0)),
                telemetry_hz=float(kwargs.get("telemetry_hz", 10.0)),
                auto_soft_kill=False,
            )
        if hasattr(fake, "set_mcu_state"):
            fake.set_mcu_state(McuState.NORMAL, send=False)
        if hasattr(fake, "set_plant_apply"):
            # armed=True (Active default) — real HostProxy.connect calls
            # hub.recover() -> NORMAL + plant_apply=True.
            fake.set_plant_apply(True, send=True)
            fake.plant_apply = True
        proxy = HostProxy(
            fake,
            owns_hub=True,
            listen_pdu=False,
            telemetry_hz=float(kwargs.get("telemetry_hz", 10.0)),
        )
        proxy._stream_hz = float(kwargs.get("stream_hz", 50.0))
        return proxy

    monkeypatch.setattr(state_module.HostProxy, "connect", classmethod(_connect))
    return fake


def test_index_serves_html(server) -> None:
    _state, base = server
    with urllib.request.urlopen(base + "/") as r:
        assert r.status == 200
        body = r.read().decode("utf-8")
    assert "<title>" in body
    assert "portSelect" in body  # connection card present
    assert "Advanced: raw per-slot Apply" in body  # control card present
    assert "/static/style.css" in body
    assert "/static/app.js" in body


def test_static_files_are_served(server) -> None:
    _state, base = server
    status, _ = (_get_raw := lambda p: urllib.request.urlopen(base + p))(
        "/static/style.css"
    ), None
    with urllib.request.urlopen(base + "/static/style.css") as r:
        assert r.status == 200
        assert "text/css" in r.headers.get("Content-Type", "")
    with urllib.request.urlopen(base + "/static/app.js") as r:
        assert r.status == 200
        assert "javascript" in r.headers.get("Content-Type", "")


def test_static_path_traversal_is_rejected(server) -> None:
    _state, base = server
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        urllib.request.urlopen(base + "/static/..%2F..%2Fapp.py")
    assert exc_info.value.code == 404


def test_state_before_any_connect_is_well_formed(server) -> None:
    _state, base = server
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["connected"] is False
    assert data["mode"] == "follow"
    assert data["soft_kill"] is True  # default at rest
    assert len(data["actuators"]) == 26  # wire slot count even with no hub yet
    assert all(a is None for a in data["actuators"])
    assert data["streaming"] is False
    assert data["held"] == [None] * 26
    # Idle must not look like a board fault (was grade=red / "disconnected").
    assert data["grade"] == "idle"
    assert "not connected" in (data.get("summary") or "").lower()


def test_connect_requests_debug_mode_not_bandwidth(server, monkeypatch) -> None:
    """See state.py module docstring: Connect must open mode="debug" (was
    "bandwidth") — verified safe (only hub.debug.* RPCs branch on it)."""

    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send=True):
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state, base = server
    status, _ = _post(base, "/api/connect", {"port": "COM5"})
    assert status == 200
    assert fake.last_connect_kwargs["mode"] == "debug"


def test_connect_enters_active_frozen_by_default(server, monkeypatch) -> None:
    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send=True):
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state, base = server
    status, _ = _post(base, "/api/connect", {"port": "COM5"})
    assert status == 200
    assert state.connected is True
    assert state.soft_kill is True
    assert fake.soft_kill is True
    status, data = _get(base, "/api/state")
    assert data["mode"] == "active"
    assert data["soft_kill"] is True


def test_soft_kill_blocks_motion_until_released(server, monkeypatch) -> None:
    """The core soft_kill contract: Apply is rejected with a clear error
    while frozen, and works once released."""

    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def __init__(self) -> None:
            self.applied = []

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send=True):
            self.applied.append((slot, desire))

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state, base = server
    _post(base, "/api/connect", {"port": "COM5"})

    status, data = _post(base, "/api/actuator/0", {"position": 1.0, "kp": 8.0, "kd": 0.4})
    assert status == 400
    assert "soft_kill is ON" in data["error"]

    status, _ = _post(base, "/api/soft_kill/release")
    assert status == 200
    assert state.soft_kill is False

    status, _ = _post(base, "/api/actuator/0", {"position": 1.0, "kp": 8.0, "kd": 0.4})
    assert status == 200
    assert any(slot == 0 for slot, _d in fake.applied)


def test_engage_soft_kill_refreezes(server, monkeypatch) -> None:
    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send=True):
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state, base = server
    _post(base, "/api/connect", {"port": "COM5"})
    _post(base, "/api/soft_kill/release")
    assert state.soft_kill is False
    status, _ = _post(base, "/api/soft_kill/engage")
    assert status == 200
    assert state.soft_kill is True
    status, data = _post(base, "/api/actuator/0", {"position": 1.0, "kp": 1.0, "kd": 0.1})
    assert status == 400
    assert "soft_kill is ON" in data["error"]


def test_idle_and_stop_work_regardless_of_soft_kill(server, monkeypatch) -> None:
    """Idle (blank torque) is never gated — only new motion commands are."""

    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def __init__(self):
            self.idled = []

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send=True):
            self.idled.append(slot)

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state, base = server
    _post(base, "/api/connect", {"port": "COM5"})
    assert state.soft_kill is True  # still frozen
    status, _ = _post(base, "/api/actuator/0/idle")
    assert status == 200
    assert 0 in fake.idled


def test_soft_kill_release_requires_connection(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/soft_kill/release")
    assert status == 400
    assert "not connected" in data["error"]


def test_held_state_reflects_active_vs_idle_commands(server, monkeypatch) -> None:
    """/api/state must distinguish an actively-held non-zero command from an
    idle-hold zero desire from a never-commanded slot."""
    from deft_controls_sdk.link import ActuatorDesire

    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def __init__(self) -> None:
            # Empty at construction — connect() anchors every slot itself
            # (kp=0, tiny nonzero position) the instant it runs, so a fresh
            # hub never has pre-existing held state to preserve (matches the
            # real Connection, whose _desires dict starts empty).
            self._held: dict = {}
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
    state.connect("COM5")
    assert fake.mcu_states and int(fake.mcu_states[-1]) == 0  # NORMAL
    assert fake.plant_apply_flags and fake.plant_apply_flags[-1] is True  # Active default
    assert state.mode == "active"
    assert state.soft_kill is True
    # Every slot is anchored by connect() — simulate an operator command
    # landing on slot 0 afterward (hub-level, independent of the soft_kill
    # HTTP guard which is exercised elsewhere).
    fake.set_actuator(0, ActuatorDesire(position=1.0, kp=8.0, kd=0.4))
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["streaming"] is True
    assert data["mode"] == "active"
    assert data["held"][0] == {
        "position": 1.0,
        "velocity": 0.0,
        "kp": 8.0,
        "kd": 0.4,
        "torque": 0.0,
        "active": True,
    }
    # Untouched slot 3 is still just the connect() anchor — idle-hold, not "active".
    assert data["held"][3]["active"] is False
    assert data["held"][3]["kp"] == 0.0


def test_state_exposes_pdb_status_from_telemetry(server) -> None:
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
    assert data["mcu_state"] == 3


def test_hard_estop_park_without_connection_writes_peer_request(server) -> None:
    """Follow mode: Hard-ESTOP park signals the CDC owner via a flag file."""
    state, base = server
    flag = state.hard_estop_request_path()
    if flag.is_file():
        flag.unlink()
    status, data = _post(base, "/api/estop/park")
    assert status == 200
    assert data.get("ok") is True
    assert data.get("mode") == "peer_request"
    assert flag.is_file()
    flag.unlink(missing_ok=True)


def test_hard_estop_park_calls_hub_when_connected(server, monkeypatch) -> None:
    class _FakeHub:
        port = "COM5"
        is_streaming = True
        parked = False

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def soft_kill_park(self) -> None:
            self.parked = True

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)

    state, base = server
    state.connect("COM5")
    status, data = _post(base, "/api/estop/park")
    assert status == 200
    assert data.get("ok") is True
    assert data.get("mode") == "direct"
    assert fake.parked is True


def test_state_exposes_session_dir_for_peer_alignment(server) -> None:
    state, base = server
    status, data = _get(base, "/api/state")
    assert status == 200
    assert data["session_dir"] == str(state.telemetry.session_dir)


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
    state, base = server
    status, data = _post(base, "/api/record/start")
    assert status == 200 and data == {"ok": True}
    assert state.telemetry.snapshot().recording is True
    status, data = _post(base, "/api/record/stop")
    assert status == 200 and data == {"ok": True}
    assert state.telemetry.snapshot().recording is False


def test_unknown_route_is_404(server) -> None:
    _state, base = server
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        urllib.request.urlopen(base + "/api/nonsense")
    assert exc_info.value.code == 404


# -- AppState in isolation (double-connect guard, locking) -----------------------------


def test_connect_refuses_while_peer_owns_com(server, monkeypatch) -> None:
    state, base = server
    state.telemetry.set_connected(True, port="COM5")
    state.telemetry.flush(timeout_s=1.0)
    assert state.peer_com_owner() is not None

    called = {"n": 0}

    def _boom(cls, port, **kwargs):
        called["n"] += 1
        raise AssertionError("HostProxy.connect must not run while peer owns COM")

    monkeypatch.setattr(state_module.HostProxy, "connect", classmethod(_boom))
    status, data = _post(base, "/api/connect", {"port": "COM5"})
    assert status == 400
    assert "already owned" in data["error"]
    assert called["n"] == 0
    assert state.connected is False


def test_connect_allowed_when_peer_state_is_stale(server, monkeypatch) -> None:
    state, base = server
    sp = state.telemetry.state_path
    sp.parent.mkdir(parents=True, exist_ok=True)
    sp.write_text(
        json.dumps(
            {
                "connected": True,
                "port": "COM5",
                "updated_at": time.time() - (state_module.PEER_OWNER_MAX_AGE_S + 1.0),
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
    class _FakeHub:
        is_streaming = True

        def __init__(self, port):
            self.port = port

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self):
            pass

    def _connect_fresh(cls, port, **kwargs):
        fake = _FakeHub(port)
        fake.send_once = lambda: None
        fake.set_led = lambda *a, **k: None
        fake.stop_streaming = lambda: None
        fake.plant_apply = False

        def _set_actuators(desires, *, send=True):
            for slot, d in desires.items():
                fake.set_actuator(int(slot), d, send=send)
        fake.set_actuators = _set_actuators
        fake.soft_kill_freeze = lambda *, send=True: None
        fake.soft_kill_unfreeze = lambda: None
        return HostProxy(fake, owns_hub=True, listen_pdu=False)

    monkeypatch.setattr(state_module.HostProxy, "connect", classmethod(_connect_fresh))

    state = state_module.AppState(persist_telemetry=False)
    state.connect("COM5")
    assert state.connected is True
    with pytest.raises(RuntimeError, match="already connected"):
        state.connect("COM6")
    state.disconnect()
    assert state.connected is False


# -- Teleop (per-slot target+cruise slew) — no hardware, everything through a fake hub -----


class _TeleopFakeHub:
    """Records set_actuator/set_servo calls and answers held_desire/held_servo
    from whatever was last written."""

    port = "COM5"
    is_streaming = True

    def __init__(self) -> None:
        from deft_controls_sdk.link import ActuatorDesire

        self._acts = {
            0: ActuatorDesire(position=0.0),
            22: ActuatorDesire(position=0.0),
            25: ActuatorDesire(position=1.0),
        }
        # connect() anchors every slot (kp=0, tiny nonzero position) the
        # instant it runs — restore these afterward (see
        # _connect_fake_teleop_hub) so seed-relative tests see real seeds.
        self.initial_acts = dict(self._acts)
        self._servos: dict = {}
        self.mcu_states: list = []
        self.auto_soft_kill = None
        self.soft_kill = False
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

    def soft_kill_freeze(self, *, send: bool = True) -> None:
        self.soft_kill = True

    def soft_kill_unfreeze(self) -> None:
        self.soft_kill = False

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
    """Connect + release soft_kill (the new equivalent of old mode="control")."""
    fake = _TeleopFakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state.connect("COM5")
    # connect() just anchored every slot — restore the fixture's intended
    # pre-seeded positions (standing in for live feedback/prior state) so
    # seed-relative jog / target tests exercise real seed values, not the
    # anchor.
    for slot, desire in fake.initial_acts.items():
        fake.set_actuator(slot, desire)
    state.release_soft_kill()
    return fake


def test_teleop_groups_endpoint_marks_unverified_slots(server) -> None:
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
    assert data["actuators"]["14"]["verified"] is False
    assert "22" not in data["actuators"]


def test_teleop_actuator_target_requires_soft_kill_released(server, monkeypatch) -> None:
    state, base = server
    fake = _TeleopFakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state.connect("COM5")  # still frozen — never released
    status, data = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 400
    assert "soft_kill is ON" in data["error"]


def test_teleop_actuator_rejects_unverified_right_arm(server, monkeypatch) -> None:
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
    assert 0.0 < snap["actuators"][0]["pos"] <= 0.5
    assert fake.set_actuator_calls
    status, _ = _post(base, "/api/teleop/actuator/0/stop")
    assert status == 200
    frozen = state.teleop.snapshot()["actuators"][0]
    assert frozen["target"] == frozen["pos"]


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


def test_teleop_stop_all_works_even_while_frozen(server, monkeypatch) -> None:
    """Stop (freeze in place) is not gated by soft_kill — only new targets are."""
    state, base = server
    fake = _TeleopFakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state.connect("COM5")
    state.release_soft_kill()
    status, _ = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 200
    state.engage_soft_kill()
    status, data = _post(base, "/api/teleop/stop_all")
    assert status == 200
    assert data.get("ok") is True


def test_teleop_actuator_jog_clamps_to_verified_rail(server, monkeypatch) -> None:
    state, base = server
    _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/actuator/22/jog", {"direction": 1, "cruise": 0.3})
    assert status == 200
    snap = state.teleop.snapshot()["actuators"][22]
    assert snap["target"] == pytest.approx(12.22, abs=1e-6)


def test_teleop_actuator_jog_damiao_base_is_seed_relative(server, monkeypatch) -> None:
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
    assert 0 not in state.teleop.snapshot()["actuators"]
    blanked = fake._acts[0]
    assert blanked.kp == 0.0 and blanked.position == 0.0
    assert 25 in fake._acts and fake._acts[25].position == 1.0


def test_idle_group_neck_clears_servos(server, monkeypatch) -> None:
    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    fake._servos[0] = object()
    status, _ = _post(base, "/api/idle_group/neck")
    assert status == 200
    assert fake._servos == {}


def test_teleop_servo_idle_releases_only_that_slot(server, monkeypatch) -> None:
    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/servo/0", {"target": 2500, "cruise": 100})
    assert status == 200
    status, _ = _post(base, "/api/teleop/servo/0/idle")
    assert status == 200
    assert 0 not in state.teleop.snapshot()["servos"]
    assert fake._servos[0].servo_id == 0
    assert 1 not in fake._servos


def test_continuous_launch_and_stop_use_injected_callables(server) -> None:
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
    from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

    state, base = server
    fake = _connect_fake_teleop_hub(monkeypatch, state, base)
    status, _ = _post(base, "/api/teleop/actuator/0", {"target": 0.5, "cruise": 0.3})
    assert status == 200
    time.sleep(0.3)
    time.sleep(1.5)
    assert state.teleop.snapshot()["actuators"][0]["target"] == 0.5

    nominal_kd = fake.set_actuator_calls[-1][1].kd
    assert state.teleop.snapshot()["actuators"][0]["flagged"] is False

    actuators = [None] * ACTUATOR_COUNT
    actuators[0] = {"position": 0.2, "velocity": -0.4, "torque": 0.0, "temperature": 30.0, "fault": 1}
    state.telemetry.update_from_feedback(
        tick=1, ack_seq=0, mcu_state=0, plant_block=0, plant_block_name="none",
        pdu_tag=None, lap_ms=1, lap_max_ms=1, ticks_pending=0, svd_present=True,
        actuators=actuators,
    )
    snap = None
    for _ in range(40):
        time.sleep(0.025)
        snap = state.teleop.snapshot()["actuators"][0]
        if snap.get("flagged"):
            break
    assert snap is not None
    assert snap["target"] == 0.5
    assert snap["flagged"] is True
    last_desire = fake.set_actuator_calls[-1][1]
    assert last_desire.position == 0.5
    assert last_desire.kd > nominal_kd


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


# -- Panels / registry / graceful degradation --------------------------------------------


def test_action_registry_import_paths_and_call_sigs_are_well_formed() -> None:
    assert ACTION_REGISTRY, "registry should not be empty"
    for key, spec in ACTION_REGISTRY.items():
        assert spec.key == key
        parts = spec.import_path.split(".")
        assert len(parts) >= 2
        assert all(p.isidentifier() for p in parts), spec.import_path
        assert isinstance(spec.call_sig, str) and "(" in spec.call_sig and spec.call_sig.endswith(")")
        assert spec.requires_mode in ("any", "follow", "active", "debug")


def test_wire_panels_gracefully_degrades_when_module_missing(server, monkeypatch) -> None:
    """A registry entry pointing at a module/function that genuinely doesn't
    exist must not raise — wire_panels() marks just that one unavailable and
    moves on. Also asserts every *real* entry resolves now that A/B/C's panel
    modules are merged into this tree (regression guard against the naming
    mismatches this test caught originally)."""
    from deft_controls_sdk.debug_dashboard.registry import ActionSpec

    broken = ActionSpec(
        key="_test_missing", label="missing", blurb="", owner="test",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.does_not_exist",
        call_sig="does_not_exist(proxy)",
    )
    monkeypatch.setitem(ACTION_REGISTRY, "_test_missing", broken)

    result = routes_module.wire_panels()
    assert set(result.keys()) == set(ACTION_REGISTRY.keys())
    assert result["_test_missing"] is None
    assert all(v is not None for k, v in result.items() if k != "_test_missing"), (
        "every real ACTION_REGISTRY entry should resolve to a callable now that "
        "debug/panels/{system,discover} are merged in"
    )
    status, data = _get(server[1], "/api/panels")
    assert status == 200
    assert data["_test_missing"]["available"] is False
    assert all(entry["available"] is True for k, entry in data.items() if k != "_test_missing")


def test_panel_run_reports_unavailable_not_500(server, monkeypatch) -> None:
    from deft_controls_sdk.debug_dashboard.registry import ActionSpec

    broken = ActionSpec(
        key="_test_missing_run", label="missing", blurb="", owner="test",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.does_not_exist",
        call_sig="does_not_exist(proxy)",
    )
    monkeypatch.setitem(ACTION_REGISTRY, "_test_missing_run", broken)
    routes_module.wire_panels()

    _state, base = server
    status, data = _post(base, "/api/panels/_test_missing_run/run", {})
    assert status == 400
    assert "not wired yet" in data["error"]


def test_panel_run_unknown_key_is_400_not_500(server) -> None:
    _state, base = server
    status, data = _post(base, "/api/panels/nonexistent_panel/run", {})
    assert status == 400
    assert "unknown panel action" in data["error"]


def test_bandwidth_panel_rejected_while_active_even_when_wired(server, monkeypatch) -> None:
    """Windows CDC is exclusive-open — bandwidth must be Follow-only,
    regardless of whether the real panel module has landed yet."""

    def _fake_bandwidth(port, **kwargs):
        return {"port": port, "ok": True}

    monkeypatch.setitem(routes_module._PANEL_FUNCS, "bandwidth", _fake_bandwidth)
    monkeypatch.setattr(routes_module, "_panel_wired", True)

    state, base = server

    # Follow mode: allowed (wired + not Active).
    status, data = _post(base, "/api/panels/bandwidth/run", {"port": "COM9", "seconds": 1.0})
    assert status == 200
    assert data["result"] == {"port": "COM9", "ok": True}

    # Now go Active and confirm the same call is rejected.
    class _FakeHub:
        port = "COM5"
        is_streaming = True

        def held_desires(self):
            return {}

        def held_desire(self, slot):
            return None

        def set_actuator(self, slot, desire, *, send: bool = True) -> None:
            pass

        def set_auto_soft_kill(self, enabled: bool) -> None:
            pass

        def close(self) -> None:
            pass

    fake = _FakeHub()
    _install_fake_proxy_connect(monkeypatch, fake)
    state.connect("COM5")
    status, data = _post(base, "/api/panels/bandwidth/run", {"port": "COM9", "seconds": 1.0})
    assert status == 400
    assert "Follow-mode only" in data["error"]


def test_panel_run_requires_active_for_debug_scoped_panels(server, monkeypatch) -> None:
    def _fake_inventory(proxy, **kwargs):
        return {"slots": []}

    monkeypatch.setitem(routes_module._PANEL_FUNCS, "inventory", _fake_inventory)
    monkeypatch.setattr(routes_module, "_panel_wired", True)

    _state, base = server
    status, data = _post(base, "/api/panels/inventory/run", {})
    assert status == 400
    assert "requires an Active connection" in data["error"]
