"""Offline tests: TeleopEngine.set_actuator_rate + the teleop_rate convenience wrapper.

No hardware — a fake hub records ``set_actuator`` calls. ``hub_getter`` is wired to
return ``None`` so the engine's real background thread (started by ``_ensure_thread``
via the public ``set_actuator_rate``/``engage_actuator`` calls) stays inert; each test
then calls ``_tick_actuators`` directly with its own fake hub for deterministic,
race-free ticks — see ``TeleopEngine._run``'s ``if hub is not None`` gate.
"""
from __future__ import annotations

import os
import sys
import time

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.actions.teleop import DM_BASE_TRAVEL, SlotSpec, TeleopEngine  # noqa: E402
from deft_controls_sdk.actions.teleop_rate import rate_teleop_actuator  # noqa: E402


class _FakeHub:
    def __init__(self):
        self.calls: dict = {}

    def set_actuator(self, slot, desire, *, send=False):
        self.calls[slot] = desire

    def set_servo(self, *a, **k):
        pass


def _drive_spec(slot=17, lo=-10.0, hi=10.0, cruise_max=1.0, kd=1.0):
    return SlotSpec(
        slot=slot, group="base", label="test-drive", protocol="robstride",
        kp=0.0, kd=kd, verified=False, lo=lo, hi=hi, seed_relative=False,
        cruise_max=cruise_max, cruise_default=0.3, role="drive",
    )


def _seed_relative_spec(slot=25, cruise_max=1.0):
    return SlotSpec(
        slot=slot, group="base", label="test-damiao", protocol="damiao",
        kp=10.0, kd=0.5, verified=True, lo=None, hi=None, seed_relative=True,
        cruise_max=cruise_max, cruise_default=0.3, role="drive",
    )


@pytest.fixture()
def engine():
    eng = TeleopEngine(hub_getter=lambda: None, hz=50.0)
    yield eng
    eng.stop()


def test_set_actuator_rate_engages_rate_mode(engine):
    spec = _drive_spec()
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=0.5, timeout_s=0.25)
    st = engine._act[17]
    assert st["mode"] == "rate"
    assert st["pos"] == 2.0
    assert st["target"] == 2.0
    assert st["rate"] == 0.5
    assert st["lo"] == -10.0 and st["hi"] == 10.0


def test_rate_clamped_to_spec_cruise_max(engine):
    spec = _drive_spec(cruise_max=1.0)
    engine.set_actuator_rate(17, spec=spec, seed=0.0, rate=5.0)
    assert engine._act[17]["rate"] == 1.0
    engine.set_actuator_rate(17, spec=spec, seed=0.0, rate=-5.0)
    assert engine._act[17]["rate"] == -1.0


def test_tick_integrates_rate_into_position(engine):
    spec = _drive_spec()
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=0.5, timeout_s=1.0)
    hub = _FakeHub()
    engine._tick_actuators(hub)
    st = engine._act[17]
    expected = 2.0 + 0.5 * engine._dt
    assert st["pos"] == pytest.approx(expected)
    assert st["target"] == pytest.approx(expected)  # target kept in sync
    desire = hub.calls[17]
    assert desire.position == pytest.approx(expected)
    assert desire.velocity == pytest.approx(0.5)  # not at a rail


def test_refresh_does_not_reseed_or_jump_position(engine):
    spec = _drive_spec()
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=0.5, timeout_s=1.0)
    hub = _FakeHub()
    engine._tick_actuators(hub)
    pos_after_first_tick = engine._act[17]["pos"]

    # Refresh with a new rate/timeout (simulates a held key / dragged slider) — must not
    # re-seed pos back to the original `seed` value.
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=-0.8, timeout_s=1.0)
    assert engine._act[17]["pos"] == pos_after_first_tick
    assert engine._act[17]["rate"] == -0.8


def test_stop_actuator_kills_rate_mode(engine):
    spec = _drive_spec()
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=0.5, timeout_s=1.0)
    hub = _FakeHub()
    engine._tick_actuators(hub)  # advance pos off the seed
    engine.stop_actuator(17)
    st = engine._act[17]
    assert st["mode"] == "target"
    assert st["rate"] == 0.0
    assert st["target"] == st["pos"]


def test_rate_mode_timeout_decays_to_zero(engine):
    """Fail-safe decay: no refresh within timeout_s -> the tick loop itself zeroes
    rate (hold in place), it does not keep integrating a stale rate forever."""
    spec = _drive_spec()
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=0.5, timeout_s=0.25)
    # Directly manipulate the deadline into the past.
    engine._act[17]["rate_deadline"] = time.monotonic() - 1.0
    hub = _FakeHub()
    engine._tick_actuators(hub)
    assert engine._act[17]["rate"] == 0.0
    # Position must not have moved (rate was zeroed before the integration step).
    assert engine._act[17]["pos"] == 2.0
    desire = hub.calls[17]
    assert desire.velocity == 0.0


def test_rate_mode_clamps_at_rail_and_zeroes_velocity(engine):
    spec = _drive_spec(lo=-10.0, hi=2.05, cruise_max=1.0)
    engine.set_actuator_rate(17, spec=spec, seed=2.0, rate=1.0, timeout_s=5.0)
    hub = _FakeHub()
    for _ in range(20):
        engine._tick_actuators(hub)
    st = engine._act[17]
    assert st["pos"] == pytest.approx(2.05)
    desire = hub.calls[17]
    assert desire.position == pytest.approx(2.05)
    assert desire.velocity == 0.0  # clamped at the rail -- no runaway velocity command


def test_rate_mode_seed_relative_slot_windows_around_seed(engine):
    """Bench Damiao (seed_relative, lo=hi=None) has no absolute rail -- rate mode
    windows +/- DM_BASE_TRAVEL around the seed, same as engage_actuator_seed_relative."""
    spec = _seed_relative_spec(slot=25)
    engine.set_actuator_rate(25, spec=spec, seed=1.0, rate=0.1, timeout_s=1.0)
    st = engine._act[25]
    assert st["lo"] == pytest.approx(1.0 - DM_BASE_TRAVEL)
    assert st["hi"] == pytest.approx(1.0 + DM_BASE_TRAVEL)


def test_rate_teleop_actuator_wrapper_engages(engine):
    spec = _drive_spec(slot=18)
    specs = {18: spec}
    engine2 = engine
    rate_teleop_actuator(
        engine2, 18, specs=specs, seed_fn=lambda slot: 0.75, rate=0.3, timeout_s=0.4,
    )
    st = engine2._act[18]
    assert st["mode"] == "rate"
    assert st["pos"] == 0.75
    assert st["rate"] == 0.3


def test_rate_teleop_actuator_missing_slot_raises(engine):
    with pytest.raises(ValueError):
        rate_teleop_actuator(
            engine, 99, specs={}, seed_fn=lambda slot: 0.0, rate=0.2,
        )
