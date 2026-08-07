"""Dashboard process state — Follow vs Active x soft_kill state machine.

Replaces the old observe/control-as-separate-modes model (see git history of
``app.py`` if you need the previous shape). The mental model now is:

* **Follow** — this process does not hold COM. It reads a peer's telemetry
  cache via ``state.json`` (e.g. ``yam_continuous_all`` owns CDC). Read-only.
* **Active** — this process holds COM (``AppState.connected`` /
  ``AppState.proxy is not None``).
  * **soft_kill ON** (the default the instant Active is entered) — frozen:
    hold whatever is currently held, at full torque (kp/kd/position
    preserved) — NOT blanked/de-energized. Structurally this means the wire
    stays ``NORMAL`` + ``plant_apply=1`` and simply stops being told to move
    (see ``ControlsPcbHub.soft_kill_freeze``). ``set_mcu_state`` is never
    called with RECOVERY/ESTOP while frozen this way.
  * **soft_kill OFF** — released; motion-issuing calls are allowed through.

soft_kill is a single board-wide flag (firmware ``plant_apply``/``mcu_state``
are single global values — there is no per-slot wire field for this). Per-slot
Idle (blanking one slot's held desire) is a separate, always-available
mechanism — "this slot isn't doing anything" vs. "the board is frozen".

Connect opens the shared proxy with ``mode="debug"`` (not the old
``mode="bandwidth"``) — verified safe: ``Connection.connect(mode=...)`` only
flips a wire header bit, nothing in the plant-motion path branches on it,
only ``hub.debug.*`` RPCs require ``mode="debug"``. This unlocks CFG/discover/
host-link panels on the same connection the operator already drives motors
through.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Optional

from deft_controls_sdk import ActuatorDesire, HostProxy, McuState, ServoDesire
from deft_controls_sdk.actions import (
    build_actuator_specs,
    build_servo_specs,
    make_teleop_engine,
)
from deft_controls_sdk.config import assembly_from_name
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD, list_ports_info  # noqa: F401 — re-exported for routes.py
from deft_controls_sdk.telemetry import TelemetryCache, default_session_dir

from . import remote_continuous

# Peer (e.g. yam_continuous_all) owns CDC while dashboard follows state.json —
# Hard-ESTOP park writes this flag; the peer parks and deletes it.
SOFT_KILL_REQUEST_NAME = "soft_kill_request"
# Refuse HostProxy.connect while a peer's state.json still looks live — opening
# COM would collide with the CDC owner (Windows exclusive open / dual writers).
PEER_OWNER_MAX_AGE_S = 2.0

DEFAULT_HTTP_PORT = 8766

_ACTIVE_EPS = 0.01
"""Matches firmware's actuator_any_non_idle_live() threshold — see
App/Src/plant/actuator.c. A held desire counts as "active" (not just idle
hold-position) if kp/kd/|velocity|/|torque| exceed this."""

_MAX_SANE_HZ = 1000.0
"""Upper bound on a per-connect hz override — well above any real plant TX
rate (100 Hz default, ~125 Hz continuous mouse); guards against a stray typo
(e.g. a pasted baud rate) rather than a deliberately fast rate."""


def _validate_hz(value: float, *, label: str) -> float:
    hz = float(value)
    if not (0.0 < hz <= _MAX_SANE_HZ):
        raise ValueError(f"{label} must be in (0, {_MAX_SANE_HZ:g}] Hz, got {hz!r}")
    return hz


class AppState:
    """Owns the (optional) HostProxy session across connect/disconnect cycles.

    One TelemetryCache lives for the whole process lifetime, independent of
    any particular connection — /api/state always has something sane to
    return, even before the first Connect, and fault history / ring buffer
    survive a board reset instead of resetting on every reconnect.
    """

    def __init__(
        self,
        *,
        session_dir: Optional[str] = None,
        persist_telemetry: bool = True,
        stream_hz: float = 40.0,
        telemetry_hz: float = 10.0,
    ) -> None:
        self._lock = threading.Lock()
        self._stream_hz = stream_hz
        self._telemetry_hz = telemetry_hz
        self.telemetry = TelemetryCache(
            session_dir=session_dir or default_session_dir(), persist=persist_telemetry
        )
        self.proxy: Optional[HostProxy] = None
        # soft_kill is meaningful only while connected (Active); kept True at
        # rest so a freshly-connected session always starts frozen regardless
        # of what it was left at on a previous session.
        self.soft_kill: bool = True

        # Teleop (per-slot target+cruise slew — actions.TeleopEngine) and its
        # slot-group model. "bench" is the default CFG map because that's what's
        # actually wired/live-verified on the current bench (base on slots 22-25).
        self.cfg_map: str = "bench"
        # When True, L-arm teleop rails use MuJoCo soft limits (no bench-clear
        # intersection) so an operator can nudge past the old clear envelope and
        # mark new min/max from live encoder FB. Capture is UI-only — nothing is
        # written back to yam_bench_clear_left.py.
        self.limit_scout: bool = False
        self.actuator_specs = build_actuator_specs(self.cfg_map)
        self.servo_specs = build_servo_specs()
        # Match continuous --mouse plant write (brace + gravity + ~60 Hz slew).
        # Plant stream TX rate is separate (--hz on __main__, default 100).
        self.teleop = make_teleop_engine(
            lambda: None if self.proxy is None else self.proxy.hub,
            feedback_getter=self._teleop_feedback_snapshot,
            hz=60.0,
            brace_left_arm=True,
            gravity=True,
        )

        # Continuous mode launch/stop over SSH — swappable so tests can drive the HTTP
        # contract without paramiko or a real host (see remote_continuous.py docstring).
        self.continuous_launcher = remote_continuous.default_launcher
        self.continuous_stopper = remote_continuous.default_stopper
        self._continuous_lock = threading.Lock()
        self._continuous_status: dict = {"state": "unknown", "detail": None}

    # -- Follow / Active -------------------------------------------------------------------

    @property
    def hub(self):
        """Wire hub for the active session, or None when disconnected (Follow)."""
        return None if self.proxy is None else self.proxy.hub

    @property
    def connected(self) -> bool:
        """True while Active (this process holds COM)."""
        return self.proxy is not None

    @property
    def mode(self) -> str:
        return "active" if self.connected else "follow"

    @property
    def stream_hz(self) -> float:
        """Plant TX rate the *next* :meth:`connect` will use, absent an override."""
        return self._stream_hz

    @property
    def telemetry_hz(self) -> float:
        """UI/state.json publish rate the *next* :meth:`connect` will use, absent an override."""
        return self._telemetry_hz

    def peer_com_owner(self) -> Optional[dict]:
        """If another process owns CDC in this session dir, return peer info.

        Detected from a fresh ``state.json`` with ``connected=true``. Freshness
        prefers ``updated_at``; if that is unset (connect-only publish), falls
        back to the file mtime. Stale files (crashed peer) older than
        ``PEER_OWNER_MAX_AGE_S`` are ignored so Connect can reclaim the port.
        Does not open COM.
        """
        sp = self.telemetry.state_path
        if not sp.is_file():
            return None
        try:
            data = json.loads(sp.read_text(encoding="utf-8"))
        except Exception:
            return None
        if not data.get("connected"):
            return None
        now = time.time()
        updated = float(data.get("updated_at") or 0.0)
        if updated > 0.0:
            age = now - updated
        else:
            try:
                age = now - sp.stat().st_mtime
            except OSError:
                return None
        if age > PEER_OWNER_MAX_AGE_S:
            return None
        return {
            "port": data.get("port"),
            "age_s": age,
            "path": str(sp),
            "fb_hz": data.get("fb_hz"),
            "summary": data.get("summary"),
        }

    def connect(
        self,
        port: str,
        *,
        baud: int = DEFAULT_BAUD,
        stream_hz: Optional[float] = None,
        telemetry_hz: Optional[float] = None,
    ) -> None:
        """Open CDC via HostProxy (mode="debug") and enter Active.

        Always enters Active with ``soft_kill=True`` (frozen/safe) — the
        operator must explicitly :meth:`release_soft_kill` before any motion
        command is accepted. Every slot is anchored (kp=0, tiny nonzero
        position — never a true p=0/kp=0 blank, see HOME_POS_EPS note in
        :meth:`set_actuator`) so ``plant_apply=1`` does not mount a stale or
        empty held image the instant it arms.

        ``stream_hz``/``telemetry_hz`` are a per-connect override of the rate
        this session opens at (a "new session" pick, per the constructor's
        ``--hz``/``--telemetry-hz`` defaults) — pass either to try a different
        plant TX / publish rate without restarting the dashboard process.
        Validated up front (rejected outright, never silently clamped, if
        <= 0 or so high it is almost certainly a typo) but only committed to
        ``self._stream_hz``/``self._telemetry_hz`` — so the next Connect after
        a Disconnect reuses it — once ``HostProxy.connect`` actually succeeds;
        a rejected/failed Connect (bad port, peer owns COM, ...) must not
        silently change what the *next* Connect attempt will use.

        Refuses to open COM while a peer is actively publishing ``state.json``
        for this session dir — stay in Follow mode instead of colliding.
        """
        with self._lock:
            if self.proxy is not None:
                raise RuntimeError(
                    f"already connected to {self.proxy.hub.port} — disconnect first"
                )
            effective_stream_hz = (
                self._stream_hz if stream_hz is None else _validate_hz(stream_hz, label="stream_hz")
            )
            effective_telemetry_hz = (
                self._telemetry_hz
                if telemetry_hz is None
                else _validate_hz(telemetry_hz, label="telemetry_hz")
            )
            peer = self.peer_com_owner()
            if peer is not None:
                peer_port = peer.get("port") or "?"
                raise RuntimeError(
                    f"COM already owned by a peer writing {peer['path']} "
                    f"(port={peer_port}, age={peer['age_s']:.1f}s). "
                    "Stay in Follow mode — do not Connect while continuous/teleop "
                    "owns CDC. Stop the peer (or Hard-ESTOP park) first."
                )
            # Desk bringup: share this process TelemetryCache so /api/state and
            # fault history survive reconnect.
            proxy = HostProxy.connect(
                port,
                baud=baud,
                stream_hz=effective_stream_hz,
                telemetry_hz=effective_telemetry_hz,
                armed=True,
                listen_pdu=False,
                telemetry=self.telemetry,
                assembly=assembly_from_name(self.cfg_map),
                mode="debug",
            )
            # Only now — HostProxy.connect succeeded — commit the override so
            # a later reconnect (post-Disconnect) picks it up as the new default.
            self._stream_hz = effective_stream_hz
            self._telemetry_hz = effective_telemetry_hz
            anchored = {s: ActuatorDesire(position=1e-6) for s in range(ACTUATOR_COUNT)}
            # proxy.set_actuators() reaches into hub._connection directly (see
            # HostProxy.set_actuators) — go through hub.set_actuators() instead,
            # the documented ControlsPcbHub surface (also what fake hubs in
            # tests implement).
            proxy.hub.set_actuators(anchored, send=False)
            self.proxy = proxy
            self._enter_active_locked()

    def _enter_active_locked(self) -> None:
        proxy = self.proxy
        assert proxy is not None
        proxy.hub.set_auto_soft_kill(True)
        self.soft_kill = True
        proxy.hub.soft_kill_freeze(send=True)

    def release_soft_kill(self) -> None:
        """Operator explicitly opts into motion — clears the freeze guard."""
        self._require_proxy()
        self.soft_kill = False
        self.hub.soft_kill_unfreeze()

    def engage_soft_kill(self) -> None:
        """Freeze right now — hold whatever is currently held, at full torque."""
        self._require_proxy()
        self.soft_kill = True
        self.hub.soft_kill_freeze(send=True)

    def set_listen_pdu(self, enabled: bool) -> None:
        """Toggle whether the host obeys PDB kill-state for soft-kill/LED policy.

        Off (bench with no PDU peer) is the dashboard's Connect default so the
        auto-trip hook (which now freezes via ``soft_kill_freeze`` rather than
        hard-ESTOP — see ``ControlsPcbHub.set_auto_soft_kill``) is a deliberate
        no-op. Flip this on when a real PDU (or pdb_uart_sim) is actually
        present on the bench.
        """
        self._require_proxy().listen_pdu = bool(enabled)

    def clear_fault_log(self) -> None:
        """Reset the black-box "faults this session" counter/ring — see
        TelemetryCache.clear_faults. Safe while disconnected (telemetry cache
        outlives any single connection); does not delete files on disk."""
        self.telemetry.clear_faults()

    def disconnect(self) -> None:
        with self._lock:
            self.teleop.disengage_all()
            proxy = self.proxy
            if proxy is not None:
                try:
                    proxy.hub.set_auto_soft_kill(False)
                    # HostProxy.close() blanks desires + NORMAL + plant_apply=0
                    # before releasing COM — safe regardless of soft_kill state.
                    proxy.close()
                except Exception:
                    pass
                self.proxy = None
            self.soft_kill = True

    def _require_proxy(self) -> HostProxy:
        proxy = self.proxy
        if proxy is None:
            raise RuntimeError("not connected — Follow mode is read-only, Connect first")
        return proxy

    def _require_motion_allowed(self) -> HostProxy:
        """Guard for every motion-issuing method — raises while frozen.

        Covers: set_actuator, teleop_actuator_target, teleop_actuator_jog,
        teleop_servo_target, and (once it lands) the passthrough to
        actions.teleop_rate.set_actuator_rate. Idle / Stop / raw MCU-state /
        plant_apply are NOT gated here — those are either inherently safe
        (Idle blanks torque) or diagnostic-only (Advanced panel raw buttons).
        """
        proxy = self._require_proxy()
        if self.soft_kill:
            raise RuntimeError("soft_kill is ON — release before commanding motion")
        return proxy

    def set_actuator(self, slot: int, *, position: float, kp: float, kd: float) -> None:
        # send=False: only update the held desire. The background stream loop
        # already write()s the plant image at stream_hz — a second write+flush
        # from this HTTP thread contended on the serial lock and stalled fb_hz.
        #
        # HOME_POS_EPS: avoid true blank (pos=0, kp=0) when the operator raises
        # gains — shared blank-bus skip drops uncommanded buses (FDCAN and MCP).
        # Match legacy eps when pos stays at 0 but kp/kd are non-zero.
        proxy = self._require_motion_allowed()
        pos = position
        if abs(pos) < 1e-6 and (abs(kp) > 1e-9 or abs(kd) > 1e-9):
            pos = 1e-6
        proxy.set_actuator(
            slot,
            ActuatorDesire(position=pos, velocity=0.0, kp=kp, kd=kd, torque=0.0),
            send=False,
        )

    def idle_actuator(self, slot: int) -> None:
        self._require_proxy().set_actuator(slot, ActuatorDesire(), send=False)

    def idle_all_actuators(self) -> None:
        """Blank every held desire — Apply accumulates slots; MCP LEDs on
        'other' buses are usually leftover holds, not cross-rail firmware TX."""
        proxy = self._require_proxy()
        for slot in range(ACTUATOR_COUNT):
            proxy.set_actuator(slot, ActuatorDesire(), send=False)

    def set_mcu_state(self, state: int) -> None:
        self._require_proxy().hub.set_mcu_state(McuState(state))

    def set_plant_apply(self, enable: bool) -> None:
        proxy = self._require_proxy()
        if enable:
            proxy.arm_plant()
        else:
            proxy.disarm_plant()

    def recover(self) -> None:
        self._require_proxy().hub.recover()

    def hard_estop_request_path(self) -> Path:
        return self.telemetry.session_dir / SOFT_KILL_REQUEST_NAME

    def hard_estop_park(self) -> dict:
        """Explicit hard-ESTOP (Advanced panel) — the old "Soft-kill Park".

        Unrelated to the soft_kill freeze flag: this blanks desires and
        latches ``McuState.ESTOP`` (``ControlsPcbHub.soft_kill_park`` — kept
        exactly as it was, unchanged, for this purpose).
        When this process owns COM: parks directly. When following a peer's
        state.json (continuous owns CDC): writes a request flag the peer
        polls — Connect COM is not required.
        """
        hub = self.hub
        if hub is not None:
            hub.soft_kill_park()
            return {"mode": "direct", "parked": True}
        flag = self.hard_estop_request_path()
        flag.parent.mkdir(parents=True, exist_ok=True)
        flag.write_text(f"{time.time():.3f}\n", encoding="utf-8")
        return {"mode": "peer_request", "path": str(flag)}

    def held_snapshot(self) -> dict:
        """Currently-commanded plant control state — distinct from measured
        feedback (SessionState.actuators) and from whatever's sitting in an
        unapplied input box. This is what the background stream thread is
        actually resending right now, per slot."""
        hub = self.hub
        if hub is None or not hub.is_streaming:
            return {"streaming": False, "held": [None] * ACTUATOR_COUNT}
        desires = hub.held_desires()
        held = []
        for slot in range(ACTUATOR_COUNT):
            d = desires.get(slot)
            if d is None:
                held.append(None)
                continue
            active = (
                abs(d.kp) > _ACTIVE_EPS
                or abs(d.kd) > _ACTIVE_EPS
                or abs(d.velocity) > _ACTIVE_EPS
                or abs(d.torque) > _ACTIVE_EPS
            )
            held.append(
                {
                    "position": d.position,
                    "velocity": d.velocity,
                    "kp": d.kp,
                    "kd": d.kd,
                    "torque": d.torque,
                    "active": active,
                }
            )
        return {"streaming": True, "held": held}

    # -- Teleop (per-slot target+cruise slew) --------------------------------------------

    def set_cfg_map(self, cfg_map: str) -> None:
        """UI-only relabeling of which slots are "base" — see
        ``actions.teleop.build_actuator_specs``. Never touches board CFG."""
        if cfg_map not in ("bench", "product"):
            raise ValueError("cfg_map must be 'bench' or 'product'")
        self.cfg_map = cfg_map
        self._rebuild_actuator_specs()

    def set_limit_scout(self, enabled: bool) -> None:
        """Widen L-arm rails to MuJoCo soft (no clear clamp) for manual limit finding."""
        self.limit_scout = bool(enabled)
        self._rebuild_actuator_specs()

    def _rebuild_actuator_specs(self) -> None:
        from dataclasses import replace

        from deft_controls_sdk.config.actuator import LEFT_ARM_SLOTS
        from deft_controls_sdk.config.yam_limits import soft_limits_q7

        specs = build_actuator_specs(self.cfg_map)
        if self.limit_scout:
            lo, hi = soft_limits_q7("left", use_bench_clear=False)
            for i, slot in enumerate(LEFT_ARM_SLOTS):
                cur = specs.get(slot)
                if cur is None or not cur.verified:
                    continue
                specs[slot] = replace(cur, lo=float(lo[i]), hi=float(hi[i]))
                self.teleop.set_actuator_limits(slot, lo=float(lo[i]), hi=float(hi[i]))
        else:
            for slot, cur in specs.items():
                if cur.group == "arm_left" and cur.verified and cur.lo is not None and cur.hi is not None:
                    self.teleop.set_actuator_limits(slot, lo=float(cur.lo), hi=float(cur.hi))
        self.actuator_specs = specs

    def teleop_groups(self) -> dict:
        def spec_dict(spec) -> dict:
            return {
                "slot": spec.slot, "group": spec.group, "label": spec.label,
                "protocol": spec.protocol, "verified": spec.verified,
                "lo": spec.lo, "hi": spec.hi, "seed_relative": spec.seed_relative,
                "cruise_max": spec.cruise_max, "cruise_default": spec.cruise_default,
            }
        return {
            "cfg_map": self.cfg_map,
            "limit_scout": self.limit_scout,
            "actuators": {slot: spec_dict(spec) for slot, spec in self.actuator_specs.items()},
            "servos": {slot: spec_dict(spec) for slot, spec in self.servo_specs.items()},
        }

    def _teleop_feedback_snapshot(self) -> dict:
        """One telemetry read per teleop tick (not per slot) — feeds the settled-hold
        damping/flag logic in TeleopEngine, never used to move a commanded target."""
        actuators = self.telemetry.snapshot().actuators
        return {i: a for i, a in enumerate(actuators) if a is not None}

    def _current_actuator_position(self, slot: int) -> Optional[float]:
        """Seed for engaging teleop — never invent a probe pose. Prefer the
        currently-held desire (what the stream is already resending); fall
        back to the latest live feedback sample; if neither exists yet, the
        caller must refuse to engage."""
        hub = self.hub
        if hub is not None:
            held = hub.held_desire(slot)
            if held is not None:
                return held.position
        fb = self.telemetry.snapshot().actuators[slot]
        if fb is not None:
            return fb.get("position")
        return None

    def teleop_actuator_target(self, slot: int, *, target: float, cruise: float) -> None:
        spec = self.actuator_specs.get(slot)
        if spec is None:
            raise ValueError(f"slot {slot} is not a known actuator in the '{self.cfg_map}' CFG map")
        if not spec.verified:
            raise ValueError(f"{spec.label} (slot {slot}) has no live-verified range on this bench yet")
        self._require_motion_allowed()
        seed = self._current_actuator_position(slot)
        if seed is None:
            raise RuntimeError(f"no live position for slot {slot} yet — wait for feedback before teleop")
        self.teleop.engage_actuator(slot, spec=spec, seed=seed, target=target, cruise=cruise)

    def teleop_actuator_jog(self, slot: int, *, direction: int, cruise: float) -> None:
        spec = self.actuator_specs.get(slot)
        if spec is None:
            raise ValueError(f"slot {slot} is not a known actuator in the '{self.cfg_map}' CFG map")
        if not spec.verified:
            raise ValueError(f"{spec.label} (slot {slot}) has no live-verified range on this bench yet")
        self._require_motion_allowed()
        seed = self._current_actuator_position(slot)
        if seed is None:
            raise RuntimeError(f"no live position for slot {slot} yet — wait for feedback before teleop")
        self.teleop.jog_actuator(slot, spec=spec, seed=seed, direction=direction, cruise=cruise)

    def teleop_actuator_stop(self, slot: int) -> None:
        self.teleop.stop_actuator(slot)

    def teleop_stop_all(self) -> None:
        """Freeze every teleop slot in place (holding torque) — not idle/slack."""
        for slot in list(self.actuator_specs.keys()):
            try:
                self.teleop.stop_actuator(int(slot))
            except Exception:
                pass
        for slot in list(self.servo_specs.keys()):
            try:
                self.teleop.stop_servo(int(slot))
            except Exception:
                pass

    def _current_servo_position(self, slot: int) -> Optional[float]:
        hub = self.hub
        if hub is None:
            return None
        held = hub.held_servo(slot)
        return held.native_step_position if held is not None else None

    def teleop_servo_target(self, slot: int, *, target: float, cruise: float) -> None:
        spec = self.servo_specs.get(slot)
        if spec is None:
            raise ValueError(f"slot {slot} is not a neck servo slot (0=pitch, 1=yaw)")
        self._require_motion_allowed()
        self.teleop.engage_servo(slot, spec=spec, target=target, cruise=cruise)

    def teleop_servo_stop(self, slot: int) -> None:
        self.teleop.stop_servo(slot)

    def teleop_servo_idle(self, slot: int) -> None:
        """Release torque on one neck servo only — the same per-slot clear
        fallback ``idle_group("neck")`` uses for both, here scoped to one."""
        spec = self.servo_specs.get(slot)
        if spec is None:
            raise ValueError(f"slot {slot} is not a neck servo slot (0=pitch, 1=yaw)")
        proxy = self._require_proxy()
        self.teleop.disengage_servo(slot)
        proxy.set_servo(slot, ServoDesire(servo_id=0), send=False)

    def idle_group(self, group: str) -> None:
        """Blank every desire in one teleop group at once — always safe
        regardless of whether the group's range is live-verified (Idle never
        commands a position, unlike target-teleop)."""
        proxy = self._require_proxy()
        if group == "neck":
            for slot in self.servo_specs:
                self.teleop.disengage_servo(slot)
            proxy.hub.clear_servos(send=False)
            return
        if group not in ("base", "arm_left", "arm_right"):
            raise ValueError("group must be one of: base, arm_left, arm_right, neck")
        for slot, spec in self.actuator_specs.items():
            if spec.group == group:
                self.teleop.disengage_actuator(slot)
                proxy.set_actuator(slot, ActuatorDesire(), send=False)

    # -- Continuous mode (SSH launch/stop on the Jetson) ----------------------------------

    def launch_continuous(self, *, duration_s: float = 0.0) -> dict:
        with self._continuous_lock:
            self._continuous_status = {"state": "launching", "detail": None}

        def _run() -> None:
            try:
                result = self.continuous_launcher(duration_s=duration_s)
                status = {"state": "launched", "detail": result}
            except Exception as exc:  # noqa: BLE001 — surface to the UI, don't crash the thread
                status = {"state": "error", "detail": str(exc)}
            with self._continuous_lock:
                self._continuous_status = status

        threading.Thread(target=_run, name="deft-dashboard-continuous-launch", daemon=True).start()
        return {"state": "launching"}

    def stop_continuous(self) -> dict:
        with self._continuous_lock:
            self._continuous_status = {"state": "stopping", "detail": None}

        def _run() -> None:
            try:
                result = self.continuous_stopper()
                status = {"state": "stopped", "detail": result}
            except Exception as exc:  # noqa: BLE001
                status = {"state": "error", "detail": str(exc)}
            with self._continuous_lock:
                self._continuous_status = status

        threading.Thread(target=_run, name="deft-dashboard-continuous-stop", daemon=True).start()
        return {"state": "stopping"}

    def continuous_status(self) -> dict:
        with self._continuous_lock:
            return dict(self._continuous_status)
