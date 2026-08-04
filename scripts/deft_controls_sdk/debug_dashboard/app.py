"""Localhost controller + telemetry dashboard.

Run:

    python -m deft_controls_sdk.debug_dashboard
    # browser: http://127.0.0.1:8766  -> pick a port, click Connect (observe)

    python -m deft_controls_sdk.debug_dashboard --port COM5
    # same UI, auto-connects in observe mode at launch

Connect defaults to **observe**: plant stream clears HOST_STALE, MCU stays
plant_apply=0 (observe), and soft-kill auto-park hooks are off — so opening the dashboard
does not latch ESTOP / yellow-red PDU LEDs from a residual soft-kill or V/I
check. Opt into plant control explicitly (Enable control → NORMAL + hooks).

HTTP defaults to **8766** so it does not collide with ``pdb_uart_sim.py
--control-port 8765``.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Literal, Optional
from urllib.parse import urlparse

from deft_controls_sdk import ActuatorDesire, HostProxy, McuState, ServoDesire
from deft_controls_sdk.actions import (
    build_actuator_specs,
    build_servo_specs,
    make_teleop_engine,
)
from deft_controls_sdk.config import assembly_from_name
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD, list_ports_info
from deft_controls_sdk.telemetry import TelemetryCache, default_session_dir

from . import remote_continuous

# Peer (e.g. yam_continuous_all) owns CDC while dashboard follows state.json —
# Soft-kill Park writes this flag; the peer parks and deletes it.
SOFT_KILL_REQUEST_NAME = "soft_kill_request"
# Refuse HostProxy.connect while a peer's state.json still looks live — opening
# COM would collide with the CDC owner (Windows exclusive open / dual writers).
PEER_OWNER_MAX_AGE_S = 2.0

ControlMode = Literal["observe", "control"]
DEFAULT_HTTP_PORT = 8766

_ACTIVE_EPS = 0.01
"""Matches firmware's actuator_any_non_idle_live() threshold — see
App/Src/plant/actuator.c. A held desire counts as "active" (not just idle
hold-position) if kp/kd/|velocity|/|torque| exceed this."""


class AppState:
    """Owns the (optional) HostProxy session across connect/disconnect cycles.

    One TelemetryCache lives for the whole process lifetime, independent of
    any particular connection — /api/state always has something sane to
    return, even before the first Connect, and fault history / ring buffer
    survive a board reset instead of resetting on every reconnect.

    UI ``observe`` / ``control`` map onto HostProxy ``armed=False`` /
    ``arm_plant()`` — same session shape as notebooks and pcb_lab.debug.
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
        self.control_mode: ControlMode = "observe"

        # Teleop (per-slot target+cruise slew — actions.TeleopEngine) and its
        # slot-group model. "bench" is the default CFG map because that's what's
        # actually wired/live-verified on the current bench (base on slots 22-25)
        # — see base-robstride-mcp.md's "Known falsehoods retired". "product" is
        # offered purely as the wizard's CFG-map hint.
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

    @property
    def hub(self):
        """Wire hub for the active session, or None when disconnected."""
        return None if self.proxy is None else self.proxy.hub

    @property
    def connected(self) -> bool:
        return self.proxy is not None

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
        mode: ControlMode = "observe",
    ) -> None:
        """Open CDC via HostProxy and start the plant stream.

        Default ``mode="observe"``: ``armed=False`` (plant_apply=0) + no auto
        soft-kill park — safe telemetry without faulting the board on Connect.
        Pass ``mode="control"`` only when the operator wants plant apply + park
        hooks (``arm_plant()``).

        Refuses to open COM while a peer is actively publishing ``state.json``
        for this session dir — stay in follow mode instead of colliding.
        """
        if mode not in ("observe", "control"):
            raise ValueError("mode must be 'observe' or 'control'")
        with self._lock:
            if self.proxy is not None:
                raise RuntimeError(
                    f"already connected to {self.proxy.hub.port} — disconnect first"
                )
            peer = self.peer_com_owner()
            if peer is not None:
                peer_port = peer.get("port") or "?"
                raise RuntimeError(
                    f"COM already owned by a peer writing {peer['path']} "
                    f"(port={peer_port}, age={peer['age_s']:.1f}s). "
                    "Stay in follow mode — do not Connect while continuous/teleop "
                    "owns CDC. Stop the peer (or Soft-kill Park) first."
                )
            # Desk bringup: share this process TelemetryCache so /api/state and
            # fault history survive reconnect. armed=False ≡ UI observe.
            proxy = HostProxy.connect(
                port,
                baud=baud,
                stream_hz=self._stream_hz,
                telemetry_hz=self._telemetry_hz,
                armed=False,
                listen_pdu=False,
                telemetry=self.telemetry,
                assembly=assembly_from_name(self.cfg_map),
                mode="bandwidth",
            )
            self.proxy = proxy
            self.control_mode = "observe"
            if mode == "control":
                self._enter_control_locked()

    def set_control_mode(self, mode: ControlMode) -> None:
        if mode not in ("observe", "control"):
            raise ValueError("mode must be 'observe' or 'control'")
        with self._lock:
            proxy = self.proxy
            if proxy is None:
                raise RuntimeError("not connected")
            if mode == "control":
                self._enter_control_locked()
            else:
                self.teleop.disengage_all()
                hub = proxy.hub
                hub.set_auto_soft_kill(False)
                for slot in range(ACTUATOR_COUNT):
                    proxy.set_actuator(slot, ActuatorDesire(), send=False)
                proxy.disarm_plant()
                self.control_mode = "observe"

    def _enter_control_locked(self) -> None:
        proxy = self.proxy
        assert proxy is not None
        proxy.hub.set_auto_soft_kill(True)
        proxy.arm_plant()
        self.control_mode = "control"

    def set_listen_pdu(self, enabled: bool) -> None:
        """Toggle whether the host obeys PDB kill-state for soft-kill/LED policy.

        Independent of Enable control / observe. Off (bench with no PDU peer)
        is the dashboard's Connect default so ``set_auto_soft_kill(True)``
        during Enable control is a deliberate no-op — see
        ``ControlsPcbHub.set_auto_soft_kill``. Flip this on when a real PDU
        (or pdb_uart_sim) is actually present on the bench and you want its
        SOFT_KILL_REQ / bad V-I to auto-park the plant.
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
                # Leave plant_apply off so disconnect does not freeze an ESTOP
                # latch from a soft-kill park that happened while connected.
                try:
                    proxy.hub.set_auto_soft_kill(False)
                    proxy.disarm_plant()
                    time.sleep(0.05)
                except Exception:
                    pass
                try:
                    proxy.close()
                except Exception:
                    pass
                self.proxy = None
            self.control_mode = "observe"

    def _require_proxy(self) -> HostProxy:
        proxy = self.proxy
        if proxy is None:
            raise RuntimeError("not connected")
        return proxy

    def set_actuator(self, slot: int, *, position: float, kp: float, kd: float) -> None:
        # send=False: only update the held desire. The background stream loop
        # already write()s the plant image at stream_hz — a second write+flush
        # from this HTTP thread contended on the serial lock and stalled fb_hz.
        #
        # HOME_POS_EPS: avoid true blank (pos=0, kp=0) when the operator raises
        # gains — shared blank-bus skip drops uncommanded buses (FDCAN and MCP).
        # Match legacy eps when pos stays at 0 but kp/kd are non-zero.
        pos = position
        if abs(pos) < 1e-6 and (abs(kp) > 1e-9 or abs(kd) > 1e-9):
            pos = 1e-6
        self._require_proxy().set_actuator(
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

    def soft_kill_request_path(self) -> Path:
        return self.telemetry.session_dir / SOFT_KILL_REQUEST_NAME

    def soft_kill_park(self) -> dict:
        """Product soft-kill park (see ControlsPcbHub.soft_kill_park).

        When this process owns COM: clear desires and latch McuState.ESTOP.
        When following a peer's state.json (continuous owns CDC): write a
        request flag the peer polls — Connect COM is not required.
        """
        hub = self.hub
        if hub is not None:
            hub.soft_kill_park()
            return {"mode": "direct", "parked": True}
        flag = self.soft_kill_request_path()
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
                self.teleop.set_actuator_limits(
                    slot, lo=float(lo[i]), hi=float(hi[i])
                )
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
        """Seed for engaging teleop — never invent a probe pose (see
        base-robstride-mcp.md). Prefer the currently-held desire (what the
        stream is already resending); fall back to the latest live feedback
        sample; if neither exists yet, the caller must refuse to engage."""
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
        self._require_proxy()
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
        self._require_proxy()
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
        self._require_proxy()
        self.teleop.engage_servo(slot, spec=spec, target=target, cruise=cruise)

    def teleop_servo_stop(self, slot: int) -> None:
        self.teleop.stop_servo(slot)

    def teleop_servo_idle(self, slot: int) -> None:
        """Release torque on one neck servo only — see dxl-neck.md's
        ``ServoDesire(servo_id=0)`` per-slot clear fallback (the same one
        ``idle_group("neck")`` uses for both, here scoped to one)."""
        spec = self.servo_specs.get(slot)
        if spec is None:
            raise ValueError(f"slot {slot} is not a neck servo slot (0=pitch, 1=yaw)")
        proxy = self._require_proxy()
        self.teleop.disengage_servo(slot)
        proxy.set_servo(slot, ServoDesire(servo_id=0), send=False)

    def idle_group(self, group: str) -> None:
        """Blank every desire in one teleop group at once — the "idle
        movement for each base actuator / neck / arm" ask. Always safe
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


_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Deft controls — telemetry</title>
<style>
  :root {
    --bg: #0e1015;
    --panel: #1a1e2b;
    --panel-2: #232838;
    --text: #eceff4;
    --muted: #9aa3b5;
    --muted-2: #6b7387;
    --accent: #5b9cf6;
    --accent-dim: color-mix(in srgb, var(--accent) 18%, transparent);
    --green: #3dcf8e;
    --yellow: #e6c35c;
    --red: #ea6a6a;
    --line: #2a3144;
    --radius: 10px;
    --header-h: 3.6rem;
  }
  * { box-sizing: border-box; }
  ::selection { background: var(--accent-dim); }
  body {
    margin: 0; font-family: "Segoe UI", system-ui, sans-serif;
    background: var(--bg); color: var(--text);
    min-height: 100vh; line-height: 1.45;
  }
  a { color: var(--accent); }
  header {
    position: sticky; top: 0; z-index: 20; height: var(--header-h);
    padding: 0 1.25rem; border-bottom: 1px solid var(--line);
    display: flex; flex-wrap: wrap; gap: 0.9rem; align-items: center;
    background: color-mix(in srgb, var(--bg) 88%, transparent);
    backdrop-filter: blur(8px); -webkit-backdrop-filter: blur(8px);
  }
  header h1 { font-size: 1rem; font-weight: 700; margin: 0; letter-spacing: 0.01em; }
  header .meta { color: var(--muted); font-size: 0.82rem; }
  .spacer { flex: 1 1 auto; }
  .grade {
    font-weight: 700; text-transform: uppercase; letter-spacing: 0.06em;
    padding: 0.25rem 0.65rem; border-radius: 999px; font-size: 0.72rem;
    border: 1px solid transparent;
  }
  .grade.green { background: color-mix(in srgb, var(--green) 20%, transparent); color: var(--green); border-color: color-mix(in srgb, var(--green) 40%, transparent); }
  .grade.yellow { background: color-mix(in srgb, var(--yellow) 20%, transparent); color: var(--yellow); border-color: color-mix(in srgb, var(--yellow) 40%, transparent); }
  .grade.red { background: color-mix(in srgb, var(--red) 20%, transparent); color: var(--red); border-color: color-mix(in srgb, var(--red) 40%, transparent); }
  .grade.idle { background: color-mix(in srgb, var(--muted) 16%, transparent); color: var(--muted); border-color: var(--line); }
  .banner {
    margin: 0.6rem 0 0; padding: 0.55rem 0.75rem; border-radius: 8px;
    background: color-mix(in srgb, var(--yellow) 10%, transparent);
    border: 1px solid color-mix(in srgb, var(--yellow) 30%, var(--line));
    color: var(--text); font-size: 0.8rem; line-height: 1.4;
  }
  .banner.ok {
    background: color-mix(in srgb, var(--green) 8%, transparent);
    border-color: color-mix(in srgb, var(--green) 26%, var(--line));
  }
  .banner.warn {
    background: color-mix(in srgb, var(--red) 10%, transparent);
    border-color: color-mix(in srgb, var(--red) 30%, var(--line));
  }
  main { padding: 1.1rem 1.25rem 3rem; display: grid; gap: 0.9rem; max-width: 1180px; margin: 0 auto; }
  .card {
    background: var(--panel); border: 1px solid var(--line); border-radius: var(--radius);
    padding: 1rem 1.15rem; box-shadow: 0 1px 0 rgba(0,0,0,0.15);
  }
  .card.tight { padding: 0.75rem 0.9rem; }
  .card h2 { margin: 0 0 0.75rem; font-size: 0.75rem; color: var(--muted);
    text-transform: uppercase; letter-spacing: 0.09em; font-weight: 700; }
  .card summary { cursor: pointer; list-style-position: outside; padding: 0.1rem 0; }
  .card summary:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; border-radius: 4px; }
  .card summary h2 { display: inline; margin: 0; }
  .card details[open] > summary { margin-bottom: 0.6rem; }
  .card details.sub { margin-top: 0.6rem; border-top: 1px dashed var(--line); padding-top: 0.6rem; }
  .card details.sub summary { font-size: 0.72rem; color: var(--muted-2); text-transform: uppercase; letter-spacing: 0.06em; font-weight: 600; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(140px, 1fr)); gap: 0.75rem; }
  .grid.compact { grid-template-columns: repeat(auto-fill, minmax(112px, 1fr)); gap: 0.6rem; }
  .kv .k { display: block; color: var(--muted); font-size: 0.68rem; text-transform: uppercase; letter-spacing: 0.05em; }
  .kv .v { font-variant-numeric: tabular-nums; font-size: 1.02rem; margin-top: 0.15rem; }
  .grid.compact .kv .v { font-size: 0.88rem; }
  .kv.wide { grid-column: 1 / -1; }
  .kv.wide .v { font-size: 0.78rem; color: var(--muted); font-variant-numeric: normal; }
  ul.context { margin: 0; padding-left: 1.1rem; color: var(--muted); }
  ul.context li { margin: 0.25rem 0; }
  table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; font-size: 0.85rem; }
  th, td { text-align: left; padding: 0.4rem 0.45rem; border-bottom: 1px solid var(--line); }
  th { color: var(--muted); font-weight: 600; font-size: 0.68rem; text-transform: uppercase; letter-spacing: 0.03em; }
  tbody tr:hover { background: color-mix(in srgb, var(--panel-2) 60%, transparent); }
  .summary { font-size: 1.1rem; margin: 0 0 0.5rem; font-weight: 600; }
  button, select, input {
    font-family: inherit; font-size: 0.82rem; background: var(--panel-2); color: var(--text);
    border: 1px solid var(--line); border-radius: 7px; padding: 0.4rem 0.7rem;
  }
  select, input { background: var(--bg); }
  button { cursor: pointer; font-weight: 600; transition: border-color 0.12s, background 0.12s, opacity 0.12s; }
  button:hover:not(:disabled) { border-color: var(--muted-2); }
  button:focus-visible, select:focus-visible, input:focus-visible { outline: 2px solid var(--accent); outline-offset: 1px; }
  button:disabled { opacity: 0.38; cursor: not-allowed; }
  button.primary { background: color-mix(in srgb, var(--accent) 22%, var(--panel-2)); border-color: color-mix(in srgb, var(--accent) 55%, var(--line)); color: #eaf1ff; }
  button.primary:hover:not(:disabled) { border-color: var(--accent); }
  button.record.on { background: color-mix(in srgb, var(--red) 25%, transparent); color: var(--red); border-color: var(--red); }
  button.estop, button.btn-danger { color: var(--red); border-color: color-mix(in srgb, var(--red) 55%, var(--line)); }
  button.estop:hover:not(:disabled), button.btn-danger:hover:not(:disabled) { background: color-mix(in srgb, var(--red) 14%, transparent); }
  button.btn-go { background: color-mix(in srgb, var(--accent) 20%, var(--panel-2)); border-color: color-mix(in srgb, var(--accent) 50%, var(--line)); }
  button.btn-stop { color: var(--green); border-color: color-mix(in srgb, var(--green) 45%, var(--line)); }
  button.btn-stop:hover:not(:disabled) { background: color-mix(in srgb, var(--green) 12%, transparent); }
  button.btn-idle { color: var(--yellow); border-color: color-mix(in srgb, var(--yellow) 45%, var(--line)); }
  button.btn-idle:hover:not(:disabled) { background: color-mix(in srgb, var(--yellow) 12%, transparent); }
  button.btn-ghost { background: transparent; }
  .segmented { display: inline-flex; border: 1px solid var(--line); border-radius: 7px; overflow: hidden; }
  .segmented button { border: none; border-radius: 0; background: var(--panel-2); }
  .segmented button:not(:last-child) { border-right: 1px solid var(--line); }
  .segmented button.active { background: color-mix(in srgb, var(--accent) 24%, var(--panel-2)); color: #eaf1ff; }
  .fault-badge { color: var(--red); font-weight: 600; }
  .row { display: flex; gap: 0.5rem; align-items: center; flex-wrap: wrap; }
  .row.toolbar {
    position: sticky; top: var(--header-h); z-index: 10; background: var(--panel);
    padding: 0.5rem 0; margin: -0.2rem 0 0.7rem; border-bottom: 1px solid var(--line);
  }
  .err { color: var(--red); font-size: 0.8rem; min-height: 1.1em; margin: 0.4rem 0 0; }
  input[type=number] { width: 4.5rem; }
  .streaming-badge { font-size: 0.8rem; color: var(--muted); margin-left: auto; }
  .streaming-badge.on { color: var(--green); }
  .active-badge { font-weight: 700; }
  .active-badge.active { color: var(--yellow); }
  .active-badge.idle { color: var(--muted); }
  td.held { font-variant-numeric: tabular-nums; }

  /* Teleop joint cards */
  .joint-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 0.6rem; }
  .joint-card {
    border: 1px solid var(--line); border-radius: 9px; padding: 0.6rem 0.7rem;
    background: var(--panel-2); font-size: 0.82rem; transition: border-color 0.12s, box-shadow 0.12s;
  }
  .joint-card.disabled { opacity: 0.5; }
  .joint-card.selected { border-color: var(--accent); box-shadow: 0 0 0 1px var(--accent); }
  .joint-card .jc-head { display: flex; justify-content: space-between; align-items: baseline; gap: 0.4rem; margin-bottom: 0.4rem; }
  .joint-card .jc-label { font-weight: 700; }
  .joint-card .jc-sub { color: var(--muted); font-size: 0.7rem; }
  .joint-card .jc-slider { display: flex; align-items: center; gap: 0.5rem; margin: 0.35rem 0; }
  .joint-card input[type=range] { flex: 1 1 auto; width: auto; accent-color: var(--accent); }
  .joint-card .val { font-variant-numeric: tabular-nums; text-align: right; min-width: 3.4rem; color: var(--text); }
  .joint-card .jc-foot { display: flex; align-items: center; gap: 0.4rem; flex-wrap: wrap; margin-top: 0.4rem; }
  .joint-card .jc-foot input[type=number] { width: 4rem; }
  .joint-card button { padding: 0.3rem 0.5rem; font-size: 0.76rem; }
  .joint-card .btn-group { display: flex; gap: 0.3rem; }
  .joint-card .cmd { flex-basis: 100%; color: var(--muted); font-size: 0.72rem; margin-top: 0.15rem; }
  .joint-card .cmd.flagged { color: var(--yellow); font-weight: 600; }
  .joint-card .jc-limits {
    display: flex; align-items: center; gap: 0.35rem; flex-wrap: wrap;
    margin-top: 0.3rem; font-size: 0.72rem; color: var(--muted);
  }
  .joint-card .jc-limits .fb { font-variant-numeric: tabular-nums; color: var(--text); min-width: 5.5rem; }
  .joint-card .jc-limits .cap { font-variant-numeric: tabular-nums; }
  .teleop-group { margin-top: 0.85rem; }
  .teleop-group + .teleop-group { border-top: 1px solid var(--line); padding-top: 0.75rem; }
  .teleop-group summary { display: flex; align-items: baseline; gap: 0.5rem; }
  .teleop-group summary .group-title { font-size: 0.85rem; font-weight: 700; }
  .teleop-group[open] summary { margin-bottom: 0.5rem; }
</style>
</head>
<body>
<header>
  <h1>Deft controls telemetry</h1>
  <span id="grade" class="grade idle">idle</span>
  <span class="meta" id="meta">not connected</span>
  <span class="spacer"></span>
  <span class="streaming-badge" id="streamingBadge">streaming: —</span>
</header>
<main>
  <section class="card">
    <h2>Connection</h2>
    <div class="row">
      <select id="portSelect"></select>
      <button class="primary" id="connectBtn" onclick="connect()">Connect (observe)</button>
      <span class="segmented">
        <button id="enableCtrlBtn" onclick="enableControl()" disabled title="Switch MCU to NORMAL and plant_apply=1. The instant this arms, firmware starts mounting every CFG-enabled actuator's held desire each tick and transmitting it on that actuator's CAN bus — every configured channel starts TX'ing/blinking at once, not just the one you move. See the Control banner below.">Enable control</button>
        <button id="observeBtn" onclick="toObserve()" disabled title="Blank desires, plant_apply=0, no auto-park">Back to observe</button>
      </span>
      <button class="btn-ghost" id="disconnectBtn" onclick="disconnect()" disabled>Disconnect</button>
      <span class="meta" id="connMeta">not connected</span>
    </div>
    <p class="banner ok" id="modeBanner">Idle — not owning COM. Connect opens observe mode (plant_apply=0); plant motors stay gated until Enable control.</p>
    <p class="err" id="connError"></p>
    <p class="meta" id="sessionMeta"></p>
  </section>

  <section class="card">
    <h2>PDU / soft-kill</h2>
    <div class="grid compact" id="pduGlance"></div>
    <div class="row" style="margin-top:0.6rem">
      <button class="estop" id="softKillBtn" onclick="softKillPark()"
        title="Independent of Enable control / ESTOP button — works in observe too, and in follow mode signals the CDC-owning peer">Soft-kill Park</button>
      <span class="meta" id="pdbMeta"></span>
    </div>
    <div class="row" style="margin-top:0.5rem">
      <button id="listenPduBtn" onclick="toggleListenPdu()" disabled
        title="Host obeys PDB kill-state for auto soft-kill park + LED policy only while this is ON. OFF (the Connect default) is a bare bench with no PDU peer — Enable control's auto-park hook is armed but is a deliberate no-op until you turn this on. Turn ON only when a real PDU or pdb_uart_sim is actually wired up.">listen_pdu: OFF</button>
      <span class="meta">no PDU peer on the bench? leave this OFF — Enable control still works, it just won't auto-park from PDU kill/V-I</span>
    </div>
    <p class="err" id="pdbError"></p>
    <p class="meta" id="pdbResult"></p>
    <details class="sub">
      <summary>Raw PDB telemetry (pack/rail V·I, peer fields)</summary>
      <div class="grid compact" id="pduAdvanced" style="margin-top:0.5rem"></div>
    </details>
  </section>

  <section class="card">
    <h2>Teleop</h2>
    <div class="row toolbar">
      <label class="meta">Slot labels
        <select id="cfgMapSelect" onchange="setCfgMap()"
          title="UI-only relabeling of which slots this Teleop panel shows as 'base' vs 'arm' — picks labels/ranges to display, nothing more. This is NOT the board's real CFG table (per-slot enabled/bus/protocol/motor_id, stored in MCU NVM) and never writes to the board. To read or edit the real CFG table, use pcb_lab: python -m pcb_lab.debug --port COMx show --cfg / set --cfg (needs its own debug-mode session, separate from this dashboard's live stream).">
          <option value="bench">bench (22–25, live-verified on this rig)</option>
          <option value="product">product (14–19, not wired on this bench)</option>
        </select>
      </label>
      <button class="btn-stop" onclick="teleopStopAll()" title="Freeze all teleop slots — keeps holding torque, does not go slack">&#9616;&#9616; Stop all (Space)</button>
      <span class="spacer"></span>
      <label class="meta" title="Widen L-arm rails to MuJoCo soft limits so you can nudge past the current clear envelope and Mark lo/hi from live encoder FB. Does not write files — copy the summary yourself.">
        <input type="checkbox" id="limitScout" onchange="setLimitScout()"> Scout limits
      </label>
      <span class="meta">selected: <b id="keysArmSel">—</b> · click a joint card, then ←/→</span>
    </div>
    <p class="banner" style="margin-top:0.4rem">"Slot labels" only changes which slots this panel calls base/arm and their ranges — it's a display picker, not the board's CFG table. Real per-slot CFG (enabled/bus/protocol/motor_id) lives in MCU NVM; view/edit it with pcb_lab: <code>python -m pcb_lab.debug --port COMx show --cfg</code> (read-only) or <code>set --cfg</code> (edit), in a separate debug-mode session — not from this dashboard.</p>
    <div class="row">
      <button class="btn-idle" onclick="idleGroup('arm_left')" title="Blanks ALL 7 left-arm joints at once — every joint loses holding torque simultaneously. For one joint, use that joint's own Idle button below.">Idle L-arm (all 7)</button>
      <button class="btn-idle" onclick="idleGroup('arm_right')" title="Blanks ALL 7 right-arm joints at once.">Idle R-arm (all 7)</button>
      <button class="btn-idle" onclick="idleGroup('base')">Idle base (all)</button>
      <button class="btn-idle" onclick="idleGroup('neck')">Idle neck (both)</button>
      <button onclick="copyCapturedLimits()" title="Copy marked lo/hi as Python tuples">Copy marked limits</button>
      <button onclick="clearCapturedLimits()">Clear marks</button>
    </div>
    <pre class="meta" id="capturedLimitsOut" style="margin-top:0.4rem;white-space:pre-wrap">Marked limits: (none yet — select a joint, ←/→ nudge, Mark lo / Mark hi at the edges)</pre>
    <p class="banner warn" style="margin-top:0.5rem"><b>Stop</b> (green) freezes a joint in place, holding torque. <b>Idle</b> (yellow) zeroes torque — that joint goes slack and can swing/drop under gravity or a neighbor's load. Prefer Stop unless you actually want it slack. The group buttons above idle every joint in that group at once; use a card's own Idle for just one joint.</p>
    <p class="banner" id="keysBanner" style="margin-top:0.4rem">Keyboard (Enable control; not while typing in a field): <code>Space</code> stop all · click joint or <code>1</code>–<code>7</code> select · <code>←</code>/<code>→</code> (or <code>[</code>/<code>]</code>) nudge selected arm · <code>↑</code>/<code>↓</code> neck pitch · <code>,</code>/<code>.</code> neck yaw · <code>z</code>/<code>x</code> <code>c</code>/<code>v</code> jog base</p>
    <p class="err" id="teleopError"></p>

    <details class="teleop-group" open>
      <summary><span class="group-title">Arm — left</span><span class="meta">bench live-verified · drag = target, host walks there</span></summary>
      <div class="joint-grid" id="armLeftRows"></div>
    </details>
    <details class="teleop-group">
      <summary><span class="group-title">Arm — right</span><span class="meta">no live-verified range on this bench yet — Idle only</span></summary>
      <div class="joint-grid" id="armRightRows"></div>
    </details>
    <details class="teleop-group" open>
      <summary><span class="group-title">Base actuators</span><span class="meta">speed + start/stop · bench CH5/CH6, slots 22–25</span></summary>
      <div class="joint-grid" id="baseRows"></div>
    </details>
    <details class="teleop-group" open>
      <summary><span class="group-title">Neck</span><span class="meta">2x Dynamixel</span></summary>
      <div class="joint-grid" id="neckRows"></div>
    </details>
  </section>

  <section class="card">
    <p class="summary" id="summary">—</p>
    <ul class="context" id="context"></ul>
    <div class="grid compact" id="healthGlance" style="margin-top:0.75rem"></div>
    <div class="row" style="margin-top:0.75rem">
      <button class="record" id="recordBtn" onclick="toggleRecord()">&#9679; Record</button>
      <span class="meta" id="recordMeta"></span>
    </div>
    <details class="sub">
      <summary>Black box (fault capture)</summary>
      <div class="grid compact" id="blackbox" style="margin-top:0.5rem"></div>
      <div class="row" style="margin-top:0.5rem">
        <button class="btn-idle" id="clearFaultsBtn" onclick="clearFaultLog()">Clear fault log</button>
        <span class="meta" id="clearFaultsMeta"></span>
      </div>
    </details>
    <details class="sub">
      <summary>Advanced timing diagnostics</summary>
      <div class="grid compact" id="healthAdvanced" style="margin-top:0.5rem"></div>
    </details>
  </section>

  <section class="card">
    <details id="plantControlDetails">
      <summary><h2 style="display:inline">Advanced: raw per-slot Apply</h2> <span class="meta" id="plantControlSummaryMeta"></span></summary>
      <div class="row" style="margin-top:0.75rem">
        <button id="mcuNormal" onclick="setMcuState(0)">NORMAL</button>
        <button id="mcuRecovery" onclick="setMcuState(1)">RECOVERY</button>
        <button id="mcuApplyOff" onclick="setPlantApply(false)" title="plant_apply=0 — observe, no actuator mount">APPLY_OFF</button>
        <button id="mcuApplyOn" onclick="setPlantApply(true)" title="plant_apply=1 — arm plant apply">APPLY_ON</button>
        <button id="mcuEstop" class="estop" onclick="setMcuState(3)"
          title="Control-only latch — needs Control mode (Enable control) to be armed. Not the same as Soft-kill Park, which works without Control mode.">ESTOP</button>
        <button id="recoverBtn" onclick="recover()">Recover</button>
        <button id="idleAllBtn" onclick="idleAll()">Idle all slots</button>
      </div>
      <p class="err" id="ctrlError"></p>
      <p class="meta">rarely needed — prefer Teleop above for normal driving. This is the raw per-slot position/kp/kd path.</p>
      <div style="overflow-x:auto">
        <table>
          <thead>
            <tr>
              <th>slot</th><th colspan="5">feedback (measured)</th>
              <th colspan="4">commanded (held — what's actually being sent)</th>
              <th colspan="4">apply new command</th>
            </tr>
            <tr>
              <th></th>
              <th>pos</th><th>vel</th><th>τ</th><th>temp</th><th>fault</th>
              <th>state</th><th>pos</th><th>kp</th><th>kd</th>
              <th>set pos</th><th>kp</th><th>kd</th><th></th>
            </tr>
          </thead>
          <tbody id="acts"></tbody>
        </table>
      </div>
    </details>
  </section>

  <section class="card">
    <details id="continuousDetails">
      <summary><h2 style="display:inline">Continuous mode</h2></summary>
      <div class="row">
        <button id="continuousLaunchBtn" onclick="continuousLaunch()">Launch continuous</button>
        <label class="meta">duration s (0 = until stopped) <input type="number" id="continuousDuration" value="0" step="1" style="width:5rem"></label>
        <button class="btn-danger" id="continuousStopBtn" onclick="continuousStop()">Stop continuous (hard)</button>
        <span class="meta" id="continuousStatus"></span>
      </div>
      <p class="banner" id="continuousBanner">Launch runs <code>yam_continuous_all.py</code> on the Jetson over SSH (syncs current files first) — the CDC port then belongs to that process, not this dashboard. Disconnect COM here before launching. Prefer Soft-kill Park above to stop it cleanly; "Stop continuous (hard)" is the pkill+CAN-blank fallback for a wedged process.</p>
    </details>
  </section>
</main>
<script>
function kv(k, v, cls) {
  return `<div class="kv${cls ? " " + cls : ""}"><span class="k">${k}</span><div class="v">${v ?? "—"}</div></div>`;
}
function fmt(n, d=2) {
  if (n === null || n === undefined) return "—";
  if (typeof n === "number") return Number.isInteger(n) ? String(n) : n.toFixed(d);
  return String(n);
}
function setConnError(msg) { document.getElementById("connError").textContent = msg || ""; }
function setCtrlError(msg) { document.getElementById("ctrlError").textContent = msg || ""; }
function setPdbError(msg) { document.getElementById("pdbError").textContent = msg || ""; }

const PEER_KILL_STATE_NAMES = {0: "normal", 1: "soft_kill_req", 2: "soft_kill_ready", 3: "hard_estop"};
function fmtVec(vec, d=2) {
  if (!vec) return "—";
  return vec.map(v => fmt(v, d)).join(" / ");
}

async function postAction(path, body, errSetter) {
  try {
    const opts = { method: "POST" };
    if (body !== undefined) {
      opts.headers = { "Content-Type": "application/json" };
      opts.body = JSON.stringify(body);
    }
    const r = await fetch(path, opts);
    const data = await r.json();
    if (!r.ok) {
      errSetter(data.error || `request failed (${r.status})`);
    } else {
      errSetter("");
    }
  } catch (e) {
    errSetter(String(e));
  }
  tick();
}

async function loadPorts() {
  try {
    const r = await fetch("/api/ports");
    const data = await r.json();
    const sel = document.getElementById("portSelect");
    const current = sel.value;
    sel.innerHTML = (data.ports || []).map(p =>
      `<option value="${p.device}">${p.device}${p.is_stm32_cdc ? " (STM32 USB CDC)" : ""}</option>`
    ).join("") || `<option value="">no ports found</option>`;
    if (current) sel.value = current;
  } catch (e) {
    setConnError("port list failed: " + e);
  }
}

function connect() {
  const port = document.getElementById("portSelect").value;
  if (!port) { setConnError("choose a port first"); return; }
  document.getElementById("connectBtn").disabled = true;
  // Always observe first — never latches ESTOP / plant apply on Connect.
  postAction("/api/connect", { port, mode: "observe" }, setConnError);
}
function disconnect() {
  document.getElementById("disconnectBtn").disabled = true;
  postAction("/api/disconnect", {}, setConnError);
}
function enableControl() { postAction("/api/control_mode", { mode: "control" }, setConnError); }
function toObserve() { postAction("/api/control_mode", { mode: "observe" }, setConnError); }
let lastListenPdu = false;
function toggleListenPdu() { postAction("/api/listen_pdu", { enable: !lastListenPdu }, setPdbError); }
function clearFaultLog() {
  const meta = document.getElementById("clearFaultsMeta");
  meta.textContent = "clearing…";
  postAction("/api/faults/clear", {}, (msg) => {
    meta.textContent = msg || `cleared at ${new Date().toLocaleTimeString()}`;
  });
}
function setMcuState(n) { postAction("/api/mcu_state", { state: n }, setCtrlError); }
function setPlantApply(on) { postAction("/api/plant_apply", { enable: !!on }, setCtrlError); }
function recover() { postAction("/api/recover", {}, setCtrlError); }
async function softKillPark() {
  const resultEl = document.getElementById("pdbResult");
  resultEl.textContent = "parking…";
  try {
    const r = await fetch("/api/pdb/soft_kill_park", { method: "POST" });
    const data = await r.json();
    if (!r.ok) {
      setPdbError(data.error || `request failed (${r.status})`);
      resultEl.textContent = "";
    } else {
      setPdbError("");
      const now = new Date().toLocaleTimeString();
      resultEl.textContent = data.mode === "direct"
        ? `parked direct (MCU → ESTOP, desires blanked) at ${now}`
        : `flag written at ${now} — ${data.path || "soft_kill_request"} (peer must poll to act)`;
    }
  } catch (e) {
    setPdbError(String(e));
    resultEl.textContent = "";
  }
  tick();
}
function idleAll() { postAction("/api/actuator/idle_all", {}, setCtrlError); }
function applyActuator(slot) {
  const g = id => parseFloat(document.getElementById(id).value);
  const position = g(`pos${slot}`) || 0.0;
  const kp = g(`kp${slot}`) || 0.0;
  const kd = g(`kd${slot}`) || 0.0;
  postAction(`/api/actuator/${slot}`, { position, kp, kd }, setCtrlError);
}
function idleActuator(slot) { postAction(`/api/actuator/${slot}/idle`, {}, setCtrlError); }

let actuatorRowsBuilt = false;
function buildActuatorRows(count) {
  let rows = "";
  for (let slot = 0; slot < count; slot++) {
    rows += `<tr>
      <td>${slot}</td>
      <td id="fbpos${slot}">—</td><td id="fbvel${slot}">—</td><td id="fbtau${slot}">—</td>
      <td id="fbtemp${slot}">—</td><td id="fbfault${slot}">—</td>
      <td id="heldstate${slot}">—</td>
      <td id="heldpos${slot}" class="held">—</td>
      <td id="heldkp${slot}" class="held">—</td>
      <td id="heldkd${slot}" class="held">—</td>
      <td><input type="number" step="0.01" id="pos${slot}" placeholder="0.0"></td>
      <td><input type="number" step="0.1" id="kp${slot}" placeholder="0.0"></td>
      <td><input type="number" step="0.01" id="kd${slot}" placeholder="0.0"></td>
      <td>
        <button class="btn-go" id="apply${slot}" onclick="applyActuator(${slot})">Apply</button>
        <button class="btn-idle" id="idle${slot}" onclick="idleActuator(${slot})">Idle</button>
      </td>
    </tr>`;
  }
  document.getElementById("acts").innerHTML = rows;
  actuatorRowsBuilt = true;
}

// -- Teleop (per-slot target+cruise slew) — replaces "type a number, hit Apply" -----------

let teleopGroups = null;
let teleopRowsBuilt = false;

async function loadTeleopGroups() {
  try {
    const r = await fetch("/api/teleop/groups");
    teleopGroups = await r.json();
    document.getElementById("cfgMapSelect").value = teleopGroups.cfg_map;
    const scoutEl = document.getElementById("limitScout");
    if (scoutEl) scoutEl.checked = !!teleopGroups.limit_scout;
    buildTeleopRows();
    if (keysArmSlot != null) _keysSetArmSlot(keysArmSlot);
    renderCapturedLimits();
  } catch (e) {
    setTeleopError("teleop groups failed: " + e);
  }
}

function setTeleopError(msg) { document.getElementById("teleopError").textContent = msg || ""; }

async function setCfgMap() {
  const map = document.getElementById("cfgMapSelect").value;
  await postAction("/api/cfg_map", { map }, setTeleopError);
  teleopRowsBuilt = false;
  await loadTeleopGroups();
}

async function setLimitScout() {
  const enabled = !!document.getElementById("limitScout").checked;
  await postAction("/api/teleop/limit_scout", { enable: enabled }, setTeleopError);
  teleopRowsBuilt = false;
  await loadTeleopGroups();
}

// Session-local marked encoder edges (not persisted; copy into yam_bench_clear_left yourself).
const capturedLimits = {};  // slot -> {lo?, hi?}
let lastArmFb = {};         // slot -> position

function selectArmSlot(slot, ev) {
  if (ev && ev.target && (ev.target.closest("button") || ev.target.closest("input"))) return;
  _keysSetArmSlot(slot);
}

function markLimit(slot, which) {
  const pos = lastArmFb[slot];
  if (pos == null || !Number.isFinite(pos)) {
    setTeleopError(`no live FB for slot ${slot} yet — wait for feedback`);
    return;
  }
  const cur = capturedLimits[slot] || {};
  if (which === "lo") cur.lo = pos;
  else cur.hi = pos;
  if (cur.lo != null && cur.hi != null && cur.hi < cur.lo) {
    const t = cur.lo; cur.lo = cur.hi; cur.hi = t;
  }
  capturedLimits[slot] = cur;
  setTeleopError("");
  renderCapturedLimits();
}

function clearCapturedLimits() {
  for (const k of Object.keys(capturedLimits)) delete capturedLimits[k];
  renderCapturedLimits();
}

function renderCapturedLimits() {
  if (!teleopGroups) return;
  const arms = Object.values(teleopGroups.actuators)
    .filter(s => s.group === "arm_left" && s.verified)
    .sort((a, b) => a.slot - b.slot);
  const lines = arms.map((s, i) => {
    const c = capturedLimits[s.slot] || {};
    const el = document.getElementById(`armCap${s.slot}`);
    if (el) {
      el.textContent = `lo ${c.lo != null ? Number(c.lo).toFixed(4) : "—"} · hi ${c.hi != null ? Number(c.hi).toFixed(4) : "—"}`;
    }
    return `J${i + 1} (slot ${s.slot}): lo=${c.lo != null ? Number(c.lo).toFixed(4) : "—"}  hi=${c.hi != null ? Number(c.hi).toFixed(4) : "—"}`;
  });
  const out = document.getElementById("capturedLimitsOut");
  if (out) out.textContent = "Marked limits (live encoder FB; not saved):\n" + lines.join("\n");
}

function copyCapturedLimits() {
  if (!teleopGroups) return;
  const arms = Object.values(teleopGroups.actuators)
    .filter(s => s.group === "arm_left" && s.verified)
    .sort((a, b) => a.slot - b.slot);
  const lo = [], hi = [];
  for (const s of arms) {
    const c = capturedLimits[s.slot] || {};
    lo.push(c.lo != null ? Number(c.lo).toFixed(4) : "None");
    hi.push(c.hi != null ? Number(c.hi).toFixed(4) : "None");
  }
  const text =
    `# marked from debug_dashboard (raw encoder edges — inset yourself before committing)\n` +
    `CLEAR_LO = (${lo.join(", ")})\n` +
    `CLEAR_HI = (${hi.join(", ")})\n`;
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).then(
      () => setTeleopError("copied CLEAR_LO / CLEAR_HI to clipboard"),
      () => setTeleopError("clipboard failed — see Marked limits text above"),
    );
  } else {
    setTeleopError("clipboard unavailable — copy from Marked limits text above");
  }
  const out = document.getElementById("capturedLimitsOut");
  if (out) out.textContent = text;
}

function idleGroup(group) { postAction(`/api/idle_group/${group}`, {}, setTeleopError); }

function teleopArmRow(spec) {
  const s = spec.slot;
  if (!spec.verified) {
    return `<div class="joint-card disabled" id="armRow${s}">
      <div class="jc-head"><span class="jc-label">${spec.label}</span></div>
      <div class="jc-sub">not live-verified on this bench — see arm-damiao-ch1.md</div>
      <div class="jc-foot">
        <span class="btn-group"><button class="btn-idle" onclick="idleActuator(${s})" title="Zero torque on this joint only — it will move/drop under gravity/load if unsupported">Idle</button></span>
      </div>
    </div>`;
  }
  const mid = ((spec.lo + spec.hi) / 2).toFixed(3);
  const cap = capturedLimits[s] || {};
  const capTxt = `lo ${cap.lo != null ? Number(cap.lo).toFixed(4) : "—"} · hi ${cap.hi != null ? Number(cap.hi).toFixed(4) : "—"}`;
  return `<div class="joint-card" id="armRow${s}" onclick="selectArmSlot(${s}, event)" title="Click to select for ←/→ keyboard nudge">
    <div class="jc-head"><span class="jc-label">${spec.label}</span><span class="jc-sub">slot ${s} · rail [${Number(spec.lo).toFixed(2)}, ${Number(spec.hi).toFixed(2)}]</span></div>
    <div class="jc-slider">
      <input type="range" id="armSlider${s}" min="${spec.lo}" max="${spec.hi}" step="0.01" value="${mid}"
        oninput="document.getElementById('armVal${s}').textContent = this.value">
      <span class="val" id="armVal${s}">${mid}</span>
    </div>
    <div class="jc-foot">
      <input type="number" id="armCruise${s}" value="${spec.cruise_default}" step="0.05" min="0" max="${spec.cruise_max}" title="cruise rad/s">
      <span class="btn-group">
        <button class="btn-go" id="armGo${s}" onclick="teleopArmSet(${s})" title="Walk to the slider's position at the cruise rate">Go</button>
        <button class="btn-stop" id="armStop${s}" onclick="teleopArmStop(${s})" title="Freeze here — keeps full holding torque, does not go slack">Stop</button>
        <button class="btn-idle" id="armIdle${s}" onclick="idleActuator(${s})" title="Zero torque on THIS joint only — it will move/drop under gravity/load from a neighbor if unsupported">Idle</button>
      </span>
      <span class="cmd" id="armCmd${s}">—</span>
    </div>
    <div class="jc-limits">
      <span class="fb" id="armFb${s}">fb —</span>
      <button type="button" onclick="markLimit(${s}, 'lo')" title="Capture current encoder FB as this joint's marked minimum">Mark lo</button>
      <button type="button" onclick="markLimit(${s}, 'hi')" title="Capture current encoder FB as this joint's marked maximum">Mark hi</button>
      <span class="cap" id="armCap${s}">${capTxt}</span>
    </div>
  </div>`;
}

function teleopBaseRow(spec) {
  const s = spec.slot;
  return `<div class="joint-card" id="baseRow${s}">
    <div class="jc-head"><span class="jc-label">${spec.label}</span><span class="jc-sub">slot ${s} · ${spec.protocol}${spec.seed_relative ? " · window is ±2π from seed" : ""}</span></div>
    <div class="jc-foot">
      <input type="number" id="baseCruise${s}" value="${spec.cruise_default}" step="0.05" min="0" max="${spec.cruise_max}" title="cruise rad/s">
      <span class="btn-group">
        <button class="btn-go" id="baseJogP${s}" onclick="teleopBaseJog(${s}, 1)">Jog +</button>
        <button class="btn-go" id="baseJogM${s}" onclick="teleopBaseJog(${s}, -1)">Jog −</button>
        <button class="btn-stop" id="baseStop${s}" onclick="teleopBaseStop(${s})" title="Freeze here — keeps holding torque">Stop</button>
        <button class="btn-idle" id="baseIdle${s}" onclick="idleActuator(${s})" title="Zero torque on this actuator only">Idle</button>
      </span>
      <span class="cmd" id="baseCmd${s}"></span>
    </div>
  </div>`;
}

function teleopNeckRow(spec) {
  const s = spec.slot;
  const mid = Math.round((spec.lo + spec.hi) / 2);
  return `<div class="joint-card" id="neckRow${s}">
    <div class="jc-head"><span class="jc-label">${spec.label}</span><span class="jc-sub">slot ${s}</span></div>
    <div class="jc-slider">
      <input type="range" id="neckSlider${s}" min="${spec.lo}" max="${spec.hi}" step="1" value="${mid}"
        oninput="document.getElementById('neckVal${s}').textContent = this.value">
      <span class="val" id="neckVal${s}">${mid}</span>
    </div>
    <div class="jc-foot">
      <input type="number" id="neckCruise${s}" value="${spec.cruise_default}" step="10" min="0" max="${spec.cruise_max}" title="cruise native-steps/s">
      <span class="btn-group">
        <button class="btn-go" id="neckGo${s}" onclick="teleopNeckSet(${s})">Go</button>
        <button class="btn-stop" id="neckStop${s}" onclick="teleopNeckStop(${s})" title="Freeze here">Stop</button>
        <button class="btn-idle" id="neckIdle${s}" onclick="teleopNeckIdle(${s})" title="Release torque on this servo only">Idle</button>
      </span>
      <span class="cmd" id="neckCmd${s}">—</span>
    </div>
  </div>`;
}

function buildTeleopRows() {
  if (!teleopGroups) return;
  const acts = teleopGroups.actuators;
  const armLeft = Object.values(acts).filter(s => s.group === "arm_left").sort((a, b) => a.slot - b.slot);
  const armRight = Object.values(acts).filter(s => s.group === "arm_right").sort((a, b) => a.slot - b.slot);
  const base = Object.values(acts).filter(s => s.group === "base").sort((a, b) => a.slot - b.slot);
  const neck = Object.values(teleopGroups.servos).sort((a, b) => a.slot - b.slot);
  document.getElementById("armLeftRows").innerHTML = armLeft.map(teleopArmRow).join("");
  document.getElementById("armRightRows").innerHTML = armRight.map(teleopArmRow).join("");
  document.getElementById("baseRows").innerHTML = base.map(teleopBaseRow).join("");
  document.getElementById("neckRows").innerHTML = neck.map(teleopNeckRow).join("");
  teleopRowsBuilt = true;
}

function teleopArmSet(slot) {
  const target = parseFloat(document.getElementById(`armSlider${slot}`).value);
  const cruise = parseFloat(document.getElementById(`armCruise${slot}`).value) || 0;
  postAction(`/api/teleop/actuator/${slot}`, { target, cruise }, setTeleopError);
}
function teleopArmStop(slot) { postAction(`/api/teleop/actuator/${slot}/stop`, {}, setTeleopError); }
function teleopBaseJog(slot, direction) {
  const cruise = parseFloat(document.getElementById(`baseCruise${slot}`).value) || 0;
  postAction(`/api/teleop/actuator/${slot}/jog`, { direction, cruise }, setTeleopError);
}
function teleopBaseStop(slot) { postAction(`/api/teleop/actuator/${slot}/stop`, {}, setTeleopError); }
function teleopNeckSet(slot) {
  const target = parseFloat(document.getElementById(`neckSlider${slot}`).value);
  const cruise = parseFloat(document.getElementById(`neckCruise${slot}`).value) || 0;
  postAction(`/api/teleop/servo/${slot}`, { target, cruise }, setTeleopError);
}
function teleopNeckStop(slot) { postAction(`/api/teleop/servo/${slot}/stop`, {}, setTeleopError); }
function teleopNeckIdle(slot) { postAction(`/api/teleop/servo/${slot}/idle`, {}, setTeleopError); }
function teleopStopAll() { postAction("/api/teleop/stop_all", {}, setTeleopError); }

// Keyboard teleop — requires control mode; ignored while focus is in form fields.
let keysControlMode = "observe";
let keysArmSlot = null;
const KEYS_ARM_STEP = 0.05;
const KEYS_NECK_STEP = 40;

function _keysTypingTarget(el) {
  if (!el) return false;
  const tag = (el.tagName || "").toLowerCase();
  return tag === "input" || tag === "select" || tag === "textarea" || el.isContentEditable;
}

function _keysVerifiedArmSlots() {
  if (!teleopGroups) return [];
  return Object.values(teleopGroups.actuators)
    .filter(s => s.group === "arm_left" && s.verified)
    .sort((a, b) => a.slot - b.slot);
}

function _keysBaseSlots() {
  if (!teleopGroups) return [];
  return Object.values(teleopGroups.actuators)
    .filter(s => s.group === "base")
    .sort((a, b) => a.slot - b.slot);
}

function _keysNudgeArm(dir) {
  if (keysArmSlot == null) return;
  const slider = document.getElementById(`armSlider${keysArmSlot}`);
  if (!slider) return;
  const lo = parseFloat(slider.min);
  const hi = parseFloat(slider.max);
  let v = parseFloat(slider.value) + dir * KEYS_ARM_STEP;
  v = Math.max(lo, Math.min(hi, v));
  slider.value = v.toFixed(3);
  const valEl = document.getElementById(`armVal${keysArmSlot}`);
  if (valEl) valEl.textContent = slider.value;
  teleopArmSet(keysArmSlot);
}

function _keysNudgeNeck(slot, dir) {
  const slider = document.getElementById(`neckSlider${slot}`);
  if (!slider) return;
  const lo = parseFloat(slider.min);
  const hi = parseFloat(slider.max);
  let v = parseFloat(slider.value) + dir * KEYS_NECK_STEP;
  v = Math.max(lo, Math.min(hi, Math.round(v)));
  slider.value = String(v);
  const valEl = document.getElementById(`neckVal${slot}`);
  if (valEl) valEl.textContent = slider.value;
  teleopNeckSet(slot);
}

function _keysUpdateArmLabel() {
  const el = document.getElementById("keysArmSel");
  if (!el) return;
  el.textContent = keysArmSlot == null ? "—" : `slot ${keysArmSlot}`;
}

function _keysSetArmSlot(slot) {
  if (keysArmSlot != null) {
    const prev = document.getElementById(`armRow${keysArmSlot}`);
    if (prev) prev.classList.remove("selected");
  }
  keysArmSlot = slot;
  if (keysArmSlot != null) {
    const cur = document.getElementById(`armRow${keysArmSlot}`);
    if (cur) cur.classList.add("selected");
  }
  _keysUpdateArmLabel();
}

window.addEventListener("keydown", (ev) => {
  if (_keysTypingTarget(ev.target)) return;
  if (keysControlMode !== "control") return;
  const k = ev.key;
  if (k === " ") {
    ev.preventDefault();
    teleopStopAll();
    return;
  }
  if (k === "ArrowUp") { ev.preventDefault(); _keysNudgeNeck(0, 1); return; }
  if (k === "ArrowDown") { ev.preventDefault(); _keysNudgeNeck(0, -1); return; }
  // ←/→ nudge the selected arm joint; neck yaw is ,/. when an arm is selected.
  if (k === "ArrowLeft") {
    ev.preventDefault();
    if (keysArmSlot != null) _keysNudgeArm(-1);
    else _keysNudgeNeck(1, -1);
    return;
  }
  if (k === "ArrowRight") {
    ev.preventDefault();
    if (keysArmSlot != null) _keysNudgeArm(1);
    else _keysNudgeNeck(1, 1);
    return;
  }
  if (k === ",") { ev.preventDefault(); _keysNudgeNeck(1, -1); return; }
  if (k === ".") { ev.preventDefault(); _keysNudgeNeck(1, 1); return; }
  if (k >= "1" && k <= "7") {
    const arms = _keysVerifiedArmSlots();
    const idx = parseInt(k, 10) - 1;
    if (idx < arms.length) {
      _keysSetArmSlot(arms[idx].slot);
    }
    return;
  }
  if (k === "[") { ev.preventDefault(); _keysNudgeArm(-1); return; }
  if (k === "]") { ev.preventDefault(); _keysNudgeArm(1); return; }
  const base = _keysBaseSlots();
  if (k === "z" || k === "Z") { if (base[0]) teleopBaseJog(base[0].slot, -1); return; }
  if (k === "x" || k === "X") { if (base[0]) teleopBaseJog(base[0].slot, 1); return; }
  if (k === "c" || k === "C") { if (base[1]) teleopBaseJog(base[1].slot, -1); return; }
  if (k === "v" || k === "V") { if (base[1]) teleopBaseJog(base[1].slot, 1); return; }
});

function continuousLaunch() {
  const duration_s = parseFloat(document.getElementById("continuousDuration").value) || 0;
  postAction("/api/continuous/launch", { duration_s }, setTeleopError);
}
function continuousStop() { postAction("/api/continuous/stop", {}, setTeleopError); }

async function tick() {
  try {
    const r = await fetch("/api/state");
    const s = await r.json();
    const g = document.getElementById("grade");
    const grade = s.grade || "idle";
    g.textContent = grade;
    g.className = "grade " + grade;
    document.getElementById("summary").textContent = s.summary || "—";
    const modeLabel = s.control_mode || (s.connected ? "observe" : "idle");
    keysControlMode = s.control_mode || "observe";
    document.getElementById("meta").textContent = s.connected
      ? `${s.port || "?"} · ${modeLabel} · poll 200ms`
      : (s.following_state_file ? `follow · poll 200ms` : `idle · poll 200ms`);
    const ctx = document.getElementById("context");
    ctx.innerHTML = (s.context || []).map(c => `<li>${c}</li>`).join("");

    document.getElementById("healthGlance").innerHTML = [
      kv("fb_hz", s.fb_hz != null ? s.fb_hz.toFixed(1) : null),
      kv("mcu_state", s.mcu_state),
      kv("plant_block", s.plant_block_name || s.plant_block),
      kv("ack_lag", s.stream_ack_lag),
      kv("tick", s.tick),
      kv("age_s", s.age_s != null ? s.age_s.toFixed(2) : null),
    ].join("");

    document.getElementById("healthAdvanced").innerHTML = [
      kv("ack_seq", s.ack_seq),
      kv("act_lap_ms", s.lap_ms),
      kv("act_lap_peak_ms", s.lap_max_ms),
      kv("periph_lap_ms", s.periph_lap_ms),
      kv("periph_lap_peak_ms", s.periph_lap_max_ms),
      kv("ticks_pend", s.ticks_pending),
      kv("host_tx_hz", s.stream_tx_hz != null ? s.stream_tx_hz.toFixed(1) : null),
      kv("host_tx_gap_p95_ms", s.stream_tx_gap_p95_ms != null ? s.stream_tx_gap_p95_ms.toFixed(1) : null),
      kv("host_tx_gap_max_ms", s.stream_tx_gap_max_ms != null ? s.stream_tx_gap_max_ms.toFixed(1) : null),
      kv("host_send_ms", s.stream_send_ms != null ? s.stream_send_ms.toFixed(1) : null),
      kv("host_poll_ms", s.stream_poll_ms != null ? s.stream_poll_ms.toFixed(1) : null),
      kv("host_credit_wait_ms", s.stream_credit_wait_ms != null ? s.stream_credit_wait_ms.toFixed(1) : null),
      kv("host_pub_ms", s.stream_publish_ms != null ? s.stream_publish_ms.toFixed(1) : null),
      kv("host_loop_ms", s.stream_loop_ms != null ? s.stream_loop_ms.toFixed(1) : null),
      kv("svd", s.svd_present ? "yes" : "no"),
      kv("ui_note", "3 jobs: plant TX / UI snapshot / opt-in Record. Healthy: ack_lag~0, fb flood when idle. If MCP Apply spikes lag, Idle all + check firmware plant MCP path.", "wide"),
    ].join("");

    document.getElementById("blackbox").innerHTML = [
      kv("faults this session", s.fault_count ?? 0),
      kv("capturing fault", s.capturing_fault
        ? `<span class="fault-badge">yes — dumping context</span>` : "no"),
      kv("last fault file", s.last_fault_path ? s.last_fault_path.split(/[\\/]/).pop() : "—"),
    ].join("");
    const recBtn = document.getElementById("recordBtn");
    const recMeta = document.getElementById("recordMeta");
    recBtn.classList.toggle("on", !!s.recording);
    recBtn.textContent = s.recording ? "■ Stop recording" : "● Record";
    recMeta.textContent = s.recording
      ? `${(s.recording_bytes/1024).toFixed(1)} KB · ${(s.recording_seconds||0).toFixed(0)}s · ${s.recording_path ? s.recording_path.split(/[\\/]/).pop() : ""}`
      : "";
    const clearFaultsBtn = document.getElementById("clearFaultsBtn");
    clearFaultsBtn.disabled = !!s.following_state_file;
    clearFaultsBtn.title = s.following_state_file
      ? "Following a peer's state.json — this dashboard isn't the one counting faults right now, so Clear has nothing to reset here. Clear on whichever process owns COM instead."
      : "Resets the 'faults this session' counter and pre-fault ring buffer in RAM. Does not delete already-written faults/fault_*.ndjson dump files on disk. If the board is still faulted, the very next tick re-arms a fresh capture — this clears bookkeeping, not the underlying condition.";

    // PDU / soft-kill — pdb_status is null until the first feedback frame
    // with a parseable system-kill block arrives (or never connected).
    const pdb = s.pdb_status;
    const hostRequested = s.mcu_state === 3;  // McuState.ESTOP — same latch plant_command.c drives to pdb_link_request_estop()
    document.getElementById("pduGlance").innerHTML = pdb ? [
      kv("kill_state (MCU view, freshness-gated)", pdb.kill_state_name),
      kv("kill_reason", pdb.kill_reason_name),
      kv("stale_failsafe", pdb.stale_failsafe ? "yes (COMMS_LOSS -> HARD)" : "no"),
      kv("host requested ESTOP", hostRequested ? "yes" : "no"),
      kv("local estop_sense (Controls PB7)", pdb.estop_sense === 1 ? "1 (allowed)" : "0 (asserted)"),
    ].join("") : `<div class="kv"><span class="k">status</span><div class="v">no PDB frame yet</div></div>`;
    document.getElementById("pduAdvanced").innerHTML = pdb ? [
      kv("peer kill_state (raw PDBF)", pdb.pdb ? PEER_KILL_STATE_NAMES[pdb.pdb.kill_state] ?? pdb.pdb.kill_state : "—"),
      kv("peer estop_sense (raw PDBF)", pdb.pdb ? (pdb.pdb.estop_sense === 1 ? "1 (allowed)" : "0 (asserted)") : "—"),
      kv("contactor_state", pdb.pdb ? pdb.pdb.contactor_state : "—"),
      kv("pack_v (V) x4", fmtVec(pdb.pack_v_V)),
      kv("rail_v (V) x4", fmtVec(pdb.rail_v_V)),
      kv("pack_i (A) x4", fmtVec(pdb.pack_i_A)),
      kv("rail_i (A) x4", fmtVec(pdb.rail_i_A)),
    ].join("") : "";
    // Soft-kill Park stays clickable while following continuous state.json
    // (peer owns CDC) — POST writes a request flag the peer honors.
    const canSoftKill = !!(s.connected || s.following_state_file || s.peer_connected);
    document.getElementById("softKillBtn").disabled = !canSoftKill;
    const pdbMeta = document.getElementById("pdbMeta");
    let pdbHint = "";
    if (pdb) {
      if (pdb.kill_state_name === "soft_kill_req")
        pdbHint = "peer requesting soft-kill — Park to ack SOFT_KILL_READY";
      else if (pdb.kill_state_name === "soft_kill_ready")
        pdbHint = "parked — SOFT_KILL_READY acked";
    }
    if (!s.connected && canSoftKill && !pdbHint)
      pdbHint = "follow mode — Park writes soft_kill_request; only yam_continuous_all polls it today (vbeta peers do not)";
    pdbMeta.textContent = pdbHint;

    // Connection card
    const inControl = !!s.connected && s.control_mode === "control";
    const inObserve = !!s.connected && s.control_mode !== "control";
    document.getElementById("connectBtn").disabled = !!s.connected;
    document.getElementById("disconnectBtn").disabled = !s.connected;
    document.getElementById("enableCtrlBtn").disabled = !inObserve;
    document.getElementById("observeBtn").disabled = !inControl;
    document.getElementById("enableCtrlBtn").classList.toggle("active", inControl);
    document.getElementById("observeBtn").classList.toggle("active", inObserve);
    document.getElementById("portSelect").disabled = !!s.connected;
    document.getElementById("connMeta").textContent = s.connected
      ? `${s.port} · ${s.control_mode || "observe"}`
      : (s.following_state_file
          ? `following ${s.state_path || "state.json"}${s.peer_connected ? " (peer live)" : ""}`
          : "not connected — Connect (observe) is safe; Enable control only when you want motors");
    document.getElementById("sessionMeta").textContent = s.session_dir
      ? `session dir: ${s.session_dir} (set DEFT_SESSION_DIR to override — this dashboard and any peer script must agree, or follow mode reads nothing)`
      : "";
    const banner = document.getElementById("modeBanner");
    if (!s.connected && !s.following_state_file) {
      banner.className = "banner ok";
      banner.textContent = "Idle — not owning COM. Connect opens observe (plant_apply=0, no auto soft-kill). PDU sim panel uses :8765; this UI defaults to :8766.";
    } else if (!s.connected && s.following_state_file) {
      banner.className = "banner ok";
      banner.textContent = "Follow mode — reading state.json only. Soft-kill Park signals the peer; do not Connect while continuous owns CDC.";
    } else if (inObserve) {
      banner.className = "banner";
      banner.textContent = "Observe — streaming telemetry, plant_apply=0, auto soft-kill off. Board will not ESTOP from PDU V/I on connect. Enable control when ready to command. Soft-kill Park works right now regardless — it does not need Control mode.";
    } else {
      banner.className = "banner warn";
      banner.textContent = "Control — MCU NORMAL + plant_apply=1. Firmware now mounts every CFG-enabled slot's held desire each tick and TX's it on that slot's CAN bus, so ALL configured channels start transmitting/blinking at once — not just the one you move (idle-hold slots still send a zero-torque keepalive to stay enabled). Idle all / Back to observe before Disconnect if you parked. Soft-kill Park ≠ Enable control: Park is the always-available kill; Enable control is what arms the ESTOP button and Apply.";
    }
    lastListenPdu = !!s.listen_pdu;
    const listenBtn = document.getElementById("listenPduBtn");
    listenBtn.disabled = !s.connected;
    listenBtn.textContent = `listen_pdu: ${lastListenPdu ? "ON" : "OFF"}`;
    listenBtn.className = lastListenPdu ? "btn-go" : "btn-ghost";

    // Plant control gating — ESTOP only when we own COM (follow mode uses Soft-kill Park)
    const plantLive = inControl;
    for (const id of ["mcuNormal", "mcuRecovery", "mcuApplyOff", "mcuApplyOn", "recoverBtn", "idleAllBtn", "mcuEstop"]) {
      document.getElementById(id).disabled = !plantLive;
    }

    // Streaming badge — the background thread that actually owns the wire.
    const streamBadge = document.getElementById("streamingBadge");
    streamBadge.classList.toggle("on", !!s.streaming);
    const activeSlots = [];
    (s.held || []).forEach((h, i) => { if (h && h.active) activeSlots.push(i); });
    const activeCount = activeSlots.length;
    streamBadge.textContent = s.streaming
      ? `streaming: ON (${activeCount} active: [${activeSlots.join(",")}] )`
      : "streaming: OFF";
    if (activeCount >= 2 && (s.stream_ack_lag || 0) >= 26) {
      streamBadge.title = "Multiple ACTIVE holds — Apply accumulates. Idle all before switching buses.";
    }
    // Glanceable even when the Plant control dropdown is collapsed.
    document.getElementById("plantControlSummaryMeta").textContent =
      `mcu=${s.mcu_state ?? "—"} · streaming=${s.streaming ? "ON" : "OFF"}${activeCount ? ` · ${activeCount} active` : ""}`;

    // Actuator rows: build once, then only touch feedback/held cells + button
    // disabled state — never re-render the <input> elements themselves, or
    // every 200ms poll would wipe out whatever the user is mid-typing.
    const acts = s.actuators || [];
    const held = s.held || [];
    if (!actuatorRowsBuilt && acts.length > 0) buildActuatorRows(acts.length);
    if (actuatorRowsBuilt) {
      const setText = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
      acts.forEach((a, slot) => {
        if (a) {
          setText(`fbpos${slot}`, fmt(a.position, 3));
          setText(`fbvel${slot}`, fmt(a.velocity, 3));
          setText(`fbtau${slot}`, fmt(a.torque, 2));
          setText(`fbtemp${slot}`, fmt(a.temperature, 1));
          setText(`fbfault${slot}`, `0x${(a.fault>>>0).toString(16)}`);
        }
        const h = held[slot];
        const stateEl = document.getElementById(`heldstate${slot}`);
        if (stateEl) {
          stateEl.innerHTML = h
            ? `<span class="active-badge ${h.active ? "active" : "idle"}">${h.active ? "ACTIVE" : "idle-hold"}</span>`
            : "—";
        }
        setText(`heldpos${slot}`, h ? fmt(h.position, 3) : "—");
        setText(`heldkp${slot}`, h ? fmt(h.kp, 1) : "—");
        setText(`heldkd${slot}`, h ? fmt(h.kd, 2) : "—");
        const applyBtn = document.getElementById(`apply${slot}`);
        const idleBtn = document.getElementById(`idle${slot}`);
        if (applyBtn) applyBtn.disabled = !inControl;
        if (idleBtn) idleBtn.disabled = !inControl;
      });
    }

    // Teleop card: build rows once groups are known, gate every control on
    // inControl same as the raw table, and show each engaged slot's live
    // commanded position (from the host-side slew engine, not the slider —
    // never touch the slider/inputs themselves here, same rule as above).
    if (!teleopRowsBuilt && teleopGroups) buildTeleopRows();
    for (const id of ["armLeftRows", "armRightRows", "baseRows", "neckRows"]) {
      document.querySelectorAll(`#${id} button`).forEach(b => { b.disabled = !inControl; });
    }
    const teleopAct = (s.teleop && s.teleop.actuators) || {};
    const teleopSrv = (s.teleop && s.teleop.servos) || {};
    if (teleopGroups) {
      Object.keys(teleopGroups.actuators).forEach(slotStr => {
        const slot = Number(slotStr);
        const st = teleopAct[slot] ?? teleopAct[slotStr];
        const armCmd = document.getElementById(`armCmd${slot}`);
        if (armCmd) {
          armCmd.textContent = st
            ? `→ ${fmt(st.target, 3)} @ ${fmt(st.cruise, 2)} rad/s${st.flagged ? " · ⚠ large tracking error — under load?" : ""}`
            : "idle";
          armCmd.classList.toggle("flagged", !!(st && st.flagged));
        }
        const armFb = document.getElementById(`armFb${slot}`);
        if (armFb) {
          const a = acts[slot];
          if (a && a.position != null) {
            lastArmFb[slot] = a.position;
            armFb.textContent = `fb ${fmt(a.position, 4)}`;
          } else {
            armFb.textContent = "fb —";
          }
        }
        const baseCmd = document.getElementById(`baseCmd${slot}`);
        if (baseCmd) {
          baseCmd.textContent = st ? `→ ${fmt(st.target, 2)}${st.flagged ? " · ⚠ tracking error" : ""}` : "";
          baseCmd.classList.toggle("flagged", !!(st && st.flagged));
        }
      });
      Object.keys(teleopGroups.servos).forEach(slotStr => {
        const slot = Number(slotStr);
        const st = teleopSrv[slot] ?? teleopSrv[slotStr];
        const neckCmd = document.getElementById(`neckCmd${slot}`);
        if (neckCmd) neckCmd.textContent = st ? (st.seeded ? `→ ${fmt(st.target, 0)} (now ${fmt(st.pos, 0)})` : "seeding present position…") : "idle";
      });
    }

    // Continuous mode card
    const cl = s.continuous_launch || { state: "unknown" };
    document.getElementById("continuousStatus").textContent =
      cl.state + (cl.detail ? `: ${typeof cl.detail === "string" ? cl.detail : JSON.stringify(cl.detail)}` : "");
    document.getElementById("continuousLaunchBtn").disabled = !!s.connected || cl.state === "launching";
    document.getElementById("continuousStopBtn").disabled = cl.state === "stopping";
  } catch (e) {
    document.getElementById("summary").textContent = "UI fetch failed: " + e;
  }
}
async function toggleRecord() {
  const btn = document.getElementById("recordBtn");
  const recording = btn.classList.contains("on");
  btn.disabled = true;
  await postAction(recording ? "/api/record/stop" : "/api/record/start", {}, setCtrlError);
  btn.disabled = false;
}
loadPorts();
loadTeleopGroups();
setInterval(tick, 200);
tick();
</script>
</body>
</html>
"""


def make_handler(state: AppState):
    # Coalesce /api/state builds — browser polls at 5 Hz; ThreadingHTTPServer can
    # overlap slower snapshot+held work and contend with the plant stream lock.
    state_cache: dict = {"t": 0.0, "payload": None}
    state_cache_lock = threading.Lock()
    state_cache_ttl_s = 0.1

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: object) -> None:
            return  # quiet

        def _send_json(self, payload_obj: object, status: int = 200) -> None:
            payload = json.dumps(payload_obj).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _read_json_body(self) -> dict:
            length = int(self.headers.get("Content-Length") or 0)
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            return json.loads(raw) if raw else {}

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                body = _HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/state":
                now = time.monotonic()
                with state_cache_lock:
                    cached = state_cache["payload"]
                    if cached is not None and (now - state_cache["t"]) < state_cache_ttl_s:
                        self._send_json(cached)
                        return
                # Not connected: serve peer state.json (e.g. yam_continuous_all
                # with persist_telemetry) so the UI can watch FB without owning COM.
                if not state.connected:
                    try:
                        sp = state.telemetry.state_path
                        if sp.is_file():
                            payload = json.loads(sp.read_text(encoding="utf-8"))
                            # Peer may have connected=true (it owns COM). Keep
                            # telemetry, but mark follow mode so UI does not
                            # enable Apply/MCU controls against a missing hub.
                            peer_connected = bool(payload.get("connected"))
                            payload["following_state_file"] = True
                            payload["state_path"] = str(sp)
                            payload["session_dir"] = str(sp.parent)
                            payload["cfg_map"] = state.cfg_map
                            payload["continuous_launch"] = state.continuous_status()
                            payload["peer_connected"] = peer_connected
                            payload["connected"] = False
                            payload["control_mode"] = "idle"
                            payload["streaming"] = bool(
                                payload.get("streaming") or peer_connected
                            )
                            if peer_connected and payload.get("fb_hz") is not None:
                                hz = float(payload["fb_hz"] or 0.0)
                                payload["summary"] = (
                                    f"following state.json · fb_hz={hz:.0f} · "
                                    f"peer={payload.get('port') or '?'}"
                                )
                                if hz >= 20.0 and payload.get("grade") == "red":
                                    payload["grade"] = "green"
                            with state_cache_lock:
                                state_cache["t"] = now
                                state_cache["payload"] = payload
                            self._send_json(payload)
                            return
                    except Exception:
                        pass
                payload = state.telemetry.snapshot_dict()
                payload.update(state.held_snapshot())
                payload["control_mode"] = state.control_mode if state.connected else "idle"
                payload["following_state_file"] = False
                payload["session_dir"] = str(state.telemetry.session_dir)
                payload["cfg_map"] = state.cfg_map
                payload["listen_pdu"] = bool(state.proxy.listen_pdu) if state.proxy is not None else None
                payload["continuous_launch"] = state.continuous_status()
                payload["teleop"] = state.teleop.snapshot()
                # Idle (no COM) must not look like a board fault — telemetry
                # cache grades disconnected as "red"; soften for the UI.
                if not state.connected:
                    payload["grade"] = "idle"
                    payload["summary"] = "idle — not connected"
                    payload["context"] = [
                        "Connect (observe) to stream telemetry without plant apply",
                        "Enable control only when you intend to command actuators",
                    ]
                with state_cache_lock:
                    state_cache["t"] = now
                    state_cache["payload"] = payload
                self._send_json(payload)
                return
            if path == "/api/ports":
                self._send_json({"ports": list_ports_info()})
                return
            if path == "/api/teleop/groups":
                self._send_json(state.teleop_groups())
                return
            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                body = self._read_json_body()
                if path == "/api/connect":
                    port = body.get("port")
                    if not port:
                        raise ValueError("port is required")
                    mode = body.get("mode") or "observe"
                    state.connect(port, mode=mode)
                elif path == "/api/disconnect":
                    state.disconnect()
                elif path == "/api/control_mode":
                    state.set_control_mode(body.get("mode") or "observe")
                elif path == "/api/listen_pdu":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    state.set_listen_pdu(bool(body.get("enable", False)))
                elif path == "/api/faults/clear":
                    state.clear_fault_log()
                elif path == "/api/record/start":
                    state.telemetry.start_recording()
                elif path == "/api/record/stop":
                    state.telemetry.stop_recording()
                elif path == "/api/recover":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.recover()
                elif path == "/api/pdb/soft_kill_park":
                    result = state.soft_kill_park()
                    self._send_json({"ok": True, **result})
                    return
                elif path == "/api/mcu_state":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.set_mcu_state(int(body["state"]))
                elif path == "/api/plant_apply":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.set_plant_apply(bool(body.get("enable", False)))
                elif path == "/api/actuator/idle_all":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.idle_all_actuators()
                elif path == "/api/cfg_map":
                    state.set_cfg_map(body.get("map") or "bench")
                elif path == "/api/teleop/limit_scout":
                    state.set_limit_scout(bool(body.get("enable", False)))
                elif path.startswith("/api/idle_group/"):
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.idle_group(path[len("/api/idle_group/") :])
                elif path.startswith("/api/teleop/servo/"):
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    rest = path[len("/api/teleop/servo/") :]
                    if rest.endswith("/stop"):
                        state.teleop_servo_stop(int(rest[: -len("/stop")]))
                    elif rest.endswith("/idle"):
                        state.teleop_servo_idle(int(rest[: -len("/idle")]))
                    else:
                        state.teleop_servo_target(
                            int(rest),
                            target=float(body["target"]),
                            cruise=float(body.get("cruise", 0.0)),
                        )
                elif path.startswith("/api/teleop/actuator/"):
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    rest = path[len("/api/teleop/actuator/") :]
                    if rest.endswith("/stop"):
                        state.teleop_actuator_stop(int(rest[: -len("/stop")]))
                    elif rest.endswith("/jog"):
                        state.teleop_actuator_jog(
                            int(rest[: -len("/jog")]),
                            direction=int(body.get("direction", 1)),
                            cruise=float(body.get("cruise", 0.0)),
                        )
                    else:
                        state.teleop_actuator_target(
                            int(rest),
                            target=float(body["target"]),
                            cruise=float(body.get("cruise", 0.0)),
                        )
                elif path == "/api/teleop/stop_all":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.teleop_stop_all()
                elif path == "/api/continuous/launch":
                    self._send_json(state.launch_continuous(duration_s=float(body.get("duration_s", 0.0))))
                    return
                elif path == "/api/continuous/stop":
                    self._send_json(state.stop_continuous())
                    return
                elif path.startswith("/api/actuator/"):
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    rest = path[len("/api/actuator/") :]
                    if rest.endswith("/idle"):
                        state.idle_actuator(int(rest[: -len("/idle")]))
                    else:
                        state.set_actuator(
                            int(rest),
                            position=float(body.get("position", 0.0)),
                            kp=float(body.get("kp", 0.0)),
                            kd=float(body.get("kd", 0.0)),
                        )
                else:
                    self.send_error(404)
                    return
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            # Do not return the full telemetry snapshot on every control POST —
            # snapshot_dict() does a deep asdict under the same lock the stream
            # thread needs for update_from_feedback. The UI already polls
            # /api/state at 5 Hz.
            self._send_json({"ok": True})

    return Handler


def serve(
    state: AppState, host: str = "127.0.0.1", http_port: int = DEFAULT_HTTP_PORT
) -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer((host, http_port), make_handler(state))
    thread = threading.Thread(target=httpd.serve_forever, name="deft-dashboard-http", daemon=True)
    thread.start()
    return httpd
