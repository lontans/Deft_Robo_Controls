"""HTTP route table + dispatch — replaces the old app.py's long if/elif chain.

Also owns :func:`wire_panels`: imports the panel action functions three other
agents are building in parallel, in separate worktrees:

* ``deft_controls_sdk.debug.panels.system.system_panel`` / ``.led_panel``
* ``deft_controls_sdk.debug.panels.discover.discover_panel`` / ``.calibrate_panel``

**None of these exist in this worktree.** :func:`wire_panels` tries each
:data:`registry.ACTION_REGISTRY` entry's ``import_path`` inside a
``try/except`` — a missing module/function just marks that action
unavailable (surfaced via ``GET /api/panels`` and a clean 400 on
``POST /api/panels/<key>/run``, never a 500 or a startup crash). Call it once
at server start (:func:`serve` does this); safe to call again.
"""
from __future__ import annotations

import importlib
import json
import logging
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Dict, Optional, Pattern, Tuple
from urllib.parse import urlparse

from deft_controls_sdk.link.exchange import list_ports_info

from . import templates
from .registry import ACTION_REGISTRY
from .state import AppState, DEFAULT_HTTP_PORT

log = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"
_STATIC_CONTENT_TYPES = {".css": "text/css; charset=utf-8", ".js": "application/javascript; charset=utf-8"}

# -- Panel wiring (graceful degradation) --------------------------------------------------

_PANEL_FUNCS: Dict[str, Optional[Callable]] = {}
_panel_wired = False


def wire_panels() -> Dict[str, Optional[Callable]]:
    """Attempt to import every ACTION_REGISTRY entry's real implementation.

    Never raises. Idempotent — safe to call more than once (e.g. once at
    server startup, again lazily from the first ``/api/panels`` request).
    """
    global _panel_wired
    for key, spec in ACTION_REGISTRY.items():
        module_path, _, func_name = spec.import_path.rpartition(".")
        try:
            mod = importlib.import_module(module_path)
            func = getattr(mod, func_name)
        except (ImportError, ModuleNotFoundError, AttributeError) as exc:
            log.warning("panel action %r not wired yet (%s): %s", key, spec.import_path, exc)
            _PANEL_FUNCS[key] = None
            continue
        _PANEL_FUNCS[key] = func
    _panel_wired = True
    return dict(_PANEL_FUNCS)


def panel_status() -> Dict[str, dict]:
    """ACTION_REGISTRY entries + whether each is actually wired right now."""
    if not _panel_wired:
        wire_panels()
    out: Dict[str, dict] = {}
    for key, spec in ACTION_REGISTRY.items():
        out[key] = {
            "key": key,
            "label": spec.label,
            "blurb": spec.blurb,
            "owner": spec.owner,
            "import_path": spec.import_path,
            "call_sig": spec.call_sig,
            "requires_mode": spec.requires_mode,
            "available": _PANEL_FUNCS.get(key) is not None,
        }
    return out


# -- POST handlers ---------------------------------------------------------------------
# Each returns None (-> {"ok": True}) or a dict (sent as-is).


def _require_active(state: AppState) -> None:
    if not state.connected:
        raise RuntimeError("not connected")


def _h_connect(state: AppState, params: dict, body: dict):
    port = body.get("port")
    if not port:
        raise ValueError("port is required")
    hz = body.get("hz")
    telemetry_hz = body.get("telemetry_hz")
    state.connect(
        port,
        stream_hz=None if hz is None else float(hz),
        telemetry_hz=None if telemetry_hz is None else float(telemetry_hz),
    )


def _h_disconnect(state: AppState, params: dict, body: dict):
    state.disconnect()


def _h_soft_kill_release(state: AppState, params: dict, body: dict):
    state.release_soft_kill()


def _h_soft_kill_engage(state: AppState, params: dict, body: dict):
    state.engage_soft_kill()


def _h_listen_pdu(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.set_listen_pdu(bool(body.get("enable", False)))


def _h_faults_clear(state: AppState, params: dict, body: dict):
    state.clear_fault_log()


def _h_record_start(state: AppState, params: dict, body: dict):
    state.telemetry.start_recording()


def _h_record_stop(state: AppState, params: dict, body: dict):
    state.telemetry.stop_recording()


def _h_recover(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.recover()


def _h_estop_park(state: AppState, params: dict, body: dict):
    result = state.hard_estop_park()
    return {"ok": True, **result}


def _h_mcu_state(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.set_mcu_state(int(body["state"]))


def _h_plant_apply(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.set_plant_apply(bool(body.get("enable", False)))


def _h_idle_all(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.idle_all_actuators()


def _h_cfg_map(state: AppState, params: dict, body: dict):
    state.set_cfg_map(body.get("map") or "bench")


def _h_limit_scout(state: AppState, params: dict, body: dict):
    state.set_limit_scout(bool(body.get("enable", False)))


def _h_idle_group(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.idle_group(params["group"])


def _h_servo_stop(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_servo_stop(int(params["slot"]))


def _h_servo_idle(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_servo_idle(int(params["slot"]))


def _h_servo_target(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_servo_target(int(params["slot"]), target=float(body["target"]), cruise=float(body.get("cruise", 0.0)))


def _h_actuator_teleop_stop(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_actuator_stop(int(params["slot"]))


def _h_actuator_teleop_jog(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_actuator_jog(
        int(params["slot"]), direction=int(body.get("direction", 1)), cruise=float(body.get("cruise", 0.0))
    )


def _h_actuator_teleop_target(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_actuator_target(
        int(params["slot"]), target=float(body["target"]), cruise=float(body.get("cruise", 0.0))
    )


def _h_teleop_stop_all(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.teleop_stop_all()


def _h_continuous_launch(state: AppState, params: dict, body: dict):
    return state.launch_continuous(duration_s=float(body.get("duration_s", 0.0)))


def _h_continuous_stop(state: AppState, params: dict, body: dict):
    return state.stop_continuous()


def _h_actuator_idle(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.idle_actuator(int(params["slot"]))


def _h_actuator_apply(state: AppState, params: dict, body: dict):
    _require_active(state)
    state.set_actuator(
        int(params["slot"]),
        position=float(body.get("position", 0.0)),
        kp=float(body.get("kp", 0.0)),
        kd=float(body.get("kd", 0.0)),
    )


def _h_panel_run(state: AppState, params: dict, body: dict):
    key = params["key"]
    spec = ACTION_REGISTRY.get(key)
    if spec is None:
        raise ValueError(f"unknown panel action {key!r}")
    if spec.requires_mode == "follow" and state.connected:
        raise RuntimeError(
            f"{spec.label} is Follow-mode only — disconnect COM first "
            "(Windows CDC is exclusive-open; a concurrent probe would corrupt both sessions)"
        )
    if spec.requires_mode in ("active", "debug") and not state.connected:
        raise RuntimeError(f"{spec.label} requires an Active connection")
    if not _panel_wired:
        wire_panels()
    func = _PANEL_FUNCS.get(key)
    if func is None:
        raise RuntimeError(f"{spec.label} is not wired yet ({spec.import_path} unavailable)")
    call_kwargs = dict(body or {})
    if spec.requires_mode == "follow":
        port = call_kwargs.pop("port", None)
        if not port:
            raise ValueError("port is required")
        result = func(port, **call_kwargs)
    else:
        result = func(state.proxy, **call_kwargs)
    return {"ok": True, "result": result}


_POST_ROUTES: Tuple[Tuple[Pattern, Callable], ...] = tuple(
    (re.compile(pattern), handler)
    for pattern, handler in [
        (r"^/api/connect$", _h_connect),
        (r"^/api/disconnect$", _h_disconnect),
        (r"^/api/soft_kill/release$", _h_soft_kill_release),
        (r"^/api/soft_kill/engage$", _h_soft_kill_engage),
        (r"^/api/listen_pdu$", _h_listen_pdu),
        (r"^/api/faults/clear$", _h_faults_clear),
        (r"^/api/record/start$", _h_record_start),
        (r"^/api/record/stop$", _h_record_stop),
        (r"^/api/recover$", _h_recover),
        (r"^/api/estop/park$", _h_estop_park),
        (r"^/api/mcu_state$", _h_mcu_state),
        (r"^/api/plant_apply$", _h_plant_apply),
        (r"^/api/actuator/idle_all$", _h_idle_all),
        (r"^/api/cfg_map$", _h_cfg_map),
        (r"^/api/teleop/limit_scout$", _h_limit_scout),
        (r"^/api/idle_group/(?P<group>[a-zA-Z_]+)$", _h_idle_group),
        (r"^/api/teleop/servo/(?P<slot>\d+)/stop$", _h_servo_stop),
        (r"^/api/teleop/servo/(?P<slot>\d+)/idle$", _h_servo_idle),
        (r"^/api/teleop/servo/(?P<slot>\d+)$", _h_servo_target),
        (r"^/api/teleop/actuator/(?P<slot>\d+)/stop$", _h_actuator_teleop_stop),
        (r"^/api/teleop/actuator/(?P<slot>\d+)/jog$", _h_actuator_teleop_jog),
        (r"^/api/teleop/actuator/(?P<slot>\d+)$", _h_actuator_teleop_target),
        (r"^/api/teleop/stop_all$", _h_teleop_stop_all),
        (r"^/api/continuous/launch$", _h_continuous_launch),
        (r"^/api/continuous/stop$", _h_continuous_stop),
        (r"^/api/actuator/(?P<slot>\d+)/idle$", _h_actuator_idle),
        (r"^/api/actuator/(?P<slot>\d+)$", _h_actuator_apply),
        (r"^/api/panels/(?P<key>[a-zA-Z_]+)/run$", _h_panel_run),
    ]
)


def dispatch_post(state: AppState, path: str, body: dict):
    """Find and run the handler for ``path``. Raises on no match (-> 404) or
    on handler error (caller turns exceptions into a 400 + {"error": ...})."""
    for pattern, handler in _POST_ROUTES:
        m = pattern.match(path)
        if m is not None:
            return handler(state, m.groupdict(), body)
    return _NO_ROUTE


_NO_ROUTE = object()


# -- GET /api/state payload --------------------------------------------------------------


def build_state_payload(state: AppState) -> dict:
    from deft_controls_sdk.link.exchange import ACTUATOR_COUNT

    if not state.connected:
        try:
            sp = state.telemetry.state_path
            if sp.is_file():
                payload = json.loads(sp.read_text(encoding="utf-8"))
                peer_connected = bool(payload.get("connected"))
                payload["following_state_file"] = True
                payload["state_path"] = str(sp)
                payload["session_dir"] = str(sp.parent)
                payload["cfg_map"] = state.cfg_map
                payload["continuous_launch"] = state.continuous_status()
                payload["peer_connected"] = peer_connected
                payload["connected"] = False
                payload["mode"] = "follow"
                payload["soft_kill"] = state.soft_kill
                payload["connect_stream_hz"] = state.stream_hz
                payload["connect_telemetry_hz"] = state.telemetry_hz
                payload["streaming"] = bool(payload.get("streaming") or peer_connected)
                if peer_connected and payload.get("fb_hz") is not None:
                    hz = float(payload["fb_hz"] or 0.0)
                    payload["summary"] = (
                        f"following state.json · fb_hz={hz:.0f} · peer={payload.get('port') or '?'}"
                    )
                    if hz >= 20.0 and payload.get("grade") == "red":
                        payload["grade"] = "green"
                return payload
        except Exception:
            pass

    payload = state.telemetry.snapshot_dict()
    payload.update(state.held_snapshot())
    payload["connected"] = state.connected
    payload["mode"] = state.mode
    payload["soft_kill"] = state.soft_kill
    payload["connect_stream_hz"] = state.stream_hz
    payload["connect_telemetry_hz"] = state.telemetry_hz
    payload["following_state_file"] = False
    payload["session_dir"] = str(state.telemetry.session_dir)
    payload["cfg_map"] = state.cfg_map
    payload["listen_pdu"] = bool(state.proxy.listen_pdu) if state.proxy is not None else None
    payload["continuous_launch"] = state.continuous_status()
    payload["teleop"] = state.teleop.snapshot()
    if not state.connected:
        payload["grade"] = "idle"
        payload["summary"] = "idle — Follow mode, not connected"
        payload["context"] = [
            "Connect to enter Active — starts frozen (soft_kill ON)",
            "Release soft-kill only when you intend to command actuators",
        ]
    return payload


# -- HTTP server --------------------------------------------------------------------------


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

        def _send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json_body(self) -> dict:
            length = int(self.headers.get("Content-Length") or 0)
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            return json.loads(raw) if raw else {}

        def _serve_static(self, name: str) -> None:
            # No subdirectories, no path traversal — only a flat basename lookup.
            safe = Path(name).name
            fp = _STATIC_DIR / safe
            ctype = _STATIC_CONTENT_TYPES.get(fp.suffix)
            if ctype is None or not fp.is_file():
                self.send_error(404)
                return
            self._send_bytes(fp.read_bytes(), ctype)

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send_bytes(templates.INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if path.startswith("/static/"):
                self._serve_static(path[len("/static/") :])
                return
            if path == "/api/state":
                now = time.monotonic()
                with state_cache_lock:
                    cached = state_cache["payload"]
                    if cached is not None and (now - state_cache["t"]) < state_cache_ttl_s:
                        self._send_json(cached)
                        return
                payload = build_state_payload(state)
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
            if path == "/api/panels":
                self._send_json(panel_status())
                return
            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                body = self._read_json_body()
                result = dispatch_post(state, path, body)
                if result is _NO_ROUTE:
                    self.send_error(404)
                    return
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            # Do not return the full telemetry snapshot on every control POST —
            # snapshot_dict() does a deep asdict under the same lock the stream
            # thread needs for update_from_feedback. The UI already polls
            # /api/state at 5 Hz.
            self._send_json(result if isinstance(result, dict) else {"ok": True})

    return Handler


def serve(state: AppState, host: str = "127.0.0.1", http_port: int = DEFAULT_HTTP_PORT) -> ThreadingHTTPServer:
    wire_panels()
    httpd = ThreadingHTTPServer((host, http_port), make_handler(state))
    thread = threading.Thread(target=httpd.serve_forever, name="deft-dashboard-http", daemon=True)
    thread.start()
    return httpd
