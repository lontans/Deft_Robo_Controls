"""Localhost controller + telemetry dashboard.

Run:

    python -m deft_controls_sdk.debug_dashboard
    # browser: http://127.0.0.1:8765  -> pick a port, click Connect

    python -m deft_controls_sdk.debug_dashboard --port COM5
    # same UI, auto-connects at launch (old one-shot workflow still works)

This process is the one COM owner while connected — AppState opens/closes a
ControlsPcbHub in response to browser actions (Connect/Disconnect), and the
same TelemetryCache instance survives across reconnects so fault history
isn't lost on a board reset. Tier 1: read-only health + black box (as
before) plus plant control (per-slot position/kp/kd hold, mcu_state,
recover) — raw MIT-hold commanding only, no ramping/homing/teleop policy,
same boundary the rest of the SDK draws. DEBUG-mode ops (discover/cfg) are
not exposed here yet.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlparse

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD, list_ports_info
from deft_controls_sdk.telemetry import TelemetryCache, default_session_dir

_ACTIVE_EPS = 0.01
"""Matches firmware's actuator_any_non_idle_live() threshold — see
App/Src/plant/actuator.c. A held desire counts as "active" (not just idle
hold-position) if kp/kd/|velocity|/|torque| exceed this."""


class AppState:
    """Owns the (optional) ControlsPcbHub across connect/disconnect cycles.

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
        self.hub: Optional[ControlsPcbHub] = None

    @property
    def connected(self) -> bool:
        return self.hub is not None

    def connect(self, port: str, *, baud: int = DEFAULT_BAUD) -> None:
        with self._lock:
            if self.hub is not None:
                raise RuntimeError(f"already connected to {self.hub.port} — disconnect first")
            hub = ControlsPcbHub.connect(port, baud=baud, telemetry=self.telemetry)
            hub.start_streaming(hz=self._stream_hz, telemetry_hz=self._telemetry_hz)
            self.hub = hub

    def disconnect(self) -> None:
        with self._lock:
            if self.hub is not None:
                self.hub.close()
                self.hub = None

    def _require_hub(self) -> ControlsPcbHub:
        hub = self.hub
        if hub is None:
            raise RuntimeError("not connected")
        return hub

    def set_actuator(self, slot: int, *, position: float, kp: float, kd: float) -> None:
        # send=False: only update the held desire. The background stream loop
        # already write()s the plant image at stream_hz — a second write+flush
        # from this HTTP thread contended on the serial lock and stalled fb_hz.
        #
        # HOME_POS_EPS: legacy plant teleop never sends position==0 on an active
        # slot — firmware treats blank MCP desires (pos=0 and idle gains) as
        # "skip SPI entirely", so a hold at true 0 with kp>0 is fine, but a
        # hold at 0 with kp=0 would go silent on CH4–6. Match legacy eps when
        # the operator leaves pos at 0 but raises kp/kd.
        pos = position
        if abs(pos) < 1e-6 and (abs(kp) > 1e-9 or abs(kd) > 1e-9):
            pos = 1e-6
        self._require_hub().set_actuator(
            slot,
            ActuatorDesire(position=pos, velocity=0.0, kp=kp, kd=kd, torque=0.0),
            send=False,
        )

    def idle_actuator(self, slot: int) -> None:
        self._require_hub().set_actuator(slot, ActuatorDesire(), send=False)

    def idle_all_actuators(self) -> None:
        """Blank every held desire — Apply accumulates slots; MCP LEDs on
        'other' buses are usually leftover holds, not cross-rail firmware TX."""
        hub = self._require_hub()
        for slot in range(ACTUATOR_COUNT):
            hub.set_actuator(slot, ActuatorDesire(), send=False)

    def set_mcu_state(self, state: int) -> None:
        self._require_hub().set_mcu_state(McuState(state))

    def recover(self) -> None:
        self._require_hub().recover()

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


_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Deft controls — telemetry</title>
<style>
  :root {
    --bg: #12141a;
    --panel: #1c2030;
    --text: #e8eaef;
    --muted: #9aa3b5;
    --green: #3dcf8e;
    --yellow: #e6c35c;
    --red: #e86a6a;
    --line: #2a3144;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; font-family: "Segoe UI", system-ui, sans-serif;
    background: var(--bg); color: var(--text);
    min-height: 100vh;
  }
  header {
    padding: 1rem 1.25rem; border-bottom: 1px solid var(--line);
    display: flex; flex-wrap: wrap; gap: 1rem; align-items: baseline;
  }
  header h1 { font-size: 1.1rem; font-weight: 600; margin: 0; letter-spacing: 0.02em; }
  header .meta { color: var(--muted); font-size: 0.85rem; }
  .grade {
    font-weight: 700; text-transform: uppercase; letter-spacing: 0.06em;
    padding: 0.2rem 0.55rem; border-radius: 4px; font-size: 0.75rem;
  }
  .grade.green { background: color-mix(in srgb, var(--green) 25%, transparent); color: var(--green); }
  .grade.yellow { background: color-mix(in srgb, var(--yellow) 25%, transparent); color: var(--yellow); }
  .grade.red { background: color-mix(in srgb, var(--red) 25%, transparent); color: var(--red); }
  main { padding: 1.25rem; display: grid; gap: 1rem; max-width: 1200px; }
  .card {
    background: var(--panel); border: 1px solid var(--line); border-radius: 8px;
    padding: 1rem 1.1rem;
  }
  .card h2 { margin: 0 0 0.75rem; font-size: 0.8rem; color: var(--muted);
    text-transform: uppercase; letter-spacing: 0.08em; font-weight: 600; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(140px, 1fr)); gap: 0.75rem; }
  .kv .k { display: block; color: var(--muted); font-size: 0.7rem; text-transform: uppercase; letter-spacing: 0.05em; }
  .kv .v { font-variant-numeric: tabular-nums; font-size: 1.05rem; margin-top: 0.15rem; }
  ul.context { margin: 0; padding-left: 1.1rem; color: var(--muted); }
  ul.context li { margin: 0.25rem 0; }
  table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; font-size: 0.85rem; }
  th, td { text-align: left; padding: 0.35rem 0.4rem; border-bottom: 1px solid var(--line); }
  th { color: var(--muted); font-weight: 500; font-size: 0.7rem; text-transform: uppercase; }
  .summary { font-size: 1.15rem; margin: 0 0 0.5rem; }
  button, select, input {
    font-family: inherit; font-size: 0.8rem; background: var(--bg); color: var(--text);
    border: 1px solid var(--line); border-radius: 6px; padding: 0.35rem 0.6rem;
  }
  button { cursor: pointer; font-weight: 600; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button.record.on { background: color-mix(in srgb, var(--red) 25%, transparent); color: var(--red); border-color: var(--red); }
  button.estop { color: var(--red); border-color: var(--red); }
  .fault-badge { color: var(--red); font-weight: 600; }
  .row { display: flex; gap: 0.5rem; align-items: center; flex-wrap: wrap; }
  .err { color: var(--red); font-size: 0.8rem; min-height: 1.1em; margin: 0.4rem 0 0; }
  input[type=number] { width: 4.5rem; }
  .streaming-badge { font-size: 0.8rem; color: var(--muted); margin-left: auto; }
  .streaming-badge.on { color: var(--green); }
  .active-badge { font-weight: 700; }
  .active-badge.active { color: var(--yellow); }
  .active-badge.idle { color: var(--muted); }
  td.held { font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<header>
  <h1>Deft controls telemetry</h1>
  <span id="grade" class="grade red">red</span>
  <span class="meta" id="meta">connecting…</span>
</header>
<main>
  <section class="card">
    <h2>Connection</h2>
    <div class="row">
      <select id="portSelect"></select>
      <button id="connectBtn" onclick="connect()">Connect</button>
      <button id="disconnectBtn" onclick="disconnect()" disabled>Disconnect</button>
      <span class="meta" id="connMeta">not connected</span>
    </div>
    <p class="err" id="connError"></p>
  </section>
  <section class="card">
    <p class="summary" id="summary">—</p>
    <ul class="context" id="context"></ul>
  </section>
  <section class="card">
    <h2>Tier-1 health</h2>
    <div class="grid" id="health"></div>
  </section>
  <section class="card">
    <h2>Black box</h2>
    <div class="grid" id="blackbox"></div>
    <button class="record" id="recordBtn" onclick="toggleRecord()">● Record</button>
    <span class="meta" id="recordMeta"></span>
  </section>
  <section class="card">
    <h2>Plant control</h2>
    <div class="row">
      <button id="mcuNormal" onclick="setMcuState(0)">NORMAL</button>
      <button id="mcuRecovery" onclick="setMcuState(1)">RECOVERY</button>
      <button id="mcuDiag" onclick="setMcuState(2)" title="Blocks plant apply per firmware">DIAG_ONLY</button>
      <button id="mcuEstop" class="estop" onclick="setMcuState(3)">ESTOP</button>
      <button id="recoverBtn" onclick="recover()">Recover</button>
      <button id="idleAllBtn" onclick="idleAll()">Idle all slots</button>
      <span class="streaming-badge" id="streamingBadge">streaming: —</span>
    </div>
    <p class="err" id="ctrlError"></p>
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
  </section>
</main>
<script>
function kv(k, v) {
  return `<div class="kv"><span class="k">${k}</span><div class="v">${v ?? "—"}</div></div>`;
}
function fmt(n, d=2) {
  if (n === null || n === undefined) return "—";
  if (typeof n === "number") return Number.isInteger(n) ? String(n) : n.toFixed(d);
  return String(n);
}
function setConnError(msg) { document.getElementById("connError").textContent = msg || ""; }
function setCtrlError(msg) { document.getElementById("ctrlError").textContent = msg || ""; }

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
  postAction("/api/connect", { port }, setConnError);
}
function disconnect() {
  document.getElementById("disconnectBtn").disabled = true;
  postAction("/api/disconnect", {}, setConnError);
}
function setMcuState(n) { postAction("/api/mcu_state", { state: n }, setCtrlError); }
function recover() { postAction("/api/recover", {}, setCtrlError); }
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
      <td><input type="number" step="0.01" id="pos${slot}" placeholder="0.0"></td>
      <td><input type="number" step="0.1" id="kp${slot}" placeholder="0.0"></td>
      <td><input type="number" step="0.01" id="kd${slot}" placeholder="0.0"></td>
      <td>
        <button id="apply${slot}" onclick="applyActuator(${slot})">Apply</button>
        <button id="idle${slot}" onclick="idleActuator(${slot})">Idle</button>
      </td>
    </tr>`;
  }
  document.getElementById("acts").innerHTML = rows;
  actuatorRowsBuilt = true;
}

async function tick() {
  try {
    const r = await fetch("/api/state");
    const s = await r.json();
    const g = document.getElementById("grade");
    g.textContent = s.grade || "red";
    g.className = "grade " + (s.grade || "red");
    document.getElementById("summary").textContent = s.summary || "—";
    document.getElementById("meta").textContent =
      `${s.port || "?"} · ${s.mode || "?"} · poll 200ms`;
    const ctx = document.getElementById("context");
    ctx.innerHTML = (s.context || []).map(c => `<li>${c}</li>`).join("");

    document.getElementById("health").innerHTML = [
      kv("tick", s.tick),
      kv("fb_hz", s.fb_hz != null ? s.fb_hz.toFixed(1) : null),
      kv("plant_block", s.plant_block_name || s.plant_block),
      kv("mcu_state", s.mcu_state),
      kv("ack_seq", s.ack_seq),
      kv("ack_lag", s.stream_ack_lag),
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
      kv("ui_note", "3 jobs: plant TX / UI snapshot / opt-in Record. Healthy: ack_lag~0, fb flood when idle. If MCP Apply spikes lag, Idle all + check firmware plant MCP path."),
      kv("svd", s.svd_present ? "yes" : "no"),
      kv("age_s", s.age_s != null ? s.age_s.toFixed(2) : null),
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

    // Connection card
    document.getElementById("connectBtn").disabled = !!s.connected;
    document.getElementById("disconnectBtn").disabled = !s.connected;
    document.getElementById("portSelect").disabled = !!s.connected;
    document.getElementById("connMeta").textContent = s.connected ? `connected: ${s.port}` : "not connected";

    // Plant control gating — never disable ESTOP so a stuck link doesn't hide the kill switch
    for (const id of ["mcuNormal", "mcuRecovery", "mcuDiag", "recoverBtn"]) {
      document.getElementById(id).disabled = !s.connected;
    }
    document.getElementById("mcuEstop").disabled = false;

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
        if (applyBtn) applyBtn.disabled = !s.connected;
        if (idleBtn) idleBtn.disabled = !s.connected;
      });
    }
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
                payload = state.telemetry.snapshot_dict()
                payload.update(state.held_snapshot())
                with state_cache_lock:
                    state_cache["t"] = now
                    state_cache["payload"] = payload
                self._send_json(payload)
                return
            if path == "/api/ports":
                self._send_json({"ports": list_ports_info()})
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
                    state.connect(port)
                elif path == "/api/disconnect":
                    state.disconnect()
                elif path == "/api/record/start":
                    state.telemetry.start_recording()
                elif path == "/api/record/stop":
                    state.telemetry.stop_recording()
                elif path == "/api/recover":
                    state.recover()
                elif path == "/api/mcu_state":
                    state.set_mcu_state(int(body["state"]))
                elif path == "/api/actuator/idle_all":
                    state.idle_all_actuators()
                elif path.startswith("/api/actuator/"):
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


def serve(state: AppState, host: str = "127.0.0.1", http_port: int = 8765) -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer((host, http_port), make_handler(state))
    thread = threading.Thread(target=httpd.serve_forever, name="deft-dashboard-http", daemon=True)
    thread.start()
    return httpd
