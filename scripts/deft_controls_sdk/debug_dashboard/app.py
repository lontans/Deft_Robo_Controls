"""Localhost controller + telemetry dashboard.

Run:

    python -m deft_controls_sdk.debug_dashboard
    # browser: http://127.0.0.1:8766  -> pick a port, click Connect (observe)

    python -m deft_controls_sdk.debug_dashboard --port COM5
    # same UI, auto-connects in observe mode at launch

Connect defaults to **observe**: plant stream clears HOST_STALE, MCU stays
DIAG_ONLY, and soft-kill auto-park hooks are off — so opening the dashboard
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

from deft_controls_sdk import ActuatorDesire, ControlsPcbHub, McuState
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD, list_ports_info
from deft_controls_sdk.telemetry import TelemetryCache, default_session_dir

# Peer (e.g. yam_continuous_all) owns CDC while dashboard follows state.json —
# Soft-kill Park writes this flag; the peer parks and deletes it.
SOFT_KILL_REQUEST_NAME = "soft_kill_request"

ControlMode = Literal["observe", "control"]
DEFAULT_HTTP_PORT = 8766

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
        self.control_mode: ControlMode = "observe"

    @property
    def connected(self) -> bool:
        return self.hub is not None

    def connect(
        self,
        port: str,
        *,
        baud: int = DEFAULT_BAUD,
        mode: ControlMode = "observe",
    ) -> None:
        """Open CDC and start the plant stream.

        Default ``mode="observe"``: DIAG_ONLY + no auto soft-kill park — safe
        telemetry without faulting the board on Connect. Pass ``mode="control"``
        only when the operator wants NORMAL plant apply + park hooks.
        """
        if mode not in ("observe", "control"):
            raise ValueError("mode must be 'observe' or 'control'")
        with self._lock:
            if self.hub is not None:
                raise RuntimeError(f"already connected to {self.hub.port} — disconnect first")
            hub = ControlsPcbHub.connect(port, baud=baud, telemetry=self.telemetry)
            # Observe first: clear HOST_STALE without latching ESTOP from PDU.
            hub.start_streaming(
                hz=self._stream_hz,
                telemetry_hz=self._telemetry_hz,
                auto_soft_kill=False,
            )
            hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
            self.hub = hub
            self.control_mode = "observe"
            if mode == "control":
                self._enter_control_locked()

    def set_control_mode(self, mode: ControlMode) -> None:
        if mode not in ("observe", "control"):
            raise ValueError("mode must be 'observe' or 'control'")
        with self._lock:
            hub = self.hub
            if hub is None:
                raise RuntimeError("not connected")
            if mode == "control":
                self._enter_control_locked()
            else:
                hub.set_auto_soft_kill(False)
                for slot in range(ACTUATOR_COUNT):
                    hub.set_actuator(slot, ActuatorDesire(), send=False)
                hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
                self.control_mode = "observe"

    def _enter_control_locked(self) -> None:
        hub = self.hub
        assert hub is not None
        hub.set_auto_soft_kill(True)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        self.control_mode = "control"

    def disconnect(self) -> None:
        with self._lock:
            if self.hub is not None:
                # Leave MCU in DIAG_ONLY so disconnect does not freeze an ESTOP
                # latch from a soft-kill park that happened while connected.
                try:
                    self.hub.set_auto_soft_kill(False)
                    self.hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
                    time.sleep(0.05)
                except Exception:
                    pass
                self.hub.close()
                self.hub = None
            self.control_mode = "observe"

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

    def soft_kill_request_path(self) -> Path:
        return self.telemetry.session_dir / SOFT_KILL_REQUEST_NAME

    def soft_kill_park(self) -> dict:
        """Product soft-kill park (see ControlsPcbHub.soft_kill_park).

        When this process owns COM: clear desires and latch McuState.ESTOP.
        When following a peer's state.json (continuous owns CDC): write a
        request flag the peer polls — Connect COM is not required.
        """
        if self.hub is not None:
            self.hub.soft_kill_park()
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
  .grade.idle { background: color-mix(in srgb, var(--muted) 22%, transparent); color: var(--muted); }
  .banner {
    margin: 0.55rem 0 0; padding: 0.45rem 0.65rem; border-radius: 6px;
    background: color-mix(in srgb, var(--yellow) 12%, transparent);
    border: 1px solid color-mix(in srgb, var(--yellow) 35%, var(--line));
    color: var(--text); font-size: 0.8rem; line-height: 1.35;
  }
  .banner.ok {
    background: color-mix(in srgb, var(--green) 10%, transparent);
    border-color: color-mix(in srgb, var(--green) 30%, var(--line));
  }
  .banner.warn {
    background: color-mix(in srgb, var(--red) 12%, transparent);
    border-color: color-mix(in srgb, var(--red) 35%, var(--line));
  }
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
  <span id="grade" class="grade idle">idle</span>
  <span class="meta" id="meta">not connected</span>
</header>
<main>
  <section class="card">
    <h2>Connection</h2>
    <div class="row">
      <select id="portSelect"></select>
      <button id="connectBtn" onclick="connect()">Connect (observe)</button>
      <button id="enableCtrlBtn" onclick="enableControl()" disabled title="Switch MCU to NORMAL and arm soft-kill auto-park">Enable control</button>
      <button id="observeBtn" onclick="toObserve()" disabled title="Blank desires, DIAG_ONLY, no auto-park">Back to observe</button>
      <button id="disconnectBtn" onclick="disconnect()" disabled>Disconnect</button>
      <span class="meta" id="connMeta">not connected</span>
    </div>
    <p class="banner ok" id="modeBanner">Idle — not owning COM. Connect opens observe mode (DIAG_ONLY); plant motors stay gated until Enable control.</p>
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
    <h2>PDU / soft-kill</h2>
    <div class="grid" id="pdu"></div>
    <div class="row" style="margin-top:0.5rem">
      <button class="estop" id="softKillBtn" onclick="softKillPark()">Soft-kill Park</button>
      <span class="meta" id="pdbMeta"></span>
    </div>
    <p class="err" id="pdbError"></p>
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
function setMcuState(n) { postAction("/api/mcu_state", { state: n }, setCtrlError); }
function recover() { postAction("/api/recover", {}, setCtrlError); }
function softKillPark() { postAction("/api/pdb/soft_kill_park", {}, setPdbError); }
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
    const grade = s.grade || "idle";
    g.textContent = grade;
    g.className = "grade " + grade;
    document.getElementById("summary").textContent = s.summary || "—";
    const modeLabel = s.control_mode || (s.connected ? "observe" : "idle");
    document.getElementById("meta").textContent = s.connected
      ? `${s.port || "?"} · ${modeLabel} · poll 200ms`
      : (s.following_state_file ? `follow · poll 200ms` : `idle · poll 200ms`);
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

    // PDU / soft-kill — pdb_status is null until the first feedback frame
    // with a parseable system-kill block arrives (or never connected).
    const pdb = s.pdb_status;
    const hostRequested = s.mcu_state === 3;  // McuState.ESTOP — same latch plant_command.c drives to pdb_link_request_estop()
    document.getElementById("pdu").innerHTML = pdb ? [
      kv("kill_state (MCU view, freshness-gated)", pdb.kill_state_name),
      kv("kill_reason", pdb.kill_reason_name),
      kv("stale_failsafe", pdb.stale_failsafe ? "yes (COMMS_LOSS -> HARD)" : "no"),
      kv("host requested ESTOP", hostRequested ? "yes" : "no"),
      kv("local estop_sense (Controls PB7)", pdb.estop_sense === 1 ? "1 (allowed)" : "0 (asserted)"),
      kv("peer kill_state (raw PDBF)", pdb.pdb ? PEER_KILL_STATE_NAMES[pdb.pdb.kill_state] ?? pdb.pdb.kill_state : "—"),
      kv("peer estop_sense (raw PDBF)", pdb.pdb ? (pdb.pdb.estop_sense === 1 ? "1 (allowed)" : "0 (asserted)") : "—"),
      kv("contactor_state", pdb.pdb ? pdb.pdb.contactor_state : "—"),
      kv("pack_v (V) x4", fmtVec(pdb.pack_v_V)),
      kv("rail_v (V) x4", fmtVec(pdb.rail_v_V)),
      kv("pack_i (A) x4", fmtVec(pdb.pack_i_A)),
      kv("rail_i (A) x4", fmtVec(pdb.rail_i_A)),
    ].join("") : `<div class="kv"><span class="k">status</span><div class="v">no PDB frame yet</div></div>`;
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
      pdbHint = "follow mode — Park signals continuous via soft_kill_request";
    pdbMeta.textContent = pdbHint;

    // Connection card
    const inControl = !!s.connected && s.control_mode === "control";
    const inObserve = !!s.connected && s.control_mode !== "control";
    document.getElementById("connectBtn").disabled = !!s.connected;
    document.getElementById("disconnectBtn").disabled = !s.connected;
    document.getElementById("enableCtrlBtn").disabled = !inObserve;
    document.getElementById("observeBtn").disabled = !inControl;
    document.getElementById("portSelect").disabled = !!s.connected;
    document.getElementById("connMeta").textContent = s.connected
      ? `${s.port} · ${s.control_mode || "observe"}`
      : (s.following_state_file
          ? `following ${s.state_path || "state.json"}${s.peer_connected ? " (peer live)" : ""}`
          : "not connected — Connect (observe) is safe; Enable control only when you want motors");
    const banner = document.getElementById("modeBanner");
    if (!s.connected && !s.following_state_file) {
      banner.className = "banner ok";
      banner.textContent = "Idle — not owning COM. Connect opens observe (DIAG_ONLY, no auto soft-kill). PDU sim panel uses :8765; this UI defaults to :8766.";
    } else if (!s.connected && s.following_state_file) {
      banner.className = "banner ok";
      banner.textContent = "Follow mode — reading state.json only. Soft-kill Park signals the peer; do not Connect while continuous owns CDC.";
    } else if (inObserve) {
      banner.className = "banner";
      banner.textContent = "Observe — streaming telemetry, MCU DIAG_ONLY, auto soft-kill off. Board will not ESTOP from PDU V/I on connect. Enable control when ready to command.";
    } else {
      banner.className = "banner warn";
      banner.textContent = "Control — MCU NORMAL + soft-kill auto-park armed. Idle all / Back to observe before Disconnect if you parked.";
    }

    // Plant control gating — ESTOP only when we own COM (follow mode uses Soft-kill Park)
    const plantLive = inControl;
    for (const id of ["mcuNormal", "mcuRecovery", "mcuDiag", "recoverBtn", "idleAllBtn", "mcuEstop"]) {
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
                elif path == "/api/actuator/idle_all":
                    if not state.connected:
                        raise RuntimeError("not connected")
                    if state.control_mode != "control":
                        raise RuntimeError("Enable control first (observe mode is read-only)")
                    state.idle_all_actuators()
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
