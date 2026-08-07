function kv(k, v, cls) {
  return `<div class="kv${cls ? " " + cls : ""}"><span class="k">${k}</span><div class="v">${v ?? "—"}</div></div>`;
}
// Bigger sibling of kv() for the small set of "read this first" numbers atop
// the Health card (stat-hero-row) — same shape, different visual weight.
function statTile(k, v, cls) {
  return `<div class="stat-tile${cls ? " " + cls : ""}"><span class="k">${k}</span><div class="v">${v ?? "—"}</div></div>`;
}
function fmt(n, d=2) {
  if (n === null || n === undefined) return "—";
  if (typeof n === "number") return Number.isInteger(n) ? String(n) : n.toFixed(d);
  return String(n);
}
function esc(s) {
  return String(s).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
}
function setConnError(msg) { document.getElementById("connError").textContent = msg || ""; }
function setCtrlError(msg) { document.getElementById("ctrlError").textContent = msg || ""; }
function setPdbError(msg) { document.getElementById("pdbError").textContent = msg || ""; }
function setCfgError(msg) { document.getElementById("cfgError").textContent = msg || ""; }
function setLedError(msg) { document.getElementById("ledError").textContent = msg || ""; }

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
    return data;
  } catch (e) {
    errSetter(String(e));
    return null;
  } finally {
    tick();
  }
}

// -- Panel actions (POST /api/panels/<key>/run) — shared by Testing + CFG + LED ------------

async function runPanel(key, body) {
  const r = await fetch(`/api/panels/${key}/run`, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}),
  });
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || `request failed (${r.status})`);
  return data.result;
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

// -- Follow / Active x soft_kill ----------------------------------------------------------

function connect() {
  const port = document.getElementById("portSelect").value;
  if (!port) { setConnError("choose a port first"); return; }
  const body = { port };
  const hzRaw = document.getElementById("connectHz").value;
  if (hzRaw) {
    const hz = parseFloat(hzRaw);
    if (!(hz > 0)) { setConnError("hz must be a positive number"); return; }
    body.hz = hz;
  }
  document.getElementById("connectBtn").disabled = true;
  postAction("/api/connect", body, setConnError);
}
function disconnect() {
  document.getElementById("disconnectBtn").disabled = true;
  postAction("/api/disconnect", {}, setConnError);
}
function releaseSoftKill() { postAction("/api/soft_kill/release", {}, setConnError); }
function engageSoftKill() { postAction("/api/soft_kill/engage", {}, setConnError); }

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

async function hardEstopPark() {
  const resultEl = document.getElementById("pdbResult");
  resultEl.textContent = "parking…";
  try {
    const r = await fetch("/api/estop/park", { method: "POST" });
    const data = await r.json();
    if (!r.ok) {
      setCtrlError(data.error || `request failed (${r.status})`);
      resultEl.textContent = "";
    } else {
      setCtrlError("");
      const now = new Date().toLocaleTimeString();
      resultEl.textContent = data.mode === "direct"
        ? `parked direct (MCU → ESTOP, desires blanked) at ${now}`
        : `flag written at ${now} — ${data.path || "soft_kill_request"} (peer must poll to act)`;
    }
  } catch (e) {
    setCtrlError(String(e));
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

// -- Panels catalog ------------------------------------------------------------------------

let lastPanels = {};
async function loadPanels() {
  try {
    const r = await fetch("/api/panels");
    lastPanels = await r.json();
    renderPanels();
  } catch (e) { /* non-fatal — informational only */ }
}
function renderPanels() {
  const el = document.getElementById("panelsList");
  if (!el) return;
  el.innerHTML = Object.values(lastPanels).map(p => `
    <div class="panel-row">
      <span>
        <span class="pr-name">${p.label}</span> · <span class="pr-blurb">${p.blurb}</span><br>
        <span class="pr-blurb"><code>${p.call_sig}</code></span>
      </span>
      <span class="pr-badge ${p.available ? "ok" : "missing"}">${p.available ? "wired" : "unavailable"}</span>
    </div>`).join("") || `<p class="meta">no panel actions registered</p>`;
}

// -- Health: bandwidth trigger -------------------------------------------------------------

async function runBandwidthTest() {
  const port = document.getElementById("bandwidthPort").value;
  const seconds = parseFloat(document.getElementById("bandwidthSeconds").value) || 2.0;
  const resultEl = document.getElementById("bandwidthResult");
  resultEl.textContent = "running…";
  try {
    const result = await runPanel("bandwidth", { port, seconds });
    resultEl.textContent = JSON.stringify(result);
  } catch (e) {
    resultEl.textContent = String(e);
  }
}

// -- CFG: sample / edit-one-slot / replace-whole-table --------------------------------------

let lastCfg = null;         // last sampled collect_cfg() result
let lastActuatorsFb = [];   // updated every tick() from /api/state — live overlay, not re-sampled

function railFor(bus) {
  const b = Number(bus);
  if (b >= 1 && b <= 3) return "fdcan";
  if (b >= 4 && b <= 6) return "mcp";
  return "?";
}

// Mirrors pcb_tui.py's slot_connected(): enabled + real motor_id + FB shows
// actual data (nonzero temp/fault, or meaningfully nonzero pos/vel/torque) —
// not just "enabled" (which only says configured, not actually replying).
function slotConnected(row, fb) {
  if (!row.enabled) return null;
  const mid = Number(row.motor_id) & 0xFF;
  if (!mid) return false;
  if (!fb) return false;
  if (Number(fb.temperature || 0) !== 0) return true;
  if (Number(fb.fault || 0) !== 0) return true;
  for (const k of ["position", "velocity", "torque"]) {
    if (Math.abs(Number(fb[k] || 0)) > 1e-4) return true;
  }
  return false;
}

function fbForSlot(slot) {
  const a = lastActuatorsFb[slot];
  return a || null;
}

async function sampleCfg() {
  const btn = document.getElementById("cfgSampleBtn");
  btn.disabled = true;
  try {
    lastCfg = await runPanel("cfg_show", {});
    setCfgError("");
    document.getElementById("cfgSampleMeta").textContent = `sampled at ${new Date().toLocaleTimeString()}`;
    renderCfgTable();
    renderCfgPeriph();
  } catch (e) {
    setCfgError(String(e));
  } finally {
    btn.disabled = false;
  }
}

function renderCfgTable() {
  const tbody = document.getElementById("cfgSlotRows");
  if (!lastCfg) { tbody.innerHTML = `<tr><td colspan="12" class="meta">Sample CFG to populate.</td></tr>`; return; }
  const rows = lastCfg.slots || [];
  if (!rows.length) { tbody.innerHTML = `<tr><td colspan="12" class="meta">(no CFG slots)</td></tr>`; return; }
  tbody.innerHTML = rows.map(r => {
    const slot = Number(r.slot);
    const fb = fbForSlot(slot);
    const conn = slotConnected(r, fb);
    const connTxt = conn === null ? "—" : (conn ? "yes" : "no");
    const connCls = conn === true ? "ok" : (conn === false ? "missing" : "");
    return `<tr>
      <td>${slot}</td>
      <td>${r.enabled ? "Y" : "."}</td>
      <td>${r.bus ?? "—"}</td>
      <td>${railFor(r.bus)}</td>
      <td>${esc(r.protocol_name ?? r.protocol ?? "?")}</td>
      <td>${esc(r.motor_id_hex ?? r.motor_id ?? "—")}</td>
      <td class="pr-badge ${connCls}">${connTxt}</td>
      <td>${fmt(fb ? fb.position : null, 3)}</td>
      <td>${fmt(fb ? fb.velocity : null, 3)}</td>
      <td>${fmt(fb ? fb.torque : null, 2)}</td>
      <td>${fmt(fb ? fb.temperature : null, 1)}</td>
      <td><button class="btn-ghost" onclick="prefillCfgEdit(${slot})">Edit</button></td>
    </tr>`;
  }).join("");
}

function renderCfgPeriph() {
  const el = document.getElementById("cfgPeriph");
  if (!lastCfg) { el.innerHTML = ""; return; }
  const servos = lastCfg.servos || [];
  const led = lastCfg.led || {};
  const servoRows = servos.map(s => `
    <tr><td>${s.slot}</td><td>${s.enabled ? "Y" : "."}</td><td>${esc(s.model_name ?? s.model ?? "?")}</td>
    <td>${esc(s.id_hex ?? s.id ?? "—")}</td><td>[${s.pos_min ?? "?"}..${s.pos_max ?? "?"}]</td></tr>`).join("");
  el.innerHTML = `
    <table>
      <thead><tr><th>slot</th><th>en</th><th>model</th><th>id</th><th>limits</th></tr></thead>
      <tbody>${servoRows || `<tr><td colspan="5" class="meta">(no NVM servo rows)</td></tr>`}</tbody>
    </table>
    <div class="grid compact" style="margin-top:0.5rem">
      ${kv("listen_pdu", lastCfg.listen_pdu ? "ON" : "off")}
      ${kv("led count", led.default_count)}
      ${kv("led mode", led.default_mode)}
      ${kv("led brightness", led.default_brightness)}
    </div>`;
}

function prefillCfgEdit(slot) {
  const row = (lastCfg && lastCfg.slots || []).find(r => Number(r.slot) === Number(slot));
  document.getElementById("cfgEditSlot").value = slot;
  if (row) {
    document.getElementById("cfgEditBus").value = row.bus ?? "";
    document.getElementById("cfgEditProtocol").value = row.protocol ?? "";
    document.getElementById("cfgEditMotorId").value = row.motor_id ?? "";
    document.getElementById("cfgEditMasterId").value = row.master_id ?? 0;
    document.getElementById("cfgEditEnabled").checked = !!row.enabled;
  }
  document.getElementById("cfgEditDetails").open = true;
  document.getElementById("cfgEditSlot").scrollIntoView({ behavior: "smooth", block: "center" });
}

async function applyCfgSlotEdit() {
  const g = id => document.getElementById(id);
  const slot = parseInt(g("cfgEditSlot").value, 10);
  if (Number.isNaN(slot)) { setCfgError("slot is required"); return; }
  const motorIdRaw = g("cfgEditMotorId").value.trim();
  const motor_id = motorIdRaw.startsWith("0x") ? parseInt(motorIdRaw, 16) : parseInt(motorIdRaw, 10);
  try {
    await runPanel("cfg_set_slot", {
      slot, bus: parseInt(g("cfgEditBus").value, 10) || 0,
      protocol: parseInt(g("cfgEditProtocol").value, 10) || 0,
      motor_id: Number.isNaN(motor_id) ? 0 : motor_id,
      master_id: parseInt(g("cfgEditMasterId").value, 10) || 0,
      enabled: !!g("cfgEditEnabled").checked,
      persist: !!g("cfgEditPersist").checked,
    });
    setCfgError("");
    await sampleCfg();
  } catch (e) {
    setCfgError(String(e));
  }
}

function copyCfgTableToEditor() {
  if (!lastCfg) { setCfgError("sample CFG first"); return; }
  const rows = (lastCfg.slots || []).filter(r => r.enabled).map(r => ({
    slot: r.slot, bus: r.bus, protocol: r.protocol, motor_id: r.motor_id,
    master_id: r.master_id || 0, enabled: true,
  }));
  document.getElementById("cfgTableJson").value = JSON.stringify(rows, null, 2);
}

async function applyCfgTableFromEditor() {
  let rows;
  try {
    rows = JSON.parse(document.getElementById("cfgTableJson").value || "[]");
  } catch (e) {
    setCfgError("invalid JSON: " + e.message);
    return;
  }
  const disable_missing = !!document.getElementById("cfgTableDisableMissing").checked;
  const persist = !!document.getElementById("cfgTablePersist").checked;
  try {
    await runPanel("cfg_apply_table", { rows, disable_missing, persist });
    setCfgError("");
    await sampleCfg();
  } catch (e) {
    setCfgError(String(e));
  }
}

async function saveCfgNvm() {
  try {
    await runPanel("cfg_save_nvm", {});
    setCfgError("saved to NVM at " + new Date().toLocaleTimeString());
  } catch (e) {
    setCfgError(String(e));
  }
}

// -- Testing: shared terminal-style output pane ----------------------------------------------

function appendTerminal(label, result, isError) {
  const el = document.getElementById("testingTerminal");
  const empty = el.querySelector(".term-empty");
  if (empty) empty.remove();
  const ts = new Date().toLocaleTimeString();
  const hasBody = result !== "" && result !== undefined;
  const body = isError ? String(result) : (hasBody ? JSON.stringify(result, null, 2) : "");
  const entry = document.createElement("div");
  entry.className = "term-entry" + (isError ? " err" : "");
  entry.innerHTML = `<div class="term-head"><span class="term-ts">${esc(ts)}</span><span class="term-label">${esc(label)}</span></div>` +
    (body ? `<pre class="term-body">${esc(body)}</pre>` : "");
  el.appendChild(entry);
  el.scrollTop = el.scrollHeight;
}
function clearTerminal() {
  document.getElementById("testingTerminal").innerHTML = `<div class="term-empty">ready.</div>`;
}

async function runInventory() {
  const preset = document.getElementById("inventoryPreset").value;
  appendTerminal(`inventory(preset=${preset}) — running…`, "", false);
  try {
    const result = await runPanel("inventory", { preset });
    appendTerminal(`inventory(preset=${preset})`, result, false);
  } catch (e) {
    appendTerminal(`inventory(preset=${preset}) FAILED`, e, true);
  }
}

async function runDiscover() {
  const buses = document.getElementById("discoverBuses").value.split(",").map(s => parseInt(s.trim(), 10)).filter(n => !Number.isNaN(n));
  const protocols = document.getElementById("discoverProtocols").value.split(",").map(s => s.trim()).filter(Boolean);
  const label = `discover(buses=${JSON.stringify(buses)}, protocols=${JSON.stringify(protocols)})`;
  appendTerminal(`${label} — running, may take a while…`, "", false);
  try {
    const result = await runPanel("discover", { buses, protocols });
    appendTerminal(label, result, false);
  } catch (e) {
    appendTerminal(`${label} FAILED`, e, true);
  }
}

async function runCalibrate() {
  const bus = parseInt(document.getElementById("calibrateBus").value, 10);
  const motor_id = parseInt(document.getElementById("calibrateMotorId").value, 10);
  const cal_listen_s = parseFloat(document.getElementById("calibrateListenS").value) || 28.0;
  if (Number.isNaN(bus) || Number.isNaN(motor_id)) { appendTerminal("calibrate FAILED", "bus and motor_id are required", true); return; }
  const label = `calibrate(bus=${bus}, motor_id=${motor_id}, cal_listen_s=${cal_listen_s})`;
  appendTerminal(`${label} — running, ~15-35s, shaft must spin freely…`, "", false);
  try {
    const result = await runPanel("calibrate", { bus, motor_id, cal_listen_s });
    appendTerminal(label, result, false);
  } catch (e) {
    appendTerminal(`${label} FAILED`, e, true);
  }
}

// -- Controls: LED --------------------------------------------------------------------------

async function setLedMode() {
  const mode = document.getElementById("ledMode").value;
  const brightness = parseInt(document.getElementById("ledBrightness").value, 10) || 0;
  const pattern = parseInt(document.getElementById("ledPattern").value, 10) || 0;
  try {
    await runPanel("led_set", { mode, brightness, pattern });
    setLedError("");
  } catch (e) {
    setLedError(String(e));
  }
}
async function applyLedPreset() {
  const preset_name = document.getElementById("ledPreset").value;
  try {
    await runPanel("led_preset", { preset_name });
    setLedError("");
  } catch (e) {
    setLedError(String(e));
  }
}

// -- Teleop (per-slot target+cruise slew) --------------------------------------------------

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

// Session-local marked encoder edges (not persisted; copy yourself).
const capturedLimits = {};  // slot -> {lo?, hi?}
let lastArmFb = {};         // slot -> position

function selectArmSlot(slot, ev) {
  if (ev && ev.target && (ev.target.closest("button") || ev.target.closest("input"))) return;
  _keysSetArmSlot(slot);
}
// Mirrors selectArmSlot() for the joint-card's own tabindex/role="button" —
// keeps the card keyboard-reachable (Tab + Enter/Space) alongside the
// existing 1-7 shortcut, without changing what selection *does*.
function selectArmRowKey(slot, ev) {
  if (ev.target && (ev.target.closest("button") || ev.target.closest("input"))) return;
  if (ev.key !== "Enter" && ev.key !== " ") return;
  ev.preventDefault();
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
      <div class="jc-sub">not live-verified on this bench</div>
      <div class="jc-foot">
        <span class="btn-group"><button class="btn-idle" onclick="idleActuator(${s})" title="Zero torque on this joint only">Idle</button></span>
      </div>
    </div>`;
  }
  const mid = ((spec.lo + spec.hi) / 2).toFixed(3);
  const cap = capturedLimits[s] || {};
  const capTxt = `lo ${cap.lo != null ? Number(cap.lo).toFixed(4) : "—"} · hi ${cap.hi != null ? Number(cap.hi).toFixed(4) : "—"}`;
  return `<div class="joint-card" id="armRow${s}" onclick="selectArmSlot(${s}, event)" onkeydown="selectArmRowKey(${s}, event)"
      tabindex="0" role="button" aria-pressed="${keysArmSlot === s}" title="Click or focus + Enter to select for ←/→ keyboard nudge">
    <div class="jc-head"><span class="jc-label">${spec.label}</span><span class="jc-sub">slot ${s} · rail [${Number(spec.lo).toFixed(2)}, ${Number(spec.hi).toFixed(2)}]</span></div>
    <div class="jc-slider">
      <input type="range" id="armSlider${s}" min="${spec.lo}" max="${spec.hi}" step="0.01" value="${mid}"
        oninput="document.getElementById('armVal${s}').textContent = this.value">
      <span class="val" id="armVal${s}">${mid}</span>
    </div>
    <div class="jc-foot">
      <input type="number" id="armCruise${s}" value="${spec.cruise_default}" step="0.05" min="0" max="${spec.cruise_max}" title="cruise rad/s">
      <span class="btn-group">
        <button class="btn-go" id="armGo${s}" onclick="teleopArmSet(${s})" title="Walk to the slider's position">Go</button>
        <button class="btn-stop" id="armStop${s}" onclick="teleopArmStop(${s})" title="Freeze here — keeps torque">Stop</button>
        <button class="btn-idle" id="armIdle${s}" onclick="idleActuator(${s})" title="Zero torque on THIS joint only">Idle</button>
      </span>
      <span class="cmd" id="armCmd${s}">—</span>
    </div>
    <div class="jc-limits">
      <span class="fb" id="armFb${s}">fb —</span>
      <button type="button" onclick="markLimit(${s}, 'lo')" title="Capture current encoder FB as minimum">Mark lo</button>
      <button type="button" onclick="markLimit(${s}, 'hi')" title="Capture current encoder FB as maximum">Mark hi</button>
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
        <button class="btn-stop" id="baseStop${s}" onclick="teleopBaseStop(${s})" title="Freeze here">Stop</button>
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

// Keyboard teleop — requires Active + soft_kill released; ignored while focus is in form fields.
let keysMotionAllowed = false;
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
    if (prev) { prev.classList.remove("selected"); prev.setAttribute("aria-pressed", "false"); }
  }
  keysArmSlot = slot;
  if (keysArmSlot != null) {
    const cur = document.getElementById(`armRow${keysArmSlot}`);
    if (cur) { cur.classList.add("selected"); cur.setAttribute("aria-pressed", "true"); }
  }
  _keysUpdateArmLabel();
}

window.addEventListener("keydown", (ev) => {
  if (_keysTypingTarget(ev.target)) return;
  if (!keysMotionAllowed) return;
  const k = ev.key;
  if (k === " ") {
    ev.preventDefault();
    teleopStopAll();
    return;
  }
  if (k === "ArrowUp") { ev.preventDefault(); _keysNudgeNeck(0, 1); return; }
  if (k === "ArrowDown") { ev.preventDefault(); _keysNudgeNeck(0, -1); return; }
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
    const active = !!s.connected;
    const softKill = !!s.soft_kill;
    const motionAllowed = active && !softKill;
    keysMotionAllowed = motionAllowed;

    let grade = s.grade || "idle";
    const gradeCls = active && softKill ? "frozen" : grade;
    g.textContent = active && softKill ? `${grade} · frozen` : grade;
    g.className = "grade " + gradeCls;
    document.getElementById("healthCard").dataset.grade = gradeCls;
    // Mirror the grade onto the rail's Health entry too — the one status
    // readable without switching off whatever panel is currently open.
    const healthRailItem = document.querySelector('.rail-item[data-panel="health"]');
    if (healthRailItem) healthRailItem.dataset.grade = gradeCls;
    document.getElementById("summary").textContent = s.summary || "—";
    document.getElementById("meta").textContent = s.connected
      ? `${s.port || "?"} · ${softKill ? "frozen" : "released"}`
      : (s.following_state_file ? `follow` : `follow (idle)`);
    const ctx = document.getElementById("context");
    ctx.innerHTML = (s.context || []).map(c => `<li>${c}</li>`).join("");

    // The 3 numbers that answer "is the board healthy" without reading
    // closely: safety kill-state, whether motion is actually live right
    // now, and link telemetry rate. Everything else is supporting detail.
    //
    // kill_state priority: the dashboard's own operator-driven states
    // (manual e-stop park, soft_kill freeze) take precedence over the raw
    // PDB mirror, because they're what's *actually* blocking motion right
    // now (see state.py's Follow/Active x soft_kill machine) — the PDB
    // field alone can't tell them apart from a genuine hardware fault.
    // stale_failsafe (no PDB ever synced / gone stale) is the MCU's
    // documented fail-safe echo (pdb_link.h PDB_STALE_FAILSAFE=1), not a
    // real estop — show it as neutral "no_pdb" instead of red hard_estop.
    const pdb = s.pdb_status;
    const manualEstop = s.mcu_state === 3; // PLANT_MCU_STATE_ESTOP (operator park request)
    const softKilled = !!s.soft_kill;
    let killState;
    let killCls;
    if (manualEstop) {
      killState = "manual_estop";
      killCls = "warn";
    } else if (softKilled) {
      killState = "soft_killed";
      killCls = "hold";
    } else if (pdb && pdb.stale_failsafe) {
      killState = "no_pdb";
      killCls = "neutral";
    } else if (pdb) {
      killState = pdb.kill_state_name;
      killCls = killState === "hard_estop" ? "crit" : killState === "normal" ? "ok" : "warn";
    } else {
      killState = null;
      killCls = "neutral";
    }
    const blockName = s.plant_block_name || (s.plant_block != null ? String(s.plant_block) : null);
    const blockLive = blockName === "none";
    const blockCls = blockName == null ? "neutral" : (blockLive ? "warn" : "hold");
    // fb_hz is an informational rate, not a severity signal — no color tint,
    // so it doesn't compete with the two tiles that actually mean "safe" vs "fault".
    const fbCls = s.fb_hz != null ? "" : "neutral";
    document.getElementById("healthHero").innerHTML = [
      statTile("kill_state", killState, killCls),
      statTile("plant_block", s.plant_block_name || s.plant_block, blockCls),
      statTile("fb_hz", s.fb_hz != null ? s.fb_hz.toFixed(1) : null, fbCls),
    ].join("");

    document.getElementById("healthGlance").innerHTML = [
      kv("mcu_state", s.mcu_state),
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
      kv("configured_hz", s.connect_stream_hz != null ? `${s.connect_stream_hz} Hz` : null),
      kv("host_tx_hz", s.stream_tx_hz != null ? s.stream_tx_hz.toFixed(1) : null),
      kv("host_tx_gap_p95_ms", s.stream_tx_gap_p95_ms != null ? s.stream_tx_gap_p95_ms.toFixed(1) : null),
      kv("host_tx_gap_max_ms", s.stream_tx_gap_max_ms != null ? s.stream_tx_gap_max_ms.toFixed(1) : null),
      kv("host_loop_ms", s.stream_loop_ms != null ? s.stream_loop_ms.toFixed(1) : null),
      kv("svd", s.svd_present ? "yes" : "no"),
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
      ? `${(s.recording_bytes/1024).toFixed(1)} KB · ${(s.recording_seconds||0).toFixed(0)}s`
      : "";
    const clearFaultsBtn = document.getElementById("clearFaultsBtn");
    clearFaultsBtn.disabled = !!s.following_state_file;

    // PDU / kill-state telemetry (kill_state itself is promoted into healthHero above)
    const hostRequested = s.mcu_state === 3;
    document.getElementById("pduGlance").innerHTML = pdb ? [
      kv("kill_reason", pdb.kill_reason_name),
      kv("stale_failsafe", pdb.stale_failsafe ? "yes" : "no"),
      kv("host ESTOP req", hostRequested ? "yes" : "no"),
      kv("estop_sense", pdb.estop_sense === 1 ? "1 (allowed)" : "0 (asserted)"),
    ].join("") : `<div class="kv"><span class="k">status</span><div class="v">no PDB frame yet</div></div>`;
    document.getElementById("pduAdvanced").innerHTML = pdb ? [
      kv("peer kill_state", pdb.pdb ? PEER_KILL_STATE_NAMES[pdb.pdb.kill_state] ?? pdb.pdb.kill_state : "—"),
      kv("contactor_state", pdb.pdb ? pdb.pdb.contactor_state : "—"),
      kv("pack_v (V) x4", fmtVec(pdb.pack_v_V)),
      kv("rail_v (V) x4", fmtVec(pdb.rail_v_V)),
      kv("pack_i (A) x4", fmtVec(pdb.pack_i_A)),
      kv("rail_i (A) x4", fmtVec(pdb.rail_i_A)),
    ].join("") : "";

    // Connection + take-control header
    document.getElementById("connectBtn").disabled = active;
    document.getElementById("disconnectBtn").disabled = !active;
    document.getElementById("releaseBtn").disabled = !active || !softKill;
    document.getElementById("engageBtn").disabled = !active || softKill;
    document.getElementById("releaseBtn").classList.toggle("active", motionAllowed);
    document.getElementById("engageBtn").classList.toggle("active", active && softKill);
    document.getElementById("portSelect").disabled = active;
    const connectHzEl = document.getElementById("connectHz");
    connectHzEl.disabled = active;
    if (s.connect_stream_hz) connectHzEl.placeholder = String(s.connect_stream_hz);
    const banner = document.getElementById("modeBanner");
    if (!active && !s.following_state_file) {
      banner.className = "banner ok";
      banner.textContent = "Follow — not owning COM. Connect enters Active frozen (soft_kill ON) until you Take control.";
    } else if (!active && s.following_state_file) {
      banner.className = "banner ok";
      banner.textContent = "Follow — reading state.json only. Hard ESTOP signals the peer.";
    } else if (softKill) {
      banner.className = "banner hold";
      banner.textContent = "Active — frozen. Holding whatever is currently held, at full torque. Take control to command new motion.";
    } else {
      banner.className = "banner warn";
      banner.textContent = "Active — control taken. Motion commands are accepted.";
    }
    lastListenPdu = !!s.listen_pdu;
    const listenBtn = document.getElementById("listenPduBtn");
    listenBtn.disabled = !active;
    listenBtn.textContent = `listen_pdu: ${lastListenPdu ? "ON" : "OFF"}`;
    listenBtn.className = lastListenPdu ? "btn-go" : "btn-ghost";

    for (const id of ["mcuNormal", "mcuRecovery", "mcuApplyOff", "mcuApplyOn", "recoverBtn", "idleAllBtn"]) {
      document.getElementById(id).disabled = !active;
    }
    document.getElementById("mcuEstop").disabled = false;

    document.getElementById("bandwidthRunBtn").disabled = active;

    const streamBadge = document.getElementById("streamingBadge");
    streamBadge.classList.toggle("on", !!s.streaming);
    const activeSlots = [];
    (s.held || []).forEach((h, i) => { if (h && h.active) activeSlots.push(i); });
    streamBadge.textContent = s.streaming ? `streaming: ON (${activeSlots.length} active)` : "streaming: OFF";
    document.getElementById("plantControlSummaryMeta").textContent =
      `mcu=${s.mcu_state ?? "—"} · streaming=${s.streaming ? "ON" : "OFF"}${activeSlots.length ? ` · ${activeSlots.length} active` : ""}`;

    // Live FB overlay used by both the Advanced raw table and the CFG card.
    const acts = s.actuators || [];
    lastActuatorsFb = acts;
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
        if (applyBtn) applyBtn.disabled = !motionAllowed;
        if (idleBtn) idleBtn.disabled = !active;
      });
    }
    // Refresh the CFG card's live overlay (conn/pos/vel/tau/temp) without re-sampling CFG itself.
    if (lastCfg) renderCfgTable();

    if (!teleopRowsBuilt && teleopGroups) buildTeleopRows();
    for (const id of ["armLeftRows", "armRightRows", "baseRows", "neckRows"]) {
      document.querySelectorAll(`#${id} button`).forEach(b => {
        const isFreezeOrIdle = /Stop|Idle/.test(b.textContent) || b.id.includes("Stop") || b.id.includes("Idle");
        b.disabled = isFreezeOrIdle ? !active : !motionAllowed;
      });
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
            ? `→ ${fmt(st.target, 3)} @ ${fmt(st.cruise, 2)} rad/s${st.flagged ? " · ⚠ tracking error" : ""}`
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
          baseCmd.textContent = st ? `→ ${fmt(st.target, 2)}${st.flagged ? " · ⚠" : ""}` : "";
          baseCmd.classList.toggle("flagged", !!(st && st.flagged));
        }
      });
      Object.keys(teleopGroups.servos).forEach(slotStr => {
        const slot = Number(slotStr);
        const st = teleopSrv[slot] ?? teleopSrv[slotStr];
        const neckCmd = document.getElementById(`neckCmd${slot}`);
        if (neckCmd) neckCmd.textContent = st ? (st.seeded ? `→ ${fmt(st.target, 0)} (now ${fmt(st.pos, 0)})` : "seeding…") : "idle";
      });
    }

    if (Object.keys(lastPanels).length) renderPanels();

    const cl = s.continuous_launch || { state: "unknown" };
    document.getElementById("continuousStatus").textContent =
      cl.state + (cl.detail ? `: ${typeof cl.detail === "string" ? cl.detail : JSON.stringify(cl.detail)}` : "");
    document.getElementById("continuousLaunchBtn").disabled = active || cl.state === "launching";
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
// -- Section nav (left rail) -----------------------------------------------------------
// Replaces the old 2-column layout (all 7 sections' content in the DOM at
// once — a very tall primary column next to an under-filled 2x2 grid of
// collapsed dropdowns). Now exactly one .panel is visible at a time; the
// rail is just a destination list, not content, so switching it never
// changes what data any panel shows — every panel keeps polling/populating
// in the background via the ids above regardless of visibility.
function initSectionNav() {
  const rail = document.getElementById("sectionRail");
  const panels = Array.from(document.querySelectorAll("#sectionPanels > .panel"));
  if (!rail || !panels.length) return;

  function activate(name) {
    panels.forEach(p => { p.hidden = p.dataset.panel !== name; });
    rail.querySelectorAll(".rail-item").forEach(b => {
      b.classList.toggle("active", b.dataset.panel === name);
    });
    try { localStorage.setItem("deft_active_panel", name); } catch (e) { /* ignore */ }
  }

  rail.addEventListener("click", (ev) => {
    const btn = ev.target.closest(".rail-item");
    if (!btn) return;
    activate(btn.dataset.panel);
  });

  let initial = "health";
  try {
    const saved = localStorage.getItem("deft_active_panel");
    if (saved && rail.querySelector(`.rail-item[data-panel="${saved}"]`)) initial = saved;
  } catch (e) { /* ignore */ }
  activate(initial);
}

initSectionNav();
loadPorts();
loadTeleopGroups();
loadPanels();
setInterval(tick, 200);
tick();
