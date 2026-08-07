"""HTML markup for the dashboard index page.

Layout (revised 2026-08-06, replacing the earlier 2-column
main-column + 2x2-collapsible-grid shape — that put all 7 sections'
content in the DOM open at once, so the page grew very tall and
left-heavy while the right-hand grid sat mostly empty at rest): a
left nav rail lists all 7 sections (Health -> Connected/CFG -> Testing
-> Controls -> Advanced -> Panels catalog -> Continuous mode, same
priority order as before); only the selected section's panel is
visible at a time, sized to whatever width the page actually has. The
"take control" (soft_kill release/engage) toggle and the live grade
pill stay in the sticky header so they're visible regardless of which
panel is open or how far it's scrolled — see app.js's ``initSectionNav``
for the show/hide + persisted-selection logic.

CSS/JS live in ``static/style.css`` / ``static/app.js`` (served by
:mod:`routes` under ``/static/``) — this module only holds the page
skeleton.
"""
from __future__ import annotations

INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Deft controls — debug</title>
<link rel="stylesheet" href="/static/style.css"/>
</head>
<body>
<header>
  <h1>Deft controls</h1>
  <span id="grade" class="grade idle">idle</span>
  <select id="portSelect" title="Serial port"></select>
  <span class="hz-field" title="Plant TX rate for this session. Pre-connect only — takes effect on the next Connect, not live. Leave blank to keep the current rate.">
    <input type="number" id="connectHz" placeholder="100" min="1" max="1000" step="1" style="width:4rem">
    <span class="hz-unit">Hz</span>
  </span>
  <button class="primary" id="connectBtn" onclick="connect()"
    title="Opens COM in mode=debug and enters Active with soft_kill ON (frozen/safe) by default.">Connect</button>
  <button class="btn-ghost" id="disconnectBtn" onclick="disconnect()" disabled>Disconnect</button>
  <span class="spacer"></span>
  <span class="segmented take-control">
    <button class="btn-release" id="releaseBtn" onclick="releaseSoftKill()" disabled
      title="Clears the freeze — motion commands (Apply / teleop target / jog) are accepted again.">Take control</button>
    <button class="btn-freeze" id="engageBtn" onclick="engageSoftKill()" disabled
      title="Freeze now — holds whatever is currently held, at full torque (not blanked). Stays NORMAL + plant_apply=1.">Freeze</button>
  </span>
  <span class="streaming-badge" id="streamingBadge">streaming: —</span>
</header>
<main>
  <p class="meta" id="meta">not connected</p>
  <p class="banner ok" id="modeBanner"></p>
  <p class="err" id="connError"></p>

  <div class="shell">
  <nav class="rail" id="sectionRail">
    <button class="rail-item" data-panel="health">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Health</span><span class="rail-sub">board status</span></span>
    </button>
    <button class="rail-item" data-panel="cfg">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Connected — CFG</span><span class="rail-sub">what's on the PCB</span></span>
    </button>
    <button class="rail-item" data-panel="testing">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Testing</span><span class="rail-sub">inventory · discover · calibrate</span></span>
    </button>
    <button class="rail-item" data-panel="controls">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Controls</span><span class="rail-sub">LED · teleop</span></span>
    </button>
    <button class="rail-item" data-panel="advanced">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Advanced</span><span class="rail-sub">raw apply · hard ESTOP</span></span>
    </button>
    <button class="rail-item" data-panel="panels">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Panel catalog</span><span class="rail-sub">dev reference</span></span>
    </button>
    <button class="rail-item" data-panel="continuous">
      <span class="rail-dot"></span>
      <span class="rail-text"><span class="rail-label">Continuous mode</span><span class="rail-sub">Jetson SSH launch</span></span>
    </button>
  </nav>
  <div class="panels" id="sectionPanels">

  <!-- ============ 1. HEALTH ============ -->
  <section class="card panel" data-panel="health" id="healthCard">
    <h2>Health</h2>
    <p class="summary" id="summary">—</p>
    <ul class="context" id="context"></ul>
    <div class="stat-hero-row" id="healthHero"></div>
    <div class="grid compact" id="healthGlance" style="margin-top:0.5rem"></div>
    <div class="grid compact" id="pduGlance" style="margin-top:0.6rem"></div>
    <p class="err" id="pdbError"></p>
    <div class="row" style="margin-top:0.6rem">
      <button id="listenPduBtn" onclick="toggleListenPdu()" disabled
        title="Host obeys PDB kill-state for auto soft_kill freeze + LED policy only while this is ON.">listen_pdu: OFF</button>
      <button class="record" id="recordBtn" onclick="toggleRecord()">&#9679; Record</button>
      <span class="meta" id="recordMeta"></span>
      <span class="spacer"></span>
      <input type="text" id="bandwidthPort" placeholder="COM5" title="Port for bandwidth probe (its own mode=bandwidth connection)" style="width:6rem">
      <input type="number" id="bandwidthSeconds" value="2" step="0.5" min="0.5" style="width:4.5rem">
      <button id="bandwidthRunBtn" onclick="runBandwidthTest()"
        title="Opens its own short-lived mode=bandwidth connection. Windows CDC is exclusive-open — Follow/disconnected only.">Run bandwidth test</button>
    </div>
    <p class="meta" id="bandwidthResult"></p>
    <details class="sub">
      <summary>Fault log</summary>
      <div class="grid compact" id="blackbox" style="margin-top:0.5rem"></div>
      <div class="row" style="margin-top:0.5rem">
        <button class="btn-idle" id="clearFaultsBtn" onclick="clearFaultLog()">Clear fault log</button>
        <span class="meta" id="clearFaultsMeta"></span>
      </div>
    </details>
    <details class="sub">
      <summary>Advanced timing + raw PDB telemetry</summary>
      <p class="grid-label">Host / MCU timing</p>
      <div class="grid compact" id="healthAdvanced"></div>
      <p class="grid-label">Raw PDB telemetry</p>
      <div class="grid compact" id="pduAdvanced"></div>
    </details>
  </section>

  <!-- ============ 2. CONNECTED / CFG ============ -->
  <section class="card panel" data-panel="cfg" id="cfgCard">
    <h2>Connected — live CFG</h2>
    <div class="row">
      <button class="primary" id="cfgSampleBtn" onclick="sampleCfg()"
        title="Reads the CFG table + peripherals from the board right now. Not polled — click to re-sample.">Sample CFG</button>
      <span class="meta" id="cfgSampleMeta">not sampled yet</span>
    </div>
    <p class="err" id="cfgError"></p>
    <div class="table-surface">
      <table>
        <thead>
          <tr>
            <th>slot</th><th>en</th><th>bus</th><th>rail</th><th>protocol</th><th>motor id</th>
            <th>conn</th><th>pos</th><th>vel</th><th>τ</th><th>temp</th><th></th>
          </tr>
        </thead>
        <tbody id="cfgSlotRows"><tr><td colspan="12" class="meta">Sample CFG to populate.</td></tr></tbody>
      </table>
    </div>
    <details class="sub" open id="cfgEditDetails">
      <summary>Edit one slot</summary>
      <div class="row" style="margin-top:0.5rem">
        <input type="number" id="cfgEditSlot" placeholder="slot" min="0" max="25" style="width:4rem">
        <input type="number" id="cfgEditBus" placeholder="bus" min="1" max="6" style="width:3.5rem">
        <input type="number" id="cfgEditProtocol" placeholder="protocol" style="width:5rem">
        <input type="text" id="cfgEditMotorId" placeholder="motor_id (0xNN or dec)" style="width:8rem">
        <input type="number" id="cfgEditMasterId" placeholder="master_id" value="0" style="width:5rem">
        <label class="meta"><input type="checkbox" id="cfgEditEnabled" checked> enabled</label>
        <label class="meta"><input type="checkbox" id="cfgEditPersist"> persist to NVM</label>
        <button class="btn-go" onclick="applyCfgSlotEdit()">Apply slot</button>
      </div>
    </details>
    <details class="sub">
      <summary>Servos + LED + listen_pdu (NVM periph)</summary>
      <div id="cfgPeriph" style="margin-top:0.5rem"></div>
    </details>
    <details class="sub">
      <summary>Replace whole table</summary>
      <div class="row" style="margin-top:0.5rem">
        <button onclick="copyCfgTableToEditor()" title="Dump the currently-sampled enabled slots as JSON, ready to tweak">Copy sampled table to editor</button>
        <label class="meta"><input type="checkbox" id="cfgTableDisableMissing" checked> disable slots not in the table (real replace)</label>
        <label class="meta"><input type="checkbox" id="cfgTablePersist"> persist to NVM</label>
        <button class="btn-go" onclick="applyCfgTableFromEditor()">Replace whole table from editor</button>
      </div>
      <textarea id="cfgTableJson" rows="8" style="width:100%;margin-top:0.5rem"
        placeholder='[{"slot":22,"bus":5,"protocol":0,"motor_id":112,"enabled":true}, ...]'></textarea>
      <button class="btn-go" style="margin-top:0.5rem" onclick="saveCfgNvm()">Save current RAM CFG to NVM</button>
    </details>
  </section>

  <!-- ============ 3. TESTING ============ -->
  <section class="card panel" data-panel="testing" id="testingCard">
    <h2>Testing</h2>
    <p class="section-intro">One-click diagnostic routines run against the board right now. Each run appends its
      result — pass, fail, or raw data — to the log at the bottom, ready to screenshot or paste into a bug report.</p>

    <div class="test-groups">
      <div class="test-group">
        <div class="test-group-head">
          <span class="test-group-title">Inventory</span>
          <span class="test-group-blurb">Enumerate live CFG-enabled actuators/servos on this bench.</span>
        </div>
        <div class="row">
          <label class="field-label">preset
            <select id="inventoryPreset">
              <option value="bench">bench</option>
              <option value="product">product</option>
            </select>
          </label>
          <button class="primary" onclick="runInventory()">Run inventory</button>
        </div>
      </div>

      <div class="test-group">
        <div class="test-group-head">
          <span class="test-group-title">Discover</span>
          <span class="test-group-blurb">Sweep CAN buses for live drives (RobStride/Damiao/Cubemars/ZeroErr).</span>
        </div>
        <div class="row">
          <label class="field-label">buses
            <input type="text" id="discoverBuses" placeholder="e.g. 1,5" value="1" style="width:6rem">
          </label>
          <label class="field-label">protocols
            <input type="text" id="discoverProtocols" placeholder="robstride,damiao,…" value="robstride" style="width:9rem">
          </label>
          <button onclick="runDiscover()">Discover</button>
        </div>
      </div>

      <div class="test-group">
        <div class="test-group-head">
          <span class="test-group-title">Calibrate <span class="meta">(RobStride only)</span></span>
          <span class="test-group-blurb">Zero/offset calibration for one live RobStride slot — shaft must spin freely; ~15–35s, blocking.</span>
        </div>
        <div class="row">
          <label class="field-label">bus
            <input type="number" id="calibrateBus" placeholder="bus" min="1" max="6" style="width:3.5rem">
          </label>
          <label class="field-label">motor_id
            <input type="number" id="calibrateMotorId" placeholder="motor_id" style="width:5.5rem">
          </label>
          <label class="field-label">listen s
            <input type="number" id="calibrateListenS" placeholder="listen s" value="28" min="10" max="60" style="width:5rem">
          </label>
          <button onclick="runCalibrate()"
            title="RobStride only. Shaft must spin freely. ~15-35s blocking.">Calibrate</button>
        </div>
      </div>
    </div>

    <div class="row" style="margin-top:0.8rem">
      <span class="meta">Results append below, most recent last.</span>
      <span class="spacer"></span>
      <button class="btn-ghost" onclick="clearTerminal()">Clear output</button>
    </div>
    <div class="terminal-surface">
      <div class="terminal-head">Output log</div>
      <div class="terminal" id="testingTerminal"><div class="term-empty">ready.</div></div>
    </div>
  </section>

  <!-- ============ 4. CONTROLS ============ -->
  <section class="card panel" data-panel="controls">
    <div class="panel-head"><h2>Controls</h2> <span class="meta">LED · teleop (arm / base / neck)</span></div>

      <div class="row" style="margin-top:0.6rem">
        <select id="ledMode">
          <option value="follow">follow</option>
          <option value="pdu">pdu</option>
          <option value="debug">debug</option>
        </select>
        <input type="number" id="ledBrightness" value="8" min="0" max="255" style="width:4.5rem" title="brightness">
        <input type="number" id="ledPattern" value="0" min="0" max="8" style="width:3.5rem" title="pattern (debug mode only)">
        <button onclick="setLedMode()">Set LED</button>
        <span class="spacer"></span>
        <select id="ledPreset">
          <option value="idle">idle</option>
          <option value="follow">follow</option>
          <option value="pdu">pdu</option>
          <option value="gen_2">gen_2</option>
        </select>
        <button onclick="applyLedPreset()">Apply preset</button>
      </div>
      <p class="err" id="ledError"></p>

      <div class="row toolbar" style="margin-top:0.75rem">
        <label class="meta">Slot labels
          <select id="cfgMapSelect" onchange="setCfgMap()"
            title="UI-only relabeling of which slots this Teleop panel shows as 'base' vs 'arm' — display only.">
            <option value="bench">bench (22–25)</option>
            <option value="product">product (14–19)</option>
          </select>
        </label>
        <button class="btn-stop" onclick="teleopStopAll()" title="Freeze all teleop slots">&#9616;&#9616; Stop all (Space)</button>
        <span class="spacer"></span>
        <label class="meta" title="Widen L-arm rails to MuJoCo soft limits for manual limit finding.">
          <input type="checkbox" id="limitScout" onchange="setLimitScout()"> Scout limits
        </label>
        <span class="meta">selected: <b id="keysArmSel">—</b></span>
      </div>
      <div class="row">
        <button class="btn-idle" onclick="idleGroup('arm_left')">Idle L-arm</button>
        <button class="btn-idle" onclick="idleGroup('arm_right')">Idle R-arm</button>
        <button class="btn-idle" onclick="idleGroup('base')">Idle base</button>
        <button class="btn-idle" onclick="idleGroup('neck')">Idle neck</button>
        <button onclick="copyCapturedLimits()" title="Copy marked lo/hi as Python tuples">Copy marked limits</button>
        <button onclick="clearCapturedLimits()">Clear marks</button>
      </div>
      <pre class="meta" id="capturedLimitsOut" style="margin-top:0.4rem;white-space:pre-wrap">Marked limits: none yet.</pre>
      <p class="err" id="teleopError"></p>

      <details class="teleop-group" open>
        <summary><span class="group-title">Arm — left</span></summary>
        <div class="joint-grid" id="armLeftRows"></div>
      </details>
      <details class="teleop-group">
        <summary><span class="group-title">Arm — right</span></summary>
        <div class="joint-grid" id="armRightRows"></div>
      </details>
      <details class="teleop-group" open>
        <summary><span class="group-title">Base actuators</span></summary>
        <div class="joint-grid" id="baseRows"></div>
      </details>
      <details class="teleop-group" open>
        <summary><span class="group-title">Neck</span></summary>
        <div class="joint-grid" id="neckRows"></div>
      </details>
  </section>

  <!-- ============ 5. ADVANCED ============ -->
  <section class="card panel" data-panel="advanced">
    <div class="panel-head"><h2>Advanced: raw per-slot Apply + hard ESTOP</h2> <span class="meta" id="plantControlSummaryMeta"></span></div>
      <div class="row" style="margin-top:0.75rem">
        <button id="mcuNormal" onclick="setMcuState(0)">NORMAL</button>
        <button id="mcuRecovery" onclick="setMcuState(1)">RECOVERY</button>
        <button id="mcuApplyOff" onclick="setPlantApply(false)">APPLY_OFF</button>
        <button id="mcuApplyOn" onclick="setPlantApply(true)">APPLY_ON</button>
        <button id="mcuEstop" class="estop" onclick="hardEstopPark()"
          title="Blanks every held desire and latches McuState.ESTOP. Different from soft_kill (freeze). Works in Follow mode too.">Hard ESTOP (park)</button>
        <button id="recoverBtn" onclick="recover()">Recover</button>
        <button id="idleAllBtn" onclick="idleAll()">Idle all slots</button>
      </div>
      <p class="err" id="ctrlError"></p>
      <p class="meta" id="pdbResult"></p>
      <div class="table-surface">
        <table id="actuatorsTable" class="col-grouped">
          <thead>
            <tr>
              <th>slot</th><th colspan="5" class="grp-fb">feedback</th>
              <th colspan="4" class="grp-held">held</th><th colspan="4" class="grp-apply">apply</th>
            </tr>
            <tr>
              <th></th><th>pos</th><th>vel</th><th>τ</th><th>temp</th><th>fault</th>
              <th>state</th><th>pos</th><th>kp</th><th>kd</th>
              <th>set pos</th><th>kp</th><th>kd</th><th></th>
            </tr>
          </thead>
          <tbody id="acts"></tbody>
        </table>
      </div>
  </section>

  <!-- ============ 6. Panels catalog (dev-facing) ============ -->
  <section class="card panel" data-panel="panels">
    <h2>Panel action catalog</h2>
    <p class="meta">Every wired debug action, its import path and call signature — groundwork for a future "copy as script" button.</p>
    <div id="panelsList"></div>
  </section>

  <!-- ============ 7. Continuous mode ============ -->
  <section class="card panel" data-panel="continuous">
    <h2>Continuous mode</h2>
    <div class="row">
      <button id="continuousLaunchBtn" onclick="continuousLaunch()">Launch continuous</button>
      <label class="meta">duration s (0 = until stopped) <input type="number" id="continuousDuration" value="0" step="1" style="width:5rem"></label>
      <button class="btn-danger" id="continuousStopBtn" onclick="continuousStop()">Stop continuous (hard)</button>
      <span class="meta" id="continuousStatus"></span>
    </div>
    <p class="banner" id="continuousBanner">Launches yam_continuous_all.py on the Jetson over SSH — CDC then belongs to that process. Disconnect COM here first.</p>
  </section>

  </div>
  </div>
</main>
<script src="/static/app.js"></script>
</body>
</html>
"""

__all__ = ["INDEX_HTML"]
