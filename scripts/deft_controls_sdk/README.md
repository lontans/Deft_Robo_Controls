# deft_controls_sdk — host SDK for the Deft Robotics controls PCB (USB CDC)

Self-contained package: no imports from `scripts/legacy/` (`controls_pcb_host`,
`controls_hub_controller`, `control_hub`, etc.). Wire helpers and `Connection`
live under `link/`; DEBUG-mode bench ops (discover/config) live under `bench/`.

## Entrypoints

**Software — PLANT mode**

```python
from deft_controls_sdk import ControlsPcbHub, ActuatorDesire, McuState

with ControlsPcbHub.connect("COM5") as hub:
    hub.start_streaming()
    print(hub.telemetry.snapshot())
    print(hub.state_path)  # ./.deft_session/state.json
```

**Software — DEBUG mode** (discover / config, under a bench lease on the same
Connection — not a second serial port):

```python
with ControlsPcbHub.connect("COM5") as hub:
    with hub.debug.lease(bus=2):
        hit = hub.debug.discover_robstride(bus=2)
    table = hub.debug.cfg_get_table()
    hub.debug.cfg_set_slot(slot=1, bus=2, protocol=1, motor_id=hit, persist=False)
```

`hub.debug.calibrate_robstride` is **not implemented yet** — see its docstring
in `bench/__init__.py` for why (timing-sensitive, spins a shaft, depends on
four more legacy modules); use `scripts/legacy/control_hub.py calibrate` today.

**Software — black box** (fault-triggered dumps happen automatically once
connected; manual recording is opt-in):

```python
with ControlsPcbHub.connect("COM5") as hub:
    hub.start_streaming()
    hub.telemetry.start_recording()   # every tick -> .deft_session/recordings/
    ...
    hub.telemetry.stop_recording()
    print(hub.telemetry.snapshot().last_fault_path)  # most recent auto-captured fault, if any
```

**Human (localhost controller + telemetry dashboard)**

```powershell
cd scripts
python -m deft_controls_sdk.debug_dashboard
# open http://127.0.0.1:8765, pick a port, click Connect

python -m deft_controls_sdk.debug_dashboard --port COM5   # old one-shot workflow still works
```

One page, one process, one COM owner. Sections: **Connection** (port dropdown,
Connect/Disconnect — opens/closes a `ControlsPcbHub` live, no restart needed
on a board reset), **Tier-1 health** + **Black box** (as before), **Plant
control** — per-slot position/kp/kd hold with Apply/Idle, `mcu_state` buttons
(NORMAL/RECOVERY/DIAG_ONLY/ESTOP — ESTOP is never disabled by connection
state), and Recover. Raw MIT-hold commanding only, same as the rest of the
SDK — no ramping/homing/teleop policy lives here. DEBUG-mode ops
(discover/cfg) are not exposed in the UI yet, `hub.debug.*` only.

One `TelemetryCache` persists for the whole process, independent of any one
connection — fault history and the ring buffer survive a Disconnect/Connect
cycle (e.g. after a board power-cycle) instead of resetting each time.

## Layout

```
controls_pcb_hub.py    # façade (what apps import) — plant / debug / telemetry
link/
  exchange/
    wire_layout.py      # constants / offsets (renamed from schema.py — "schema"
                         # already means something else in telemetry/cache.py)
    transport.py        # serial + frame reader
    pack.py              # build command image bytes (plant path, pdu=0)
    parse.py              # parse feedback image bytes (plant path)
    bench.py              # tagged-pdu (DEBUG) pack/parse: RS2/DM0/CFG + CAN bus helpers
  api_types.py         # ActuatorDesire, FeedbackImage, … (renamed from types.py)
  connection.py        # Connection — socket + held desires + streaming/telemetry
                         # publish (formerly a separate LinkSession; merged —
                         # see connection.py's module docstring for why)
bench/                  # DEBUG mode: discover / config, under Connection.exchange_raw
  lease.py              # RS2 SESSION_BEGIN/END bracket (hub.debug.lease())
  robstride.py           # RS02 discover + probe (calibrate NOT ported — see above)
  damiao.py              # Damiao DM0 discover (known-IDs-first order preserved)
  config.py             # CFG PDU: actuator table get/set/save (NVM)
telemetry/              # shared cache (scripts + dashboard); mode reflects
                         # idle | plant_stream | debug_lease | discover | cfg
  cache.py              # latest-snapshot + grading (state.json enqueue — never blocks)
  persist.py            # SessionDiskWriter: background state/NDJSON/fault I/O
  recorder.py           # FaultRecorder: ring buffer + fault-triggered dumps + manual record
debug_dashboard/        # localhost controller + telemetry UI — the one COM
                         # owner while connected (AppState opens/closes the hub)
  app.py                # AppState + HTTP routes + the page itself
  __main__.py           # python -m deft_controls_sdk.debug_dashboard [--port COM5]
```

## Dashboard HTTP routes

| Route | Method | Body | Notes |
|-------|--------|------|-------|
| `/` | GET | — | the page |
| `/api/state` | GET | — | full `SessionState` snapshot; always well-formed, even disconnected |
| `/api/ports` | GET | — | `list_ports_info()` — device/description/vid/pid/is_stm32_cdc |
| `/api/connect` | POST | `{port}` | 400 if already connected or the port can't be opened |
| `/api/disconnect` | POST | — | no-op (200) if not connected |
| `/api/actuator/<slot>` | POST | `{position, kp, kd}` | 400 if not connected |
| `/api/actuator/<slot>/idle` | POST | — | zero desire for that slot |
| `/api/mcu_state` | POST | `{state}` (0-3) | |
| `/api/recover` | POST | — | |
| `/api/record/start`, `/api/record/stop` | POST | — | manual black-box recording |

Every action route returns the current `/api/state` snapshot on success, or `{"error": "..."}` with a 400 on failure — the connect flow especially can fail (wrong port, device busy) and must not 500.

## Telemetry permanence

| Artifact | Behavior |
|----------|----------|
| In-memory `TelemetryCache` | Latest snapshot; `hub.telemetry.snapshot()` |
| `state.json` | Latest only; atomic tmp→replace under `.deft_session/` — never grows. Written on a **background thread** at ~10 Hz (coalesced); the plant stream loop never blocks on disk. Dashboard `/api/state` reads RAM and does not need this file. |
| Fault-triggered dumps | `.deft_session/faults/fault_<time>_<n>_<reason>.ndjson` — automatic on a `plant_block`/actuator-fault/`ESTOP` transition, bounded ring window before + fixed tick count after; oldest pruned past `max_fault_files` (default 20). A `startup_grace_s` window (default 3.0s from first observed tick) suppresses triggers during connect — the first feedback frame can legitimately read `HOST_STALE` with `fb_hz` still ramping from cold before streaming stabilizes; without the grace window every connect fired a spurious dump (confirmed on a real bench capture: `plant_block=HOST_STALE` on tick 0, cleared 264 ms later, steady by ~1.25s). Dumps are written on the same background disk thread. |
| Manual recording | `.deft_session/recordings/record_<time>.ndjson` — `hub.telemetry.start_recording()`/`stop_recording()`, or the Record button in the dashboard; every observed tick while on, unbounded until stopped (size/duration shown live in `state.json` and the dashboard so it's never silent). Appends are queued to the background writer (no per-tick `flush()` on the stream thread). |

Works against currently flashed firmware (562 B `CMDH`/`HBHF`). No firmware changes — this is host-side only.

## Porting status vs `scripts/legacy/`

Ported and tested (no-hardware golden tests in `scripts/tests/test_deft_controls_sdk_bench.py`):
`hub.debug.lease()`, RobStride discover/probe, Damiao discover, CFG get/set/save
(RAM apply and flash persist reported as distinct outcomes — flash SAVE is
unreliable in practice, not just in theory; see `bench/config.py`).

**Not ported** — do not assume this SDK replaces these yet:
- RS02 encoder calibrate (`hub.debug.calibrate_robstride` raises `NotImplementedError`)
- Damiao register scan / discover *without* a caller-supplied `known_ids` list
  (the SDK has no actuator-config table of its own — pass known IDs explicitly)
- Teleop (arrow keys, YAM limits, homing/brace) — app-layer policy, deliberately
  out of scope for this SDK; see `docs/plan-host-api-streamline.md` P3

All of the above must be verified against real hardware before
`scripts/legacy/` is deleted — the golden tests above only prove the wire
encode/decode and control flow are self-consistent, not that they match the
real MCU's behavior.

Older host CLIs and packages live under [`scripts/legacy/`](../legacy/README.md)
(frozen — do not extend).
