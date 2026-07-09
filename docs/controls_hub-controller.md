# controls_hub-controller — design prompt

Design brief for a **production Python library** that commands actuators on the Deft Robotics Controls PCB over USB CDC (or UART). This library will become the **normal-operation host interface** — not a teleop demo.

Use this document as the instruction set for designing (and eventually implementing) the `controls_hub` controller API.

---

## Repository context

Repo: `DeftRoboticsControlsPCB` (STM32G4 firmware + Python host scripts).

### Wire protocol (source of truth)

| Resource | Role |
|----------|------|
| `App/Inc/host/host_exchange_schema.h` | C structs, layout v1 |
| `docs/host-exchange-v1.md` | 562-byte image layout |
| `docs/architecture.md` | Data flow, plant gates, dual paths |

### Existing Python (study, refactor — do not wrap teleop)

| Path | Role |
|------|------|
| `scripts/controls_pcb_host/commands.py` | `build_plant_command()`, `patch_actuator_desire()` |
| `scripts/controls_pcb_host/session.py` | `PcbSession` (serial, seq, send/poll) |
| `scripts/controls_pcb_host/feedback.py` | Feedback parsers |
| `scripts/controls_pcb_host/actuator_config.py` | Slot table mirror of `plant_config.c` |
| `scripts/control_hub/teleop/plant.py` | **Explicitly out of scope** (ramps, homing, arrow keys, lead caps) |

### Firmware path for normal operation (audit these)

| File | Role |
|------|------|
| `App/Src/host/host_link.c` | RX/TX, freshness |
| `App/Src/plant/plant_command.c` | Dispatch: PDU backdoors vs plant mount |
| `App/Src/plant/actuator.c` | `actuator_command_mount`, `actuator_apply_desire` |
| `App/Src/plant/control_loop.c` | TIM6 @ 500 Hz |
| `App/Src/plant/diag/diag_gates.c` | `plant_runtime_actuator_can_apply()`, `host_stale` (500 ms) |
| `App/Src/plant/plugins/robstride.c` | MIT apply, MCP vs FDCAN, host position interpolation |
| `App/Src/plant/plugins/damiao.c` | Damiao MIT path |
| `App/Src/plant/plant_config.c` | Six enabled slots |

---

## Goal

Design a Python package API where application code can:

1. Open a session to the PCB (`COM5` / `/dev/ttyACM*`)
2. Set `mcu_state=NORMAL`, `pdu` all zero (no RS2/DM/DXL bench tags)
3. Write **raw MIT desires** into `actuator_commands[slot]`:
   - `(position, velocity, kp, kd, torque)` per slot
4. Stream commands at the app's rate (e.g. 50–500 Hz)
5. Read **562 B feedback** with parsed `actuator_feedback[slot]`, `plant_block`, `tick`, `ack_seq`
6. Get **predictable motor behavior**: host values map to CAN MIT frames with minimal hidden policy

### Non-goals

- No arrow-key teleop, homing slews, ramp-up/down, lead caps, or "backdrivable idle" logic
- No RS2/DM PDU bench probes in the core API (optional `bench` submodule is fine)
- No firmware changes in the design task — **flag gaps**, don't implement C

---

## Critical design constraint: firmware is not a dumb passthrough

Cross-check C and document what the MCU **actually does** between host write and motor:

| Layer | Behavior to verify |
|-------|-------------------|
| Mount | `actuator_command_mount` copies all 6 slots on each fresh command image |
| Apply rate | TIM6 500 Hz; host desire held between updates |
| Gates | `plant_block`: bench_session, probe_busy, quiet_period, DIAG_ONLY, **host_stale** (>500 ms without fresh command) |
| RS02 interp | `robstride_host_desire_updated` + `robstride_interp_desire` — **500 Hz position interpolation** between host updates when `velocity≈0` |
| MCP CH4–6 | `tx_burst=1`, SPI scheduling — known ~300 ms feedback chunking (see `docs/bringup.md` §7) |
| FDCAN CH1–3 | `tx_burst=3` |
| Recovery | `mcu_state=RECOVERY` triggers `plant_recovery_all()` (reset/disable, clears desires) |
| Slot table | Wire has 25 slots; firmware uses `ACTUATOR_COUNT=6` |

The API must make these behaviors **visible and documented**, not hidden behind teleop-style smoothing.

---

## Deliverables

Produce a design doc + proposed package layout.

### 1. Package structure

Example target (may be improved):

```
controls_pcb/
  __init__.py
  schema.py          # offsets, structs, magic — mirror host_exchange_schema.h
  command.py         # CommandImage builder (mutable or immutable)
  feedback.py        # FeedbackImage parser
  session.py         # Transport + seq + pump thread optional
  plant.py           # PlantRuntime: set_desire(), stream(), read_state()
  config.py          # SlotConfig from actuator_config / plant_config mirror
  exceptions.py
  bench/             # optional: RS2/DM PDU wrappers, clearly separated
```

### 2. Core API sketch (with types)

Design classes/functions for:

- `PlantSession.connect(port)` / context manager
- `session.set_mcu_state(NORMAL | RECOVERY | ESTOP | DIAG_ONLY)`
- `session.write_command(image: CommandImage)` — raw 562 B
- `session.set_actuator(slot, desire: ActuatorDesire)` — patch one slot, send
- `session.set_actuators({slot: desire, ...})` — batch
- `session.read_feedback() -> FeedbackImage` — latest or blocking
- `session.plant_block -> PlantBlockReason` — enum matching firmware
- `ActuatorDesire(position, velocity, kp, kd, torque)` — no implicit defaults that surprise callers
- `FeedbackState` per slot: position, velocity, torque, temperature, fault

Also design:

- **Streaming loop helper** (`run_at_hz(hz, callback)`) vs **fire-and-forget** (`send_once`)
- **Hold-last-command** semantics (document that MCU holds until next frame)
- **Stale watchdog**: host must send ≥1 cmd / 500 ms or `host_stale` blocks apply

### 3. Firmware cross-check matrix (required)

Build a table: **API field / call → C function → motor effect → gaps**

Must cover at minimum:

- Command image validation (`host_command_image_valid`)
- When `pdu` non-zero blocks plant apply
- `host_stale` bypass when `actuator_any_non_idle_live()` (kp/kd/vel/torque non-idle)
- RS02 interpolation: host sends `v=0, p=θ` at 40 Hz → firmware extrapolates to 500 Hz — **is this desired for production API or a gap?**
- Damiao slot 2: MIT field mapping, enable latch, differences from RobStride
- Servo slots (`servos[2]`) and LED slot — include or explicitly defer
- Feedback `header.seq` not incremented (documented gap in `host-exchange-v1.md`)
- Host actuator config not verified against MCU (`actuator_config.py` comment: NVM PDU pending)
- MCP feedback cadence / grouped movement — operational limitation vs API bug

Flag each gap as: **document**, **API workaround**, or **firmware fix needed**.

### 4. Usage examples (no teleop)

**A. Single MIT hold**

```python
# slot 3, CH4 MCP, id 0x70 — hold position with kp
session.set_actuator(3, ActuatorDesire(p=0.0, v=0.0, kp=8.0, kd=0.45, tau=0.0))
```

**B. Velocity-mode stream at 100 Hz**

```python
# App owns trajectory; sends (p, v, kp, kd) each tick — no host-side ramp
for t in trajectory:
    session.set_actuator(3, t.desire)
    session.sleep_until_next_tick(100.0)
```

**C. Multi-slot frame**

```python
session.set_actuators({0: d0, 1: d1})
```

### 5. Relationship to existing code

Specify:

- What to **keep** from `controls_pcb_host` (wire builders, parsers)
- What to **delete/deprecate** over time (teleop in `control_hub/teleop/plant.py` stays separate)
- Whether `PcbSession` becomes the base or gets replaced
- Migration path for `python scripts/control_hub.py teleop` vs new `from controls_pcb import PlantSession`

### 6. Testing plan

- Unit tests: pack/unpack round-trip vs `host_exchange_schema.h` offsets
- Loopback: send command, parse feedback, assert `ack_seq` tracks `seq & 0xFF`
- Integration (documented manual): slot 3 on COM5, step `kp` 0→8, verify `plant_block=none` and `fb` moves
- Regression: ensure `pdu` zero does not trigger `plant_diag`

---

## Known issues to investigate (from bringup)

Do not ignore these — they affect whether "write 562 B → corresponding change" is literally true:

1. **Grouped movement on MCP** (~300 ms `fb` jumps) — `docs/bringup.md` §7
2. **RS02 host interpolation** — may smooth between sparse host updates; production callers may want explicit `velocity` instead of relying on interp
3. **`host_stale` at 500 ms** — streaming apps must meet minimum rate or set non-idle desires
4. **Superloop vs 500 Hz** when MCP SPI blocks — effective apply cadence may be <500 Hz

---

## Acceptance criteria

- [ ] Caller can command motors with **only** `actuator_commands[]` + `mcu_state=NORMAL`, zero PDU
- [ ] No teleop ramping, homing, or arrow logic in core API
- [ ] Every hidden firmware policy is listed in the cross-check matrix
- [ ] Gaps have severity (P0/P1/P2) and owner (host vs firmware)
- [ ] API is suitable as the **filter layer for normal robot operation** (policy stack sits above this, not inside it)
- [ ] Wire layout v1 unchanged unless v2 is proposed with migration notes

---

## Output format (for the designer)

1. Executive summary (1 paragraph)
2. Proposed API (signatures + docstrings)
3. Firmware cross-check matrix
4. Gap list with recommendations
5. Example application code
6. Phased implementation plan (MVP → production)

**Do not** write teleop. **Do not** add host-side smoothing unless explicitly labeled optional. Prefer thin, honest mapping to the 562-byte contract.

---

## Implementation review (Jul 2026)

Independent review of the delivered package at `scripts/controls_hub_controller/` and design doc `docs/controls_hub-controller-design.md`, verified against C source and wire layout.

### Verdict

**Shippable as the normal-operation motor API.** The cross-check matrix is the real deliverable and it holds up. Fix `recover()` and make `set_mcu_state` + send semantics obvious before building a GUI on top.

### What checks out

| Area | Finding |
|------|---------|
| Wire contract | `CommandImage` only touches `actuator_commands[]` + `mcu_state`; `pdu` / `servos` / `leds` stay zero. |
| Mount semantics | `actuator_command_mount` copies all 6 slots every frame (`actuator.c:31–41`). `PlantSession` resending the full `_desires` dict on every `send_once()` / `run_at_hz()` tick is **required**, not optional. |
| Gates & RS02 interp | `diag_gates.c` and `robstride_interp_desire` match the design doc. P1 RS02 interpolation finding and host workaround (explicit nonzero `velocity`) are accurate. |
| Slot table | `actuator_config.py` matches `plant_config.c` (CH1/0x76, CH2/0x70, CH3/Damiao/0x06, CH4–6 MCP/0x70). |
| Tests | 15 unit tests cover header packing, MIT field offsets, unset-slot-is-zero, batch set, slot validation, feedback parsing, bad-frame rejection. |
| Non-goals | No teleop imports, no PDU builders, no host-side ramping in the library. |

### Gaps and nits

| Issue | Severity | Notes |
|-------|----------|-------|
| `recover()` never sends after `NORMAL` | **P1 for GUI** | Sets `RECOVERY`, sends once, sleeps, sets `NORMAL` locally — but doesn't `send_once()` in `NORMAL`. MCU may sit in RECOVERY until the next command. |
| `set_mcu_state()` doesn't send | **P1 for GUI** | ESTOP / RECOVERY / DIAG_ONLY buttons must call `send_once()` explicitly. Easy to miss in a dashboard. |
| `FeedbackImage` thinner than `parse_feedback_header` | P2 | No `pdu_tag`, `fb_seq`, heartbeat bits — a debug dashboard will want `controls_pcb_host.feedback.parse_feedback_header` or an extended parser. |
| Sine example uses `velocity=0.0` | P2 (pedagogical) | Triggers RS02 host interpolation — the thing the brief warns about. Fine as a demo, but ironic. |
| Servos / LEDs deferred | Expected | Correct for the library; dashboard must patch wire offsets 516–529 via `controls_pcb_host.commands` while keeping `pdu` zero. |
| Package name | Cosmetic | Brief suggested `controls_pcb/`; delivered as `controls_hub_controller/`. Fine. |
| Hardware / loopback tests | Documented gap | Manual HIL still outstanding per design doc §7. |

### Delivered artifacts

| Path | Role |
|------|------|
| `scripts/controls_hub_controller/` | `PlantSession`, `CommandImage`, `FeedbackImage`, `ActuatorDesire`, `McuState`, `PlantBlockReason`, `config.py`, `exceptions.py` |
| `scripts/controls_hub_controller_example.py` | Runnable hold / sine examples (no ramping) |
| `scripts/tests/test_controls_hub_controller.py` | 15 unit tests |
| `docs/controls_hub-controller-design.md` | Executive summary, API sketch, 13-row firmware cross-check matrix, gap list, phased plan |

### Acceptance criteria (post-implementation)

- [x] Caller can command motors with only `actuator_commands[]` + `mcu_state=NORMAL`, zero PDU
- [x] No teleop ramping, homing, or arrow logic in core API
- [x] Every hidden firmware policy listed in the cross-check matrix
- [x] Gaps have severity (P0/P1/P2) and owner (host vs firmware)
- [x] API suitable as the filter layer for normal robot operation
- [x] Wire layout v1 unchanged

### Firmware items flagged, not touched

1. **P1** — RS02 slots always interpolate position when host sends `velocity≈0` (`robstride_interp_desire`). Host workaround: send explicit nonzero `velocity`; no firmware change made.
2. **P2** — `diag_gates.c` `plant_runtime_actuator_can_apply()` has an `else` branch that labels an unreachable case as `BENCH_SESSION`. Harmless today (dead code).
3. **P2** — feedback `header.seq` is never incremented (stays 0); cosmetic — `ack_seq` already covers correlation.

Full cross-check detail: `docs/controls_hub-controller-design.md` §3.
