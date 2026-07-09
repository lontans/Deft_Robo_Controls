# controls_hub_dashboard — design prompt

Design brief for a **desktop GUI** that controls and debugs the Deft Robotics Controls PCB over the **562-byte host exchange contract only** — the same normal-operation path as `controls_hub_controller`, not PDU bench backdoors.

Use this document as the instruction set for implementing `scripts/controls_hub_dashboard.py`.

---

## Context

Repo: `DeftRoboticsControlsPCB`. A production host library exists at `scripts/controls_hub_controller/` with design doc `docs/controls_hub-controller-design.md` and implementation review in `docs/controls_hub-controller.md` §Implementation review.

### Read first (source of truth)

| Resource | Role |
|----------|------|
| `docs/controls_hub-controller-design.md` | Firmware behaviors, gaps, held-desire semantics |
| `docs/host-exchange-v1.md` | Byte layout (command + feedback) |
| `scripts/controls_hub_controller/` | `PlantSession`, `ActuatorDesire`, `FeedbackImage`, `McuState`, `PlantBlockReason` |
| `scripts/controls_pcb_host/commands.py` | `patch_servo_command`, `pack_led_word`, wire offsets |
| `scripts/controls_pcb_host/feedback.py` | `parse_feedback_header`, `parse_servo_feedback`, `format_status_line` |
| `scripts/controls_hub_controller/config.py` | `slot_config`, `host_table` (6 actuator slots) |

---

## Goal

Single runnable script:

```bash
python scripts/controls_hub_dashboard.py
python scripts/controls_hub_dashboard.py --port COM5
```

A GUI where an operator can:

1. Connect / disconnect USB CDC (`COM*` / `/dev/ttyACM*`)
2. Command **all plant-path peripherals** on the 562 B image: 6 actuators, 2 servos, 1 LED strip command, `mcu_state`
3. See **live debug readouts**: `tick`, `ack_seq`, `plant_block`, `mcu_state` readback, per-actuator feedback, servo feedback, fault flags, frame rates, stale warnings
4. Run a background command stream so `host_stale` (>500 ms) doesn't silently block apply when holding motors

---

## Hard constraints (do not violate)

1. **`pdu` must always be zero** on every command frame. No RS2/DM/DXL/UART bench PDU tags. No `PcbSession` probe/calibrate/discover plugins. No imports from `control_hub/teleop` or `controls_pcb_host/plugins/*`.
2. **STM32-agnostic** — only use fields defined in `host_exchange_schema.h` / `host-exchange-v1.md` plus the hand-maintained `host_table`. Do not hardcode C function names, TIM6 details, or plugin internals in the UI beyond what feedback already exposes (`plant_block`, `tick`, etc.).
3. **562 B frames only** — compose commands via `controls_hub_controller` + `controls_pcb_host.commands` wire patch helpers. Do not open a second code path.
4. **Hold-last semantics** — firmware mounts all 6 actuator slots every frame. The dashboard must resend the **full** held actuator + servo + LED state every stream tick (same pattern as `PlantSession._desires`).
5. **No host-side smoothing / ramping** for actuators — sliders set MIT desires directly; no teleop homing, arrow keys, or lead caps.
6. **Stdlib GUI preferred** — use `tkinter` (works on Windows bench laptops). Do not add PyQt / Dear PyGui / etc. to `requirements.txt` without explicit approval. `pyserial` is the only dep.

---

## Recommended architecture

### Extend the library minimally (preferred)

Add `scripts/controls_hub_controller/hub_session.py` (or extend `PlantSession`) with:

```python
class HubSession(PlantSession):
    # Host-held state mirrors (like _desires for actuators):
    # _servo_cmds: Dict[int, ServoCommand]   # slot 0..1
    # _led_cmd: LedCommand | None

    def set_servo(slot, position, speed, servo_id, torque_enable=1, operating_mode=3) -> None
    def set_led(mode, brightness, led_count) -> None
    def build_command() -> CommandImage   # actuators + servos + leds + mcu_state, pdu untouched
```

`build_command()` starts from `PlantSession.build_command()` then patches `servos[]` at offset 516 and `leds[]` at offset 528 using existing `controls_pcb_host.commands` helpers. Assert `buf[530:562] == b'\x00' * 32` in debug builds or unit test.

**Fix `recover()`** while you're here: after `set_mcu_state(NORMAL)`, call `send_once()` so the MCU actually leaves RECOVERY. See implementation review in `docs/controls_hub-controller.md`.

Consider `set_mcu_state(..., send=True)` or document clearly that state changes need an immediate send.

### Dashboard process model

```
┌─────────────────────────────────────────────────┐
│  Main thread: tkinter UI (sliders, labels, btns) │
│  - reads latest FeedbackSnapshot (thread-safe)   │
│  - writes desired state into HubSession (locked) │
└─────────────────────────────────────────────────┘
         ▲                              │
         │  FeedbackSnapshot            │ held desires
         │                              ▼
┌─────────────────────────────────────────────────┐
│  Worker thread: stream loop @ configurable Hz  │
│  - session.send_once() every tick (default 50Hz) │
│  - session.poll_feedback() → update snapshot     │
│  - compute fb_hz, tick_delta, ack_seq tracking   │
└─────────────────────────────────────────────────┘
```

- Default stream rate: **50 Hz** (safe margin under 500 ms stale watchdog). User-adjustable 10–200 Hz.
- On connect: start worker. On disconnect: stop worker, `close()`.
- UI updates at ~10–20 Hz from the snapshot (don't block tkinter on serial I/O).

### Feedback parsing

Use `FeedbackImage` for actuators. For debug header fields not on `FeedbackImage`, also call `parse_feedback_header(raw)` and `parse_servo_feedback(raw, slot)` from `controls_pcb_host.feedback`.

Expose a `FeedbackSnapshot` dataclass the UI binds to:

- `tick`, `ack_seq`, `mcu_state`, `plant_block`, `pdu_tag` (expect `0x00` / empty)
- `actuators[0..5]`: position, velocity, torque, temperature, fault (hex)
- `servos[0..1]`: present_position, speed, moving, motor_source_id
- `cmd_seq_low` vs `ack_seq` mismatch flag
- `fb_hz`, `cmd_hz`, `tick_hz` (derived)
- `host_stale_warning` if `plant_block == HOST_STALE` or cmd loop gap > 400 ms

---

## GUI layout (minimum viable)

### Top bar

- Port dropdown (`controls_pcb_host.transport.list_serial_ports` / `auto_pick_port`)
- Connect / Disconnect
- Stream rate (Hz) spinbox
- **Recover** button → `session.recover()`
- **MCU state** radio/buttons: NORMAL | RECOVERY | ESTOP | DIAG_ONLY (with warning tooltip: DIAG_ONLY blocks plant apply per firmware)

### Status strip (always visible, color-coded)

- `plant_block` — green=NONE, red=HOST_STALE/ESTOP, amber=others
- `tick`, `ack_seq`, `mcu_state`, `fb_hz`, `cmd_hz`
- Banner when `plant_block != NONE` explaining what it means (use `PlantBlockReason` names from design doc)

### Actuator panel (6 tabs or scrollable grid)

Per slot, label from `slot_config(slot)`:

- `CH{n} | {protocol} | id 0x{id}`
- Command: `position`, `velocity`, `kp`, `kd`, `torque` — entry widgets + "Apply slot" / live-update toggle
- Feedback: `pos`, `vel`, `torque`, `temp`, `fault=0x........`
- **Idle** button → all-zero `ActuatorDesire`
- Tooltip on MCP buses (CH4–6): "feedback may update in grouped bursts — see bringup §7"
- When `|velocity| < 0.01` and protocol is robstride, show small warning: "RS02 may interpolate position between host updates (P1 gap)"

### Servo panel (2 slots)

- `servo_id` (default 1 and 2 per `plant_config.c`), `goal_position` (int), `speed`, torque enable checkbox
- Feedback: present position, speed, moving
- Patch via `patch_servo_command` into the composed frame

### LED panel

- `mode` (0=test scan, 1=off), `brightness` 0–31, `led_count` 0–63
- Uses `pack_led_word` / offset 528 per `host-exchange-v1.md`

### Advanced / debug (collapsible)

- Hex dump of last command / last feedback (first 64 bytes + pdu region showing zeros on cmd)
- `format_status_line()` text log (scrolling, last 100 lines)
- Checkbox: "Stream only when connected" (default on)

---

## Explicit non-goals

- No RS02 calibrate / pararead / discover UI (that's PDU bench — out of scope)
- No Damiao register scan
- No Dynamixel PDU probe
- No arrow-key teleop
- No trajectory generator / sine wave demo (keep `controls_hub_controller_example.py` for that)
- No firmware changes

---

## Files to create / modify

| File | Action |
|------|--------|
| `scripts/controls_hub_dashboard.py` | **New** — GUI entry point |
| `scripts/controls_hub_controller/hub_session.py` | **New** (preferred) — servo/LED compose + `recover()` fix |
| `scripts/controls_hub_controller/__init__.py` | Export `HubSession` if added |
| `scripts/tests/test_hub_session.py` | Unit tests: servo/LED patches, pdu stays zero, full compose round-trip |
| Operator notes | Optional appendix in this doc or comment block in `controls_hub_dashboard.py`: launch, panels, stale watchdog, `plant_block` meanings |

Do **not** modify STM32 firmware.

---

## Acceptance criteria

- [ ] Connect to a live board (or mock serial in tests); GUI streams 562 B frames with **zero pdu**
- [ ] All 6 actuator slots can be commanded independently; unset slots are **not** zeroed mid-session (held state resend)
- [ ] Servos and LED command fields are patched on the wire and survive streaming alongside actuators
- [ ] `plant_block`, `tick`, `ack_seq`, actuator + servo feedback update live in the UI
- [ ] `HOST_STALE` visible when stream paused or rate < ~2 Hz with idle desires
- [ ] Recover transitions MCU to NORMAL with a confirming send
- [ ] No imports from PDU bench / teleop paths
- [ ] `python -m pytest scripts/tests/` still passes (add hub_session tests)

---

## Testing

1. **Unit (no hardware):** build a `HubSession.build_command()` frame, assert offsets 516–529 for servos/LED, assert `pdu[0:3] == b'\x00\x00\x00'`, actuator slot 3 round-trip unchanged.
2. **Manual HIL:** connect COM port, hold slot 3 with `kp=8`, confirm `plant_block=none` and position feedback moves; toggle LED brightness; move servo 0; press Recover; verify `ack_seq` tracks command seq low byte.

---

## Implementation notes

- Bootstrap: same `sys.path.insert(0, scripts_dir)` pattern as `controls_hub_controller_example.py`.
- Thread safety: one lock around `HubSession` desire dicts; worker owns serial writes.
- On ESTOP: clear held actuator desires (mirror `PlantSession.set_mcu_state` behavior) and send immediately.
- `DIAG_ONLY` in the GUI is for visibility/testing only — show a prominent warning that plant actuator apply is blocked (`plant_block=DIAG_ONLY`).
- Keep the dashboard file focused; put compose logic in `hub_session.py`, not 800 lines of tkinter mixed with wire math.

**Do not** cheat around firmware by using PDU backdoors. The whole point is an honest normal-operation host tool on the 562-byte contract.
