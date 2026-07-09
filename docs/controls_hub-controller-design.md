# controls_hub_controller — design doc

Implements the brief in `docs/controls_hub-controller.md`. Package lives at
`scripts/controls_hub_controller/`; implemented in this pass (not just designed).

## 1. Executive summary

`controls_hub_controller` is a thin Python layer over the 562 B host exchange image
(`App/Inc/host/host_exchange_schema.h`, `docs/host-exchange-v1.md`) that lets application
code drive motors with raw MIT desires (`position, velocity, kp, kd, torque`) per
actuator slot and read back parsed feedback, without any of the ramping/homing/arrow-key
policy that lives in `scripts/control_hub/teleop/plant.py`. It reuses
`controls_pcb_host`'s wire-offset constants and low-level pack/parse functions (single
source of truth for the byte layout) but exposes a new, narrower surface —
`PlantSession`, `CommandImage`, `FeedbackImage`, `ActuatorDesire`, `McuState`,
`PlantBlockReason` — that only touches `actuator_commands[]` and `system.mcu_state`,
never `pdu`/`servos`/`leds`. The firmware is not a dumb passthrough (mount-every-frame
semantics, a 500 ms stale watchdog with a non-idle bypass, and RS02-specific host
position interpolation all sit between a host write and a motor moving); this doc's
cross-check matrix makes each of those behaviors explicit rather than papering over them.

## 2. Proposed API (as implemented)

```python
from controls_hub_controller import PlantSession, ActuatorDesire, McuState, PlantBlockReason

class ActuatorDesire:  # frozen dataclass
    position: float = 0.0   # rad
    velocity: float = 0.0   # rad/s
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0     # Nm
    # No implicit defaults beyond IEEE-754 zero — an all-zero desire is a real
    # idle/no-torque command, not "leave unchanged".

class McuState(IntEnum):
    NORMAL = 0; RECOVERY = 1; DIAG_ONLY = 2; ESTOP = 3

class PlantBlockReason(IntEnum):
    NONE = 0; BENCH_SESSION = 1; PROBE_BUSY = 2; QUIET_PERIOD = 3
    DIAG_ONLY = 4; HOST_STALE = 5; SERVO_SESSION = 6

class CommandImage:
    def __init__(self, seq: int = 0, mcu_state: McuState = McuState.NORMAL): ...
    def set_mcu_state(self, state: McuState) -> "CommandImage": ...
    def set_actuator(self, slot: int, desire: ActuatorDesire) -> "CommandImage": ...
    def set_actuators(self, desires: Mapping[int, ActuatorDesire]) -> "CommandImage": ...
    def desire(self, slot: int) -> ActuatorDesire: ...
    def to_bytes(self) -> bytes: ...          # raw 562 B

class FeedbackImage:
    def __init__(self, raw: bytes): ...        # raises InvalidFrameError on bad magic/size
    tick: int; ack_seq: int; mcu_state: int
    plant_block: PlantBlockReason | int
    def actuator(self, slot: int) -> FeedbackState | None: ...
    actuators: list[FeedbackState | None]

class PlantSession:
    @classmethod
    def connect(cls, port: str, *, baud: int = 115200) -> "PlantSession": ...
    def open(self) -> None: ...
    def close(self) -> None: ...
    def set_mcu_state(self, state: McuState) -> None: ...
    def recover(self) -> None: ...              # RECOVERY -> NORMAL, clears held desires
    def write_command(self, image: CommandImage) -> None: ...   # raw, caller-built frame
    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None: ...
    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = True) -> None: ...
    def poll_feedback(self) -> FeedbackImage | None: ...        # non-blocking, latest
    def read_feedback(self, *, timeout_s: float = 1.0, latest: bool = True) -> FeedbackImage: ...
    def sleep_until_next_tick(self, hz: float) -> None: ...     # app-owns-the-loop pacing
    def run_at_hz(self, hz: float, callback, *, stop=None) -> None: ...  # streaming loop helper
```

`send_once()` (fire-and-forget) vs `run_at_hz()` (streaming loop helper) both exist, per
the deliverable. `PlantSession` keeps a host-side dict of currently-held desires and
resends *all* of them on every frame — required because of cross-check row 1 below, not
an API nicety.

## 3. Firmware cross-check matrix

All rows verified against current C source (not docs) in this session.

| # | API surface | C function | Motor effect | Gap |
|---|---|---|---|---|
| 1 | `set_actuator`/`set_actuators` write only the slots you pass | `actuator_command_mount` (`actuator.c:31-41`) copies **all 6** `ACTUATOR_COUNT` slots from *every* fresh command image, unconditionally, no change-detection | Any slot not included in a given 562 B frame is mounted as an **all-zero desire** — it is not "left alone" | **document** — `PlantSession` resends the full held-desire dict every frame specifically to hide this; callers building `CommandImage` by hand must do the same |
| 2 | `pdu` never written by this API | `plant_command.c:42-45,68,74-77` — tag match on `pdu.data[0..2]`; all-zero `pdu` falls straight through to normal plant mount | Zero `pdu` never triggers `plant_diag` | none — confirmed safe by construction |
| 3 | n/a (host_stale is implicit) | `diag_gates.c` `plant_runtime_actuator_can_apply()`: `!host_link_command_is_fresh(500) && !actuator_any_non_idle_live()` → `HOST_STALE`, blocks CAN apply | Streaming below ~2 Hz effective (or gaps > 500 ms) with an idle-ish desire silently stops motion; `plant_block` in feedback reports it | **document** — exposed via `FeedbackImage.plant_block`; `PlantBlockReason.HOST_STALE` is the value to watch |
| 4 | n/a | `actuator_any_non_idle_live()` (`actuator.c:53-67`): bypasses staleness if kp/kd/\|vel\|/\|torque\| > 0.01 on any enabled slot | A held nonzero-kp position hold keeps applying even if the host briefly stalls >500 ms; a zero-kp/kd/vel/torque ("truly idle") desire does not get this grace period | **document** — this is a real firmware policy, not a bug; flagged here rather than hidden |
| 5 | RS02 slots, `velocity≈0` desires | `robstride_host_desire_updated` + `robstride_interp_desire` (`robstride.c:668-727`): when `\|velocity\| < 0.01`, firmware computes an implied velocity from the last two *host* update timestamps and extrapolates position at 500 Hz between host updates (only if the host-to-host gap was ≤200 ms) | A sparse `p=θ, v=0` stream (e.g. 40 Hz) gets smoothed into an implicit slew on RS02 slots — this is host-invisible interpolation, exactly what `docs/controls_hub-controller.md` non-goals ask to avoid hiding | **document, P1** — production callers that want *no* implicit smoothing should send an explicit nonzero `velocity` every frame (bypasses `\|v\|<0.01` trigger); this is not toggle-able from the host today |
| 6 | Damiao slot (protocol=DAMIAO) | `damiao_apply_cycle` (`damiao.c:434-464`): auto clear-fault+enable on first non-recovery command, latched per-slot; no host "session begin" required, no position-interpolation analog | First frame after `recover()`/boot silently arms the motor — there is no explicit "enable" call in this API, it is implicit in `set_actuator` | **document** — different from RobStride's plain MIT-apply; no action needed unless an explicit-enable API is later required |
| 7 | `set_mcu_state(RECOVERY \| ESTOP)` | `plant_command.c:33-36` — RECOVERY/ESTOP short-circuits dispatch straight into `plant_recovery_all()`, actuator mount/apply is skipped entirely while in that state | `actuator_commands[]` sent during RECOVERY/ESTOP has **no effect** until NORMAL resumes | **document** — `PlantSession.set_mcu_state` clears the host-held desire dict on RECOVERY/ESTOP to mirror `plant_recovery_all()` clearing live desires, so old commands don't reappear the instant NORMAL resumes |
| 8 | `servos[2]`, `leds[1]` — **not exposed** by this API | `servo.c` (Dynamixel unicast) and `led.c` (SK9822 @ ~30 Hz) are fully live firmware consumers, not stubs | Out of scope for `controls_hub_controller`; `controls_pcb_host.commands.build_plant_servo_command` / `sk9822_led_test.py` still cover them | **document** — explicitly deferred, not a gap; revisit if app-layer needs servo/LED control through the same session object |
| 9 | `FeedbackImage.actuators` mirrors wire | Wire carries `HOST_EXCHANGE_ACTUATOR_SLOTS=25`, firmware only ever populates slots 0..5 (`ACTUATOR_COUNT`) | Slots 6..24 are always zero on the wire; API only exposes 0..5 (`ACTUATOR_COUNT`) and raises `InvalidSlotError` outside that range | **document** — deliberate, matches `plant_config.c`'s 6 enabled slots |
| 10 | `FeedbackImage` — no `header.seq` tracking | `host_feedback_image_fetch` (`host_link.c:130-147`) never assigns `header.seq`; always 0 | Use `ack_seq` (echo of the *command's* `header.seq & 0xFF`) for correlation, not the feedback header's own `seq` | **firmware gap, P2** — cosmetic; `ack_seq` is sufficient for the "did my command land" use case, so no host workaround needed |
| 11 | MCP slots (CH4-6, i.e. slot indices whose configured bus is MCP) | `robstride_apply_cycle` (`robstride.c:729-799`): `tx_burst = mcp ? 1 : 3`; MCP path has no alternating para-read supplemental feedback that FDCAN gets | MCP-bus actuators (see slot table below) get a lower TX burst and a documented, empirically-observed grouped-feedback-cadence effect (`docs/bringup.md` §7); no code-level ~300 ms constant was found — treat this as an operational limitation, not a hard-coded delay | **document, P2** — surfaced via `tick`/timestamps the app can already read from `FeedbackImage`; no API change proposed |
| 12 | `diag_gates.c` `plant_block` labeling | `plant_runtime_actuator_can_apply()` step 2 (`diag_gates.c`) falls back to `PLANT_BLOCK_BENCH_SESSION` in its final `else` branch, but that branch is reached via `plant_diag_skip_actuator_can()` which is itself only true when one of bench/probe/quiet/servo-session already holds — so the fallback is currently unreachable dead code, not a live mislabel | none observed in practice | **flag for firmware, P2** — not fixed here (no C changes made); harmless as-is, noted for awareness only |
| 13 | Host actuator config (`bus`, `protocol`, `motor_id` per slot) | `controls_pcb_host.actuator_config._DEFAULT_TABLE` re-checked against `plant_config.c:16-57` this session | **Matches exactly** (slot0 CH1/RS/0x76 … slot2 CH3/Damiao/0x06 … slot5 CH6/RS/0x70) | no gap currently — table was in sync; no live NVM/config-readback PDU exists in C (`plant_config_apply` absent repo-wide), so this remains a hand-maintained mirror, not a firmware bug |

## 4. Gap list

| Gap | Severity | Owner | Recommendation |
|---|---|---|---|
| RS02 host-position interpolation is always-on for `\|velocity\|<0.01` desires (#5) | P1 | firmware (opt-out) or host (workaround now) | Host workaround available today: send explicit nonzero `velocity` to bypass; longer term, an `mcu_state`/pdu flag to disable interpolation per-slot would make this an explicit choice instead of implicit firmware policy |
| Feedback `header.seq` never incremented (#10) | P2 | firmware | Cosmetic — `ack_seq` (command-seq echo) already covers correlation; low priority fix, only needed if something wants to detect duplicate/dropped feedback frames independently of command cadence |
| MCP grouped-feedback cadence (#11) | P2 | firmware/hardware | No host-side fix possible (SPI-CAN scheduling); document expected cadence per bus in slot config so app code on MCP slots doesn't assume FDCAN-like feedback freshness |
| `diag_gates.c` bench_session fallback branch is unreachable (#12) | P2 (code health, not a bug today) | firmware | **Not touched in this pass** — flagging per your instruction not to edit STM32 C in this task. Worth a follow-up: either prove it truly can't be hit, or give the final `else` its own `PLANT_BLOCK_UNKNOWN` label so future refactors don't silently mislabel a real path. |
| No live config-readback/NVM PDU (#13) | P2 | firmware | `actuator_config.py`/`controls_hub_controller.config` remain a hand-maintained mirror; low urgency since the mirror currently matches, but will silently drift if `plant_config.c` changes without a corresponding host edit |

## 5. Example application code

See `scripts/controls_hub_controller_example.py` for runnable versions.

```python
# A. Single MIT hold — slot 3
from controls_hub_controller import PlantSession, ActuatorDesire

with PlantSession.connect("COM5") as session:
    session.set_actuator(3, ActuatorDesire(position=0.0, velocity=0.0, kp=8.0, kd=0.45, torque=0.0))

# B. Velocity-mode stream at 100 Hz — app owns the trajectory, no host-side ramp
with PlantSession.connect("COM5") as session:
    for t in trajectory:
        session.set_actuator(3, t.desire, send=False)
        session.send_once()
        session.sleep_until_next_tick(100.0)

# C. Multi-slot frame
with PlantSession.connect("COM5") as session:
    session.set_actuators({0: d0, 1: d1})
```

## 6. Relationship to existing code

- **Kept, reused directly**: `controls_pcb_host.protocol` (wire offsets — single source of
  truth), `controls_pcb_host.commands.{patch_actuator_desire,patch_system_mcu_state}`,
  `controls_pcb_host.feedback.{parse_feedback_header,parse_actuator_feedback}`,
  `controls_pcb_host.transport.{FrameReader,SerialRxPump,open_serial}`,
  `controls_pcb_host.actuator_config` (re-exported via `controls_hub_controller.config`).
- **Not touched / out of scope**: `scripts/control_hub/teleop/plant.py` (ramps, homing,
  arrow keys, lead caps) stays exactly as-is and is not imported by
  `controls_hub_controller`. RS2/DM/DXL/UB bench PDU builders in
  `controls_pcb_host.commands` are not re-exported here — bench work stays on
  `controls_pcb_host`/`PcbSession` directly.
- **`PcbSession` vs `PlantSession`**: not replaced. `PcbSession` remains the bench/diag
  session (RS2/DM session begin/end, probes). `PlantSession` is new and narrower —
  normal-operation only. They can coexist against the same port sequentially (not
  concurrently — one serial connection at a time).
- **Migration path**: existing `python scripts/control_hub.py teleop` is unaffected.
  New application code should `from controls_hub_controller import PlantSession` instead
  of hand-building frames with `controls_pcb_host.commands.build_plant_command`.

## 7. Testing plan

- **Implemented**: `scripts/tests/test_controls_hub_controller.py` — pack/unpack
  round-trip against wire offsets, unset-slot-is-zero, batch set, slot-range validation,
  feedback header/plant_block/ack_seq parsing, bad-magic/bad-size rejection. Run with
  `python -m pytest scripts/tests/`. 19/19 passing (includes existing `test_protocol.py`,
  `test_rs02_decode.py`).
- **Documented, manual (needs hardware)**: loopback on a live port — send a command,
  read feedback, assert `ack_seq == cmd.seq & 0xFF`; step slot 3 `kp` 0→8 and verify
  `plant_block == PlantBlockReason.NONE` and `fb.actuator(3).position` moves; regression
  check that zero-`pdu` frames never flip `plant_block` to a diag-related value.

## 8. Phased implementation plan

- **MVP (done this pass)**: `CommandImage`, `FeedbackImage`, `PlantSession` with
  `set_actuator(s)`, `send_once`, `poll_feedback`/`read_feedback`, `run_at_hz`,
  `sleep_until_next_tick`, `recover`. Unit tests. Example script.
- **Next**: hardware-in-the-loop test against a live board (manual, per §7) to confirm
  `ack_seq`/`plant_block` behavior matches this doc under real timing; add an
  `hz`-aware warning/log when the caller's own loop is running slower than
  `HOST_STALE_MS` would tolerate.
- **Later**: optional `bench/` submodule if application code ever needs RS2/DM probes
  through the same `PlantSession` object (not needed today — `controls_pcb_host` covers
  it). Possible firmware follow-up on the P1/P2 gaps above, tracked separately since no
  C was changed in this task.

## Acceptance criteria check

- [x] Caller can command motors with only `actuator_commands[]` + `mcu_state=NORMAL`,
      zero `pdu` — `CommandImage`/`PlantSession` never write `pdu`/`servos`/`leds`.
- [x] No teleop ramping, homing, or arrow logic in core API.
- [x] Every hidden firmware policy found is listed in §3.
- [x] Gaps have severity and owner (§4).
- [x] API suitable as a filter layer — no policy stack embedded (trajectories, ramps,
      safety logic are the caller's job).
- [x] Wire layout v1 unchanged — no schema/offset edits made.
