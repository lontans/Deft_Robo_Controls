# RFC: CubeMars Servo-Current dialect for AKH70-48 (no MIT)

**CORRECTION (post-hoc, unverified premise):** the "AKH70-48 has no MIT
mode" claim below cites `CubeMars_AK_Driver_Doc_Generalised.pdf` §5.1-5.2,
but that same PDF's §5.3 documents MIT Mode Communication Protocol in full
(enable/disable/zero opcodes, std-ID DLC=8 frame, byte-identical to
`cubemars_motor_driver_doc.pdf`'s §5.3) with no per-model exclusion — and
neither PDF mentions "AKH" anywhere in its text (checked via full-text
extraction, zero hits in both). Both docs are titled/scoped to the "AK
Series" generically, presenting Servo Mode and MIT Mode as two capabilities
of the same driver board line, not per-model-exclusive dialects. The
"no MIT mode" premise this RFC is built on does not appear to be supported
by either source PDF — treat as unconfirmed, not fact, until bench-checked
(send `0xFC` + a zero-effort MIT frame to a real AKH70-48 and look for the
FB echo `cubemars_probe_id`/`cubemars.c`'s hot path already expects). Do not
build the dialect-split plumbing below off this premise without that check.

Status: brainstorm / not implemented. Scope: `App/.../plugins/cubemars.*`,
`App/.../diag/diag_cubemars.c`, CFG/host labeling only. **Does not touch**
Damiao / RobStride / ZeroErr apply paths, and does not migrate other
CubeMars units off MIT.

## 1. Hardware fact this RFC is built around

AKH70-48 has no MIT mode. The only usable wire dialect for torque-ish
control is **Servo Mode → Current Loop Mode** (`CAN_PACKET_SET_CURRENT = 1`,
PDF §5.1.2 / §5.2.1). Position / Pos-Speed Servo modes have no torque
feed-forward on the wire and are out of scope for a torque-controlled joint.

Confirmed from `CubeMars_AK_Driver_Doc_Generalised.pdf` §5.1–5.2 (full text
extracted, not assumed):

**TX — Current Loop Mode (§5.1.2)**
- Ext CAN ID: `(1 << 8) | node_id` — i.e. `cubemars_build_ext_id(CUBEMARS_MODE_CURRENT, node_id)`, already implemented and correct in `cubemars.c`.
- DLC **4**, not 8: big-endian `int32_t` = `(int32_t)(current_A * 1000.0f)`, raw range `-60000..60000` → `±60 A`.
- No enter/exit opcode. A Current-mode frame commands Iq the instant it's decoded — there is no MIT-style `0xFC` handshake for Servo Mode at all.

**RX — periodic status upload (§5.2.1, 1–500 Hz configurable)**, 8 bytes:

| bytes | field | raw type | scale |
|---|---|---|---|
| [0:1] | position | int16 | ×0.1 → deg, range ±3200° |
| [2:3] | speed | int16 | ×10 → **electrical** RPM, range ±320000 |
| [4:5] | current | int16 | ×0.01 → A, range ±60 A |
| [6] | motor temperature | int8 | °C |
| [7] | error code | uint8 | 0 none / 1 motor-overtemp / 2 overcurrent / 3 overvoltage / 4 undervoltage / 5 encoder / 6 MOSFET-overtemp / 7 stall |

This is byte-identical to what `cubemars.c` already has written (and
compiled out) under `#if CUBEMARS_ENABLE_SERVO_MODE` as
`cubemars_servo_parse_rx` — that reference code just needs its `torque`
and unit handling fixed (see §3) and promoting from "reference" to a real
dispatch path.

**Open item — feedback CAN ID.** The generalized PDF doesn't restate the ID
field for the upload frame in §5.2.1 (only the TX table in §5.1 documents
`[28:8]=mode / [7:0]=node`). The existing reference code
(`cubemars_servo_parse_rx`) sidesteps this by keying off `cfg->master_id` as
an opaque, bringup-discovered value rather than a hardcoded formula — keep
that approach. **Verify on the bench** (bringup step 3 below) and record the
actual ext ID in a captured-frame note next to the KT/pole-pair constants.

*Weaker lead than it first looked — corrected.* An earlier pass at this RFC
suggested `CAN_PACKET_STATUS=9` (stock VESC numbering) as a first guess for
the upload frame's ext ID. On closer reading that doesn't hold up: the
documented CAN enum here is `0 SET_DUTY, 1 SET_CURRENT, 2
SET_CURRENT_BRAKE, 3 SET_RPM, 4 SET_POS, 5 SET_ORIGIN_HERE, 6 SET_POS_SPD`
— but indices 5/6 in stock VESC are `CAN_PACKET_FILL_RX_BUFFER` /
`CAN_PACKET_FILL_RX_BUFFER_LONG`. CubeMars has evidently **overwritten**
those slots with their own servo packet types, not merely left them
undocumented, which means nothing past index 4 can be assumed to match
stock VESC numbering (contrast §6's serial `COMM_PACKET_ID` gap, which is a
clean truncation with no renumbering evidence — a materially stronger
lead). No safe guess for the upload frame's ext ID follows from the CAN
enum alone; go straight to a blind listen-any-ext-ID sniff for this one.

**Open items — per-unit constants, not in this PDF (it's the "Generalised" edition, no per-model table):**
- `KT` (Nm/A) for AKH70-48.
- Pole-pair count and gear ratio, needed to turn electrical RPM into output-shaft rad/s.

Per the existing project rule ("PDF scales are often wrong/inconsistent —
prefer measured bringup"), these three go in a small per-model constant
table (mirrors `k_ak_limits[]` already in `cubemars.c`) filled from bench
measurement, not copied from a marketing datasheet.

## 2. Dialect selection (CFG / per-slot)

`actuator_config_t` (bus, protocol, motor_id, master_id, enabled) is shared
across all protocols and has no CubeMars-specific field — do not add one
there (would touch the 694 B host-exchange wire schema for every protocol).

`cubemars.c` already has exactly the right shape of side-table for this:
`s_model[ACTUATOR_COUNT]` + `cubemars_set_model()/cubemars_get_model()`,
called out in a `TODO(cfg-model)` comment as "the hook for a future CFG/diag
wire." Dynamixel's `servo_table[].model` (`PLANT_SERVO_MODEL_XL430` /
`_XL330_M288`, in `dynamixel.h`) is the one other place this project has
solved the same problem — a small per-slot enum living in a plugin-owned
side table, not in the shared actuator schema.

**Recommendation:** extend the existing CubeMars side-table with a
`dialect` byte, same pattern, same maturity level as the model field it
already sits next to:

```c
// cubemars.h
typedef enum {
    CUBEMARS_DIALECT_MIT           = 0u,  // current default for all slots
    CUBEMARS_DIALECT_SERVO_CURRENT = 1u,  // AKH70-48 and future non-MIT AKs
} cubemars_dialect_t;

void cubemars_set_dialect(uint8_t slot, cubemars_dialect_t dialect);
cubemars_dialect_t cubemars_get_dialect(uint8_t slot);
```

Default stays `CUBEMARS_DIALECT_MIT` (today's only behavior, zero risk to
existing MIT AKs). AKH70-48 slots get `CUBEMARS_DIALECT_SERVO_CURRENT` set
at `plant_config_load_factory_defaults()` time for now — same maturity as
`CUBEMARS_MIT_DEFAULT_MODEL` hardcoding AK80-9 today. A host-facing CFG
mailbox opcode to set this per slot at runtime is a natural follow-up (would
mirror how `servo_table[].model` would eventually get a setter) but isn't
required for first bringup and isn't proposed here — avoids inventing wire
RPCs before the dialect itself is proven on hardware.

Add a second per-model constant table next to `k_ak_limits[]`:

```c
typedef struct {
    float kt_nm_per_a;
    float pole_pairs;
    float gear_ratio;      // output-shaft turns per electrical-RPM turn source
    float current_limit_a; // conservative smoke-test-derived ceiling, not the 60A wire max
} cubemars_servo_current_params_t;
```
one entry, `CUBEMARS_AKH70_48`, filled in from bringup measurement (§5),
not from the PDF.

## 3. Plant changes (Cubemars-only)

Keep `cubemars_apply_cycle()` as the single entry point; branch on
`cubemars_get_dialect(slot)` at the top instead of unconditionally doing
MIT. No change to Damiao/RobStride/ZeroErr call sites.

```
cubemars_apply_cycle(cfg, desire, state_out):
    if dialect == MIT:      // unchanged, byte-for-byte today's path
        ... existing enable-latch + cubemars_pack_mit ...
    else:                    // SERVO_CURRENT
        iq = desire->torque / kt_nm_per_a(slot)   // Nm -> A
        clamp iq to [-current_limit_a, +current_limit_a]  (bringup ceiling, not ±60A)
        pack SET_CURRENT ext frame (DLC 4, big-endian int32 milliamps-ish per §1)
        enqueue — no enable latch, no 0xFC/0xFD, ever
```

Lifecycle differences from MIT worth being explicit about, since the header
comment for MIT ("apply_cycle never sends disable... blank desire is a valid
zero-effort command") doesn't hold as-is for Servo Mode:

- **No enter/exit opcode exists in Servo Mode.** `s_cubemars_enable_latched[]`
  and `cubemars_send_enable/disable` are MIT-only; the servo-current branch
  never touches that latch and never sends `0xFC`/`0xFD`/`0xFE` — those are
  MIT special codes and are meaningless (probably ignored, unverified) to a
  drive running in Servo Mode.
- **Idle / recovery semantics** for the servo-current branch: send
  `Iq = 0` (SET_CURRENT with raw 0), not Current-Brake. Current-Brake
  actively resists motion at the current position — a reasonable *optional*
  E-stop behavior later, but the wrong default for "disable" (should
  freewheel, matching what Damiao/RobStride disable already does
  conceptually). `plant_recovery_all()` for a servo-current slot = one
  `Iq=0` frame, not a special exit opcode.
- **Command-loss behavior is unverified.** Whether the AKH70-48 drive
  free-wheels or holds last command if CAN frames stop arriving needs a
  bench check (§5) before relying on "host silence = safe."

**Feedback parsing** — promote `cubemars_servo_parse_rx` out of
`#if CUBEMARS_ENABLE_SERVO_MODE` for slots with dialect==SERVO_CURRENT only
(MIT slots keep using `cubemars_parse_mit`, dispatched the same way as
pack). Fix two unit gaps in the existing reference code before relying on it:

1. `state_out->velocity = speed_raw * 10.0f` currently returns *electrical
   RPM*, not rad/s. Needs `(eRPM * 2π) / (60 * pole_pairs * gear_ratio)` to
   land in the same rad·s⁻¹ units Damiao/RobStride/ZeroErr states use.
2. `state_out->torque = cur_raw * 0.01f` currently stores raw Amps into a
   field the host contract defines as Nm (SI). Needs `× kt_nm_per_a` — the
   inverse of the pack-side conversion in §3 — done at the same
   plugin-internal layer, not pushed onto the host, so Cubemars stays
   drop-in with the shared `ActuatorDesire`/state SI contract other
   protocols use.

Position (`pos_raw * 0.1f` → degrees) needs `× π/180` for the same reason.

None of this touches `damiao.c`/`robstride.c`/`zeroerr.c` — it's entirely
inside `cubemars_pack_*`/`cubemars_parse_*` dispatch.

## 4. Discover / calibrate / dashboard

Today's `diag_cubemars.c` / `cubemars_probe_id[_range]` sends a zero-effort
**MIT** frame and listens for the FB echo (`data[0] == Drive ID`). That
probe strategy doesn't apply to a servo-current-only unit: there's no MIT FB
echo to key on, and per §1 the periodic upload's CAN ID isn't yet confirmed.

Two options for a servo-current-aware discover, in order of how much new
surface they need:

- **A (recommended for first bringup): listen-only probe.** Don't transmit
  anything — Servo Mode uploads periodically on its own once configured. Add
  `CUBEMARS_PROBE_SERVO_LISTEN`: open a listen window on the bus, accept any
  ext-ID frame whose DLC==8 and error byte is in `[0,7]`, report the ext ID
  seen back to the host as candidate `master_id`. This is how the "open
  feedback ID" unknown from §1 actually gets resolved on the bench, so it
  has to come before a send-and-match probe would even be reliable.
- **B (once A has pinned the ID convention): send-and-match probe**, same
  shape as today's MIT sweep — send `Iq=0` to a candidate node_id ext ID,
  listen for its upload frame, same "host re-sweeps from hit+1" contract as
  the existing `cubemars_probe_id_range`.

Either way this is new code in `cubemars_probe_*` + a new `kind` in
`diag_cubemars.c`'s dispatch — it must **not** touch
`cubemars_apply_cycle`'s enable latch, same rule the file's header comment
already states for the existing MIT probe.

Dashboard / Tab-2: out of scope per the brief's non-goals; a servo-current
slot should render through the same state fields (`position`, `velocity`,
`torque`, `fault`) once §3's unit fixes land, so no dashboard-specific work
should be needed beyond what CFG labeling already implies.

## 5. Bringup checklist (hardware, safety-first)

1. **Power + CAN only, no commands.** Confirm bus termination/bitrate (PDF
   implies 1 Mbps, same as MIT AKs elsewhere in this repo). Sniff the bus
   for any unsolicited upload frames — resolves the §1 "which ext ID"
   question before writing any decode code against a guess.
2. **Node ID.** Read/confirm the drive's configured node ID (factory
   default per CubeMars tooling, not assumed to be the same convention as
   MIT `motor_id`). Record it in the CFG row for the slot.
3. **Upload rate.** Confirm the factory-default 1–500 Hz upload rate first
   (measure frame timing directly — no tool needed); only chase changing it
   via §7 if the default genuinely doesn't fit the control loop. Note the
   value used either way (affects staleness checks later, same idea as
   Damiao/RobStride timeout handling).
4. **Iq smoke test, current source disconnected from load if possible, or
   joint free to move with no coupled hazard.** Send a *small* Iq (e.g.
   0.2–0.5 A, nowhere near the 60 A wire ceiling) via the discover/probe
   path from §4, confirm: (a) motor responds mechanically in the expected
   direction, (b) upload frame's current field roughly tracks commanded Iq,
   (c) error byte stays 0.
5. **KT estimate.** With the joint either torque-loaded against a known
   mass/lever arm or blocked against a load cell, step Iq and measure output
   torque to back out `kt_nm_per_a`; note pole-pairs/gear-ratio from the
   nameplate or from comparing commanded eRPM-scale speed against measured
   output-shaft speed. Fill in `cubemars_servo_current_params_t` from this,
   not from the generalized PDF (no per-model table in it) and not from a
   marketing datasheet number taken on faith.
6. **Command-loss check.** Stop sending CAN frames mid-motion at low Iq;
   confirm whether the drive free-wheels or holds — determines whether the
   MCU needs its own watchdog re-send-`Iq=0` behavior on top of
   `plant_recovery_all()`.
7. **Host torque teleop**, small gains, once 1–6 are clean — same
   host/Jetson-side PD-plus-gravity outer loop already used for other arms;
   nothing new needed there since the MCU-side dialect fix keeps the shared
   `ActuatorDesire.torque` (Nm) contract intact.

## 6. Configuring the driver without CubeMars' tool

Two different things live under "config": **control commands** (mode
select, set-origin, current/position setpoints) are §1–§5 above and are
already plain CAN frames — no tool ever needed. **Persistent driver
settings** (CAN node ID, MIT-vs-Servo firmware personality, upload rate,
current/voltage limits) are normally done through CubeMars' GUI and are not
in the "Generalised" PDF. This section is about that second category.

**A lead, not a guess — and only on the serial side.** §5.2.2's
`COMM_PACKET_ID` enum lists `COMM_SET_DETECT` then jumps straight from
index 11 to `COMM_ROTOR_POSITION = 22`, silently skipping 12–21. Those
missing indices line up exactly with stock open-source VESC firmware's own
enum (`SET_SERVO_POS=12, SET_MCCONF=13, GET_MCCONF=14,
GET_MCCONF_DEFAULT=15, SET_APPCONF=16, GET_APPCONF=17,
GET_APPCONF_DEFAULT=18, SAMPLE_PRINT=19, TERMINAL_CMD=20, PRINT=21`) — a
clean truncation, nothing renumbered, real evidence CubeMars forked VESC
and just didn't publish these. `GET_APPCONF`/`SET_APPCONF` hold CAN node ID
and CAN status rate in stock VESC; `GET_MCCONF`/`SET_MCCONF` hold
current/voltage limits.

The equivalent CAN-side enum (§1) is **not** clean evidence the same way —
CubeMars has demonstrably overwritten indices 5/6 (stock VESC's
`FILL_RX_BUFFER`/`FILL_RX_BUFFER_LONG`) with their own `SET_ORIGIN_HERE`/
`SET_POS_SPD`. Stock VESC uses that same buffer-fill mechanism (plus
`PROCESS_RX_BUFFER`/`PROCESS_SHORT_BUFFER`) to tunnel an arbitrary serial
`COMM_*` packet — including `SET_APPCONF`/`SET_MCCONF` — over CAN in 8-byte
chunks, so a config write would need no serial cable at all. Whether that
tunnel still exists in this fork, and at which indices, is unknown given
the renumbering already observed — treat "config over CAN alone" as
unconfirmed until sniffed or confirmed by CubeMars, not as available by
default.

**No hardware, but the tool itself?** Two hardware-free ways to still get
real bytes out of it: (a) unpack the installer/install directory and look
for literal parameter-definition resource files (XML/JSON) rather than
compiled logic — tools in this lineage often ship the mcconf/appconf field
layouts that way for GUI auto-generation; (b) put a null-modem/loopback
virtual serial port (`com0com` on Windows, a `socat` PTY pair on Linux)
between the tool and a script that fakes just enough of a connect handshake
(a plausible, CRC-valid reply to whatever `COMM_FW_VERSION`/
`COMM_GET_VALUES`-style probe it opens with) to get the tool to treat it as
a live drive — then read the exact `SET_APPCONF`/`SET_MCCONF` bytes it
emits when you change a setting in the GUI, without ever needing a real
motor.

**Recommended order, cheapest/safest first:**

1. **Zero risk:** confirm the serial link round-trips at all using the
   already-documented `COMM_GET_VALUES`.
2. **Zero risk:** probe `COMM_GET_APPCONF (17)` / `COMM_GET_MCCONF (14)`
   with an empty payload. A large struct blob back (vs. silence/error)
   confirms live config-read — pure observation.
3. **Real risk — read-modify-write only, never a guessed partial struct.**
   Decode one field against a reference VESC struct layout, cross-check it
   against ground truth you can already observe independently (node ID from
   which CAN ID it answers on; upload rate from measured frame timing). If
   the known fields decode correctly, flip the one field you want and write
   the whole blob back unchanged elsewhere. If they don't line up, stop —
   it isn't the same layout, and guessing further risks corrupting a config
   sector.
4. **Lower-risk alternative, if the tool is reachable even once (a VM, a
   borrowed machine):** sniff it instead of reverse-engineering blind. Tap
   the serial line during one "set node ID" / "set upload rate" session,
   capture the exact bytes, replay them verbatim afterward. Turns an
   unknown protocol into a known one with zero struct-layout risk, and the
   tool is never needed again after that one capture.
5. **Ask CubeMars support directly** for the AKH70-48 config command spec —
   a reasonable ask even when the public PDF omits it.

**Before any of this:** bringup step 1 (power+CAN, listen only) likely
answers node ID and upload rate for free, since AKH70-48 should already ship
in Servo Mode broadcasting on its own. Only pursue this section if the
factory defaults actually don't work for the integration.

## 7. Non-goals (unchanged from the brief)

- No plant-wide "everything goes current-loop" migration — MIT AKs keep
  today's `cubemars_apply_cycle` behavior verbatim when
  `dialect == CUBEMARS_DIALECT_MIT` (the default).
- No Damiao / RobStride / ZeroErr apply-path changes.
- No dashboard/Tab-2 rewrite; CFG labeling only if a tiny addition is
  needed to show dialect in an existing view.
- No new host `ActuatorDesire`/`ActuatorFeedback` shape — Nm-in/Nm-out stays
  the wire contract; the A↔Nm and eRPM↔rad/s conversions are internal to
  the Cubemars plugin, same information-hiding the MIT path already has via
  `k_ak_limits[]`.

## File touch list (when this moves from RFC to patch)

- `App/Inc/plant/plugins/cubemars.h` — `cubemars_dialect_t`,
  `cubemars_set_dialect`/`get_dialect`, `cubemars_servo_current_params_t`,
  `CUBEMARS_PROBE_SERVO_LISTEN`; promote the `#if CUBEMARS_ENABLE_SERVO_MODE`
  block from reference-only to a real (gated per-slot, not compile-time-off)
  path.
- `App/Src/plant/plugins/cubemars.c` — dialect dispatch in
  `cubemars_apply_cycle`/`cubemars_on_rx_frame`, current-mode pack (DLC 4),
  fixed-unit servo parse, idle-frame (`Iq=0`) instead of MIT disable for
  servo-current slots, new probe path.
- `App/Src/plant/diag/diag_cubemars.c` — new probe `kind` dispatch for
  listen-only / send-and-match servo-current discover.
- `docs/plant.md`, `docs/vendor.md` — replace the current "Servo mode
  compile-gated off" line with the dual-dialect description once real.
- `docs/bringup.md` — append the §5 checklist as an AKH70-48-specific
  section.
- `scripts/deft_controls_sdk/debug/cubemars.py` — bench helper for the
  servo-current smoke test (send bounded Iq, print decoded upload frame).
- CFG/host labeling: whichever slot-naming file backs `product_cfg.py`'s
  row tables, to add an AKH70-48 bench-profile row — no schema change,
  additive row only.
