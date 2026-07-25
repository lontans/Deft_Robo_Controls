# FeatherSDK lift ("torso") teardown + de-stub plan

**Status:** research only, no firmware/SDK changes made by this doc. Companion
to [`vbeta-pcb-adapter.md`](vbeta-pcb-adapter.md)'s "Lift (stub)" section and
[`zeroerr-firmware-bringup.md`](zeroerr-firmware-bringup.md) (the CANopen
scaffolding this plan reuses).

**Bottom line:** FeatherSDK's lift is a CANopen actuator (confirmed by a code
comment in the vendored client, not by inspecting `feathersdk` itself — that
package is closed-source and was never vendored anywhere in the reference
dump). Our firmware already has a working CiA-402-over-CANopen stack built
for a different drive (ZeroErr eDriver); de-stubbing the lift is mostly
*reusing that stack against an unknown object dictionary*, and the one real
blocker is that the lift's CANopen node ID / OD indices / encoder scale are
not in any file we have — they can only come from a bench SDO scan once the
torso motor's CAN wire is landed on Controls PCB CH3.

---

## 1. What `docs/deft_vbeta_ref` actually contains (and doesn't)

`docs/deft_vbeta_ref/deft_vbeta/` is Cursor's checkout of the **client-side**
AMR repo (`lerobot`-based). It calls into `feathersdk`, a separate installed
Python package, via plain `import feathersdk...` / `import lib.robot.robot`
— that package's source is **not** in this checkout, not pinned in any
`requirements.txt`/`setup.py` in the tree, and not findable anywhere else on
this machine. It is loaded at runtime from `/feathersys/user_config/conf` on
the Jetson (a path referenced in the client code, itself not vendored). So:

- Everything below the `feather_platform.torso` Python object is a black box
  to us. We only know what the **call sites** reveal.
- There is no EDS file, no object-dictionary listing, no CAN node ID, no
  encoder-scale constant, no schematic for the lift drive anywhere in this
  repo or the reference dump. Contrast this with ZeroErr, where
  `zeroerr-firmware-bringup.md` had a byte-exact EDS to work from — we have
  no equivalent for the lift.

## 2. The lift ("torso") API surface, as used by the client

Grep across `docs/deft_vbeta_ref/deft_vbeta` for every `torso.*` call site
(`feather_platform_client.py`, `deft_mobile.py`, `scripts/torso_lower.py`,
`scripts/estop.py`) gives the complete observed surface:

| Call | Where | Meaning |
|------|-------|---------|
| `torso.healthy_state` | `feather_platform_client.py:82`, `torso_lower.py:29`, `estop.py:82` | bool — calibrated/ready |
| `torso.recalibrate()` | same sites | blocking-ish kickoff of a homing/calibration routine |
| `torso.wait_for_recalibration()` | same sites | blocks until calibration finishes |
| `torso.update_velocity(v: float)` | `feather_platform_client.py:127,150`, `deft_mobile.py:355,357,359`, `estop.py:39` | **mm/s**, +up/-down by sign (see teleop below); `0` = stop |
| `torso.get_state()` | `feather_platform_client.py:160`, `deft_mobile.py:346`, `torso_lower.py:36,57,67` | dict with at least `"height"` (mm) and `"velocity"` (mm/s) keys |
| `torso.go_to(target_mm: float)` | `torso_lower.py:46` | absolute position command, **only used in this one offline utility script** — not in the live teleop/policy hot path |
| `torso.moving` | `torso_lower.py:50,63` | bool |
| `torso.bottom_height_mm` | `torso_lower.py:35` | attribute, presumably paired with a `top_height_mm` we never see referenced |

Velocity sign convention (`deft_mobile.py:352-358`, mirrored in
`yam_ai_mobile.py:441-460`'s `teleop_base_lift`/`send_lift_command`):

```python
if abs(right_y_thumbstick) > self.lift_threshold:
    if right_y_thumbstick > 0:
        self.torso.update_velocity(self.lift_velocity)   # e.g. +60 or +150 mm/s
    else:
        self.torso.update_velocity(-self.lift_velocity)
else:
    self.torso.update_velocity(0)
```

**Only velocity control is used in the live teleop/policy path.** `go_to` /
`bottom_height_mm` only appear in the standalone `scripts/torso_lower.py`
maintenance script (home the lift to its floor before shutdown/transport).
This matters for scoping: the CANopen mode we need *first* is Profile
Velocity (PV), not Profile Position (PP) — absolute homing can come later.

The one and only protocol hint, `torso_lower.py:47-49`:

```python
torso.go_to(target_mm)
# Wait for the background CANopen transaction to start the move.
for _ in range(50):
    if torso.moving:
        break
```

`"CANopen transaction"` is the sole confirmation that the lift drive speaks
CANopen. No node ID, baud rate, or vendor name is ever mentioned in any
comment or string literal in the whole reference tree.

## 3. Current stub, in our code (what actually needs to change)

| File | What's there today |
|------|--------------------|
| `scripts/deft_controls_sdk/vbeta/platform.py:56,123-126` | `PcbPlatformClient` tracks `self._lift_vel_cmd` on `lift_cmd` but **never enqueues an `ActuatorDesire`** — comment: `"Stub: accept mm/s, do not enqueue plant."` |
| `scripts/deft_controls_sdk/vbeta/platform.py:241-243` | `get_state()` hardcodes `lift_velocity: 0.0`, `lift_height: 0.0`, `lift_unimplemented: 1.0` |
| `scripts/deft_controls_sdk/vbeta/slots.py:21,70-71` | `LIFT_SLOT = 20` reserved; `yam_product_rows()` emits `(bus=3, enabled=False, PROTO_NONE, 0, 0)` for it |
| `docs/vbeta-pcb-adapter.md` "Lift (stub)" section | Documents the above as intentional, "Bring up later via FeatherSDK (likely CANopen)" |
| Firmware CFG (`plant_config_nvm.c` factory defaults / CFG SET) | Slot 20 disabled — no SPI/FDCAN traffic for it, by design (`ch4-mcp2518-bringup-postmortem.md`-style "don't spam a disabled slot") |

None of this is broken — it's the deliberate stub the vbeta plan called for.
De-stubbing means: wire slot 20 to a real protocol handler, flip it enabled
in the product CFG, and replace the three no-ops above with real
`ActuatorDesire`/feedback plumbing.

## 4. What we already have to build on

We are **not** starting from zero on the firmware side. `App/Src/plant/can/canopen.c`
+ `App/Inc/plant/can/canopen.h` are a working, bus-agnostic CiA-301 master
(NMT send, SYNC send, expedited SDO read/write u8/u16/u32, PDO1 pack/parse
for a `controlword + int32 position/velocity` mapping) — built for ZeroErr
but written generically (`canopen_sdo_write_u32(bus, node, index, sub, ...)`
takes no ZeroErr-specific assumption). `App/Src/plant/plugins/zeroerr.c` is
a full worked example of *using* that layer for a real CiA-402 drive:

- A boot state machine (`zeroerr_boot_step`, ~20 phases) that does
  NMT stop → reset comm → pre-op → set mode-of-operation via SDO → remap
  TxPDO1/RxPDO1 via SDO (disable → clear → map two objects → set count →
  set transmission type → re-enable) → NMT start → CiA-402 enable sequence
  (shutdown → switch-on → enable-operation controlwords).
- A steady-state `apply_cycle` that packs `controlword + target` into the
  mapped RxPDO1 every plant tick once `ZE_PHASE_OPERATIONAL`, with the
  "new setpoint" rising-edge-on-bit4 handshake CiA-402 profile-position mode
  requires.
- `actuator.c` already special-cases `PROTO_ZEROERR` in the three places a
  new lift protocol would also need it: `actuator_apply_desire`
  (`App/Src/plant/actuator.c:270-274`), `actuator_dispatch_bus_rx`
  (`:181-185`), and `plant_recovery_all` (`:105-109`).
- `App/Inc/plant/actuator.h:8-13` has the `protocol_t` enum
  (`PROTO_NONE/ROBSTRIDE/CUBEMARS/DAMIAO/ZEROERR/COUNT`) and
  `App/Src/plant/plugin_schema/plugin_table.c` the generic dispatch table —
  both are exactly where a new `PROTO_*` for the lift gets registered.
- `docs/zeroerr-firmware-bringup.md` §1-2 is a template for the "confirmed
  vs. must-discover-on-bench" bus/identity table below — mirror its
  structure once real numbers exist.

This means the lift plugin is realistically a **new `.c` file shaped like
`zeroerr.c`**, not new architecture — assuming the real drive is CiA-402
compliant (a safe bet: essentially every commercial CANopen linear-actuator/
servo-drive controller is), with **Profile Velocity mode** as the first
target instead of ZeroErr's Profile Position, since that's the only mode the
live teleop path actually uses (§2 above).

## 5. What is genuinely unknown (bench-only, cannot be resolved from docs)

Everything the ZeroErr doc got for free from the EDS file, we don't have for
the lift:

| Unknown | Why it matters | How to get it |
|---------|-----------------|----------------|
| CANopen node ID | Every SDO/PDO COB-ID is `base + node` | SDO/NMT scan across `1..127` on CH3 once wired, same pattern as `legacy/damiao_scan.py --discover` |
| Baud rate | ZeroErr's EDS only supports 1 Mbps; lift drive may differ | Scope the bus or sweep bitrates during discovery |
| Object dictionary (mode-of-operation index, PDO mapping objects, identity `0x1018`) | Needed to remap PDO1 the way `zeroerr_boot_step` does | SDO read `0x1000`/`0x1018` (generic CiA-301, safe to probe); vendor OD beyond that needs either a datasheet/EDS from the lift drive's manufacturer or systematic SDO probing |
| Encoder / gearing scale (counts ↔ mm) | `zeroerr_rad_to_counts` has a hardcoded `ZEROERR_ENCODER_RES` for *that* drive; the lift's is unrelated | Measure on the bench: command a known velocity for a known time, compare commanded vs. `get_state()`-reported height delta (once we can even read it — chicken/egg, so start by reading raw encoder counts via SDO before trusting any scale) |
| `bottom_height_mm` / (presumed) `top_height_mm` | Needed for soft limits — this is a mechanical lift with end-of-travel, not a free-spinning joint | Either read back from the real drive's limit-switch/homing behavior, or measure by hand and hardcode like YAM arm soft limits in `bringup.md` §2 |
| What `recalibrate()` physically does | If it's "drive to a hard stop to zero the encoder," our firmware needs an equivalent homing routine; if it's just "clear faults," it's much simpler | Watch the CAN bus (or the Feather board's own firmware if source is ever available) during a real `torso.recalibrate()` call |
| Direction sign at the wire level | The Python-level `+velocity = up` convention doesn't guarantee the CANopen target-velocity sign matches without a first bench test | Confirm during first-motion bench test, in a rig where the lift is unloaded and free to move a few mm safely |

None of these can be answered from documentation research alone — the
current step, correctly, was FeatherSDK research; the *next* step is
physical: land the lift drive's CANopen wire on Controls PCB CH3 (FDCAN2,
per `bringup.md` §2's schematic mapping) and run a discovery pass.

## 6. De-stub plan (ordered)

### Phase 0 — bench discovery (blocking, hardware required)

1. Wire the lift drive's CAN H/L (+ GND, + power per its own supply
   requirements) to Controls PCB CH3.
2. Node/baud scan: adapt `legacy/damiao_scan.py --discover --host-only`'s
   sweep pattern (or a new small script using `canopen_sdo_read_u32(..., 0x1000, ...)`)
   across candidate node IDs at 1 Mbps first (matches every other bus on
   this board), falling back to other standard CANopen baud rates if silent.
3. Once a node responds: read `0x1000` (device type) and `0x1018`
   (identity: vendor/product/revision — same read `zeroerr_read_identity()`
   already does generically) to confirm CiA-402 and get a vendor lead. If
   the vendor is identifiable, look up their EDS/datasheet — that would
   immediately fill in most of §5's unknowns the way ZeroErr's EDS did.
4. If no EDS is obtainable, probe the standard CiA-402 objects directly:
   `0x6060` (mode of operation), `0x606C` (actual velocity), `0x6064`
   (actual position), `0x60FF` (target velocity), `0x607A` (target
   position), `0x6040`/`0x6041` (controlword/statusword) — these indices are
   part of the CiA-402 spec itself, not vendor-specific, so they're a
   reasonable first guess even blind.
5. Determine counts↔mm scale and travel limits per §5.

### Phase 1 — firmware plugin (mirrors `zeroerr.c`)

6. Add `PROTO_LIFT` (name TBD) to `protocol_t` in `App/Inc/plant/actuator.h`
   and register it in `App/Src/plant/plugin_schema/plugin_table.c`.
7. New `App/Src/plant/plugins/lift_torso.c` + header: boot/enable state
   machine shaped like `zeroerr_boot_step`, but targeting **Profile
   Velocity** mode (CiA-402 mode `0x03`) instead of Profile Position, PDO1
   mapped to `controlword + target_velocity` (cmd) / `statusword + actual_velocity`
   (or `actual_position`, if we want height feedback over PDO too — likely
   want both, which may need a second PDO pair depending on the real drive's
   max mapped-object count).
8. Wire the three `actuator.c` call sites the same way `PROTO_ZEROERR` is
   wired today (`actuator_apply_desire`, `actuator_dispatch_bus_rx`,
   `plant_recovery_all`).
9. Enable slot 20 in the product CFG: `yam_product_rows()` in
   `scripts/deft_controls_sdk/vbeta/slots.py` — flip `enabled=False` to
   `True`, `PROTO_NONE` to the new protocol, `bus=3`, `motor_id=<discovered node id>`.

### Phase 2 — host API (`PcbPlatformClient`)

10. `platform.py`'s `lift_cmd` branch (~line 123): replace the
    `self._lift_vel_cmd = float(...)` no-op with an `ActuatorDesire`
    enqueued to `LIFT_SLOT` via `self._session.set_actuators(...)` — velocity
    converted from FeatherSDK's mm/s to whatever unit the firmware plugin
    expects (rad/s-equivalent counts/s, matching the arm/base pattern of
    keeping SI-ish units at the plant boundary and converting in the host
    layer, consistent with `deft_utils.py`'s existing
    `to_policy_lift_action`/`from_policy_lift_action` mm↔m conversions).
11. `get_state()` (~line 231): read slot 20's `actuator_state_live` off the
    feedback image (same pattern as the existing `bwc_angle`/`bpc_velocity`
    reads just above it) and convert back to mm/mm-s; drop
    `lift_unimplemented` once real feedback is flowing (keep it as a
    transitional flag if Phase 0/1 lands before Phase 2, so partial rollout
    is still honest about what's real).
12. Decide the mm↔plant-unit scale and where it lives (probably a constant
    in `slots.py` next to `DEFAULT_ARM_KP` etc.), once Phase 0 measures it.

### Phase 3 — docs + smoke

13. Update `vbeta-pcb-adapter.md`'s "Lift (stub)" section to describe the
    real wire-up once shipped; drop the "no-op" language.
14. Add `scripts/vbeta_lift_smoke.py` alongside the existing
    `vbeta_smoke.py arm`/`vbeta_smoke.py base`/`vbeta_smoke.py neck` —
    same exclusive-COM pattern, small creep-and-back motion.
15. Extend `scripts/tests/test_deft_controls_sdk_vbeta.py`'s fake-hub
    contract tests to cover the lift path once it does something other than
    return the stub dict.

## 7. Risk callouts for whoever does Phase 0

- **This is a load-bearing mechanical actuator holding up a torso** — unlike
  arm/base bring-up, a wrong velocity-mode sign or a runaway command can
  drive it into a hard stop. Do the first bench test **unloaded** and with a
  hand on a physical E-stop / power cutoff, not just a software one — the
  PDB hard-ESTOP wire (`docs/pdb-uart-v1.md`) doesn't exist as delivered
  hardware yet either, so don't assume it as a safety net during this
  specific bring-up.
- Keep slot 20 CFG-disabled until Phase 0 is far enough along to have a
  concrete node ID and sign convention in hand — an enabled slot with the
  wrong node ID will silently address some *other* node on CH3 (there
  shouldn't be one there today, but don't assume).
