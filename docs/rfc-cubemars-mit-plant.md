# RFC: CubeMars plant plugin → MIT Power Mode (std-ID)

Status: proposal drafted — SDK wire + tests landed; App patch is proposal-only
(`git apply --check` verified). Offline Agent1 — no build/flash/COM5.
Executor: Cursor (owns `App/`/`Core/`, COM5, soft-DFU).
Patch: [`docs/patches/cubemars-mit-plant.patch`](patches/cubemars-mit-plant.patch).
Proposed full sources: [`docs/patches/cubemars_mit_proposed/`](patches/cubemars_mit_proposed/).

## Problem

[`App/Src/plant/plugins/cubemars.c`](../App/Src/plant/plugins/cubemars.c) today
implements **Servo Mode Position-Speed Loop (control mode 6)** over **extended
29-bit IDs**. That path:

- does not match the MIT impedance surface that `ActuatorDesire` /
  `PcbArmDriver` / i2rt already speak (pos/vel/kp/kd/torque),
- has no enable/disable/zero lifecycle (Servo Mode has none documented),
- and cannot share a bus cleanly with Damiao-style std-ID MIT tooling.

YAM product arms stay on Damiao. This RFC does **not** flip default YAM CFG.

## Target: MIT Power Mode (PDF §5.3)

Source of truth:
[`External_Documentation/CubeMars/cubemars_motor_driver_doc.pdf`](../External_Documentation/CubeMars/cubemars_motor_driver_doc.pdf)
**§5.3 MIT power mode communication protocol** (pp. 43–47).

| Item | Spec |
|---|---|
| Frame type | **Standard** 11-bit ID |
| TX ID | motor / Drive ID (default 1) |
| Enable | `{0xFF×7, 0xFC}` on TX ID — “enter motor control mode” |
| Disable | `{0xFF×7, 0xFD}` |
| Zero | `{0xFF×7, 0xFE}` — set current position to 0 |
| MIT DLC | 8 |
| MIT layout | identical bit packing to Damiao MIT (`damiao_pack_tx`) |
| Kp / Kd | **0–500** / **0–5** (12-bit each) |
| RX ID | `0x00 + Drive ID` (PDF) |
| RX payload | `D[0]=id`, `D[1..2]=p(16)`, `D[3..5]=v(12)\|t(12)`, `D[6]=temp`, `D[7]=err` |

Host / plant desire surface stays the existing MIT tuple — same shape as
i2rt / `PcbArmDriver` / `ActuatorDesire(position, velocity, kp, kd, torque)`.

## Parallels to `damiao.c` (do not reinvent)

| CubeMars MIT | Damiao analogue |
|---|---|
| `cubemars_apply_cycle` | `damiao_apply_cycle` — latch enable, then one MIT/tick |
| `cubemars_pack_mit` (static, feeds `cubemars_apply_cycle` + `cubemars_ops.pack_tx`) | `damiao_pack_tx` — same `float_to_uint` + byte layout |
| `cubemars_on_rx_frame` / `cubemars_parse_mit` (static, feeds `cubemars_ops.parse_rx`) | `damiao_parse_rx` — same p/v/t unpack; id/err byte differs |
| enable `0xFC` / disable `0xFD` / zero `0xFE` | same opcodes; Damiao also has `0xFB` clear-fault (CubeMars PDF does **not** document FB — do not invent it) |
| actuator.c apply / RX / recovery branches | mirror Damiao (see below) |

Damiao enable latch today waits for feedback `ERR==1` after clear+enable.
CubeMars PDF only requires "enter mode" (`0xFC`) before MIT, and documents no
fault/status semantics for MIT mode at all — so the implemented latch is
**TX-driven, not RX-gated**: send `0xFC` once per session (first call after
`cubemars_reset_enable_latch()`), then stream one MIT frame per tick forever
after, idle or not. This is deliberately **more** Damiao-like than the
original sketch above ("latch on first RX match") — like
`damiao_apply_cycle`, `cubemars_apply_cycle` never sends the exit-mode
(`0xFD`) frame itself; that only happens in `plant_recovery_all()`. A blank
desire (`kp=kd=0`) is just a legitimate MIT zero-effort command once
entered, not a reason to leave MIT mode — matching the arm-class framing in
§1 (stay ready, don't re-enter mode on every command gap). Gating the latch
on RX would require inventing a fault bit the PDF doesn't document, which
the "PDF sample bugs" section below and the RFC's own non-goals already say
not to do.

## Per-AK MIT limits (PDF table, §5.3)

Position is **±12.5 rad** for every listed module. Kp/Kd ranges are shared.
Speed / torque differ per AK:

| Model | V (rad/s) | T (N·m) |
|---|---:|---:|
| AK10-9 | ±50 | ±65 |
| AK60-6 | ±50 | ±15 |
| AK70-10 | ±50 | ±25 |
| AK80-6 | ±76 | ±12 |
| AK80-9 | ±50 | ±18 |
| AK80-80 | ±8 | ±144 |

Default model in the proposal: **AK80-9** (common arm-class). Per-slot override
via `cubemars_set_model(slot, model)` — not via YAM product CFG.

**Firmware plumbing for `cubemars_set_model` (this patch's actual scope):**
`actuator_config_t` has no spare per-slot field for a model index today —
`master_id` is already spoken for elsewhere (RX-ID override, mirrored from
Damiao's feedback-ID use, kept as-is rather than double-purposed). Rather
than invent a new CFG wire field to select a model host-side in this same
patch, the proposal matches Damiao's own current maturity level: one
compiled-in default (`CUBEMARS_MIT_DEFAULT_MODEL = AK80-9`, matching the SDK's
`DEFAULT_MODEL`) applied to every `PROTO_CUBEMARS` slot, exactly as
`damiao_limits()` already hardcodes DM4310 for every Damiao slot regardless
of `cfg` (`(void)cfg;` in that function today — no per-slot Damiao model
selection exists either). `cubemars_set_model(slot, model)` is written and
exported so a later CFG/diag hook can call it, but nothing in this patch
calls it yet — same "scaffold now, wire later" shape as the rest of this
sprint's secondary work. Do not build the CFG plumbing speculatively; note
it as a named follow-up instead.

## PDF sample bugs (do not copy)

The §5.3 “Sends routine code” / “Receive routine code” samples have several
defects. Plant + SDK must follow the **bit-field tables** and the known-good
Damiao packing, not the broken sample:

1. **`data[6]` packs `kp_int>>8` instead of `t_int>>8`.** Comment says
   “torque 4 bit higher”; code reuses KP. Correct (Damiao):
   `((kd & 0xF) << 4) | ((t >> 8) & 0xF)`, then `data[7] = t & 0xFF`.
2. **`float_to_uint` uses `(1<<bits)` while `uint_to_float` uses
   `(1<<bits)-1`.** Asymmetric; Damiao uses `(1<<bits)-1` both ways.
3. **Sample clamp limits (`P±95.5`, `V±30`, `T±18`) contradict the per-AK
   table** (all models `P±12.5`, model-specific V/T). Do not use the sample
   clamps.
4. **RX DLC text says 6 bytes** but the field table includes `DATA[6]` temp
   and `DATA[7]` error (8 bytes). Accept `dlc >= 6` for motion; prefer 8.
5. **`unpack_reply` uses undefined `I_MAX` / `P_MIN`…** — use the same
   per-model T limits as TX.
6. **Identifier vs “Current value” naming** — TX table says “Current”; pack
   API is torque feed-forward (`t_ff`). Treat as torque (Nm), same as Damiao.

Version history in the PDF (Ver 1.0.1–1.0.6) repeatedly “Correct the Can code
of 5.3” — treat samples as untrusted; tables + Damiao as trusted.

## Servo Mode (mode 6) disposition

Keep Servo Mode helpers behind `#if CUBEMARS_ENABLE_SERVO_MODE` (default **0**)
for reference / future diagnostic probes. **Hot plant path is MIT only** —
`cubemars_ops.pack_tx` / `parse_rx` are the MIT implementations.
Do not call Servo Mode from `actuator_apply_desire`.

## `actuator.c` branches required

Mirror Damiao's shape — CubeMars MIT is not a stateless single-frame plugin —
but **not** Damiao's RX-gated latch-clearing (see previous section: no
documented fault bit to gate on):

1. **`actuator_apply_desire`**: `PROTO_CUBEMARS` → `cubemars_apply_cycle`
   (not generic `plugin_pack_tx`), same call shape as the existing
   `PROTO_DAMIAO`/`PROTO_ZEROERR` branches.
2. **Blank-idle FDCAN skip exemption**: treat `PROTO_CUBEMARS` like
   `PROTO_DAMIAO` in the "blank FDCAN bus, nothing else commanded" skip —
   `cubemars_apply_cycle` must run every tick regardless of blank so the
   continuous MIT stream + one-time enable latch actually execute (same
   reason Damiao is exempted there today).
3. **`actuator_dispatch_bus_rx`**: **direct** call to `cubemars_on_rx_frame`
   per matching slot, immediately (no deferred `had_rx` array). Damiao's
   deferred dispatch exists specifically to avoid clearing its enable latch
   on a same-bus frame that isn't really a match; CubeMars's latch is
   TX-driven only (§ above) and never reacts to RX content, so there is no
   equivalent daisy-chain hazard to defer against — this is a real,
   intentional simplification versus Damiao, not an oversight.
4. **`plant_recovery_all`**: `cubemars_reset_enable_latch(i)` +
   `cubemars_send_disable()` (`0xFD`), enqueued the same way as the existing
   `PROTO_DAMIAO` branch there.

## Explicit non-goals

- **Do NOT flip default YAM Damiao CFG** (`yam_product_rows()` stays PROTO_DAMIAO).
- Optional host scaffold only: `cubemars_yam_rows()` in
  `scripts/deft_controls_sdk/vbeta/slots.py` + `PROTO_CUBEMARS = 2` — never
  wired into `ensure_yam_product_cfg` / smoke defaults.
- No soft-DFU / flash / COM5 from this agent.
- No expansion of `scripts/legacy/controls_pcb_host/protocol/cubemars.py`
  (Servo Mode reference only).
- No CubeMars `0xFB` clear-fault (undocumented).

## SDK deliverables (this agent)

| Path | Role |
|---|---|
| `scripts/deft_controls_sdk/protocol/cubemars_mit.py` | pack/unpack MIT + enable/disable/zero |
| `scripts/tests/test_cubemars_mit_wire.py` | golden vectors (incl. Damiao-layout parity + PDF bug regression) |
| `slots.cubemars_yam_rows()` | non-default CFG helper |

## How to apply (executor)

```powershell
git apply --check docs/patches/cubemars-mit-plant.patch
git apply docs/patches/cubemars-mit-plant.patch
```

Then rebuild / soft-DFU as usual. Bench: assign one CH with
`protocol=PROTO_CUBEMARS`, AK model matching the motor, enable, stream
`ActuatorDesire` MIT — same host path as Damiao arms.

## Matrix / bring-up checklist (executor)

- [ ] `git apply --check` clean on tree that already has RX-index + RobStride stagger
- [ ] Unit: `pytest scripts/tests/test_cubemars_mit_wire.py`
- [ ] Single CubeMars on spare bus: enable latch → MIT hold → disable on recovery
- [ ] Confirm YAM product CFG still Damiao after any host script changes
- [ ] Sniff: TX std-ID = motor_id; RX std-ID = motor_id (`0x00+Drive`); no ext-ID Servo frames on the hot path
