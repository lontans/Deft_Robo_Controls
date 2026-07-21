# 2026-07-10 workstreams — CubeMars plugin (+ ZeroErr reserved slot)

Everything under this folder mirrors the real `App/`/`Core/` paths but lives **outside**
the live STM32 build tree on purpose — the actual build stays untouched while other work
(dual-arm Damiao teleop, the host SDK) is active concurrently. Nothing here has been
compiled or flashed. Review and merge by hand (or point Cursor at this folder) when a
flasher is available.

**2026-07-20 update:** re-audited against the current live tree. The thermocouple/MAX31855
half of this folder is now **superseded** — the live tree already has a real, different,
working implementation. Only the **CubeMars** files are still unmerged and relevant. Added
a **ZeroErr** reserved-slot scaffold (no vendor doc exists for it — see below, do not
expect a working plugin).

## CubeMars — still unmerged, ready to review (small)

Turns out to be a smaller merge than originally scoped: CubeMars is pure CAN (no PDU tag,
no SPI, no dispatch change) — the only things below are what's actually needed.

**New files** (don't exist in the live tree, can be copied in as-is):
- `App/Inc/plant/plugins/cubemars.h`, `App/Src/plant/plugins/cubemars.c`

**Duplicated-and-edited** (real file copied here, then modified — diff against the live
version at the matching path before merging):
- `App/Src/plant/plugin_schema/plugin_table.c` — registers `cubemars_ops` for
  `PROTO_CUBEMARS` (was `NULL` live) **and** `zeroerr_ops` for the new `PROTO_ZEROERR`
  reserved slot (see ZeroErr section below)

That's it for CubeMars integration plumbing. `PROTO_CUBEMARS` already exists in the live
`protocol_t` enum; the generic plugin dispatch path (`plugin_pack_tx`/`plugin_parse_rx`)
already existed; a CubeMars slot gets assigned at runtime via CFG SET
(`Protocol.CUBEMARS=2`) once a motor is available, not a factory-default table edit.
`plant_command.c`, `plant_feedback.c`, `app.c`, and the SPI4/thermo files that used to be
listed here are **not** part of the CubeMars merge — see below.

### Before merging CubeMars — resolve these first (not guessed, left as explicit placeholders)

1. **CubeMars feedback CAN ID** (`cubemars.c`'s `parse_rx`) — undocumented in the vendor
   PDF. Wired via `actuator_config_t.master_id`, default 0 = inert/no-op. Bench step: CAN
   sniff with the motor in servo mode, no host TX, observe the periodic broadcast ID, then
   set it at runtime via CFG SET (no firmware redeploy needed).
2. **CubeMars electrical-vs-mechanical unit scaling** — `pack_tx`/`parse_rx` currently pass
   `ActuatorDesire`'s rad/rad·s⁻¹ straight through as placeholder wire units. Needs the
   motor's pole-pair count to verify. **Do not command a loaded joint with this until
   confirmed** — see the P0 comment block in `cubemars.h`.

(The old placeholders #3-#5 about SPI instance/pins/CPOL were about the thermocouple, not
CubeMars — moot now, see below.)

## Thermocouple / MAX31855 / SPI4 — SUPERSEDED, do not merge

The live tree already resolved this differently while this draft sat unmerged:

| This workstream assumed | Live tree actually did |
|---|---|
| Dedicated **SPI4**, half-duplex 1-line (`hspi4`) | Shares **SPI3** with SK9822 via exclusive `spi3_role` arbitration — no new SPI peripheral |
| CS on `GPIOE` pin 4 (placeholder, unconfirmed) | CS on **PB7** |
| `HAL_SPI_Receive` (1-line, MOSI-as-RX) | Full-duplex `HAL_SPI_TransmitReceive` with dummy TX bytes |
| New `plant_command.c`/`plant_feedback.c`/`app.c` dispatch | Already live and working (`thermo_is_command`/`thermo_on_command`/`thermo_feedback_fill`/`thermo_init`/`thermo_service` all wired end to end) |

Verified by diffing every file below against its live counterpart on 2026-07-20; each now
carries a `SUPERSEDED — do not merge` banner pointing back here:
`App/Inc/plant/thermo.h`, `App/Src/plant/thermo.c`, `App/Inc/plant/plugins/max31855.h`,
`App/Src/plant/plugins/max31855.c`, `Core/Src/spi.c`, `Core/Inc/spi.h`.

**Do not copy these into the live tree.** Kept in place only as historical reference for
*why* SPI3-shared-role was chosen over a dedicated SPI4 (the live version's comments
explain the real constraint: the breakout has no separate MISO, and the live fix was to
keep SPI3 full-duplex with dummy TX bytes plus exclusive role arbitration with the LED
strip, rather than reconfiguring a whole second peripheral for half-duplex).

## ZeroErr — reserved slot only, not a working plugin

Added ahead of any real ZeroErr work so the generic dispatch path has somewhere safe to
land: `PROTO_ZEROERR` in a duplicated `App/Inc/plant/actuator.h`, `zeroerr_ops` in
`plugin_table.c`, and new `App/Inc/plant/plugins/zeroerr.h` + `App/Src/plant/plugins/zeroerr.c`.

**There is no ZeroErr vendor protocol document anywhere in this repo** (`External_Documentation/`
has CubeMars/Damiao/Dynamixel/RobStride/SK9822/MCP2518FD — no ZeroErr folder). Both
`zeroerr_pack_tx`/`zeroerr_parse_rx` unconditionally return `PLUGIN_ERR_UNSUPPORTED` — this
is intentional, not a stub to quietly fill in. See `zeroerr.h`'s comment block for what's
actually blocking real work:

1. Which ZeroErr product line / firmware — their catalog isn't necessarily one uniform
   interface.
2. Whether it's a raw single-frame protocol (same shape as RobStride/Damiao/CubeMars,
   "position/kp/kd packed into 8 bytes of one CAN frame") or **CANopen** (CiA 402 profile).
   If CANopen, the `plugin_ops_t` pack_tx/parse_rx shape used by every other plugin here is
   the **wrong** integration point — CANopen needs SDO/PDO mapping and an NMT state
   machine, which is a real design pass touching `can_router.c`'s RX dispatch, not "fill in
   two functions." Don't force it through this shape without checking first.

Net effect of merging just the reserved slot (independent of CubeMars): `PROTO_ZEROERR`
becomes selectable via CFG SET without crashing or misbehaving — it just cleanly does
nothing, same as an empty slot, until a real vendor doc shows up.

## Python side (not duplicated — additive, doesn't touch the STM32 build)

Went straight into `scripts/` as normal, already committed to the working tree, tests pass:
- `scripts/controls_pcb_host/protocol/cubemars.py`, `.../protocol/thermo.py`
- `scripts/controls_pcb_host/commands.py` — `build_thermo_probe_command()`
- `scripts/controls_pcb_host/feedback.py` — `parse_thermo_feedback()`
- `scripts/thermocouple_read.py` — bench CLI (works once the firmware side is merged/flashed —
  note this now talks to the *live* SPI3-based thermo implementation, not this folder's)
- `scripts/tests/test_cubemars_wire.py`, `scripts/tests/test_thermo_wire.py`

`test_max31855_decode_bit_layout_*` in `test_thermo_wire.py` is a genuine hardware-independent
correctness test (MAX31855's bit format is a fixed datasheet fact) — the CubeMars tests are
internal-consistency-only, since nothing about that protocol is verifiable without the motor.
No Python-side changes needed for the ZeroErr reserved slot — there's nothing to talk to yet.
