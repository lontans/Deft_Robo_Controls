# 2026-07-10 workstreams — CubeMars plugin + MAX31855 thermocouple

Everything under this folder mirrors the real `App/`/`Core/` paths but lives **outside**
the live STM32 build tree on purpose — the actual build was left completely untouched
while Damiao teleop testing was active in Cursor concurrently. Nothing here has been
compiled or flashed. Review and merge by hand (or point Cursor at this folder) when ready.

Workstream 2 (`controls_hub_controller` config surface) was intentionally **not** started —
skipped per instruction, revisit later.

## What's new vs. duplicated-and-edited

**New files** (don't exist in the live tree, can be copied in as-is):
- `App/Inc/plant/plugins/cubemars.h`, `App/Src/plant/plugins/cubemars.c`
- `App/Inc/plant/plugins/max31855.h`, `App/Src/plant/plugins/max31855.c`
- `App/Inc/plant/thermo.h`, `App/Src/plant/thermo.c`

**Duplicated-and-edited** (real file copied here, then modified — diff against the live
version at the matching path before merging):
- `App/Src/plant/plugin_schema/plugin_table.c` — registers `cubemars_ops` for `PROTO_CUBEMARS` (was `NULL`)
- `App/Src/plant/plant_command.c` — dispatches the new `TMP` PDU tag to `thermo_on_command()`
- `App/Src/plant/plant_feedback.c` — calls `thermo_feedback_fill()`; adds `'t'` to the tag-preservation whitelist (**this one's load-bearing** — without it, `servo_diag_feedback_fill()` silently stomps the thermo reply every cycle it isn't the only tag; verified against the live file before writing this)
- `App/Src/app.c` — adds `thermo_init()` to `app_init()`, `thermo_service()` to `app_run()`
- `Core/Src/spi.c`, `Core/Inc/spi.h` — adds `MX_SPI4_Init()` (half-duplex 1-line) + `hspi4`

None of the actuator.c / plant_config.c files needed edits — `PROTO_CUBEMARS` and the
generic plugin dispatch path already existed; a CubeMars slot gets assigned at runtime via
the CFG SET path (`Protocol.CUBEMARS=2`) once a motor is available, not a factory-default
table edit.

## Before merging — resolve these first (not guessed, left as explicit placeholders)

1. **CubeMars feedback CAN ID** (`cubemars.c`'s `parse_rx`) — undocumented in the vendor
   PDF. Wired via `actuator_config_t.master_id`, default 0 = inert/no-op. Bench step: CAN
   sniff with the motor in servo mode, no host TX, observe the periodic broadcast ID, then
   set it at runtime via CFG SET (no firmware redeploy needed).
2. **CubeMars electrical-vs-mechanical unit scaling** — `pack_tx`/`parse_rx` currently pass
   `ActuatorDesire`'s rad/rad·s⁻¹ straight through as placeholder wire units. Needs the
   motor's pole-pair count to verify. **Do not command a loaded joint with this until
   confirmed** — see the P0 comment block in `cubemars.h`.
3. **Thermocouple SPI instance** — this assumes **SPI4** (SPI2 was the other free option).
   If the breakout is actually on SPI2, swap `Core/Src/spi.c`/`spi.h` and the `&hspi4`
   reference in `thermo.c` accordingly.
4. **Thermocouple SCK/CS pin numbers** — `THERMO_SPI4_SCK_PIN`/`THERMO_SPI4_MOSI_PIN` in
   `spi.c` and `THERMO_CS_GPIO_Port`/`THERMO_CS_Pin` in `thermo.h` are placeholders
   (`GPIOE`/pins 2, 4, 6), not verified against the actual harness wiring.
5. **MAX31855 CPOL/CPHA** — assumed Mode 0 (matches the common convention) but not
   double-checked against the MW BA017's exact datasheet page.

## Python side (not duplicated — additive, doesn't touch the STM32 build)

Went straight into `scripts/` as normal, already committed to the working tree, tests pass:
- `scripts/controls_pcb_host/protocol/cubemars.py`, `.../protocol/thermo.py`
- `scripts/controls_pcb_host/commands.py` — `build_thermo_probe_command()`
- `scripts/controls_pcb_host/feedback.py` — `parse_thermo_feedback()`
- `scripts/thermocouple_read.py` — bench CLI (works once the firmware side is merged/flashed)
- `scripts/tests/test_cubemars_wire.py`, `scripts/tests/test_thermo_wire.py` — `python -m pytest scripts/tests/` → 59/59 passing

`test_max31855_decode_bit_layout_*` in `test_thermo_wire.py` is a genuine hardware-independent
correctness test (MAX31855's bit format is a fixed datasheet fact) — the CubeMars tests are
internal-consistency-only, since nothing about that protocol is verifiable without the motor.
