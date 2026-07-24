# RFC: SK9822 factory / traffic-light LED modes (5-bit enum only)

Status: SDK + pack tests landed; firmware patch drafted under
[`docs/patches/led-factory-patterns.patch`](patches/led-factory-patterns.patch)
(`git apply --check` clean). Author: Agent 2 (offline) — no soft-DFU / COM5 /
flash. Executor: Cursor owns `App/` apply + board prove.

## Constraint (no layout bump)

`host_led_command_t` stays **2 B** inside the **672 B** command image:

| Field | Bits | Notes |
|-------|-----:|-------|
| `mode` | 5 | 0..31; this RFC only assigns new codes in that space |
| `master_brightness` | 5 | 0..31 → SK9822 global brightness (unchanged wire) |
| `led_count` | 6 | 0 ⇒ firmware `LED_STRIP_MAX` (300) |

No new fields, no `IMAGE_BYTES` change, no schema version bump.

## Mode enum (extend only; keep 0/1/2)

| Code | Name | Meaning |
|-----:|------|---------|
| 0 | `OFF` | Blank strip (existing) |
| 1 | `TEST` | Single-pixel chase / snake (existing — **do not duplicate**) |
| 2 | `FLASH` | Full-strip ~2 Hz red blink (existing bringup flash) |
| 3 | `SOLID_GREEN` | Factory OK / ready |
| 4 | `SOLID_YELLOW` | Factory attention / hold |
| 5 | `SOLID_RED` | Factory fail / blocked (steady) |
| 6 | `BLINK_YELLOW_SLOW` | Caution (slow yellow blink) |
| 7 | `BLINK_RED_FAST` | E-stop / fault attention (fast red blink) |
| 8 | `IDLE_CORNFLOWER` | Idle: cornflower `#6495ED` (100,149,237); 500 on / 500 off (1 Hz 50%) |
| 9..31 | — | Reserved; MCU treats as blank (same as unknown today) |

Factory / traffic-light patterns are **solid or blink fills** of one SK9822
band. They intentionally do **not** re-implement the TEST chase.

`FLASH` (2) stays the bringup/debug blink. `BLINK_RED_FAST` (7) is the
operator-facing fault cadence (faster); hosts that already send `mode=2`
keep working without reinterpretation.

## Timing / RGB table

Animator uses `HAL_GetTick()`. Solids TX once on mode/brightness/count change
(or first entry); blinks TX on phase edge only (same SPI thrift as `FLASH`).

| Mode | `period_ms` | Duty (on) | RGB (on) | RGB (off) | `master_brightness` |
|------|------------:|----------:|----------|-----------|---------------------|
| `OFF` | — | 0% | 0,0,0 | — | Ignored (TX brightness 0) |
| `TEST` | 50 (step) | 1 LED | 255,0,0 on lit pixel | 0,0,0 | Passed to `sk9822_transmit*` |
| `FLASH` | 500 (250/250) | 50% | 255,0,0 | 0,0,0 | Passed through |
| `SOLID_GREEN` | — (edge) | 100% | 0,255,0 | — | Passed through |
| `SOLID_YELLOW` | — (edge) | 100% | 255,180,0 | — | Passed through |
| `SOLID_RED` | — (edge) | 100% | 255,0,0 | — | Passed through |
| `BLINK_YELLOW_SLOW` | 1000 (500/500) | 50% | 255,180,0 | 0,0,0 | Passed through |
| `BLINK_RED_FAST` | 200 (100/100) | 50% | 255,0,0 | 0,0,0 | Passed through |

**Brightness field behavior:** pattern RGB is full-scale channel values; the
5-bit `master_brightness` is the SK9822 global brightness nibble passed
unchanged into `sk9822_transmit` / `sk9822_transmit_blocking`. Hosts clamp
0..31. `led_count=0` ⇒ full configured chain (same as today).

## Host API

```python
from deft_controls_sdk.link import LedDesire, LED_MODE_SOLID_GREEN

LedDesire(mode=LED_MODE_SOLID_GREEN, master_brightness=8, led_count=0)
# or: LedDesire(mode=3, master_brightness=8, led_count=0)
```

Named constants live next to `LedDesire` / pack (`LED_MODE_*`). Thin helpers
in `scripts/deft_controls_sdk/vbeta/leds.py` (`led_solid_green`, …). Pack path
remains `patch_led_command` into the existing 2 B at `LED_CMD_OFF` (606).

## Future MCU (`led.c`)

1. Extend the 5-bit mode `#define`s (3..7).
2. Add a small mode table + animator in `led_apply_mode` / `led_service`
   (solid fill vs phase-edge blink).
3. **`sk9822_transmit` / `sk9822_transmit_blocking` unchanged** — only pixel
   fill + when to TX changes.
4. Patch sketch: [`docs/patches/led-factory-patterns.patch`](patches/led-factory-patterns.patch)
   (proposed tree under `docs/patches/led_factory_proposed/`).

## Optional ESTOP / PDB kill override (doc only — not in this patch)

Future (separate change): when `system.mcu_state == ESTOP` and/or a PDB hard-kill
line asserts, firmware may **force** `BLINK_RED_FAST` (or `OFF` after a
timeout) regardless of the mounted `LedDesire`. Host desire remains staged;
override is local to `led_service` so recovery returns to the last host mode
without a re-send. Not implemented here — no plant/ESTOP/PDB wiring in the
patch.

## Non-goals

- No 672 B layout / schema version change.
- No second LED slot, no per-pixel host RGB stream.
- No replacement of `TEST` chase or reinterpretation of `FLASH=2`.
- No COM5 prove / soft-DFU by the offline author.
