# Lessons, bugs, and bring-up facts

Durable findings from bench work. **Current how-to:** [bringup.md](bringup.md).  
**Do not reintroduce** the closed regressions below.

---

## Open issues

| Issue | Where | Notes |
|-------|-------|-------|
| UART TX blocks main loop | `host_transport_uart.c` | Jetson UART path; USB CDC OK |
| Silent CAN TX drop | `actuator_apply_desire` | Full TX queue → skip, no fault in feedback |
| Encoder cal **NOISE** on daisy-chain | RS cal `0x05` | Power-cycle; one motor at a time; `--recovery` first |
| Ctrl+C mid-probe wedges MCU | `robstride_probe_id` | `--recovery` / USB replug |
| Mixed-bus bitrate | Shared CAN branch | All nodes **1 Mbps** nominal |
| Both USB+UART objects linked | Host project | Duplicate RX rings when one mode unused |
| RS 3× MOTOR_CTRL / tick | `robstride_apply_cycle` | Extra CAN load (Damiao already 1× MIT/tick) |
| RS2 session blocks plant | `plant_diag_skip_actuator_can` | Intentional — don’t mix with teleop |
| MCP idle = no ACT LED | `actuator.c` | Blank desire skips SPI on CH4–6 (by design) |
| Feedback `header.seq` always 0 | Firmware | Track via `ack_seq` for now |
| CubeMars not in live build | workstreams | Empty live stubs; scaling + RX ID unknown |

**Host backlog (SDK / legacy teleop):** soft-limit stop in plant teleop; `joint home`; batch joint script; absolute moves gated until zeros calibrated.

---

## Closed lessons (keep)

### NVM CFG SAVE

- **`flash_err` on `--persist`:** G4 `FLASH_CR_PNB` is 7 bits; page 255 needs **`FLASH_CR_BKER`**. Without it, erase hit page 127 (`0x0803F800`) while program/verify used `0x0807F800` → verify always failed. Fixed in `plant_config_nvm.c` `ramfunc_flash_erase_page`. RAM SET still worked until power cycle.

### Host exchange layout

- **v1 562 B → v2 672 B:** expand `system` to 32 B (timing leaves the DEBUG mailbox), actuators 22 B (+2 meta), `pdb[64]` at offset 608. Hosts still on v1 are rejected (`layout_version` / `byte_size`). Soft-DFU flash both ends together.

### Damiao

- **Scan-order flood:** probing ESC 1→N with heavy REG_SCAN before the real ID can silence the drive. Prefer MCU ID_SWEEP, then known slot IDs, then range. Lighter TX + RX drain after each probe.
- **Termination:** `tx>0` / `rx_raw=0` after scan-order ruled out → motor-end **120 Ω** (no software termination on 4310/4340). ~60 Ω H–L if both ends terminated.
- **Master ID:** typically `ESC_ID + 0x10`; confirm via regs `0x08` / `0x07`.
- **4310 vs 4340:** same MIT / `0x7FF` map — no separate protocol. Silence → baud (CAN FD), ID, termination — not a different wire format.
- **Daisy chain:** un-enabled motors mid-harness block teleop behind them — map+enable every unit.
- **Dual-arm firmware:** paginated CFG GET for 14 slots; Damiao 1× MIT/tick (was 3×); thermo must not clobber CFG PDU; **CH2 mixed std+ext** required for Damiao arm2.

### FDCAN mixed std/ext

- Damiao = 11-bit std; RobStride = 29-bit ext. One HW RX FIFO; demux by `IdType` in software.
- Install **std + ext** accept-all filters on mixed buses. Fan-out each frame to all slots on that bus — do **not** use per-plugin exclusive `while (can_rx_pop)` (starves the second protocol).
- Schematic CH2 → `hfdcan3`, CH3 → `hfdcan2`.

Deep reference (as-built filters/FIFOs): [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) — note **CH2 is mixed now** (older § claiming ext-only is wrong).

### MCP2518FD + MCP2562 (CH4–6)

Full debug timeline + ranked bugs: [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md) (also [bringup.md](bringup.md) §8). Short form:

- Bit time must be ~**1.0 µs** on scope (TSEG1=15 / 18 TQ). “1 Mbps” at TSEG1=17 measured ~1.15 µs → recessive ACK, TEC+8.
- Never treat TXQEIF (queue empty) as TX complete on one-deep TXQ.
- RX: first RX FIFO = channel 1; **FnBP=1**. Wrong FnBP/SFR → scope sees reply, firmware `rx=0`.
- FRESET TXQ only in Config; careful UINC/TXREQ in Normal.
- Ammeter ground truth: ~0.02 A rest / ~0.07 A enabled often beats misleading `mms` alone.

### Plant superloop (do not reintroduce)

- No blocking MCP/RobStride send on the 500 Hz MIT path — fire-and-forget; blocking OK for probes only.
- Skip Dynamixel UART unless servo host session (50 ms RX timeout blows `lap_ms`).
- Eager MCP rail init at boot.
- USB init before long `app_init`; TIM6 + USB_LP NVIC priority **0**.

### Teleop / host behavior

- Hold full joint `SLOT_KP` on release (no limp gravity sag).
- Brace non-target joints at current position — don’t zero-fill other slots in a single-slot send.
- Home to **current fb**, not fixed 0 rad, until zeros exist.
- Dynamixel: unicast only; idle latch; HW-error reboot path; servo session skips actuator CAN.
- SK9822: correct BE packing / end-frame length; one `led_table` owner.

### FreeRTOS (if enabled)

- TIM6 ISR: pending counter only (no FromISR). Don’t let CubeIDE push TIM6/USB back to low NVIC priority or empty SVC/PendSV stubs.

---

## CubeMars (merged, not motor-ready)

`cubemars.c`/`cubemars.h` merged into the live tree from the (now-deleted)
`2026-07-10 workstreams/` draft; `PROTO_CUBEMARS` → `&cubemars_ops` in
`plugin_table.c` (was `NULL`). AK-series Servo Mode, Position-Speed Loop
(control mode 6) only — see `App/Inc/plant/plugins/cubemars.h`.

**Before trusting motion — two unresolved P0s, both runtime-fixable via CFG
SET (no firmware redeploy needed) once known:**

1. Feedback CAN ID is undocumented in the vendor PDF — bench-sniff (motor in
   servo mode, no host TX, observe the periodic broadcast ID) and set via CFG
   `master_id`. Default `0` = never matches, safe no-op.
2. `pos`/`speed` units are passed through as placeholders (deg/eRPM assumed
   1:1 with `ActuatorDesire`'s rad/rad·s⁻¹) — needs the motor's pole-pair
   count to verify real scaling. **Do not command a loaded joint** until
   confirmed (see the P0 comment block in `cubemars.h`).

Host tests (`scripts/legacy/tests/test_cubemars_wire.py`, against
`scripts/legacy/controls_pcb_host/protocol/cubemars.py`) are encode/decode
internal-consistency only — nothing about this protocol is verifiable
without the motor. Not yet ported to `deft_controls_sdk` (the current
canonical SDK, see `docs/api.md`) — CFG SET already exposes
`protocol=2` (`PROTO_CUBEMARS`) generically via `hub.debug.cfg_set_slot()`,
but there's no `deft_controls_sdk`-side CubeMars encode/decode helper yet.

---

## Related

- [bringup.md](bringup.md) — commands and stories
- [ch4-mcp2518-bringup-postmortem.md](ch4-mcp2518-bringup-postmortem.md) — CH4 MCP2518FD + MCP2562
- [architecture.md](architecture.md) — modes + wire current vs target
- [decisions.md](decisions.md) — 672 B / PDB UART ADR
- [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) — mixed CAN detail
