# Host exchange — layout v2

Fixed **672-byte** binary images in both directions. Same layout on USB CDC and UART.

**Supersedes** [host-exchange-v1.md](host-exchange-v1.md) (562 B). Decision record:
[decisions.md](decisions.md) ADR-001.

**Source of truth:** `App/Inc/host/host_exchange_schema.h`,
`scripts/deft_controls_sdk/link/exchange/wire_layout.py`.

## Identifiers

| Field | Command | Feedback |
|-------|---------|----------|
| Magic | `0x434D4448` (`"CMDH"`) | `0x46424848` (`"HBHF"`) |
| `layout_version` | `2` | `2` |
| `byte_size` | `672` | `672` |

Hosts that send v1 (562 / version 1) are rejected by `host_command_image_valid()`.

## Image layout (672 B)

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 12 | `header` — magic, layout_version, byte_size, seq |
| 12 | 32 | `system` — health + plant timing (not DEBUG) |
| 44 | 550 | `actuator_*[25]` — **22 B** each (20 B MIT + 2 B meta) |
| 594 | 12 | `servos[2]` — 6 B each |
| 606 | 2 | `leds[1]` |
| 608 | 64 | `pdb[]` — power-board mirror only (plant path keeps tags out) |

Total = 12 + 32 + 550 + 12 + 2 + 64 = **672**.

Tagged DEBUG ops use separate **`DBGC` / `DBGF`** frames — see
[host-debug-v1.md](host-debug-v1.md).

### System feedback (offset 12, 32 B)

| Offset in system | Size | Field |
|-----------------:|-----:|-------|
| 0 | 4 | bitfield word0 (same meaning as v1 u32): tick:12, estop:1, mcu:3, hb:1, last_cmd_seq:8, plant_block:7 |
| 4 | 2 | `lap_ms` |
| 6 | 2 | `lap_max_ms` |
| 8 | 1 | `ticks_svc` |
| 9 | 1 | `ticks_pending` |
| 10 | 2 | `usb_rx_drop` (reserved until filled) |
| 12 | 2 | `can_rx_drop` (reserved until filled) |
| 14 | 1 | `kill_state` (PDB soft-kill; 0 until PDB UART) |
| 15 | 1 | `kill_reason` |
| 16 | 1 | `estop_sense` |
| 17 | 1 | reserved0 |
| 18 | 14 | reserved |

Command `system` is 32 B; first 4 B keep v1 e_stop / mcu_state / heartbeat bitfields; remainder reserved.

### Actuator slot (22 B)

Command: 5×float desire + `uint16 meta` (host writes 0).

Feedback: position, velocity, torque, temperature, fault + `uint16 meta`:

| Bits | Field |
|------|--------|
| 0–2 | protocol |
| 3–5 | bus (1–6) |
| 6–13 | motor_id |
| 14 | enabled |
| 15 | fb_valid |

### `pdb[64]`

- **PLANT stream (`CMDH` / `HBHF`):** `pdb` must be zero for tags; reserved for
  PDB UART power telemetry ([ADR-001](decisions.md)).
- **DEBUG (`DBGC` / `DBGF`):** 32 B tag mailbox at offset 608 — see
  [host-debug-v1.md](host-debug-v1.md).

Plant timing lives in `system[]` only (v1 SVD overlay removed).

## Rates

Host stream ~30–100 Hz at 672 B duplex remains within USB FS headroom. MCU plant remains 500 Hz hold-last.
