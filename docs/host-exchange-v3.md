# Host exchange — layout v3

Fixed **694-byte** binary images in both directions. Same layout on USB CDC and UART.

**Supersedes** [legacy/host-exchange-v2.md](legacy/host-exchange-v2.md) (672 B / 25 actuators). Decision record:
[decisions.md](decisions.md) ADR-001.

**Source of truth:** `App/Inc/host/host_exchange_schema.h`,
`scripts/deft_controls_sdk/link/exchange/wire_layout.py`.

## Identifiers

| Field | Command | Feedback |
|-------|---------|----------|
| Magic | `0x434D4448` (`"CMDH"`) | `0x46424848` (`"HBHF"`) |
| `layout_version` | `3` | `3` |
| `byte_size` | `694` | `694` |

Hosts that send v2 (672 / version 2) are rejected by `host_command_image_valid()`.

## Image layout (694 B)

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 12 | `header` — magic, layout_version, byte_size, seq |
| 12 | 32 | `system` — health + actuator/peripheral timing + seq readbacks |
| 44 | 572 | `actuator_*[26]` — **22 B** each (20 B MIT + 2 B meta) |
| 616 | 12 | `servos[2]` — 6 B each |
| 628 | 2 | `leds[1]` |
| 630 | 64 | `pdb[]` — power-board mirror only (plant path keeps tags out) |

Total = 12 + 32 + 572 + 12 + 2 + 64 = **694**.

Factory / load-matrix product CFG: CH1×8, CH2×8, CH3×4, CH4–6×2 each (= 26).

Tagged DEBUG ops use separate **`DBGC` / `DBGF`** frames — see
[host-debug-v1.md](host-debug-v1.md). DEBUG mailbox remains `pdb[0..31]` at
offset **630**.

### System feedback (offset 12, 32 B)

Same field map as v2. Kill / freshness:

| Offset in system | Size | Field |
|-----------------:|-----:|-------|
| 14 | 1 | `kill_state` (from `pdb_link`; **HARD_ESTOP** when PDB peer stale) |
| 15 | 1 | `kill_reason` (**COMMS_LOSS** when stale) |
| 16 | 1 | `estop_sense` (local PB7 wire) |

See [pdb-uart-v1.md](pdb-uart-v1.md) freshness contract and SDK
`hub.pdb_status()` / `deft_controls_sdk.pdb.PdbStatus`.

### `pdb[64]`

- **PLANT stream (`CMDH` / `HBHF`):** verbatim last valid PDBF (or zeros).
- **DEBUG (`DBGC` / `DBGF`):** 32 B tag mailbox at offset 630.
