# Host DEBUG frames v1

Tagged bench ops (CFG, RS2, DM, DFU, thermo, …) use **dedicated USB frames**,
not the plant cyclic `pdb[]` region. Plant images keep `pdb[64]` clear for
power-board telemetry ([ADR-001](decisions.md)).

Same **672 B** size as plant frames so CDC framing stays simple.

## Magics

| Direction | Magic | ASCII |
|-----------|-------|-------|
| Command | `0x44424743` | `DBGC` |
| Feedback | `0x46424744` | `DBGF` |

`layout_version` = 2, `byte_size` = 672 (same as [host-exchange-v2.md](host-exchange-v2.md)).

## Layout

Identical region map to the plant image. Only these matter for DEBUG:

| Offset | Size | Use |
|-------:|-----:|-----|
| 0 | 12 | Header (`DBGC` / `DBGF`) |
| 12 | 32 | `system` (mcu_state, etc. — same bitfields) |
| 44 | 550 | Actuators — used when an RS2 ctrl probe mounts desires; else zero |
| 608 | 32 | **DEBUG mailbox** (former plant `pdu` / `pdb[0..31]`) |
| 640 | 32 | unused / zero |

Tag bytes in the mailbox are unchanged from v1/v2 transitional (`CFG`, `RS2`,
`DM0`, `DFU!`, …).

## Host rules

- Plant stream sends only `CMDH` / expects `HBHF` (mailbox ignored on RX).
- SDK bench ops (`hub.debug.*`, soft-DFU enter) send `DBGC` and wait for `DBGF`.
- Firmware: one `DBGF` after each `DBGC`; plant TX never puts tags in `HBHF.pdb`.

## Source of truth

- Firmware: `HOST_DEBUG_*_MAGIC` in `App/Inc/host/host_exchange_schema.h`,
  dispatch in `App/Src/host/host_link.c`
- SDK: `HOST_DEBUG_*` in `wire_layout.py`, builders in `link/exchange/bench.py`
