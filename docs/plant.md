# Plant — buses, protocols, PDB

## Hardware rails

| Bus | Hardware | Typical use |
|-----|----------|-------------|
| CH1 | FDCAN1 @ 1 Mbps | Left Damiao arm |
| CH2 | FDCAN3 @ 1 Mbps | Right Damiao arm |
| CH3 | FDCAN2 @ 1 Mbps | Lift / spare (often disabled) |
| CH4–6 | MCP2518FD + MCP2562 SPI-CAN | Base RobStride |

All three FDCAN channels accept **std + ext** (`FDCAN_RX_STD_AND_EXT`). Older “CH2 ext-only” notes are obsolete.

```
Wire → FDCAN/MCP RX → sw rx_rings[bus] → actuator_dispatch_bus_rx → plugins
plugins → tx_queues → HW TX → Wire
```

Mixed Damiao (11-bit) + RobStride (29-bit) on one pair is filter-safe; demux is by CFG slot + plugin reject, not auto-detect. Same-bus ID aliasing (two ESCs both `1`) will cross-feed.

MCP path: INT-gated RX when idle (0 SPI); non-blocking TXQ; poll budgets on plant tick. Do not reorder MCP init priority.

## Protocols (plugin summary)

### Damiao (live)

STD MIT, enable latch RX-gated. ESC + master IDs in CFG. Pack nibble-interleave in `damiao.c` — trust code over PDF samples when they disagree.

### RobStride (live on base / channel bringup)

EXT 29-bit. Maintain budget per tick; MCP flush coalesced per rail. Channel CLI: `rs02_channel_bringup.py`.

### CubeMars (code present, not HW-proven)

Damiao-shaped MIT; TX-driven enable latch (`0xFC`/`0xFD`/`0xFE`). **Do not copy vendor PDF pack samples** (known bugs, e.g. data[6]). Default model AK80-9; no CFG model field yet. Servo mode compile-gated off. Discover (`CM0` bench PDU, `diag_cubemars.c`) is MIT-frame ID sweep only — no register-read scheme like Damiao’s; never touches `cubemars_apply_cycle`.

### ZeroErr (code present, not bench-proven)

CANopen PP: `motor_id` = node ID. Prefer FDCAN @ 1 Mbps; MCP CH4–6 also works (canopen.c SDO/NMT TX uses the MCP `send_now` bypass, same fix as Damiao/RobStride probes). PDO1 DLC6 (`cw`+target / status+actual). Boot FSM one NMT/SDO action per tick; SDO wait can steal RX from cohabitants — don’t spam multi-node boot. Discover (`ZE0` bench PDU, `diag_zeroerr.c`) sweeps node ids via SDO-read `0x1018`; never touches `zeroerr_apply_cycle`’s boot FSM.

### Dynamixel neck / SK9822 LEDs

PeripheralTask. DXL UART polled under `vTaskSuspendAll`. LED chain on SPI3.

## PDB kill

Separate from USB host exchange. Controls ↔ PDB MCU over **UART4** PC10/PC11, 115200 8N1, 64 B frames, CRC16-CCITT.

| | Controls→PDB | PDB→Controls |
|--|--------------|--------------|
| Magic | `PDBC` (`0x43424450`) | `PDBF` (`0x46424450`) |
| version | 1 | 1 |

`UART4_MODE` must be `UART4_MODE_PDB` when the connector is populated. Hard ESTOP wire: **PB7**, active-low (HIGH=ok).

| State | Value |
|-------|------:|
| NORMAL | 0 |
| SOFT_KILL_REQ | 1 |
| SOFT_KILL_READY | 2 |
| HARD_ESTOP | 3 |

```text
NORMAL → SOFT_KILL_REQ → (plant_recovery_all) → SOFT_KILL_READY → HARD_ESTOP
```

Stale PDBF (>200 ms) → HARD + COMMS_LOSS (fail-safe on comms loss). PB7 low
is reported separately as `system.estop_sense`; the PDB itself must set
`kill_state=HARD_ESTOP` in its own feedback frame when it drives PB7 low —
Controls' kill-state mirror does not independently OR in the raw GPIO read.
Soft kill alone must not open main power; PDB is sole rail-switch authority.
USB FB mirrors kill into `system` + `pdb[64]`. Hub: `soft_kill_park*`. Sim:
`scripts/pcb_lab/legacy/pdb_uart_sim.py`.

Source: `App/Inc/host/pdb_link.h`, `App/Src/host/pdb_link.c`. Full byte/bit-level
wire contract, state machine, handshake sequence, and TX rate detail:
[pdb-uart-v1.md](pdb-uart-v1.md).

## Soft-DFU / NVM

Flash path: [host-contract.md](host-contract.md)#soft-dfu. CFG / plant table live in NVM via DEBUG `CFG` — not on the 500 Hz desire path. No NVM “packer VM” on NORMAL plant; DIAG + Soft-DFU for malleability.
