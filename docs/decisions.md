# Architecture decisions

Dated decisions. On-wire contract today: [host-contract.md](host-contract.md) (694 B, layout v3). Narrative: [architecture.md](architecture.md).

---

## ADR-001 — Host exchange + PDB UART

| | |
|--|--|
| **Status** | Implemented; layout bumped to **v3 / 694 B** (26×22 actuator bytes). Historical v2 was 672 B / 25 slots. |
| **Date** | 2026-07-20 (v2); v3 follow-on Jul 2026 |

### Decision (still binding)

1. **USB cyclic image** carries header + 32 B system + actuators + servos + LEDs + **64 B `pdb[]` mirror**. Plant path keeps DEBUG tags out of `pdb[]`.
2. **Actuator +2 B meta** on FB for identity readback; command meta reserved.
3. **Controls ↔ PDB** is a **separate** UART link (64 B, ~20–50 Hz) for rail V/I, enable bits, soft-kill SM. Hard ESTOP remains active-low GPIO; soft kill is in-band so actuators can park before the wire asserts. PDB alone switches power.
4. Host stream ~30 Hz design; 50–100 Hz still fine on USB FS at this size.
5. DEBUG bench ops use tagged frames (`DBGC`/`DBGF`), not the plant mailbox.

### Naming

Do not confuse USB DEBUG `pdu` mailbox with PDB/PDU power kill — see [architecture.md](architecture.md).

---

## ADR-002 — Soft-DFU via option bytes

| | |
|--|--|
| **Status** | Implemented |
| **Date** | 2026-07 |

Enter ROM DFU by programming **nBOOT0=0** after DEBUG `DFU!`, not soft MEMRMP. Leave trampoline restores nBOOT0. ST-Link = recovery only. Detail: [host-contract.md](host-contract.md)#soft-dfu.

**Superseded enter path (ADR-004):** prefer `stm32_mode=2` at connect/enter; keep `DFU!` as deprecated alias.

---

## ADR-003 — Mixed std+ext on all FDCAN

| | |
|--|--|
| **Status** | Implemented / bench-verified |
| **Date** | Jul 2026 |

CH1–3 accept std+ext so Damiao + RobStride can share a branch. Protocol assignment is CFG + plugin reject, not sniff. Detail: [plant.md](plant.md).

---

## ADR-004 — Link mode + debug lanes (no PDB overlay)

| | |
|--|--|
| **Status** | Accepted / implementing |
| **Date** | 2026-07-29 |

### Context

DEBUG RPC historically reused `pdb[0..31]` on `DBGC`/`DBGF`, cannibalizing the PDU mirror region. Discover already used DBGC; the problem was the shared 32 B overlay, not “discover on plant.”

### Decision

1. **`stm32_mode`** on plant system command (bits 9–10), distinct from **`mcu_state`**. Set at `ControlsPcbHub.connect(mode=...)`; change only via disconnect/reconnect.
2. **Mode 0** (`bandwidth`): plant-only pipe; full rate; `HBHF.pdb` = PDU mirror; no debug-lanes frame; `hub.debug.*` that needs lanes refuses.
3. **Mode 1** (`debug`): plant `CMDH`/`HBHF` continues; host may interleave **debug lanes** `DBGC`/`DBGF` frames with header `DL\x01` + 10×32 B lanes (RS, CM, ZE, DM, LED, servo, PDU, 3×reserved). Subsystems arm via `arm_mask`.
4. **Mode 2** Soft-DFU: enter ROM DFU without using plant `pdb[]` as mailbox (legacy `DFU!` kept briefly).
5. Plant apply trusts actuator desires only on **CMDH**. Debug replies live in debug lanes / DBGF, never as tags in plant `HBHF.pdb`.

### Consequences

- Bandwidth tests use `mode="bandwidth"` so fb_hz / gaps stay valid.
- Multiple debug scripts share one standardized lane map.
- Host DEBUG RPC always uses debug lanes; legacy offset-630 TX path removed from SDK. FW may still accept legacy inbound when `DL` header is absent.

Detail: [host-contract.md](host-contract.md).
