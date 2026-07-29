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

---

## ADR-003 — Mixed std+ext on all FDCAN

| | |
|--|--|
| **Status** | Implemented / bench-verified |
| **Date** | Jul 2026 |

CH1–3 accept std+ext so Damiao + RobStride can share a branch. Protocol assignment is CFG + plugin reject, not sniff. Detail: [plant.md](plant.md).
