# Architecture decisions

Dated decisions for the Deft controls stack. **Current on-wire contract remains [host-exchange-v1.md](host-exchange-v1.md) (562 B)** until a deliberate layout bump is implemented and flashed. This file is the vision to implement against when that work starts.

Related narrative: [architecture.md](architecture.md).

---

## ADR-001 — Host exchange target layout (~672 B) + PDB UART 64 B

| | |
|--|--|
| **Status** | Implemented (layout v2, 672 B) — see [host-exchange-v2.md](host-exchange-v2.md) |
| **Date** | 2026-07-20 (implemented 2026-07-21) |
| **Context** | Plant health was cannibalizing the 32 B USB `pdu` mailbox (SVD timing vs CFG/discover). Power-distribution (PDB) telemetry needs 8×(V,I) plus soft-kill choreography and does not fit cleanly in 32 B UART. Dual-arm plant keeps 25 wire actuator slots. Host control loop ~30 Hz; MCU plant 500 Hz hold-last. |

### Decision

#### 1. USB host ↔ controls cyclic image (target “v2”)

Bump `HOST_LAYOUT_VERSION` and ship a new contract doc (`host-exchange-v2.md`) when implementing — **do not silently edit v1**.

| Region | Bytes | Notes |
|--------|------:|-------|
| Header | 12 | magic, `layout_version`, `byte_size`, `seq` (fb seq must increment) |
| **System** | **32** | Health + soft-kill *mirror* (not full rail V/I). Replaces packing health into USB `pdu`. |
| **Actuators** | **25 × 22 = 550** | 20 B MIT desire/state (unchanged meaning) + **2 B meta** per slot |
| Servos | 12 | unchanged |
| LEDs | 2 | unchanged |
| **`pdb[]`** | **64** | Mirror of power-board telemetry for host/SDK |
| USB debug mailbox | **0 on PLANT path** | Tagged DEBUG ops move off the cyclic plant image (lease / separate DEBUG messages); plant frames keep this region absent or always zero |

**Total = 12 + 32 + 550 + 12 + 2 + 64 = 672 B** (command and feedback same size).

##### Actuator +2 B meta (`uint16` LE)

| Side | Use |
|------|-----|
| **Feedback** | Identity readback: pack `protocol`, `bus`, `motor_id` (and optional valid/enabled flags) |
| **Command** | **Same 2 B present for struct symmetry; reserved — host writes 0; firmware ignores** |

Identity **writes** stay on CFG / debugger, not the 50 Hz desire path. Motion fields stay normalized SI (rad, rad/s, N·m); plugins translate per protocol.

Suggested fb bit layout (implementer’s choice; lock in v2 doc):

| Bits | Field |
|------|--------|
| 0–2 | protocol |
| 3–5 | bus (1–6) |
| 6–13 | motor_id |
| 14–15 | flags (e.g. enabled, fb_valid) |

Damiao master ID (if needed) stays in CFG / wider config — not required in this `uint16`.

##### System 32 B (intent)

Enough for: today’s packed tick / `mcu_state` / `ack_seq` / `plant_block`; loop timing (lap, lap_max, pending); CAN/USB drop counters; **PDB soft-kill summary** (`kill_state`, reason, estop wire sense). Full 8×V/I live in `pdb[64]`, not in system.

#### 2. Controls ↔ PDB (power distribution) UART

Separate link from USB host exchange. **Fixed 64 B frames** both directions (~20–50 Hz).

Carries:

- 4× battery pack (48 V) + 4× rails (central 48 V, 19 V, 12 V, 5 V) → **8× voltage + 8× current**
- Contactor / rail enable bits
- Soft-kill state machine fields
- CRC / seq / version

Hard ESTOP remains an **active-low GPIO** driven by the controls PCB (PDU MCU death fails safe to ESTOP). Soft kill is **in-band UART status** so controls can park actuators under power before asserting the wire. PDU must **not** open main power on soft-kill alone.

Staged shutdown:

```text
NORMAL → SOFT_KILL_REQ → (controls safe pose) → SOFT_KILL_READY → HARD_ESTOP (wire)
```

#### 3. Rates / USB FS headroom

- Host stream **~30 Hz** (product control loop) is the design point; **50–100 Hz** still comfortable on USB FS CDC at 672 B duplex.
- MCU plant remains **500 Hz** hold-last; host need not match plant rate.
- 672 B ≈ 11 FS bulk packets/image; duplex @ 30 Hz ≈ 40 KB/s — far below USB FS practical limits.

#### 4. Naming clarity

- **USB `pdu` / debug mailbox** = bench/diag sidecar on the host image (legacy v1).
- **PDB** = power distribution board (UART + ESTOP wire).
- Do not overload “PDU” in new docs without qualifying which one.

### Consequences

- Implementing requires coordinated firmware + `deft_controls_sdk` `wire_layout` / pack / parse + layout version bump; old 562 B hosts will not speak v2.
- Prefer keeping **25 wire slots** for continuity with v1 padding; firmware still applies `ACTUATOR_COUNT` (14 dual-arm today).
- Until v2 ships: remain on **562 B v1**; health may still use SVD in the mailbox; PDB contract can be prototyped on UART independently.

### Explicitly deferred / rejected for this ADR

- Shrinking wire slots to 14 only (optional later optimization, not required).
- Putting full 8×V/I only inside `system` (use `pdb[64]`).
- Growing USB image only for DEBUG discover/cal (keep DEBUG off the plant cyclic path).
- Exact 576 B “9×64 USB packets” sizing (short final packet preferred when choosing totals; 672 = 10×64 + 32 short — fine).

### Implementation checklist

- [x] Draft `docs/host-exchange-v2.md` with locked offsets + `_Static_assert` sizes
- [ ] Draft `docs/pdb-uart-v1.md` (64 B cmd/fb, kill_state enum, LSB scales)
- [x] Firmware: schema structs, feedback identity meta, system timing fill
- [ ] PDB UART service + ESTOP policy
- [x] SDK: `wire_layout` IMAGE_BYTES=672, pack/parse (timing from system[])
- [ ] Dashboard / health strip: soft-kill visible before hard ESTOP
- [x] Layout version bump — mismatched v1 hosts rejected
- [ ] Move DEBUG tags off `pdb[0..31]` to a dedicated DEBUG message (transitional: still on mailbox)

---

## Related

- [architecture.md](architecture.md) — runtime + host API modes
- [host-exchange-v1.md](host-exchange-v1.md) — **current** 562 B contract
- [bringup.md](bringup.md) — how to run
- [lessons.md](lessons.md) — bugs and durable findings
