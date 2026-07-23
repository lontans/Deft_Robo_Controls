# Controls ↔ PDB power-distribution UART — layout v1

Fixed **64-byte** binary frames, both directions, over a dedicated UART link
between the Controls PCB and the Power Distribution Board (PDB) MCU.
**Separate from** the USB host_exchange link ([host-exchange-v2.md](host-exchange-v2.md))
— this is a distinct physical connection with its own framing, not a tag on
the USB image. Decision record: [decisions.md](decisions.md) ADR-001.

**Source of truth (controls side):** `App/Inc/host/pdb_link.h`, `App/Src/host/pdb_link.c`.
This doc is also the contract for whoever implements the PDB-side firmware —
the controls-PCB implementation only exists in this repo; the PDB MCU's own
firmware is out of scope here.

**Naming:** *PDB* = Power Distribution Board (this link + the hard-ESTOP wire).
Don't confuse with the USB link's legacy 32 B `pdu` debug mailbox at the same
byte offset in the host-exchange image (see ADR-001 §4) — same word, different
concept.

---

## Physical layer

UART4, **PC10 (TX)** / **PC11 (RX)** on the Controls PCB, 115200 8N1.

`App/Inc/host/uart4_mode.h` must have `UART4_MODE` set to `UART4_MODE_PDB` for
this link to be active. The alternate roles that used to live on this same
pin pair (`UART4_MODE_TELEM`, `UART4_MODE_DAMIAO_BRIDGE`) are retired on any
board where the PDB connector is populated — selecting either of those on
such a board drives the wrong protocol onto wires that go to the power board.

---

## Identifiers

| Field | Command (Controls→PDB) | Feedback (PDB→Controls) |
|-------|-------------------------|--------------------------|
| Magic | `0x43424450` (`"PDBC"`) | `0x46424450` (`"PDBF"`) |
| `version` | `1` | `1` |
| Frame size | 64 B | 64 B |

A frame failing magic, version, or CRC check is treated as **no frame this
cycle** — never partially trusted, and never counted toward link freshness.

---

## Common header (8 B, both directions)

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 4 | `magic` |
| 4 | 1 | `version` (= 1) |
| 5 | 1 | `seq` — 8-bit wraparound, incremented by the sender each frame |
| 6 | 1 | reserved (flags; currently unused, send 0) |
| 7 | 1 | reserved/pad |

## Command frame (Controls→PDB), offset 8 onward

| Offset | Size | Field |
|-------:|-----:|-------|
| 8 | 1 | `rail_enable_cmd` — bitmask, 1 bit/rail. Controls *requests*; **PDB is the sole authority on actually switching a rail.** |
| 9 | 1 | `kill_request` — `0` NORMAL, `1` SOFT_KILL_ACK, `2` SOFT_KILL_READY (see state machine below) |
| 10 | 1 | `heartbeat` — free-running counter; lets the PDB tell "controls frozen" from "controls silent" |
| 11–61 | 51 | reserved, sent as 0 |
| 62–63 | 2 | CRC16 over bytes 0–61 |

## Feedback frame (PDB→Controls), offset 8 onward

| Offset | Size | Field |
|-------:|-----:|-------|
| 8–15 | 8 | `pack_v[4]` — 4× battery pack voltage, `uint16` |
| 16–23 | 8 | `rail_v[4]` — 4× rail voltage (48 V central, 19 V, 12 V, 5 V), `uint16` |
| 24–31 | 8 | `pack_i[4]` — 4× battery pack current, `uint16` |
| 32–39 | 8 | `rail_i[4]` — 4× rail current, `uint16` |
| 40 | 1 | `contactor_state` — readback bitmask (measured/commanded actual state per rail) |
| 41 | 1 | `kill_state` — `0` NORMAL, `1` SOFT_KILL_REQ, `2` SOFT_KILL_READY, `3` HARD_ESTOP |
| 42 | 1 | `kill_reason` — `0` none, `1` host-requested, `2` undervoltage, `3` overcurrent, `4` overtemp, `5` comms-loss, `6` button, `7` other |
| 43 | 1 | `estop_sense` — PDB's own readback of the hard-ESTOP wire level (cross-check against what controls thinks it's driving) |
| 44 | 1 | `fault_flags` — bitmask, PDB-defined |
| 45 | 1 | `heartbeat_echo` — echo of the last command `heartbeat` byte seen |
| 46–61 | 16 | reserved, sent as 0 |
| 62–63 | 2 | CRC16 over bytes 0–61 |

### LSB scales — **placeholder, not locked**

`pack_v`/`rail_v` proposed at **10 mV/count** (0–655.35 V range), `pack_i`/`rail_i`
at **10 mA/count**. These depend on the real sensor resolution/range in the PDB
hardware design and must be pinned down (and this doc updated) before the PDB
firmware is written against it — do not assume these are final.

---

## CRC16

Bit-banged CRC16-CCITT: poly `0x1021`, init `0xFFFF`, computed MSB-first over
the input bytes, **no final XOR, no reflection**. Reference implementation
(`App/Src/host/pdb_link.c` `pdb_crc16`):

```c
uint16_t crc = 0xFFFF;
for (each byte b in the frame, offsets 0..61) {
    crc ^= (uint16_t)b << 8;
    for (8 iterations) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
}
/* store crc as two bytes, offset 62 = low byte, offset 63 = high byte */
```

Chosen bit-banged (not table-driven) deliberately — trivial for the PDB
team's toolchain to reproduce bit-for-bit without needing to share a
generated table.

---

## Soft-kill / hard-ESTOP state machine

Staged sequence (ADR-001):

```text
NORMAL -> SOFT_KILL_REQ -> (controls reaches a safe pose) -> SOFT_KILL_READY -> HARD_ESTOP (wire)
```

**Invariant: the PDB must never open main power on soft-kill status alone** —
only after controls' `SOFT_KILL_READY` handshake in the command frame, or
independently via the unconditional hard-ESTOP wire.

1. **NORMAL** — feedback `kill_state == NORMAL`, link fresh. Controls drives
   the hard-ESTOP GPIO HIGH (power allowed).
2. **Kill triggered** — either the PDB reports `kill_state == SOFT_KILL_REQ`
   (its own over-current/under-voltage/button event), or controls decides to
   kill locally (host-commanded E-STOP, a local fault). Controls begins
   parking actuators to a safe pose.
3. Once parked, controls sends `kill_request = SOFT_KILL_READY` in the next
   command frame.
4. The PDB, seeing that ack, proceeds to actually open contactors.
5. **HARD_ESTOP** — controls drives the hard-ESTOP GPIO LOW **immediately and
   unconditionally** on: explicit host E-STOP, an unrecoverable local fault,
   or the PDB link going stale (no valid frame for the documented timeout).
   This bypasses the staged negotiation entirely — it is the physical
   backstop, not a fourth step in the handshake.

### Hard-ESTOP GPIO

Active-low: **HIGH = power allowed, LOW = asserted/cut**. Driven by the
Controls PCB, single owner is `pdb_link.c`. Must default to asserted (LOW)
before firmware has run at all — an external pull-down on this net is
required so an unprogrammed or crashed MCU fails safe.

**Pin: placeholder, not yet confirmed against the schematic** — currently
`PA0` in `pdb_link.c` (chosen only because nothing else in this firmware
claims it). Update `PDB_ESTOP_GPIO_PORT`/`PDB_ESTOP_GPIO_PIN` in
`App/Src/host/pdb_link.c` once the real pin is assigned.

### Freshness / fail-safe timing

- Link considered fresh if a **valid** frame (magic + version + CRC all pass)
  has been received within the last **200 ms** (`PDB_STALE_MS`) — roughly 4
  missed frames at a 20 Hz nominal rate.
- A corrupted or version-mismatched frame does **not** refresh this timer —
  it is fail-safe by construction: a stream of garbage degrades to the same
  path as silence, not a false "link is fine."
- Command frames are sent at **50 Hz** (`PDB_TX_PERIOD_MS` = 20 ms).

---

## Rates

~20–50 Hz nominal both directions, well under UART4's 115200 baud budget
(~11.5 KB/s available vs ~3.2 KB/s needed for 64 B duplex at 50 Hz).

---

## Jetson sim (bring-up without real PDB firmware)

`scripts/pdb_uart_sim.py` stands in for the PDB MCU on a spare UART (Jetson
header or USB-UART adapter — **never** the Controls board's own USB CDC
port) so the physical UART4 wiring and the USB `pdb[64]` mirror can be
exercised before real PDB firmware exists:

```bash
python pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20
python pdb_uart_sim.py --port /dev/ttyUSB0 --hz 20 --rail-v 4800 1900 1200 500
python pdb_uart_sim.py --port /dev/ttyUSB0 --simulate-kill-after 10   # exercise the soft-kill handshake

# Continuous randomized telemetry + repeated random fault-cycling, live
# ESTOP sense off a Jetson header pin (BOARD numbering, e.g. 16 = GPIO08):
python pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 --random --gpio-estop 16 --seed 1
```

It sends valid `PDBF` frames forever at `--hz`, parses incoming `PDBC`
frames, echoes the last-seen `heartbeat` into `heartbeat_echo`, and stubs
the soft-kill state machine (`--simulate-kill-after` injects a single
`SOFT_KILL_REQ`, then transitions to `SOFT_KILL_READY` only once it sees the
controls board's ack in a command frame — same non-negotiable ordering as
the real handshake). TX is non-blocking: with nothing on the other end
(board unplugged, wiring not connected), the sim keeps running and printing
its `tx_seq`/status line rather than blocking on a full serial buffer.

**`--random`** replaces the fixed `--pack-v`/`--rail-v`/`--pack-i`/`--rail-i`
values with a bounded random walk around those same centers (`--voltage-
jitter-pct`, default ±2%; `--current-jitter-pct`, default ±40% — currents
swing more in reality) and repeats the soft-kill handshake indefinitely at
random intervals (`--fault-interval-s`, default 20–60 s) with a randomly
chosen plausible reason (undervoltage/overcurrent/overtemp/button), holding
`SOFT_KILL_READY` for a random duration (`--fault-hold-s`, default 3–10 s)
before auto-recovering to `NORMAL` and repeating. It still never reports
contactors open without the controls-board ack — same invariant as the
scripted `--simulate-kill-after` path, just looped. `contactor_state`
readback is forced to `0` while `SOFT_KILL_READY` regardless of
`--contactor-state`, so that byte reflects the simulated open/closed state
rather than always echoing the CLI flag.

**`--gpio-estop BOARD_PIN`** live-reads the hard-ESTOP wire off a Jetson
header pin instead of the fixed `--estop-sense` value — needs `Jetson.GPIO`
and must run on the Jetson itself. This is a **read only**: Controls drives
the wire (active-low, HIGH = power allowed / LOW = asserted), the sim/PDB
side only cross-checks it into the `estop_sense` feedback byte, matching the
real PDB's documented role above.

The pack/parse/CRC contract it uses (`deft_controls_sdk/pdb/`) is a
bit-exact Python port of `App/Src/host/pdb_link.c` — see
`scripts/tests/test_pdb_link_frames.py` for the golden vectors.

### Cursor prove-out notes (2026-07-23)

| Check | Result |
|-------|--------|
| Agent1 pytest `test_pdb_link_frames.py` | 16 passed |
| USB CDC `pdb[64]` with **no** UART4 peer | all zeros |
| USB `system.kill_state` / `kill_reason` (no peer) | `HARD_ESTOP` (3) / `COMMS_LOSS` (5) — fail-safe as designed |
| Live Jetson/USB-UART ↔ UART4 (PC10/PC11) + fresh `PDBF` mirror | **blocked** this sprint — no spare USB-UART on the bench COM list (only COM5 CDC + ST-Link VCP + com0com pairs). Needs Jetson `ttyTHS*` or a USB-UART wired TX→PC11 / RX←PC10 / GND @ 115200 8N1, then re-run sim + confirm non-zero `pdb[64]` + `kill_state==NORMAL` while fresh. |

## Related

- [decisions.md](decisions.md) — ADR-001, the decision record this implements
- [architecture.md](architecture.md) — three-layer power path narrative
- [host-exchange-v2.md](host-exchange-v2.md) — the USB link this mirrors into (`system.kill_state`/`kill_reason`/`estop_sense`, `pdb[64]`)
