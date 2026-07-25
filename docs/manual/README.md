# Deft Controls PCB — Manual

Human instruction book for the Controls PCB stack. This is a **table of
contents that links out** — the deep, verified content lives in the docs
each chapter points to; this page does not duplicate it. If a chapter and
its linked doc ever disagree, the linked doc wins (this page is a map, not
the territory).

**Operator workbook (exercises):** [user-tutorial.md](../user-tutorial.md) —
flash, discover, vbeta/continuous prove, cleanup. Use that when you want
checkboxes; use this TOC when you want the chapter map.

## Contents

1. [Setup / Jetson / serial ownership](#1-setup--jetson--serial-ownership)
2. [Soft-DFU flash](#2-soft-dfu-flash)
3. [Plant map](#3-plant-map)
4. [PDU / soft-kill](#4-pdu--soft-kill)
5. [Arms / base / neck](#5-arms--base--neck)
6. [Continuous vs vbeta stack](#6-continuous-vs-vbeta-stack)
7. [Recovery / E-STOP / COMMS_LOSS](#7-recovery--e-stop--comms_loss)

---

## 1. Setup / Jetson / serial ownership

- Repo on the bench Jetson lives at `~/controls_pcb`, host `192.168.50.48`,
  user `deft-robotics`, password from env `JETSON_PASS` (bench default
  `4565`). SSH / Paramiko is the standard path in.
- **One process owns the board's USB CDC port at a time.** Continuous
  driver, debug dashboard (`Connect COM`), and any teleop stack all compete
  for the same serial port — never run two at once. The dashboard has a
  read-only **follow mode** (`.deft_session/state.json`) for when something
  else owns COM; see [§4](#4-pdu--soft-kill).
- Full transport/build setup: [bringup.md §1](../bringup.md#1-transport--flash).
- SDK quick start (`ControlsPcbHub`, one import surface): [api.md](../api.md),
  [`scripts/deft_controls_sdk/README.md`](../../scripts/deft_controls_sdk/README.md).

## 2. Soft-DFU flash

USB-only firmware flash, no ST-Link needed for a normal cycle (ST-Link SWD
is recovery-only). Full procedure, success criteria, and how it works:
**[soft-dfu.md](../soft-dfu.md)**.

```bash
python scripts/soft_dfu_flash.py scan   # identify board serial first
python scripts/soft_dfu_flash.py --serial <board> --require-usb-dfu
```

With more than one Controls PCB reachable (e.g. a Soft-DFU dev board
alongside the live product board), always pin `--serial` — never flash by
"whichever board answers first."

## 3. Plant map

Wire layout is **v3, 694 B, 26 actuator slots** (`HOST_EXCHANGE_ACTUATOR_SLOTS`
in `App/Inc/host/host_exchange_schema.h`) — see
[host-exchange-v3.md](../host-exchange-v3.md) and
[architecture.md](../architecture.md#wire-contracts-layout-v3--shipped).

**Which slots mean what depends on which driver stack is running** — there
is more than one live slot *map* over those 26 wire slots. Don't assume a
slot number without checking which stack you're reading it from; see
[§6](#6-continuous-vs-vbeta-stack) for the two maps.

Constant across both maps: dual YAM Damiao arms, CH1/CH2. Lift (CANopen) is
stubbed off — out of scope until explicitly picked back up.

## 4. PDU / soft-kill

Wire contract (64 B UART4 frames, Controls↔PDB): [pdb-uart-v1.md](../pdb-uart-v1.md).
Operating manual (freshness, `COMMS_LOSS`, dashboard follow-mode park):
[peripherals/pdu-uart-soft-kill.md](../peripherals/pdu-uart-soft-kill.md).

**V/I acceptability thresholds** (new this pass — provisional, tune after a
real PDB capture; see `docs/pdb-uart-v1.md`'s placeholder-scale note):

| Check | Range | Applies to |
|-------|-------|------------|
| Pack voltage | 40–55 V | each populated `pack_v[0..3]` channel |
| Central 48 V rail | 42–52 V | `rail_v[0]` only, when that rail looks active |
| Pack / rail current | ≤ 30 A abs | each active `pack_i[]` / `rail_i[]` channel |

Two independent layers apply the same numbers (kept in lockstep — see the
cross-reference comments in both files):

- **Firmware** (`App/Inc/host/pdb_vi_limits.h`, `pdb_link.c`
  `pdb_vi_reject_reason()`) overlays `kill_state=SOFT_KILL_REQ` +
  `kill_reason` (undervoltage/overcurrent) onto the USB mirror when a fresh
  `PDBF` from the PDB is out of range while the peer itself reports `NORMAL`.
- **Host SDK** (`scripts/deft_controls_sdk/pdb/limits.py`,
  `ControlsPcbHub.soft_kill_park_if_bad_vi()`) independently recomputes the
  same check from the raw `pdb[]` mirror and parks proactively — this is the
  belt-and-suspenders layer, and it's the one that protects a board
  running **older firmware that predates the overlay above**. Both layers
  are wired into `hub.start_streaming()` automatically; nothing extra to
  call from driver code.
- A stale PDB link (no valid frame for 200 ms) always fails safe to
  `HARD_ESTOP`/`COMMS_LOSS` regardless of V/I — see
  [pdu-uart-soft-kill.md](../peripherals/pdu-uart-soft-kill.md) for why that
  looks identical to a real hard-ESTOP on USB alone.
- Prove offline (no board, no motors):
  `python -m pytest scripts/tests/test_deft_controls_sdk_pdb_limits.py -q`
  — injects bad `pack_v`/`pack_i` the same way `pdb_uart_sim.py` would and
  asserts the host-side park fires.

## 5. Arms / base / neck

Live-verified, per-peripheral operating manuals (AI quickstart + human deep
dive + what's actually been proven on this bench) — **[§6](#6-continuous-vs-vbeta-stack)
first** if you're about to run something, then:

- [peripherals/arm-damiao-ch1.md](../peripherals/arm-damiao-ch1.md) — arm CH1 Damiao (CFG, enable
  latch, fault=1, CLEAR)
- [peripherals/dxl-neck.md](../peripherals/dxl-neck.md) — neck pitch/yaw Dynamixel
- [peripherals/base-robstride-mcp.md](../peripherals/base-robstride-mcp.md) — base RobStride on MCP
  CH5/CH6
- [peripherals/base-damiao-ch6.md](../peripherals/base-damiao-ch6.md) — base Damiao sharing CH6

## 6. Continuous vs vbeta stack

Two driver stacks, two different slot maps over the same 26-slot wire — pick
the one matching what you're actually running before trusting a slot number:

| | Continuous (bench, today) | vbeta / product (new this pass) |
|--|---------------------------|----------------------------------|
| Driver | `scripts/yam_continuous_all.py` | `deft_vbeta/` (repo-root working copy) via `scripts/deft_controls_sdk/vbeta/` adapters |
| Arms | slots 0–13, CH1/CH2 | slots 0–13, CH1/CH2 (same) |
| Base | slots **22–25**, bench IDs `0x70`/`0x74`/`0x75`/`0x06` — see [base-robstride-mcp.md](../peripherals/base-robstride-mcp.md) | slots **14–19**, product IDs `0x01`/`0x02` @ CH4–6 — see `vbeta/slots.py::yam_product_rows()`, [vbeta-pcb-adapter.md](../vbeta-pcb-adapter.md) |
| Slot map source of truth | `yam_continuous_all.py` `BASE_ROWS` | `scripts/deft_controls_sdk/vbeta/slots.py` |
| Op manual | [peripherals/continuous-ops.md](../peripherals/continuous-ops.md) | `deft_vbeta/` own docs (not forked here — see note below) |

**Do not mix the two base maps** — driving slots 22–25 from the vbeta/product
path or slots 14–19 from continuous is a wiring mismatch, not a config
choice. `deft_vbeta/` is a separate, actively-developed working tree (fresh
clone, not the read-only `docs/deft_vbeta_ref/deft_vbeta` submodule) — this
manual links to it once it lands verified content of its own; it is not
rewritten here.

## 7. Recovery / E-STOP / COMMS_LOSS

- **Kill states**: `0 NORMAL → 1 SOFT_KILL_REQ → 2 SOFT_KILL_READY → 3 HARD_ESTOP`.
  Full state machine: [pdb-uart-v1.md](../pdb-uart-v1.md#soft-kill--hard-estop-state-machine).
- **`stale_failsafe`** (`kill_state==HARD_ESTOP and kill_reason==COMMS_LOSS`)
  is the *only* USB-visible link-freshness signal — there's no separate
  "link stale" byte. A dead PDB link looks identical to a real hard-ESTOP
  from USB alone; cross-check `estop_sense` / the PDB's own log to tell them
  apart. Details: [peripherals/pdu-uart-soft-kill.md](../peripherals/pdu-uart-soft-kill.md#why-stale-and-hard-faulted-look-identical-on-usb).
- **Manual park**: `hub.soft_kill_park(send=True)` if you hold COM directly;
  otherwise write `.deft_session/soft_kill_request` (dashboard follow-mode
  path) and let the COM-owning process pick it up — see
  [peripherals/pdu-uart-soft-kill.md](../peripherals/pdu-uart-soft-kill.md).
- **Auto-park** already wired into `hub.start_streaming()`: PDB-requested
  soft-kill (`soft_kill_park_if_requested`) and out-of-range V/I
  (`soft_kill_park_if_bad_vi`, [§4](#4-pdu--soft-kill)) both park without
  driver code doing anything extra. `HARD_ESTOP`/`COMMS_LOSS` itself does
  **not** currently auto-park — the hard-ESTOP wire is the PDB's own
  fail-safe cutoff, not something Controls needs to react to by parking
  (there's nothing left to park safely into once contactors are already
  cut). Revisit if that assumption changes.
- Compact open-bugs list: [lessons.md](../lessons.md).
