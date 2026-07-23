# RFC: Lift Phase 0 — bench discovery (CANopen torso)

Status: discovery checklist only. Extracted from
[`feathersdk-lift-teardown.md`](feathersdk-lift-teardown.md) §§5–6.
Author: Agent 2 (offline). Blocking on hardware — no firmware/SDK work until
node ID / OD / scale are measured on CH3.

Companion: [`vbeta-pcb-adapter.md`](vbeta-pcb-adapter.md) (Lift stub),
[`zeroerr-firmware-bringup.md`](zeroerr-firmware-bringup.md) (CiA-402 stack to
reuse).

## Why Phase 0 is blocking

Everything ZeroErr got from an EDS is unknown for the FeatherSDK lift
("torso"). Docs/research cannot fill these; they need a live SDO scan once
the drive's CAN is on Controls PCB **CH3** (FDCAN2).

## Unknowns (from teardown §5)

| Unknown | Why it matters | How to get it |
|---------|----------------|---------------|
| CANopen node ID | Every SDO/PDO COB-ID is `base + node` | SDO/NMT scan `1..127` on CH3 |
| Baud rate | May differ from ZeroErr's 1 Mbps | Scope or bitrate sweep |
| Object dictionary (mode-of-op, PDO map, `0x1018`) | Remap PDO1 like `zeroerr_boot_step` | SDO `0x1000`/`0x1018`; vendor OD via EDS or probe |
| Encoder / gearing (counts ↔ mm) | Soft limits + velocity scale | Bench: known motion vs raw counts / `get_state()` height |
| `bottom_height_mm` / `top_height_mm` | Mechanical end-of-travel | Limit-switch/homing behavior or hand measure |
| What `recalibrate()` does | Homing vs clear-faults | Watch CAN (or Feather FW) during real call |
| Direction sign at wire | Python `+velocity = up` may not match CANopen | First-motion test, unloaded |

## Phase 0 checklist (ordered)

1. Wire lift CAN H/L (+ GND, + power per drive) to Controls PCB **CH3**.
2. Node/baud scan: adapt `legacy/damiao_scan.py --discover --host-only` (or
   `canopen_sdo_read_u32(..., 0x1000, ...)`) — try **1 Mbps** first, then
   other CANopen bauds if silent.
3. On first response: read `0x1000` (device type) and `0x1018` (identity) —
   confirm CiA-402 / vendor; chase EDS/datasheet if identifiable.
4. If no EDS: probe CiA-402 standards — `0x6060`, `0x606C`, `0x6064`,
   `0x60FF`, `0x607A`, `0x6040`/`0x6041`.
5. Measure counts↔mm and travel limits (table above).

## After Phase 0 (pointer only — teardown §6 Phases 1–3)

- **Phase 1:** `PROTO_LIFT` plugin mirroring `zeroerr.c`, Profile Velocity
  (`0x03`), slot 20 CFG enable.
- **Phase 2:** `PcbPlatformClient.lift_cmd` / `get_state()` real wire-up.
- **Phase 3:** docs + `vbeta_lift_smoke.py` + fake-hub tests.

## Safety (Phase 0 callouts)

Load-bearing torso actuator — first motion **unloaded**, physical E-stop /
power cutoff in hand. PDB hard-ESTOP is not a delivered safety net yet
([`pdb-uart-v1.md`](pdb-uart-v1.md)). Keep slot 20 CFG-disabled until
discovery is far enough for a trusted node ID + scale.
