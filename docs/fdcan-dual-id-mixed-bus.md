# FDCAN dual ID-type (11-bit + 29-bit) on one physical bus

**Status (Jul 2026):** Implemented and bench-verified. **CH1, CH2, and CH3** use mixed std+ext when Damiao (std) and RobStride (ext) may share a branch. Older notes that said “CH2 remains ext-only” are **obsolete** (dual-arm Damiao on CH2 required mixed filters). Short summary: [lessons.md](lessons.md). Detailed FIFO / filter / demux model follows.

---

## 0. As-built bringup — FIFOs, filters, demux

### 0.1 What “mixed frame” means here

Classic CAN only (`FDCAN_FRAME_CLASSIC`). On one twisted pair:

| Protocol | Wire ID type | Typical IDs |
|----------|--------------|-------------|
| Damiao | **11-bit standard** (`CAN_ID_STD`) | ESC ID, `0x7FF` param read, Master ID replies |
| RobStride | **29-bit extended** (`CAN_ID_EXT`) | Packed comm-mode + motor ID |

The IDE bit in the CAN header is the only wire-level discriminator. Firmware never collapses std into a 29-bit namespace.

### 0.2 Storage layers (three places a frame can live)

```
Wire ──► FDCAN HW RX FIFO0 ──► software rx_rings[bus] ──► actuator_dispatch_bus_rx
              ▲                         ▲
         Message RAM              RAM ring (depth 128)
         (peripheral)             can_frame_t {id, id_type, dlc, data}

TX path (reverse):
actuator plugin ──► tx_queues[bus] ──► FDCAN HW TX FIFO/Queue ──► Wire
```

| Layer | Where | Depth / notes |
|-------|--------|----------------|
| **HW RX FIFO0** | STM32 FDCAN Message RAM | Default Cube FIFO0; **both** std and ext accepted frames land here (not split by ID type). Firmware only reads `FDCAN_RX_FIFO0` — FIFO1 unused. |
| **SW RX ring** | `rx_rings[CAN_FDCAN_COUNT]` in `can_router.c` | 128 × `can_frame_t` per FDCAN bus (CH1–3). Head/tail circular; overrun drops oldest. |
| **SW TX queue** | `tx_queues[CAN_FDCAN_COUNT]` | Same depth; plugins enqueue, `can_tx_flush` / poll pushes to HW. |
| **HW TX FIFO** | `TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION` | Peripheral TX FIFO/queue; `HAL_FDCAN_AddMessageToTxFifoQ`. |

**Important:** There is **not** a separate HW FIFO for standard vs extended. Both frame types share RX FIFO0. Separation is carried in each element’s `IdType` field when the HAL pops the message.

MCP CH4–6 are a different backend (`spi_can_router` + MCP2518 RX FIFO1). They
install **dual accept-all filters** (std + ext, MIDE=1) so Damiao (11-bit) and
RobStride (29-bit) can share a rail — same demux rules as FDCAN (`id_type` +
plugin `parse_rx`).

### 0.3 How the peripheral decides std vs ext (filters + bitmasks)

Cube allocates filter banks in Message RAM:

| Instance | Schematic | `StdFiltersNbr` | `ExtFiltersNbr` | RX mode |
|----------|-----------|-----------------|-----------------|---------|
| `hfdcan1` | CH1 | **1** | **1** | `FDCAN_RX_STD_AND_EXT` |
| `hfdcan3` | CH2 | 0 | 0 | `FDCAN_RX_EXT_ONLY` (global accept-ext) |
| `hfdcan2` | CH3 | **1** | **1** | `FDCAN_RX_STD_AND_EXT` |

On mixed buses, `fdcan_bus_start()` installs **two** mask filters, both routed to RX FIFO0:

```c
/* Accept-all standard: ID=0, mask=0 → every bit "don't care" */
filter.IdType       = FDCAN_STANDARD_ID;
filter.FilterType   = FDCAN_FILTER_MASK;
filter.FilterID1    = 0;   /* ID   */
filter.FilterID2    = 0;   /* mask — 0 means match any 11-bit ID */
filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

/* Accept-all extended: same pattern for 29-bit */
filter.IdType       = FDCAN_EXTENDED_ID;
filter.FilterID1    = 0;
filter.FilterID2    = 0;
```

**Mask semantics (FDCAN_FILTER_MASK):** for each ID bit, `mask bit = 0` → don’t care; `mask bit = 1` → must equal ID bit. `FilterID1=0` + `FilterID2=0` is therefore **accept all** of that ID type.

**Global filter** (non-matching frames) on mixed buses:

```c
HAL_FDCAN_ConfigGlobalFilter(h,
    FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching standard */
    FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching extended */
    FDCAN_FILTER_REMOTE,
    FDCAN_FILTER_REMOTE);
```

CH2 ext-only uses global `REJECT` for std + `ACCEPT_IN_RX_FIFO0` for ext (with `ExtFiltersNbr=0` the configured mask filter is inert — the global filter is what actually accepts RobStride).

### 0.4 How software identifies the frame type after RX

`fdcan_backend_recv()` pops **only** FIFO0 and copies HAL `IdType` into `can_frame_t`:

```c
frame->id      = rx_header.Identifier;
frame->id_type = (rx_header.IdType == FDCAN_EXTENDED_ID)
                 ? CAN_ID_EXT : CAN_ID_STD;
```

ID width masks (`can_frame.h`):

| Constant | Value | Use |
|----------|-------|-----|
| `CAN_STD_ID_MASK` | `0x7FF` | 11-bit field on TX/RX |
| `CAN_EXT_MASK` | `0x1FFFFFFF` | 29-bit field on TX/RX |

Optional debug / map key (does **not** go on the wire):

```c
can_route_key(f):
  EXT → 0x80000000 | (id & CAN_EXT_MASK)
  STD → id & CAN_STD_ID_MASK
```

So numeric ID `0x06` std and `0x06` ext are distinct keys.

### 0.5 Plant demux (fan-out) — why filters alone were not enough

Hardware acceptance gets both types into `rx_rings[bus]`. Demux is **software**:

1. Each 500 Hz tick builds a **`poll_buses` bitmask** — bit `N` set if any enabled slot on `CAN_BUS_CH{N+1}` needs TX/RX this tick (`actuator_apply_desire`).
2. For each set bit: `can_router_poll_bus(bus)` drains HW FIFO0 → SW ring.
3. **`actuator_dispatch_bus_rx(bus)`** pops each SW frame **once** and offers it to **every** enabled slot on that bus:
   - Damiao `parse_rx`: rejects if `id_type != CAN_ID_STD`, then matches ESC/Master ID.
   - RobStride `robstride_on_rx_frame`: rejects if `id_type != CAN_ID_EXT`, then matches motor / comm; pararead fallback on same copy.

Before fan-out, each plugin’s `while (can_rx_pop)` **consumed the whole ring**, so the second protocol on a shared bus starved. That was the second bring-up fix after dual filters.

### 0.6 TX path (unchanged model)

Plugins set `id_type` when packing:

- Damiao → always `CAN_ID_STD`
- RobStride → always `CAN_ID_EXT`

`fdcan_backend_send()` maps that to `FDCAN_TxHeaderTypeDef.IdType` and masks the identifier. Shared `tx_queues[bus]` + HW arbitration; no firmware merge of std/ext streams.

### 0.7 Host policy

`scripts/controls_pcb_host/protocol/can_bus.py`:

- `FDCAN_MIXED_BUSES = {1, 3}`
- `is_fdcan_mixed_bus(bus)` for tooling

Plant teleop no longer hard-rejects RobStride on CH3 or Damiao on CH1; slot config (NVM / `config set`) is the source of truth.

### 0.8 Bench outcome (Jul 2026)

| Check | Result |
|-------|--------|
| Dual filters on CH1/CH3 | OK — std Damiao + ext RS RX |
| Fan-out multi-slot same bus | OK — Damiao + RS on CH1 |
| Multi-bus plant teleop CH1–3 | OK — `lap≈0–1 ms` typical |
| Damiao-only daisy (4310) | Homing OK; teleop blocked when un-enabled 4340s sit mid-chain (harness, not FIFO) |

Operator notes: [bringup.md](bringup.md) § Damiao CH1 daisy chain / Mixed std/ext.

---

## Agent prompt (historical — paste this to the implementing agent)

```
Implement mixed standard (11-bit) and extended (29-bit) classic CAN on shared FDCAN
buses in DeftRoboticsControlsPCB. Read and follow the full spec in:
  docs/fdcan-dual-id-mixed-bus.md

Goal: CH3 (and optionally CH1/CH2) must RX and TX both frame types so Damiao
(std) and RobStride (ext) — and later CANopen (std) — can coexist on the same
harness when configured on different actuator slots sharing one schematic bus.

Do NOT collapse id_type into a single 29-bit namespace. Keep can_frame_t.id_type.

Deliverables:
1. FDCAN init: dual std+ext accept filters + global filter on mixed buses (CH3 first).
2. can_router: FDCAN_FILTER_DUAL mode; update fdcan_mode_for_bus() policy.
3. RX fan-out: fix destructive can_rx_pop drain so multiple actuator slots on the
   same bus each get matching frames (actuator.c + refactor robstride/damiao apply
   paths to use shared dispatcher — see §4).
4. Host: relax assert_plant_teleop_slot / is_rs02_plant_bus CH3 exclusion once
   firmware path works; allow --plant-teleop --plant-slots with mixed protocols on
   different buses AND same bus (after fan-out).
5. Unit-style firmware tests if feasible; otherwise document bench commands in spec.
6. Update docs/lessons.md / bringup.md with mixed-bus bringup notes.

Constraints:
- Minimize scope: classic CAN only (FDCAN_FRAME_CLASSIC), no CAN-FD changes.
- MCP CH4–6 (SPI) unchanged unless trivially reusable.
- Match existing code style; no over-abstraction.
- CubeMX fdcan.c: bump StdFiltersNbr/ExtFiltersNbr only where required (§3.2).
- Bench probes (diag_dm, robstride_probe_id) must still work on mixed buses.

Verify on bench (user will run — document commands, do not assume hardware):
  discover/probe both protocols on CH3 after config set
  --plant-teleop --plant-slots with damiao + robstride on same and different buses

If a step is blocked (e.g. filter bank limits), document blocker in the PR/spec
and implement the smallest working subset (CH3 dual-RX + fan-out).
```

---

## 1. Problem statement

> **Historical.** §0 documents the as-built fix. This section describes the pre-fix failure mode.

### 1.1 Behavior before mixed bring-up

| Layer | Behavior |
|-------|----------|
| **FDCAN CH1/CH2** | Extended-only RX filter (`FDCAN_FILTER_EXT_ALL`). RobStride OK. Standard frames **rejected** at peripheral. |
| **FDCAN CH3** | Standard-only RX filter (`FDCAN_FILTER_STD_ALL`). Damiao OK. Extended frames **rejected** at peripheral. |
| **TX** | Already per-frame: `fdcan_backend_send()` uses `can_frame_t.id_type` (std or ext). **No TX change required** beyond testing. |
| **Host** | `assert_plant_teleop_slot()` in `scripts/control_hub/link.py` enforces Damiao→CH3 only, RobStride→CH1/CH2/CH4–6 only. |
| **Plugins** | `damiao_parse_rx` requires `CAN_ID_STD`; `robstride_parse_rx` requires `CAN_ID_EXT`. Already correct for demux **if** both reach the ring. |

### 1.2 Why mixed bus fails today (even if host check removed)

1. **Hardware filter:** CH3 drops all 29-bit RX before software sees them.
2. **Software drain:** `damiao_apply_cycle()` and `robstride_apply_drain_rx()` each call `while (can_rx_pop(bus, &frame))` and **consume the entire RX ring** for that bus. If two actuator slots share a bus (different protocols), the first plugin in the actuator loop eats frames meant for the second.

### 1.3 Target behavior

- One physical CAN branch (e.g. schematic **CH3**, `CAN_BUS_CH3`, `hfdcan2`) accepts **both** standard and extended classic CAN frames into `rx_rings[bus]`.
- Multiple `actuator_table[]` slots may reference the **same** `bus` with **different** `protocol` values.
- Each slot only updates its `actuator_state_live[i]` when its plugin’s `parse_rx` accepts the frame (`PLUGIN_OK`).
- Host plant teleop may combine slots across buses without artificial “CH3 = Damiao only” policy (config table is source of truth).
- Foundation for future **CANopen 2.0** (11-bit) on the same wires as RobStride extended traffic.

### 1.4 Non-goals (this task)

- CAN-FD / BRS.
- CANopen protocol plugin implementation (only RX path readiness).
- Changing MCP2518 SPI-CAN behavior (already ext; different backend).
- Merging `id_type` into a single 29-bit ID field (“ignore upper 15 bits”) — **rejected**; see §2.3.

---

## 2. Design principles

### 2.1 Physical bus vs logical bus

- **Physical:** One twisted pair, one transceiver, one FDCAN peripheral instance.
- **Logical:** `can_bus_id_t` (`CAN_BUS_CH1` … `CAN_BUS_CH3`) maps to `bus_handle[]` in `can_router.c`.
- **Schematic CH3** = `CAN_BUS_CH3` = `&hfdcan2` (not Cube “FDCAN3” label — see comments in `can_router.c`).

### 2.2 Frame model (unchanged)

```c
typedef struct {
    uint32_t id;
    can_id_type_t id_type;  // CAN_ID_STD or CAN_ID_EXT
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;
```

TX and RX paths must preserve `id_type` from HAL (`FDCAN_RxHeaderTypeDef.IdType` / `FDCAN_TxHeaderTypeDef.IdType`).

### 2.3 ID routing key (software only)

Do **not** treat standard IDs as extended with zero high bits on the wire. For dispatch maps / debug, optional unified key:

```c
static inline uint32_t can_route_key(const can_frame_t *f) {
    if (f->id_type == CAN_ID_EXT)
        return 0x80000000u | (f->id & CAN_EXT_MASK);
    return f->id & CAN_STD_ID_MASK;
}
```

Plugins continue to gate on `id_type` first; RobStride and Damiao ID spaces remain separate by protocol.

### 2.4 Bus capability enum

Introduce explicit per-bus RX capability (firmware):

```c
typedef enum {
    FDCAN_RX_EXT_ONLY = 0,   // CH1/CH2 default (keep unless product asks)
    FDCAN_RX_STD_ONLY,       // legacy CH3 (remove after dual validated)
    FDCAN_RX_STD_AND_EXT,    // mixed bus
} fdcan_rx_mode_t;
```

**As-built policy:** CH1 and CH3 → `FDCAN_RX_STD_AND_EXT`; CH2 → `FDCAN_RX_EXT_ONLY`. See §0.3.

```c
static fdcan_rx_mode_t fdcan_rx_mode_for_bus(can_bus_id_t bus) {
    switch (bus) {
    case CAN_BUS_CH1:
    case CAN_BUS_CH3:
        return FDCAN_RX_STD_AND_EXT;
    default:
        return FDCAN_RX_EXT_ONLY;  // CH2
    }
}
```

**Rollout note (original):** CH1 moved to mixed when Damiao was assigned there; CH2 stays ext-only unless CANopen/Damiao is needed on that branch.

---

## 3. FDCAN peripheral changes

### 3.1 Files

| File | Change |
|------|--------|
| `App/Src/plant/can/can_router.c` | Dual filter init, `fdcan_bus_start()` refactor |
| `Core/Src/fdcan.c` | Filter bank counts for `hfdcan2` (minimum) |
| `App/Inc/plant/can/can_router.h` | Optional: export `can_router_fdcan_rx_mode()` for debug |

### 3.2 Cube / `fdcan.c` init

Today all three instances have:

```c
hfdcanX.Init.StdFiltersNbr = 0;
hfdcanX.Init.ExtFiltersNbr = 0;
```

For **dual mode** the HAL requires at least **one standard filter element and one extended filter element** configured before `HAL_FDCAN_Start`.

**Required change for `hfdcan2` (CH3):**

```c
hfdcan2.Init.StdFiltersNbr = 1;
hfdcan2.Init.ExtFiltersNbr = 1;
```

Leave CH1/CH3 as ext-only filter counts unless you also enable dual on those buses.

Re-run Cube code gen carefully: preserve user sections in `fdcan.c` / `HAL_FDCAN_MspInit` if regenerating; or hand-edit counts only.

### 3.3 `fdcan_bus_start()` — add `FDCAN_FILTER_DUAL`

Replace mutually exclusive std **or** ext setup with:

```c
static void fdcan_bus_start(FDCAN_HandleTypeDef *h, fdcan_rx_mode_t mode)
{
    FDCAN_FilterTypeDef filter = {0};

    if (mode == FDCAN_RX_STD_AND_EXT) {
        /* Standard filter 0: accept all std → FIFO0 */
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = 0;
        filter.FilterID2 = 0;
        HAL_FDCAN_ConfigFilter(h, &filter);

        /* Extended filter 0: accept all ext → FIFO0 */
        filter.IdType = FDCAN_EXTENDED_ID;
        filter.FilterIndex = 0;
        HAL_FDCAN_ConfigFilter(h, &filter);

        HAL_FDCAN_ConfigGlobalFilter(h,
            FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching standard */
            FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching extended */
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE);
    } else if (mode == FDCAN_RX_STD_ONLY) {
        /* existing FDCAN_FILTER_STD_ALL branch */
    } else {
        /* existing FDCAN_FILTER_EXT_ALL branch */
    }

    HAL_FDCAN_Start(h);
}
```

**Validation:** Confirm against STM32G474 reference / existing comment at `can_router.c:150` (ext-only path needed global filter because `ExtFiltersNbr=0` made mask filter inert). Dual mode **must** be tested with a loopback or bench motor — do not assume filter config without RX traffic.

### 3.4 `can_router_init()` / restart

```c
fdcan_bus_start(&hfdcan2, fdcan_rx_mode_for_bus(CAN_BUS_CH3));
```

`can_router_restart_fdcan(bus)` must use the same mode policy (used after bench sessions / bus recovery).

### 3.5 TX path

**No code change expected.** Verify:

- Damiao TX: `CAN_ID_STD`, IDs e.g. `0x7FF`, motor ID.
- RobStride TX: `CAN_ID_EXT`, packed 29-bit comm IDs.

Optional bench test: enqueue one std and one ext frame on CH3 in diag code; scope or CAN analyzer if available.

---

## 4. RX fan-out (critical)

### 4.1 Bug

`actuator_apply_desire()` loop order:

1. For each slot: `robstride_apply_cycle` or `damiao_apply_cycle` → each drains **all** `can_rx_pop(bus)`.
2. Later generic `plugin_parse_rx` loop skips RobStride/Damiao.

If slot 1 (RobStride) and slot 2 (Damiao) share `CAN_BUS_CH3`, whichever `apply_cycle` runs first **removes** frames the other needs.

Bench probe paths (`damiao_probe_listen_window`, `robstride_probe_id`) have the same pattern but only one protocol active per session — OK for probes.

### 4.2 Recommended design: `actuator_bus_dispatch_rx()`

**New function** in `App/Src/plant/actuator.c` (or `can_router.c` if preferred — actuator is better since it knows slots):

```c
/*
 * After can_router_poll_bus* for a bus, dispatch every queued RX frame to all
 * enabled actuator slots on that bus. Each plugin parse_rx ignores non-matching
 * id_type / motor_id. Frame is removed from ring once (single consumer).
 */
static void actuator_dispatch_bus_rx(can_bus_id_t bus)
{
    can_frame_t frame;

    while (can_rx_pop(bus, &frame) == CAN_OK) {
        for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
            if (!actuator_table[i].enabled)
                continue;
            if (actuator_table[i].bus != bus)
                continue;
            (void)plugin_parse_rx(&actuator_table[i], &frame,
                                  &actuator_state_live[i]);
        }
    }
}
```

**Refactor apply cycles:**

| Location | Change |
|----------|--------|
| `damiao_apply_cycle()` | Remove inner `while (can_rx_pop...)`. After `can_router_poll_bus(bus)`, call `actuator_dispatch_bus_rx(bus)` **OR** only parse for this slot via `plugin_parse_rx(cfg, ...)` inside a **non-destructive** peek API (see §4.3). |
| `robstride_apply_cycle()` / `robstride_apply_drain_rx()` | Same: stop exclusive drain; rely on shared dispatch. |

**Preferred integration (single drain per bus per tick):**

At end of `actuator_apply_desire()`, after all TX enqueue + `can_router_poll_bus`:

```c
for (each bus in poll_buses mask)
    actuator_dispatch_bus_rx(bus);
```

Then **remove** RX drain loops from inside `damiao_apply_cycle` and `robstride_apply_cycle` (keep TX + enable logic there).

**RobStride pararead side channel:** `robstride_apply_drain_rx()` has extra logic for promiscuous pararead frames after failed `plugin_parse_rx`. Options:

- **A)** Move pararead handling into dispatch: if `plugin_parse_rx` fails for RS slot, call existing `robstride_try_pararead_capture()` helper with same frame (only for RS slots).
- **B)** Peek-based drain: dispatch pops once, passes frame to all slots, RS slot runs extended pararead fallback on same copy.

Document chosen approach in PR. Do not duplicate frames on the wire — only duplicate **in RAM** per pop.

### 4.3 Alternative: `can_rx_peek` + selective pop

Higher complexity; use only if multi-slot dispatch must not call `parse_rx` multiple times per frame for performance. Not recommended for v1 (6 slots, 500 Hz).

### 4.4 Probe / diag paths

`damiao_probe_listen_window` and `robstride_probe_id` run **outside** plant actuator loop with a single protocol. They may keep exclusive `can_rx_pop` loops **or** call a shared `bus_drain_for_probe(bus, matcher_fn)` — **do not** call `actuator_dispatch_bus_rx` during blocking probes unless bench sessions are isolated (they are: `plant_diag_skip_actuator_can`).

**Rule:** Plant path uses fan-out; bench probe path may keep dedicated drain (single consumer) — no change required for v1 if probes remain session-gated.

### 4.5 MCP backends

`spi_can_router_rx_pop` unchanged. Fan-out applies only to `bus < CAN_FDCAN_COUNT`.

---

## 5. Actuator / plugin edge cases

### 5.1 Same bus, two RobStride motors

Already supported: both ext; `parse_rx` filters by motor ID / comm mode. Fan-out delivers same frame to both slots; only matching slot updates state.

### 5.2 Same bus, Damiao + RobStride

After §3 + §4: Damiao std feedback and RobStride ext feedback both reach the ring; each slot ignores wrong `id_type`.

### 5.3 TX collision

Both plugins enqueue to same `tx_queues[bus]`. Arbitration is CAN hardware — no firmware merge needed. Ensure queue depth (`CAN_QUEUE_DEPTH` 128) sufficient for 500 Hz × multiple slots.

### 5.4 `damiao_enable_latched`

Unchanged. Damiao slot still needs `apply_cycle` every tick while idle (see `actuator.c` blank-desire exemption for `PROTO_DAMIAO`). Fan-out does not replace Damiao TX/enable path.

### 5.5 Bitrate

All devices on a mixed bus must share nominal bit timing (`fdcan.c` prescaler/seg). CANopen + RobStride at same 1 Mbps classic CAN is fine; do not mix bitrates.

---

## 6. Host / Python changes

### 6.1 Files

| File | Change |
|------|--------|
| `scripts/controls_pcb_host/protocol/can_bus.py` | Add `FDCAN_MIXED_BUSES = frozenset({3})` or document CH3 as std+ext capable. Optionally add `is_fdcan_mixed_bus(bus)`. |
| `scripts/control_hub/link.py` | `assert_plant_teleop_slot()` |
| `scripts/control_hub/teleop/plant.py` | Assert loop at ~1017: include damiao slots or replace with config validation helper |

### 6.2 Relax plant teleop assertions

**Replace** hard-coded bus↔protocol map with:

```python
def assert_plant_teleop_slot(slot: int, bus: int, protocol_name: str) -> None:
    if protocol_name == "damiao":
        if not is_fdcan_bus(bus):  # 1..3
            raise PlantRuntimeError(...)
        return
    if protocol_name == "robstride":
        if not (is_fdcan_bus(bus) or is_mcp_bus(bus)):
            raise PlantRuntimeError(...)
        return
    raise PlantRuntimeError(f"slot {slot}: protocol {protocol_name!r} has no plant teleop.")
```

Remove `is_rs02_plant_bus` check that excludes CH3. **Keep** MCP/FDCAN distinction for telemetry (`home_on_fb`, etc.).

### 6.3 `warmup_plant_actuators`

Already branches on `protocol_name`. No change except mixed-bus CH3 may arm both Damiao (enable probe) and RS (plant arm) if config has two slots on CH3 — both paths should run.

### 6.4 Config / teleop UX

Document example:

```powershell
python scripts/control_hub.py config set --port COM5 --slot 1 --protocol robstride --bus 3 --motor-id 0x75
python scripts/control_hub.py config set --port COM5 --slot 2 --protocol damiao   --bus 3 --motor-id 0x06
python scripts/control_hub.py --plant-teleop --plant-slots 1,2 --port COM5
```

(User runs on bench; agent documents only.)

---

## 7. Testing plan

### 7.1 Firmware unit / host tests (no hardware)

| Test | Description |
|------|-------------|
| `can_route_key` | Std vs ext keys differ for same numeric 0x06 |
| Host `assert_plant_teleop_slot` | RobStride CH3 no longer raises |
| Optional C test | Mock ring: two slots, one std frame only updates Damiao slot state |

### 7.2 Bench — CH3 dual RX (hardware)

1. Reflash MCU with dual filters on `hfdcan2`.
2. Damiao discover: `control_hub.py discover --protocol damiao --bus 3` → FOUND.
3. RobStride discover on CH3 (if motor wired or second device): `discover --protocol robstride --bus 3`.
4. Bus analyzer / activity LED PB15: traffic on both frame types when alternating probe commands.

### 7.3 Bench — mixed plant teleop

Config: slot A RobStride CH3 + slot B Damiao CH3 (or CH1 RS + CH3 Damiao for simpler v1).

```powershell
python scripts/control_hub.py recover --port COM5 --bus 3
python scripts/control_hub.py --plant-teleop --plant-slots <slots> --port COM5
```

Pass criteria:

- Both slots show `feedback_synced` / non-zero `fb` where motors exist.
- Arrow motion on selected bus moves correct motor only.
- `fault=1` on Damiao when enabled; RS comm mode sane on other slot.
- No regression: CH1/CH2 RS-only teleop still smooth (`lap≈0–1 ms`).

### 7.4 Regression

- Damiao-only CH3 teleop (existing bench path).
- RS2 CH1/CH2 teleop.
- DM bench session (`dm_session_begin` + REG_SCAN) with plant loop idle.
- RS2 bench cal on CH2 after teleop (§8 bringup.md).

---

## 8. Documentation updates

| Doc | Update |
|-----|--------|
| `docs/bringup.md` | Dual-arm map; mixed std/ext on CH1–CH3 |
| `docs/lessons.md` | Mixed-bus + bitrate / termination lessons |
| `docs/fdcan-dual-id-mixed-bus.md` | This spec (as-built §0) |

---

## 9. Implementation checklist (ordered)

- [x] **3.2** `fdcan.c`: `hfdcan1` + `hfdcan2` StdFiltersNbr=1, ExtFiltersNbr=1
- [x] **3.3** `fdcan_bus_start()` dual mode + `fdcan_rx_mode_for_bus()` (CH1 + CH3)
- [x] **3.4** Wire CH1/CH3 to `FDCAN_RX_STD_AND_EXT` in init/restart
- [x] **4.2** `actuator_dispatch_bus_rx()` + remove per-plugin full drain
- [x] **4.2** RobStride pararead fallback via `robstride_on_rx_frame()`
- [x] **6.2** Host assert relaxation + `FDCAN_MIXED_BUSES = {1, 3}`
- [x] **7** Bench verified (mixed teleop; see §0.8)
- [x] **8** Doc updates — §0 as-built; [bringup.md](bringup.md)

---

## 10. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Filter mis-config → no RX | LED PB15 + `raw_frames` in probe PDUs; loopback test frame in diag smoke |
| Fan-out perf at 500 Hz × 6 slots | Only dispatch buses in `poll_buses` mask; max 6 parse_rx per frame |
| ID clash std 0x7FF vs ext | Different `id_type`; plugins gate correctly |
| Cube regen wipes filter counts | Document hand-edit or USER CODE sections |
| User configures two RS same ID same bus | Pre-existing misconfig; optional CFG validation later |

---

## 11. Future: CANopen 2.0

- Add `PROTO_CANOPEN` plugin (std IDs only, NMT/SDO/PDO state machine).
- Same `FDCAN_RX_STD_AND_EXT` bus mode.
- Host teleop/bench routing same fan-out model.
- Consider hardware filter narrowing per protocol only if RX ring overload becomes an issue (not v1).

---

## 12. Key file reference

```
App/Src/plant/can/can_router.c      # Filters, HW FIFO0↔SW rings, TX/RX HAL
Core/Src/fdcan.c                      # StdFiltersNbr / ExtFiltersNbr
App/Src/plant/actuator.c              # poll_buses mask, actuator_dispatch_bus_rx
App/Src/plant/plugins/damiao.c        # std TX/RX; damiao_apply_cycle (TX-only)
App/Src/plant/plugins/robstride.c     # ext TX/RX; robstride_on_rx_frame
App/Inc/plant/can/can_frame.h         # id_type, CAN_STD_ID_MASK, CAN_EXT_MASK, can_route_key
App/Inc/plant/can/can_router.h        # fdcan_rx_mode_t, can_router_fdcan_rx_mode
scripts/control_hub/link.py           # assert_plant_teleop_slot
scripts/controls_pcb_host/protocol/can_bus.py  # FDCAN_MIXED_BUSES
scripts/control_hub/teleop/plant.py   # multi-slot teleop
docs/fdcan-dual-id-mixed-bus.md       # this file — §0 as-built
```

---

## 13. Acceptance criteria (summary)

1. ~~CH3~~ **CH1 and CH3** FDCAN receive **both** standard and extended classic CAN frames into `rx_rings[]` via shared HW RX FIFO0 + SW ring (§0).
2. Two actuator slots on the same bus with different protocols both receive correct feedback without starving each other (`actuator_dispatch_bus_rx`).
3. Host `control_hub.py --plant-teleop` does not reject RobStride on CH3 / Damiao on CH1 by policy alone.
4. Existing single-protocol bench paths (Damiao discover, RS2 CH1/CH2 teleop) remain working.
5. Spec + bringup docs updated — **§0 as-built** is the operator reference for FIFOs / filters / demux.
