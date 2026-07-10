# Agent plan: Damiao DM-J4340 + multi-slot YAM bring-up

**Audience:** Cursor agent continuing Damiao work while the primary owner is OOO / on YAM bench.  
**Date context:** Jul 2026 — mixed std+ext FDCAN and DM-J4310 plant path are working; **DM-J4340 discover/teleop is the open gap**.  
**Style:** minimize changes to the working stack. Prefer host `config set` over firmware edits. No FreeRTOS. No IK / 6DOF Cartesian teleop.

---

## 0. Agent prompt (paste)

```
Bring up Damiao DM-J4340 on the controls PCB and map YAM arm joints to plant
actuator slots. Follow docs/plan-damiao-4340-bringup.md end-to-end.

Constraints (do not violate):
- Keep USE_FREERTOS_SCHEDULER=0.
- Do not change FDCAN mixed-bus filters, fan-out, or can_router RX path unless
  a bench failure proves they are broken for 4340.
- Prefer runtime NVM config (control_hub.py config set --persist) over editing
  plant_config_nvm.c factory defaults.
- ACTUATOR_COUNT stays 6 unless the user explicitly asks to expand the host
  exchange schema (562 B image + host ACTUATOR_COUNT).
- Reuse PROTO_DAMIAO / damiao.c — do not add a separate "4340 protocol" unless
  the MIT / reg-scan wire format is proven incompatible.
- Joint-space teleop only (teleop --slot / --plant-teleop). No IK.
- Update docs/bringup.md with IDs found and pass/fail; keep changes small.

Start with §2 discovery. Only touch firmware after isolate tests show a real
protocol/limit mismatch (e.g. different P_MAX), not for "more slots."
```

---

## 1. Ground truth (do not re-litigate)

| Item | State |
|------|--------|
| Mixed CH1/CH3 std+ext | Working — [fdcan-dual-id-mixed-bus.md](fdcan-dual-id-mixed-bus.md) §0 |
| DM-J4310 discover on CH1 | OK via reg-scan |
| DM-J4340 discover | **FAIL** on `--host-only --start 1 --end 16` (Jul 2026) |
| Homing configured 4310s | OK |
| Teleop with un-enabled 4340s mid-daisy | 4310s **fault** — every motor on the physical chain that must pass traffic should be discovered + enabled in plant slots (or physically bypassed) |
| Plant loop | Bare-metal superloop; `lap≈0–1 ms` with 3 FDCAN motors |
| Slot budget | **`ACTUATOR_COUNT = 7`** (`App/Inc/plant/actuator.h` + `scripts/controls_pcb_host/protocol/schema.py`); wire image still 25 slots / 562 B |

Factory defaults (`plant_config_load_factory_defaults`):

| Slot | Bus | Protocol | ID |
|------|-----|----------|-----|
| 0 | CH1 | Damiao | `0x06` |
| 1 | CH2 | RobStride | `0x70` |
| 2 | CH3 | RobStride | `0x75` |
| 3–5 | CH4–6 | RobStride | MCP |

YAM work typically **repurposes slots via NVM** so several Damiao motors share **CH1** (daisy). RobStride slots can be disabled or moved aside for the arm session.

---

## 2. Goals (ordered)

1. **Find every Damiao ESC_ID on the YAM / CH1 harness** (4310 + 4340).
2. **Assign each needed joint to a plant slot** (`protocol=damiao`, `bus=1`, unique `motor_id`) and `--persist`.
3. **Single-joint teleop** each slot (`teleop --slot N`) — prove enable + `fb` tracks.
4. **Multi-slot plant teleop** for the YAM set — home + gentle jog (joint-space).
5. Document joint ↔ slot ↔ ESC_ID in [bringup.md](bringup.md).

**Non-goals:** FreeRTOS, CubeMars, SPI temp, IK, expanding beyond 6 slots, rewriting `damiao.c` “for cleanliness.”

---

## 3. How to add Damiao actuator slots (minimal path)

### 3.1 Preferred: runtime config only (no firmware rebuild)

The working stack already supports Damiao on any FDCAN bus (1–3) via CFG PDU + flash NVM.

```powershell
# Show current MCU table
python scripts/control_hub.py config show --port COM5

# Map a slot to Damiao on CH1 (example IDs — replace with discover results)
python scripts/control_hub.py config set --port COM5 --slot 0 --protocol damiao --bus 1 --motor-id 0x01 --persist
python scripts/control_hub.py config set --port COM5 --slot 1 --protocol damiao --bus 1 --motor-id 0x02 --persist
python scripts/control_hub.py config set --port COM5 --slot 2 --protocol damiao --bus 1 --motor-id 0x06 --persist
# ... up to slot 5 if needed

# Optional: disable unused RobStride slots so they do not TX on other buses
python scripts/control_hub.py config set --port COM5 --slot 3 --enabled 0 --persist
```

Rules:

- **One ESC_ID per slot**; never two slots with the same `(bus, motor_id)`.
- **All motors that remain electrically on the daisy** should either be assigned+enabled or known-idle; unconfigured mid-chain 4340s previously caused teleop faults on neighbors.
- `master_id`: leave default / AUTO unless discover prints a Master ID that must be fixed; Damiao plugin accepts AUTO/`0`.
- Host mirror: after `config show` / set, host table syncs from MCU — do not hand-edit `actuator_config.py` unless factory defaults must change for a fresh board with empty NVM.

### 3.2 When to edit firmware factory defaults

Only if a **blank NVM** board must boot into the YAM map without a host `config set` session:

- Edit `plant_config_load_factory_defaults()` in `App/Src/plant/plant_config_nvm.c`
- Mirror the same rows in `scripts/controls_pcb_host/actuator_config.py` `_DEFAULT_TABLE`
- Reflash

Do **not** change `ACTUATOR_COUNT` for “we have 8 motors.” Options if >6 motors on the arm:

| Approach | Change size |
|----------|-------------|
| **A. Session subsets** — NVM map of 6; swap configs for other joints | Minimal (preferred) |
| **B. Expand `ACTUATOR_COUNT`** | Large — host schema, 562 B packing, feedback arrays, tests |
| **C. Physical subset** — only wire 6 drives for first bring-up | None |

Default for this plan: **A or C**. Ask the user before B.

### 3.3 Do not add a second Damiao protocol

`PROTO_DAMIAO` + `damiao.c` already speak MIT + `0x7FF` reg-scan (std CAN). 4340 should use the same plugin if Assistant confirms classic CAN MIT at 1 Mbps.

Firmware touch allowed **only if** bench proves mismatch, e.g.:

- Different MIT `P_MAX` / `V_MAX` / `T_MAX` than `DM4310_*` in `App/Inc/plant/plugins/damiao.h` — then add `DM4340_*` constants and select by config flag **or** widen limits carefully; avoid breaking 4310.
- Different register map for ESC_ID read — extend probe RID list; keep one plugin.

---

## 4. Discovery procedure (4340 focus)

### 4.1 Full per-ID scan (lists many motors)

`control_hub.py discover` returns **first hit only**. Use expert scan:

```powershell
python scripts/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 1 --end 16
# If 4340 still missing, widen:
python scripts/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 32 --listen-ms 60
python scripts/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 127 --listen-ms 60
```

Record every `FOUND` line: `esc_id`, `master_rx`, which physical motor (4310 vs 4340) if labeled on the harness.

### 4.2 Isolate a single 4340 (if scan still empty)

1. Damiao Assistant + USB2CAN on **one** 4340: note ESC_ID, Master ID, baud (must be **1 Mbps** to match `fdcan.c`).
2. Probe that ID only:

```powershell
python scripts/control_hub.py probe --port COM5 --protocol damiao --bus 1 --id 0xNN
python scripts/damiao_scan.py --port COM5 --probe-id 0xNN --bus 1 --reg-scan
python scripts/damiao_scan.py --port COM5 --probe-id 0xNN --bus 1 --enable --hold-ms 2000
```

3. If Assistant works but MCU `rx_raw=0`: termination / wiring (120 Ω at bus ends), not plugin logic.
4. If MCU TX OK but wrong baud on motor: fix motor config in Assistant; do not change FDCAN timing for one model unless the whole bus moves.

### 4.3 Hypotheses if 4340 never answers reg-scan

| Hypothesis | Next check |
|------------|------------|
| ESC_ID outside 1..16 | Widen `--start/--end` |
| Not at 1 Mbps | Assistant bitrate |
| Needs enable before param read | `--enable` then MIT/`--mit-fallback` |
| Different param RID | Compare 4310 vs 4340 in vendor PDF; extend `DM_REG_*` only if needed |
| Silent on daisy until neighbors enabled | Isolate single motor on CH1 |

---

## 5. Enable + joint-space teleop (no IK)

After slots are mapped and persisted:

```powershell
python scripts/control_hub.py config show --port COM5
python scripts/control_hub.py recover --port COM5 --bus 1

# One joint at a time (safest)
python scripts/control_hub.py teleop --port COM5 --slot 0
python scripts/control_hub.py teleop --port COM5 --slot 1

# Multi-joint joint-space (arrows move active bus; key 1 = CH1 only)
python scripts/control_hub.py --port COM5 --plant-teleop --plant-slots 0,1,2,3
```

Pass criteria per slot:

- Feedback syncs; after home, arrow jog moves shaft
- `lap≈0–1 ms`, `pend` not pegged
- No cascade faults on other Damiao slots

If multi-slot faults but single-slot works: check mid-chain motor not in `--plant-slots` / not enabled; add it or power it down off-bus.

---

## 6. Allowed vs forbidden code changes

### Allowed (small)

| Area | Change |
|------|--------|
| Docs | `bringup.md` ID table, 4340 status |
| Host defaults | `_DEFAULT_TABLE` only if factory must match YAM |
| `damiao.h` / MIT limits | Only after measured 4340 range differs |
| Discover UX | Optional: print all hits in `damiao.discover` / expert path (nice-to-have, not required) |
| Tests | Host unit tests for config validation if you add helpers |

### Forbidden without explicit user ask

- `USE_FREERTOS_SCHEDULER=1`
- Changing `StdFiltersNbr` / fan-out / `actuator_dispatch_bus_rx`
- Raising `ACTUATOR_COUNT` / host 562 B layout
- New `PROTO_DAMIAO_4340`
- IK, Cartesian teleop, dashboard work
- CubeMars / SPI temp in the same PR

---

## 7. Verification checklist

```
[ ] damiao_scan --host-only lists all expected ESC_IDs (4310 + 4340)
[ ] Each YAM joint has a unique slot: damiao, bus 1, motor_id=ESC_ID, --persist
[ ] config show matches harness labels
[ ] teleop --slot N OK for each mapped joint
[ ] --plant-teleop --plant-slots … homes and jogs without mid-chain faults
[ ] bringup.md updated with joint map + 4340 discover notes
[ ] No unrelated refactors in the diff
```

---

## 8. Key files (read before editing)

```
docs/bringup.md                          # status + operator commands
docs/fdcan-dual-id-mixed-bus.md          # §0 FIFOs/filters — do not break
docs/plan-damiao-4340-bringup.md         # this plan
App/Inc/plant/actuator.h                 # ACTUATOR_COUNT = 7
App/Src/plant/plant_config_nvm.c         # factory defaults + CFG 3 B/slot GET
App/Src/plant/plugins/damiao.c           # MIT + reg-scan (reuse)
App/Inc/plant/plugins/damiao.h           # DM4310_* limits
scripts/controls_pcb_host/actuator_config.py
scripts/controls_pcb_host/plugins/damiao.py
scripts/damiao_scan.py / damiao_expert.py
scripts/control_hub/teleop/plant.py      # joint-space teleop only
```

---

## 9. Handoff note for MechE / software (no IK)

Joint-space only:

```powershell
python scripts/control_hub.py teleop --port COM5 --slot N
python scripts/control_hub.py --port COM5 --plant-teleop --plant-slots 0,1,2,...
```

Keys: `1` = CH1 group, arrows jog, `q` quit. Cartesian / 6DOF is out of scope for this board bring-up.
