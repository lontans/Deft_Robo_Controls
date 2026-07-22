# Handoff: cheap MCP poll → 200–500 Hz plant/host capability (2026-07-22)

**Status:** Good fallback committed/pushed earlier today. Afternoon iteration (TIM6 mount + burst=1 + USB memcpy + skip duplicate end-of-lap FDCAN) makes **all×25 @ 500 Hz host plant-healthy** (`fb~470–492`, `svc≈1`, `pend≤1`). Still uncommitted unless noted. `ack_lag_max` at 500 Hz host remains coalesce-noisy.

**Goal:** Keep *more* bus information *more often* (not RR/decimate as the strategy). Make SPI/USB/plant cheap enough that TIM6 500 Hz and host 200–500 Hz are robust.

---

## What landed (firmware)

### MCP / plant (busy + idle)

| Change | Where | Why |
|--------|-------|-----|
| INT-gated idle RX | `spi_can_router.c` `spi_poll_rx_one` | Skip FIFOSTA when `!int_active && !rx_irq_pending`; clear flag + re-check level (no CS — sticky INT self-heals) |
| `rx_irq_pending` accessors | `mcp2518fd.c` / `.h` | Flag was write-only before |
| Drop discarded FIFOSTA in `mcp2518_poll_rx` | `mcp2518fd.c` | Smoke path paid SPI for unused result |
| Batch 16 B RX/TX RAM | `mcp_hw_pop_rx` / `mcp_hw_txq_load` | One SPI burst vs 4×32-bit |
| TXQ service returns ready; skip `clear_abat` unless aborting | `mcp_txq_service_nonblock` | Share STA; less C1CON SPI |
| `prepare_tx` ↔ `try_send` STA handoff | `s_txq_prep_*` | No double service on same flush |
| **No FRESET on routine full TXQ** | `mcp2518_try_send` | Was catastrophic at MIT÷1 on empty-bus CH4–6×6 |
| TREC ~50 ms rate-limit in `prepare_tx` | `mcp2518fd.c` | Fixed SPI tax every flush |
| Maintain enable non-blocking + MCP coalesce | `robstride.c` | Blocking `send_now`+`HAL_Delay` × N slots pegged laps |
| Poll all commanded buses / tick | `actuator.c` | RR≤3 removed |
| End-of-lap **FDCAN only** | `diag_core.c` | Avoid second MCP SPI pass on busy holds |
| `SPI_POLL_TX_MAX` 4→2 | `spi_can_router.c` | After coalesce flush |
| FDCAN **1 MIT / plant tick** (not burst 3) | `robstride.c` | Burst 3 didn’t raise control rate; dominated all×25 |
| `RS02_MCP_APPLY_DIV` / heavy FDCAN ÷ = **1** | `robstride.c` | Heavy multi-domain effectively **off** |

### Host link (high host Hz)

| Change | Where | Why |
|--------|-------|-----|
| Bulk RX into frame buffer | `host_link_poll_rx` | Was byte-at-a-time |
| Coalesce plant CMD → **latest per lap** | `host_link.c` | 200–500 Hz host must not mount 25 slots for every queued frame; **debug CMD still immediate** |
| Plant FB ≤ **once per TIM6 tick** | `host_link_poll_tx` | Superloop was rebuilding 672 B every spin |

---

## How to build / flash

```powershell
# From repo root — CubeIDE toolchain on PATH (or use IDE build)
cd Debug
make -j8 all

cd ..
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf
```

CDC is typically **COM5** (`VID_0483`/`PID_5740`).

---

## How to test the matrix

```powershell
cd scripts
$env:PYTHONIOENCODING='utf-8'

# Classic matrix (host 40 Hz) — should OVERALL ack_lag OK
python _tmp_mcp_timing_probe.py --port COM5 --seconds 3.0 --hz 40

# Elevated host rate (capability check)
python _tmp_mcp_timing_probe.py --port COM5 --seconds 3.0 --hz 200
```

**Focused CH4–6×6 / all×25 gate** (paste or script):

```powershell
cd scripts
$env:PYTHONIOENCODING='utf-8'
python -c @"
import sys
sys.path.insert(0, '.')
from deft_controls_sdk import ActuatorDesire, ControlsPcbHub
from _tmp_mcp_timing_probe import ensure_product_cfg, measure, slots_for
hold = ActuatorDesire(position=0.01, kp=2.0, kd=0.5)
with ControlsPcbHub.connect('COM5', persist_telemetry=False) as hub:
    by_bus = ensure_product_cfg(hub)
    mcp = {s: hold for s in slots_for(by_bus, (4,5,6))}
    all25 = {s: hold for s in slots_for(by_bus, (1,2,3,4,5,6))}
    measure(hub, 'warm', {}, seconds=0.5, hz=40)
    for name, d, hz in [('CH4-6', mcp, 200), ('all25', all25, 200), ('all25', all25, 500)]:
        r = measure(hub, f'{name}_{hz}', d, seconds=2.0, hz=float(hz))
        print(name, hz, 'fb', r.get('raw_fb_hz'), 'lag_max', r.get('ack_lag_max'), 'ok_lag', r.get('ok_lag'))
"@
```

**RS02 on CH4** (motor on MCP):

```powershell
cd scripts
$env:PYTHONIOENCODING='utf-8'
python rs02_channel_bringup.py --bus 4 --port COM5
# or after cali already done:
python rs02_channel_bringup.py --bus 4 --port COM5 --motor-id 0x70 --skip-cali
```

### Metric caveats at high host Hz

- Prefer **`lap_ms` / `ticks_svc` / `ticks_pending` / `raw_fb_hz`** over raw `ack_lag_max`.
- Plant CMD **coalesce skips seqs** → `ack_lag_max` can spike (10–21) while p95≈1 and plant `svc≈1`. Don’t chase max-lag alone.
- Plant FB is capped ~500 Hz (one per TIM6 tick); blank `fb_hz` ~500–550 is expected (was ~900 when FB spammed every superloop spin).

---

## Updated stats (bench, COM5, after latest flash)

### Matrix `@ --hz 40` (PASS)

| Phase | fb_hz | ack_lag_max | notes |
|-------|------:|------------:|-------|
| blank / CH1–3 / FDCAN | ~508–514 | 0–1 | `svc≈1` |
| **CH4–6 MCP ×6** | **~490** | **0** | was lag 17 / pend 188 |
| **all×25** | **~467** | **0** | was fb ~134 / `svc~4` |
| CH4×2 | ~510 | 0 | |

**OVERALL ack_lag OK: True**

### Capability `@ host 200 Hz` (plant healthy)

| Hold | fb_hz | lag_max | lap mean | svc |
|------|------:|--------:|---------:|----:|
| CH4–6 ×6 | ~489–518 | ≤2 | ~0.4 | ~1.0 |
| all×25 | ~384–411 | ≤2 | ~2.6–2.8 | ~1.2–1.4 |

Matrix `@ --hz 200`: all×25 **lag_ok**; some FDCAN phases fail **only** on `ack_lag_max` spikes (plant p95 fine — coalesce artifact).

### Host 500 Hz (after 2026-07-22 afternoon iteration)

| Hold | fb_hz | svc | pend | Verdict |
|------|------:|----:|-----:|---------|
| CH4–6 ×6 | ~490–495 | ~1.0 | ≤1 | Plant OK; `ack_lag_max` 3–4 is coalesce noise |
| **all×25** | **~470–492** | **~1.0** | **≤1** | **Plant OK** — was fb~63 / svc~8 / pend climb |

Classic matrix `@ --hz 40`: **OVERALL ack_lag OK: True** (all×25 fb~488).
RS02 `--bus 4 --skip-cali` teleop: **OVERALL PASS**.

Remaining toward “ack_lag ≤1 and fb≈500 on all phases”: host coalesce still inflates `ack_lag_max` at 500 Hz TX; plant path is under the TIM6 budget (`lap` mean ~1.1 ms, pend stable).

---

## What landed this afternoon (on top of Claude mount-align)

1. Claude: TIM6-gated plant CMD mount (`host_link_apply_pending_plant`) — alone did **not** fix all×25@500 (still fb~63).
2. `CONTROL_TICK_BURST_MAX` 8→1 — stops FB starvation death spiral (`svc=8` → `svc=1`, fb 63→~460).
3. USB RX ring contiguous memcpy (ISR push + `usb_read`).
4. `can_led_poll` at most once per ms.
5. FDCAN TX drain under one lock; drop non-idle FDCAN pararead; pass slot into `robstride_apply_cycle`.
6. End-of-lap diag skips FDCAN buses already polled by apply — **this stabilized all×25 pend≤1**.
7. SDK cali `mechPos=None` + cp1252 arrows (Claude) — tested via bringup skip-cali path.

---

## What to try next

1. Tighten host↔plant ack accounting / coalesce so `ack_lag_max` at 500 Hz host is honest (or gate on p95 + plant pend).
2. More apply/SPI shave if wanting fb closer to 500 under all×25 (still ~470–490).
3. Optional: `-Os` / Release compare — Debug is `-O0`.

**Do not regress:** CH4–6×6 @ 40/200 Hz (`fb≳250`, plant `svc≈1`); RS02 `--bus 4` teleop; INT-gated idle RX; no FRESET-on-busy in `try_send`; burst=1 (do not restore 8 without FB proof).

---

## Key files

- `App/Src/plant/can/mcp2518fd.c`, `spi_can_router.c`, `spi_can_port.c`, `can_router.c`
- `App/Src/plant/plugins/robstride.c`, `actuator.c`, `diag/diag_core.c`, `control_loop.c`
- `App/Src/host/host_link.c`, `host_transport_usb.c`
- `scripts/_tmp_mcp_timing_probe.py`, `scripts/rs02_channel_bringup.py`

## Related docs

- `docs/bringup.md` §7 / timing matrix  
- `docs/handoff-mcp-fb-bringup-2026-07-20.md` (older MCP FB bringup)  
- `docs/api.md` — bandwidth matrix command snippet  
