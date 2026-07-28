# Debug/test playbook — Controls PCB

Task-oriented: "I want to test X" → the exact command. For *what each call does*
see [api.md](api.md); for *bench history/postmortems* see [bringup.md](bringup.md).
Everything here assumes `cd scripts` first unless noted, and that only **one**
process owns the COM port at a time (dashboard, script, or pytest against real
hardware — never two at once).

---

## 0. Is the board even alive?

| Check | Expect |
|-------|--------|
| PC2 | HIGH once USB CDC enumerates |
| PC3 | ~2 Hz blink = plant loop alive (`app_run`/TIM6) |
| PC7 | CAN activity blink (250 ms window) |
| `python -c "from deft_controls_sdk import find_cdc_port; print(find_cdc_port())"` | Prints the COM port, e.g. `COM5` |
| `python -c "from deft_controls_sdk import list_cdc_ports; print(list_cdc_ports())"` | Lists every serial device, flags which is the STM32 CDC |

If none of PC2/PC3 light up: check power, then soft-DFU flash (§8) before anything else.

---

## 1. Connect, recover, blank everything

```python
from deft_controls_sdk import ControlsPcbHub, ActuatorDesire
with ControlsPcbHub.connect("COM5") as hub:
    hub.recover()                 # RECOVERY -> NORMAL (plant_recovery_all)
    print(hub.telemetry.snapshot())
    for slot in range(25):
        hub.set_actuator(slot, ActuatorDesire(), send=False)   # blank/idle
    hub.send_once()
```

Run this first after any flash or after anything went sideways — it's the reset button.

---

## 2. Discover a motor on a bus

```powershell
# RobStride (RS02) — sweep an ID range
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    print(hub.debug.discover_robstride(bus=4, start=0x40, end=0x80))
"

# Probe one specific ID instead of sweeping
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    print(hub.debug.probe_robstride(bus=4, motor_id=0x70))
"

# Damiao — pass known/configured IDs first (scan-order flood risk otherwise)
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    print(hub.debug.discover_damiao(bus=1, start=1, end=16, known_ids=[1,2,3]))
"

# List every Damiao ID on a bus (hub discover is first-hit only)
python legacy/damiao_scan.py --port COM5 --discover --host-only --bus 1 --start 0 --end 32 --listen-ms 60
```

Buses are schematic **CH1–CH6** (1-indexed): CH1–3 = FDCAN, CH4–6 = MCP2518 SPI-CAN.

---

## 3. Calibrate a RobStride motor

**Shaft must be free to spin. Supply 24–60 V** (30 V/9A PSU can UV-trip mid-cal on an un-calibrated motor).

```powershell
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    ok = hub.debug.calibrate_robstride(bus=4, motor_id=0x70, cal_listen_s=28.0)
    print('cal ok:', ok)
"
```

Or via the full single-channel bringup script below, which does discover → CFG → metrics → cal → teleop in one shot.

---

## 4. Single-channel bringup (the "just tell me if this motor+bus works" script)

```powershell
# RobStride — move the cable, only change --bus
python rs02_channel_bringup.py --bus 4
python rs02_channel_bringup.py --bus 1 --motor-id 0x70 --skip-cali

# Damiao — --slot is required (daisy-chain bus, no canonical single-motor slot);
# does NOT touch sibling slots on the bus
python damiao_channel_bringup.py --bus 2 --slot 8 --motor-id 0x01
python damiao_channel_bringup.py --bus 2 --slot 9 --motor-id 0x02 --known-ids 0x01,0x02,0x03
```

Both print a `PASS`/`FAIL` per phase (discover, CFG, metrics hold, cali, tiny teleop) and an overall verdict — this is the fastest "is this specific motor+bus healthy" check. Exit code is 0 on overall PASS.

---

## 5. Inspect / change the actuator config table

```powershell
# Read the live table (25 rows always — CFG/NVM overrides factory defaults)
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    for i, row in enumerate(hub.debug.cfg_get_table()):
        print(i, row)
"

# Assign a slot (RAM only unless persist=True)
python -c "
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect('COM5') as hub:
    hub.debug.cfg_set_slot(slot=19, bus=4, protocol=1, motor_id=0x70, persist=True)
"
```

`protocol`: 0 none, 1 RobStride, 2 CubeMars (not motion-ready), 3 Damiao, 4 ZeroErr.
After `persist=True`, power-cycle and re-read the table to confirm it survived.

---

## 6. Bandwidth / timing stress tests

The one to run after **any** firmware change touching the plant loop, CAN/SPI drivers, or host link:

```powershell
$env:PYTHONIOENCODING='utf-8'

# Classic matrix, host 40 Hz — should be OVERALL ack_lag OK
python bench_load_matrix.py --port COM5 --hz 40 --scenario all

# Stress: elevated host rate (capability check, not required to pass)
python bench_load_matrix.py --port COM5 --hz 200 --scenario all
python bench_load_matrix.py --port COM5 --hz 500 --scenario all
```

Watch `raw_fb_hz`, `cmd_seq_lag` (8-bit `last_command_seq`), `act_lap_ms`/`act_lap_peak_ms` (PlantTask), `periph_lap_ms`/`periph_lap_peak_ms` (PeripheralTask), and `ticks_pending`/`ticks_svc`. Cross-check `cmd_rx_seq` (USB RX stage) vs `cmd_applied_seq` (plant mount) when diagnosing lag — Host starvation shows high cmd lag with healthy act_lap.

`bench_load_matrix.py` is the durable successor to the retired
`_tmp_mcp_timing_probe.py` / `_tmp_rate_rx_sweep.py` / `_tmp_load_matrix_report.py`
(see [scripts-hygiene.md](scripts-hygiene.md)) — full spec in
[bench-optimize-and-load-matrix-plan.md](legacy/bench/bench-optimize-and-load-matrix-plan.md).
For chasing a specific symptom, narrow the scenario instead of running `all`:

```powershell
# Isolate one bus (TX-only vs plant-apply cost on just that channel)
python bench_load_matrix.py --port COM5 --hz 40 --scenario ch1

# CH4-6 together — the Apply-accumulate footgun (docs/api.md §"blank" note)
python bench_load_matrix.py --port COM5 --hz 40 --scenario mcp

# TX-only baseline, no RX-sim, no plant-apply cost at all
python bench_load_matrix.py --port COM5 --hz 40 --scenario idle
```

`_tmp_bus6_real_hw.py` (full real-hardware smoke: RS02 CH6 + Dynamixels +
LEDs) is archived to `scripts/legacy/` — import-broken (referenced the
now-deleted `_tmp_mcp_timing_probe.py`) before this pass, kept as reference
only.

---

## 7. Soft-DFU flash (no ST-Link needed)

```powershell
# One-shot: enter bootloader -> program -> leave
python scripts/soft_dfu_flash.py
python scripts/soft_dfu_flash.py --image ../Debug/DeftRoboticsControlsPCB.elf   # explicit image
```

```python
from deft_controls_sdk.bench import find_cdc_port, enter_bootloader, leave_bootloader, flash_firmware
print(find_cdc_port())
flash_firmware(confirm=True)
```

If CDC drops and doesn't come back: check for `0483:DF11` in Device Manager (Windows) — that's the ROM DFU device, `leave_bootloader()` should reset it back to CDC.

---

## 8. Build firmware

```powershell
cd Debug
make -j8 all
```

(CubeIDE toolchain must be on `PATH`, or build via the IDE.) Check for zero warnings on any file you touched before flashing — `-Wall` is on.

---

## 9. Live dashboard (visual monitoring while you do something else)

```powershell
python -m deft_controls_sdk.debug_dashboard --port COM5
```

→ http://127.0.0.1:8765. This **owns the COM port** — stop it before running any script above against the same port. Useful routes if scripting against it instead of the UI: `GET /api/state`, `POST /api/actuator/<slot>`, `POST /api/actuator/<slot>/idle`, `POST /api/recover`.

---

## 10. Black-box recording (capture a session for later analysis)

```python
from deft_controls_sdk import ControlsPcbHub
with ControlsPcbHub.connect("COM5") as hub:
    hub.start_streaming()
    hub.telemetry.start_recording()
    # ... exercise whatever you're testing ...
    hub.telemetry.stop_recording()
    print(hub.telemetry.snapshot().recording_path)   # NDJSON under .deft_session/recordings/
```

---

## 11. Host-side unit tests (no hardware required)

```powershell
cd scripts
python -m pytest tests/ -q
```

Run this after *any* SDK/Python change — it's fast (~30s) and doesn't need a board connected. Test files, if you want to target one area:

| File | Covers |
|------|--------|
| `test_deft_controls_sdk_bench.py` | discover/probe/CFG packing, fake-connection harness pattern |
| `test_deft_controls_sdk_robstride_calibrate.py` | RS02 cal sequence, pararead echo handling |
| `test_deft_controls_sdk_layout_v2.py` | 672 B wire layout offsets |
| `test_deft_controls_sdk_connection_locking.py` / `..._write_drain.py` | Connection thread-safety, write draining |
| `test_deft_controls_sdk_telemetry.py` / `..._recorder.py` | Telemetry snapshot fields, NDJSON recording |
| `test_deft_controls_sdk_soft_dfu.py` | DFU enter/leave/flash helpers |
| `test_deft_controls_sdk_dashboard.py` | Dashboard HTTP routes |

---

## 12. Recovery / E-STOP paths

```python
from deft_controls_sdk import ControlsPcbHub, McuState   # NORMAL / RECOVERY / DIAG_ONLY / ESTOP
with ControlsPcbHub.connect("COM5") as hub:
    hub.set_mcu_state(McuState.ESTOP)
    print(hub.telemetry.snapshot())   # check plant_block / mcu_state_readback
    hub.recover()                     # back to NORMAL
```

`plant_block` reasons to know while debugging why nothing's moving: `1` bench_session (a DEBUG lease is held), `2` probe_busy, `3` quiet_period (just after a bench session ended), `4` diag_only, `5` host_stale (no fresh CMD in >500ms — keep streaming), `6` servo_session.

---

## 13. Known symptom → check (quick table, see bringup.md for full stories)

| Symptom | First thing to run |
|---------|---------------------|
| No feedback on one bus | `probe_robstride`/`discover_damiao` on that bus — check it's actually woken, and `--bus` matches schematic, not Cube slot position |
| CH4–6 idle but LED never blinks | Expected — blank desire (`kp/kd/vel/τ≈0`, `position==0`) skips SPI on MCP buses. Push a non-blank desire or run a probe. |
| `ack_lag_max` spikes only at high host Hz | Likely coalesce artifact, not real — check `lap` mean / `ticks_svc` first (§6) |
| Damiao `tx>0 rx_raw=0` | Motor-end 120Ω termination missing, or scan-order flood (§2's `known_ids` fix) |
| Cal listen never sees `mms=cali` | Shaft not free, wrong bus/ID, or motor not at rest — reset first |
| Ctrl+C mid-probe wedges the MCU | `hub.recover()` or USB replug |
| CFG persist doesn't survive power-cycle | G4 `FLASH_CR_BKER` NVM erase fix must be flashed — re-read table after cycling to confirm |

---

## Related

- [api.md](api.md) — what each call does (reference)
- [bringup.md](bringup.md) — bench history, postmortems, dual-arm/Damiao stories
- [host-exchange-v3.md](host-exchange-v3.md) / [host-debug-v1.md](host-debug-v1.md) — wire formats
- [lessons.md](lessons.md) — durable bug list
