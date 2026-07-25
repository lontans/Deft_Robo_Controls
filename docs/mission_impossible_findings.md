# Mission Impossible — findings

Stress suite for Controls PCB failure points under elevated-but-plausible ops.
Orchestrator: [`scripts/mission_impossible.py`](../scripts/mission_impossible.py).
Board: Jetson CDC (`/dev/ttyACM0`) unless noted. One CDC owner while a mission is `RUNNING`.

---

## Status board

| Mission | Status | Last update |
|---------|--------|-------------|
| M1 TX bandwidth | PASS | 2026-07-25 06:30:17Z |
| M2 PDU soft-kill / V/I | PASS | 2026-07-25 06:33:51Z |
| M3 Multi-joint CLEAR arm | PASS | 2026-07-25 06:34:40Z |
| M4 Faster base + DXL | PASS | 2026-07-25 06:37:06Z |
| M5 Soft-DFU stress | BLOCKED | 2026-07-25 06:31:39Z |

**Suite:** M1–M4 PASS. M5 BLOCKED (safe abort on Jetson — no soft-enter without `0483:DF11`).

---

## Lessons that unblocked M2 / M3

- Warm PDU UART sim and wait until `stale_failsafe` / COMMS_LOSS clears before KillSim or V/I phases (`soft_kill_park_if_bad_vi` only acts when kill is `NORMAL`).
- Wrap progressive CFG in `pause_plant_stream` (CFG replies get stolen while plant stream runs → `TimeoutError`).
- `stop_can` / blank+DIAG between missions and after soft-kill park (do not inherit ESTOP into the next CFG/latch).

---

## Operator observation — arm vibration

Bench felt heavy vibration during **M1** and **M2** (M3 possible); **M4 felt fine**.

Likely causes (orchestrator posture, not product FAIL):

- **M1** — `bench_load_matrix` scenario `all` stiff-holds many enabled slots for TX stress (not Goal=FB soft latch).
- **M2A** — left arm held at `position=0.0, kp=8, kd=0.5` while waiting for KillSim (snap-to-zero vs current pose).
- **M3** — multi-joint CLEAR bounce at latch `kp×0.5` can chatter near rails; progressive latch itself is continuous-like.
- **M4** — `yam_continuous_all` Goal=FB soft-engage + J2 CLEAR brace; re-run 2026-07-25 06:36Z also PASS and quiet.

Non-blocking follow-up: M1 kd-only / no motion hold; M2 seed Goal=FB before `kp>0`; optionally soften M3 cruise gains/rates.

---

## M5 Soft-DFU — brick root cause (recovered)

First M5 soft-entered DFU on Jetson; **DF11 never appeared**; CDC gone.

**Root cause:** Soft-DFU programs option bytes (`App/Src/host/soft_dfu.c`): enter sets `nBOOT0=0` (force system-memory / ROM DFU). Leave restores `nBOOT0=1`. Jetson never saw DF11, so leave never ran → every reset stayed in system memory → no app heartbeat / no `0483:5740` CDC.

**Recovery (ST-Link):**

1. `-ob nSWBOOT0=0 nBOOT0=1` (force Main Flash boot)
2. Mass erase + flash `Debug/DeftRoboticsControlsPCB.elf` + hard reset
3. Jetson: `/dev/ttyACM0` + `0483:5740` back

Orchestrator now refuses Linux soft-enter without DF11. Do not soft-enter on Jetson unless the leave path is proven — stuck `nBOOT0=0` is the brick. True Soft-DFU stress belongs on a host that enumerates DF11 (typically laptop USB).

---

## Final mission results (condensed)

### M1 TX bandwidth — PASS

| tx Hz | trials | fb_hz | ack_max | ok |
|---:|---:|---:|---:|---|
| 40 | 2 | ~457 | 0 | 2/2 |
| 100 | 2 | ~470 | 0 | 2/2 |
| 200 | 2 | ~436 | 1 | 2/2 |

Live stream samples at 20/40/100 Hz also healthy (`ack=0`). Hard gate 40 Hz ok.

### M2 PDU soft-kill / V/I — PASS

After PDU warm (`live=True`, not COMMS_LOSS):

- KillSim `--simulate-kill-after` → park on `soft_kill_req` / reason other
- UV `pack_v=3900` → park on undervoltage
- OC `pack_i=3100` → park on overcurrent

### M3 Multi-joint CLEAR — PASS

Progressive MIT latch J1–J7 all green; J2–J4 CLEAR cruise then stage2 all-7; `hard_fault=False`, `clear_breach=False`, `faults_end=[1×7]`.

### M4 Faster base + DXL — PASS (reconfirmed quiet)

`yam_continuous_all --duration 25 --base-rate 1.0`: MIT green, DXL present, base cruise armed, clean exit. Operator: quiet vs M1/M2.

### M5 Soft-DFU stress — BLOCKED

Scan showed CDC OK, DF11 absent → safe abort (no soft-enter). See brick note above.

---

## Earlier failures (resolved)

| Symptom | Cause | Fix |
|---------|--------|-----|
| M2 handshake/UV miss; frozen `seq` + COMMS_LOSS | No PDU sim through M1; phases started before UART live | Warm sim + wait for live PDU; stop_can between phases |
| M3 `TimeoutError` CFG / `faults=[0×7]` | CFG while streaming; no progressive latch; ESTOP leftover | `pause_plant_stream`; continuous-style latch; stop_can before M3 |
| M5 brick after soft-enter | Jetson never sees DF11; `nBOOT0` stuck 0 | Refuse soft-enter without DF11; SWD OB + reflash recover |

---

## Follow-up backlog

- Soften M1/M2/M3 holds so TX/PDU proves do not thrash the arm.
- Optional: laptop-only Soft-DFU timing prove (DF11 path).
- Keep Jetson PDU sim (`pdb_uart_sim` on `/dev/ttyTHS1`) up between sessions so COMMS_LOSS does not sticky-trap the next run.
- **CH5 `0x74`:** silent on CAN as of 2026-07-25 post-suite (`found=0` / no discover); `0x70` OK. See [`peripherals/continuous-ops.md`](peripherals/continuous-ops.md) open issues.
- **DXL:** present reads work; torque goals did not move FB in the same session (pitch/yaw stuck).
- **J2 brace:** when lifting J2 in CLEAR cruise, other joints sometimes fail to hold pose hard enough — raise brace gains or slow J2.
