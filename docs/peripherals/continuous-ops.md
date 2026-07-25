# Continuous ops — launch/stop, stream health, recording, "what good looks like"

Operational manual for `scripts/yam_continuous_all.py`, the all-peripherals bench cruise driver that
the other docs in this directory ([arm-damiao-ch1.md](arm-damiao-ch1.md),
[dxl-neck.md](dxl-neck.md), [base-robstride-mcp.md](base-robstride-mcp.md),
[base-damiao-ch6.md](base-damiao-ch6.md), [pdu-uart-soft-kill.md](pdu-uart-soft-kill.md)) all draw
their live evidence from. Read this one first if you're about to run continuous on the live board.

## AI quickstart

- **One-shot remote launch (recommended)**: `python scripts/launch_continuous.py` from a
  machine with SSH to the Jetson (`192.168.50.48`, user `deft-robotics`, password from env
  `JETSON_PASS`, bench default `4565`). It: kills any stale `yam_continuous_all.py`/`pdb_uart_sim.py`,
  clears a leftover `soft_kill_request` flag, syncs the current local copies of the driver + SDK
  files it needs via SFTP, runs `stop_can.py` remotely to blank any leftover CAN state, starts
  `pdb_uart_sim.py` and `yam_continuous_all.py --record --duration 50` in the background, waits for
  latch+cruise, writes a follow-mode `soft_kill_request`, and pulls back the log tail. This is the
  proven, reproducible path — prefer it over ad hoc manual SSH unless you're actively debugging one
  step of it.
- **Direct launch** (already on the Jetson or a host with the live board's CDC): `python3
  yam_continuous_all.py [--no-base] [--no-dxl] [--duration SEC] [--estop-after SEC] [--record]`.
  Auto-detects the port via `find_cdc_port()` if `--port` omitted.
- **Stop it**: Ctrl-C (SIGINT/SIGTERM both hooked, runs `_cleanup()` — blanks all desires, clears
  DXL, sets `DIAG_ONLY`, restores idle LED). **`killall -9` does NOT stop CAN** — a hard-killed
  process leaves the last-commanded MIT frames latched on the bus. If you had to hard-kill, run
  `python3 stop_can.py` afterward to blank everything.
- **Dashboard must stay in follow mode** while continuous owns CDC — do not click Connect COM in the
  debug dashboard against the same port continuous is using. The dashboard reads
  `.deft_session/state.json` (written by `persist_telemetry=True`) for live display, and its
  Soft-kill Park button uses the `.deft_session/soft_kill_request` flag-file path in follow mode —
  see [pdu-uart-soft-kill.md](pdu-uart-soft-kill.md).
- **Recording**: `--record` writes per-tick NDJSON to `.deft_session/recordings/record_<UTC
  timestamp>.ndjson` on whichever machine runs the script (the Jetson, for the live-board runs this
  doc set is based on). Recording stops automatically on any exit path (duration reached, soft-kill,
  Ctrl-C, ESTOP-respect check).
- **`--estop-after SEC`**: after SEC seconds of cruise, asserts host `McuState.ESTOP` and runs a
  2.5 s check that (a) MCU readback shows ESTOP and (b) base position moved less than 0.20 rad while
  held — process exit code is `2` if that check fails. Use this to prove ESTOP respect, separately
  from the soft-kill-park path.
- **Don't** run two drivers (continuous + `yam_dxl_clear_teleop.py`, or two continuous instances)
  against the same board's CDC port concurrently — only one process can own it.

## Human deep dive

### Why the launch script syncs files instead of just SSH-running in place

`launch_continuous.py` runs from a local checkout and `sftp.put()`s a fixed list of files
(`yam_continuous_all.py`, `pdb_uart_sim.py`, `stop_can.py`, `rs02_channel_bringup.py`, and a
handful of `deft_controls_sdk/` modules) to the Jetson's working copy before launching anything
remotely. This exists because the Jetson's checkout can otherwise drift from whatever's being
actively edited locally — syncing just the files the run actually touches (not a full repo push)
keeps the loop fast while guaranteeing the remote run reflects local changes, not a stale copy.

### Reading the per-tick status line

Every `--status-s` seconds (default 2.0 s) the loop prints one combined line, e.g.:
```
J2 dir=-1 cmd/fb=-2.917/-2.743 tau=-10.13 faults=[1,1,1,1,1,1,1] | s22=... s23=... s24=... s25=... dxl=.../...|.../ ... | fb=401 tx=20 ack=0 gap95=52.3ms pdb=hard_estop/COMMS_LOSS estop_gpio=0 reason=comms_loss
```
Breaking down the trailing stream/PDU segment (`_bw_pdb_line()`):
- `fb=` — measured feedback rate in Hz from the board (arm/base/DXL FB image), **not** the command
  rate. Healthy is several hundred Hz (this driver's CDC link runs the plant FB stream well above
  the 20 Hz host command rate).
- `tx=` — host→board command TX rate; should track `--stream-hz` (default `STREAM_HZ=20.0`).
- `ack=` (`stream_ack_lag`) — 0 is healthy; nonzero means the host is getting behind on ack'd sends.
- `gap95=` — 95th-percentile gap between TX sends in ms; at `tx=20 Hz` a ~50 ms `gap95` is exactly
  nominal (1000/20), not a warning sign by itself.
- `pdb=` — mirrors `hub.pdb_status()`; see [pdu-uart-soft-kill.md](pdu-uart-soft-kill.md) for what
  each state/reason means, including why `hard_estop/COMMS_LOSS` does **not** by itself mean the
  arm/base/DXL peripherals are unhealthy (they're driven independently of the PDU link).

### What "good" looks like for a 30–50 s cruise

Based on the 2026-07-24 live run this doc set is built from (`--duration 50`, soft-killed at ~26 s
into cruise via the follow-mode flag):
1. **Discover**: all 7 CH1 Damiao ESCs found within the 5-attempt kick/discover loop.
2. **Progressive latch**: all 7 arm joints reach `fault=1` (MIT green) — some joints may need a
   `_recover_rearm()` retry (especially J4/index 3), that's normal, not a failure, as long as the
   final pass converges.
3. **DXL present**: both neck IDs report a present position within the 2.5 s discover window.
4. **Base CFG + probe**: all 3 RobStride IDs (`0x70`, `0x74`, `0x75`) probe `found=1`; the Damiao ID
   on CH6 discovers and its slot gets a live plant-FB seed (not a `0.0` placeholder).
5. **Soft-engage**: no fault trips during the 2.4 s ramp from latch gains to full gains.
6. **Cruise**: `faults=[1,1,1,1,1,1,1]` holds for the whole window; J2 visibly bounces between the
   CLEAR bounds with clean arrive/stuck-reverse logging; base slots 22/23/24 move together and
   reverse together at their rail; base slot 25 bounces inside its soft window; DXL cmd/fb track
   within a few hundred native steps.
7. **Soft-kill park**: whichever trigger you use (flag file, `service_soft_kill()`, `--estop-after`)
   produces a clean `cleanup()` + `done` exit, not a hang or traceback.
8. **PDU status is a separate axis** — a fully healthy peripheral cruise can still show
   `pdb=hard_estop/COMMS_LOSS` if the UART sim/link is down (observed live 2026-07-24, see
   [pdu-uart-soft-kill.md](pdu-uart-soft-kill.md)); don't conflate that with a peripheral-tracking
   failure.

A run that *doesn't* look like this — a joint stuck at `fault=0` after the final green pass, `fb=`
dropping toward single digits, or an unhandled traceback instead of `cleanup()`/`done` — is the
actual failure signature worth investigating, not the PDU state.

### Open bench issues (2026-07-25)

Captured after Mission Impossible / continuous follow-up on the same Jetson board. Continuous was
stopped; dedicated discovers/probes used (not “skip and keep cruising”).

1. **CH5 RobStride `0x74` silent on CAN**
   - `discover_robstride_all(bus=5, start=0x74, end=0x74)` → no hit.
   - `probe_robstride(bus=5, motor_id=0x74)` → `found=0`, `raw_frames=0` (no CAN reply), including
     after MCP SESSION kick and after resetting sibling `0x70`. Same miss on bus 6.
   - Control: `0x70` on CH5 still probes `found=1` (same session).
   - Earlier the same day (M4 / continuous) `0x74` had probed `found=1` at ~`+2.51` rad — so this is
     a **regression / HW-or-ID state**, not “never wired.” Host continuous cannot arm what the MCP
     path never hears. Next checks: power/CAN drop on the daisy second node, confirm ID with the
     RobStride tool, reseat CH5 chain.

2. **DXL neck: present OK, torque goals do not move**
   - Stream present discover still returns both IDs (observed `pitch=1754`, `yaw=645`).
   - Commanding torque-on goals for ~5 s (`pitch→2300`, `yaw→1400`) with stream + paced
     `send_once` each tick left `end_fb` unchanged (`1754` / `645`).
   - Note: yaw present `645` is **below** firmware `servo_table[1].pos_min=700` — even so, pitch is
     in-range and also did not move. Treat as PeripheralTask / torque path / bus issue, not only
     clamp math. Earlier 2026-07-24 continuous had tracking cmd/fb; this session does not.

3. **J2 CLEAR lift vs brace hold**
   - When J2 drives toward the CLEAR high/low, other arm joints sometimes **do not hold pose hard
     enough** (brace kp/kd insufficient under the moving J2 load). That can look like multi-joint
     sag or intermittent MIT green loss even when latch initially succeeded. Follow-up: raise brace
     gains on non-J2 slots during cruise, or slow J2 rates when brace torque peaks.

## Verified

**Date:** 2026-07-24, live board on Jetson (`192.168.50.48`, `/dev/ttyACM0`), full run via
`python scripts/launch_continuous.py`, `yam_continuous_all.py --cruise-up 0.18 --cruise-down
0.12 --engage-s 2.4 --base-rate 0.7854 --record --duration 50`, soft-killed via follow-mode flag
after ~26 s of cruise (well past the 30 s continuous-tracking bar once boot+latch+engage time is
included — see the peripheral docs for the exact per-system evidence). Clean exit:
```
  dashboard soft_kill_request → soft_kill_park()
recording stopped: /home/deft-robotics/controls_pcb/scripts/.deft_session/recordings/record_20260724T214351.ndjson
cleanup: blank arm/base + clear DXL + DIAG...
done
```
Recording: `.deft_session/recordings/record_20260724T214351.ndjson` (~2.7 MB) on the Jetson — full
per-tick raw data behind every 2 s status-line sample quoted across this doc set.

Pre-run board/process check (useful as a "is it safe to launch" checklist):
```
ps -ef | grep -E "yam_continuous|pdb_uart_sim|debug_dashboard"
# only pdb_uart_sim.py (bring-up sim) + debug_dashboard (follow mode) — no yam_continuous_all.py
python3 -c "import serial.tools.list_ports as p; [print(x.device, x.hwid) for x in p.comports()]"
# /dev/ttyACM0 USB VID:PID=0483:5740 SER=3167376F3435 — live board CDC present, unheld
```

## Known falsehoods retired

- **"`killall -9` is a safe way to stop continuous."** False — it stops the Python process but
  leaves the last MIT command frames latched on the CAN bus. Always Ctrl-C (clean `_cleanup()`) or,
  if you already hard-killed, follow up with `stop_can.py`.
- **"The dashboard needs to Connect COM to see live data during continuous."** False — follow mode
  (reading `.deft_session/state.json`) gives full live telemetry without taking the port, and is the
  *required* mode whenever another process owns CDC.
- **"A `pdb=hard_estop/COMMS_LOSS` reading means the run failed."** Not necessarily — see the
  worked example above and [pdu-uart-soft-kill.md](pdu-uart-soft-kill.md): the PDU UART link and the
  CAN/servo peripheral loop are independent axes of health.
