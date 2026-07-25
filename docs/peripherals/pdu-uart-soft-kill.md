# PDU UART + soft-kill (freshness, COMMS_LOSS, dashboard follow-mode)

Live-verified operating manual for the Controls↔PDB UART link's USB-visible status and the
soft-kill park handshake. Wire-level contract: [pdb-uart-v1.md](../pdb-uart-v1.md) (authoritative —
this doc covers **operating/observing** it from the host side, not the frame layout). Source of
truth (host side): `scripts/deft_controls_sdk/pdb/status.py`, `scripts/deft_controls_sdk/controls_pcb_hub.py`
(`pdb_status`, `soft_kill_park`), `scripts/deft_controls_sdk/debug_dashboard/app.py`
(`soft_kill_park()` follow-mode branch), driver `scripts/yam_continuous_all.py`
(`session.service_soft_kill()`, `_dashboard_soft_kill_requested()`).

## AI quickstart

- **Read status**: `hub.pdb_status()` → `PdbStatus` with `.kill_state_name`, `.kill_reason_name`,
  `.estop_sense`, `.stale_failsafe` (bool). No separate USB "is the PDU link fresh" flag exists —
  `stale_failsafe` **is** that check (`kill_state == HARD_ESTOP and kill_reason == COMMS_LOSS`).
- **Kill states**: `0 NORMAL`, `1 SOFT_KILL_REQ`, `2 SOFT_KILL_READY`, `3 HARD_ESTOP`.
- **Trigger a soft-kill park two ways**:
  1. Direct COM owner: `hub.soft_kill_park(send=True)` — blanks all actuator/servo desires and
     latches `McuState.ESTOP`.
  2. **Follow mode** (dashboard not holding COM, e.g. while `yam_continuous_all.py` owns it):
     dashboard's "Soft-kill Park" button writes a timestamp to
     `.deft_session/soft_kill_request` instead of touching the port directly. The process that
     *does* own COM must be polling for that flag — `yam_continuous_all.py`'s main loop calls
     `_dashboard_soft_kill_requested()` every tick, which checks-and-deletes the flag file, then
     calls `hub.soft_kill_park(send=True)` on the peer's behalf.
  3. There's also `session.service_soft_kill()` — checks live `pdb_status().soft_kill_req` (i.e. the
     PDB itself asked for a kill, not the dashboard) and parks automatically; called every tick in
     the continuous loop for observability/response, independent of the dashboard flag path.
- **`--force-kill-state 0`** on `pdb_uart_sim.py` forces `kill_state=NORMAL` in the simulated PDU
  feedback — use this when you need a stable non-faulting sim baseline for bring-up, as opposed to
  `--random`/`--wander` which cycles faults.
- **Don't** poll for a PDU "freshness" USB byte that doesn't exist — check `stale_failsafe` on the
  parsed system-kill status instead; a garbled or missing physical PDB frame degrades to the same
  `HARD_ESTOP`/`COMMS_LOSS` signature you'd see from an actually-tripped hard ESTOP, by design (see
  Human deep dive).
- **Don't** assume `pdb_uart_sim.py` running == a fresh PDU link. It can silently die on the serial
  read (`/dev/ttyTHS1` is flaky on this Jetson — see Verified section) and the last thing anyone
  sees is `stale_failsafe: true` on the next `pdb_status()` read, same as if the PDB were physically
  unplugged. Check `ps -ef | grep pdb_uart_sim` / `/tmp/pdb_uart_sim.log` on the Jetson if kill state
  won't leave `HARD_ESTOP/COMMS_LOSS`.

## Human deep dive

### Why "stale" and "hard-faulted" look identical on USB

Per [pdb-uart-v1.md](../pdb-uart-v1.md), the PDB link's freshness fail-safe is: if no **valid**
frame (magic + version + CRC all pass) has arrived within `PDB_STALE_MS = 200 ms`, firmware reports
`kill_state = HARD_ESTOP` / `kill_reason = COMMS_LOSS` on the USB mirror — there is deliberately no
separate "link stale but otherwise fine" state. `parse_system_kill()`'s `stale_failsafe` field on the
host is just that same combination re-checked in Python
(`kill_state==KILL_HARD_ESTOP and kill_reason==KILL_REASON_COMMS_LOSS`). This is intentional
fail-safe design (a garbled byte stream must never be silently trusted as "probably still fine"),
but it means you cannot distinguish, from USB alone, "the physical PDB link died" from "someone
actually hit hard-ESTOP" — both look like `HARD_ESTOP/COMMS_LOSS` (or `HARD_ESTOP` with a different
`kill_reason` for a real button/fault). Cross-check `estop_sense` and the sim/PDB's own log if you
need to tell them apart.

### The soft-kill handshake is staged, not instant

Per the wire doc's state machine: `NORMAL → SOFT_KILL_REQ → (controls parks) → SOFT_KILL_READY →
HARD_ESTOP`. The PDB is never supposed to open contactors on `SOFT_KILL_REQ` alone — it waits for
controls to report `SOFT_KILL_READY` in the *command* frame once actuators are actually parked. On
the host side this means calling `soft_kill_park()` isn't "instant safe" from the PDB's perspective;
it's the controls side doing its part of a handshake the PDB side completes. `pdb_uart_sim.py
--simulate-kill-after N` / `--random` exercises this exact ordering in simulation (see
[pdb-uart-v1.md](../pdb-uart-v1.md) for the sim's CLI).

### Dashboard follow-mode: why a flag file instead of a shared port

Only one process may hold the Controls board's USB CDC port at a time. When `yam_continuous_all.py`
(or any other driver) owns it, the debug dashboard cannot also `Connect COM` without stealing the
port — so it runs in **follow mode**, reading `.deft_session/state.json` for display only. Its
Soft-kill Park button still needs to *do* something in that mode, so instead of touching the serial
port it writes `.deft_session/soft_kill_request` (a plain timestamp file). Any COM-owning process
that wants to honor dashboard-initiated kills must poll for that file and delete it once consumed
(`_dashboard_soft_kill_requested()` does both in one call) — this is a **cooperative** protocol, not
enforced by the SDK; a driver script that doesn't call it will never honor a follow-mode park
request.

## Verified

**Date:** 2026-07-24, live board on Jetson (`192.168.50.48`), via
`python scripts/launch_continuous.py`.

**Follow-mode soft-kill park — worked end to end.** With `yam_continuous_all.py` owning COM and the
dashboard running separately in follow mode, writing the flag file directly (same mechanism the
dashboard button uses) was picked up on the very next poll and triggered a clean park + exit:
```
writing soft_kill_request (dashboard follow-mode path)…
-rw-rw-r-- 1 deft-robotics deft-robotics 15 Jul 24 21:45 .../.deft_session/soft_kill_request
...
  dashboard soft_kill_request → soft_kill_park()
recording stopped: .../recordings/record_20260724T214351.ndjson
cleanup: blank arm/base + clear DXL + DIAG...
done
```

**PDU link was actually stale for this entire run — a real fail-safe observation, not a happy-path
one.** Every status line for the full ~26 s cruise read `pdb=hard_estop/COMMS_LOSS estop_gpio=0
reason=comms_loss`, e.g.:
```
stream/pdb: fb=401 tx=20 ack=0 gap95=52.3ms pdb=hard_estop/COMMS_LOSS estop_gpio=0 reason=comms_loss
```
Root cause found on the Jetson: `pdb_uart_sim.py` (launched fresh on `/dev/ttyTHS1 --hz 20` at the
start of this run) crashed partway through —
```
$ tail /tmp/pdb_uart_sim.log
  File ".../pdb_uart_sim.py", line 690, in main
    chunk = ser.read(4096)
  File ".../serial/serialposix.py", line 595, in read
    raise SerialException(
serial.serialutil.SerialException: device reports readiness to read but returned no data
  (device disconnected or multiple access on port?)
```
i.e. the simulated PDB stopped sending `PDBF` frames, the 200 ms freshness window elapsed, and
firmware correctly fell back to `HARD_ESTOP`/`COMMS_LOSS` on the USB mirror exactly as documented —
this is the fail-safe contract working, not a bug in the arm/base/DXL peripherals (which all kept
tracking correctly regardless, per the other peripheral docs). It does mean **this bench currently
cannot demonstrate a `pdb=normal` steady state** until `/dev/ttyTHS1` on this Jetson is confirmed
stable (matches the "blocked this sprint" note already in
[pdb-uart-v1.md](../pdb-uart-v1.md#jetson-uart-hygiene) about no confirmed live `ttyTHS*`↔UART4
loop on this specific hardware) — treat `ttyTHS1` as unreliable on this Jetson until re-verified,
not as "PDU integration is broken."

## Known falsehoods retired

- **"pdb=normal is the expected steady state on this bench right now."** Not currently true — the
  live-verified state on 2026-07-24 is `HARD_ESTOP/COMMS_LOSS` because the sim process on the
  Jetson's `/dev/ttyTHS1` crashes. This is the documented fail-safe behavior, not a regression in
  the arm/base/DXL/soft-kill-handshake paths, all of which were separately confirmed healthy in the
  same run.
- **"There's a USB freshness flag separate from kill_state."** No — `stale_failsafe` is derived
  purely from `kill_state==HARD_ESTOP and kill_reason==COMMS_LOSS`; there is no independent
  freshness byte on the wire or in `PdbStatus`.
- **"The dashboard needs Connect COM to soft-kill park."** False when a peer (e.g. continuous) owns
  the port — the flag-file follow-mode path was live-verified above to park successfully without the
  dashboard ever touching the serial port.
