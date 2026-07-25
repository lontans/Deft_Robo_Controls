# Base — RobStride on MCP CH5/CH6 (slots 22–24)

Live-verified operating manual for the RobStride RS02/RS01 drives on the MCP SPI-CAN channels used
by the current bench base rig. Source of truth: `scripts/yam_continuous_all.py` `BASE_ROWS`,
`scripts/rs02_channel_bringup.py`, `scripts/deft_controls_sdk/link/exchange.py` (`is_mcp_bus`,
`build_rs2_probe_command`, `build_rs2_scan_command`). For the Damiao motor that shares CH6, see
[base-damiao-ch6.md](base-damiao-ch6.md).

## AI quickstart

- **Bus/slot map (bench, current)** — this is the *live wiring on this bench*, not the idealized
  product map in `slots.py::yam_product_rows()` (which puts base steer/drive on slots 14–19 with
  motor IDs `0x01`/`0x02`). The bench rig actually uses:

  | Slot | Bus | Motor ID | Label | Notes |
  |-----:|----:|---------:|-------|-------|
  | 22 | 5 (MCP CH5) | `0x70` | CH5 RS02 | |
  | 23 | 5 (MCP CH5) | `0x74` | CH5 RS01 | daisy-chained with 0x70 |
  | 24 | 6 (MCP CH6) | `0x75` | CH6 RS01 | |

  (Slot 25 / CH6 `0x06` is the Damiao motor sharing bus 6 — separate doc.)
- **MIT travel rail**: `RS_P_MIN = -12.57`, `RS_P_MAX = +12.57` rad (raw RS02 MIT limits), with a
  `RS_MARGIN = 0.35` rad safety inset applied by `_rail_clip()` → effective commandable range
  `[-12.22, +12.22]`.
- **Gains**: `RS_KP = 20.0`, `RS_KD = 1.0` (continuous cruise; conservative hold gains elsewhere are
  `DEFAULT_STEER_KP=40`/`DEFAULT_STEER_KD=1`/`DEFAULT_DRIVE_KD=2` in `slots.py`, not used by the
  bench continuous script).
- **Waking an idle MCP bus**: send RS2 `SESSION_BEGIN`/`SESSION_END` scan frames on **both** bus 5
  and bus 6 before probing (`_kick_mcp_buses`) — an idle MCP ring will not answer probes otherwise.
  `is_mcp_bus(bus)` gates a longer 2.0 s timeout for MCP buses vs. 0.55 s for native FDCAN.
- **Sibling reset before enabling — order matters**: reset **every** RobStride ID on the bus first
  (`_rs_reset_id` for each, back to back), *then* probe/enable each one. Do **not** interleave
  reset-then-enable-then-reset-next — resetting a sibling after another has already been enabled
  drops the already-armed one back out of MIT. This matters specifically for the bus-5 daisy chain
  (`0x70` + `0x74`): a leftover MIT-enabled motor blocks its chain-mate from accepting a fresh
  enable.
- **Rail-reverse policy**: the spin direction flips **only when the command integrator itself
  reaches a rail**, never from feedback lag mid-travel. Don't "fix" apparent stalls by reversing on
  FB-vs-cmd error — that was the old triangle-wave flip-flop bug. `BASE_LEAD = 0.45` rad is the max
  allowed lead of desired position over live FB; beyond that the tick still advances the integrator
  but slews the *desire* toward FB at half rate, it does not reverse.
- **Don't** invent a probe pose for a RobStride slot that returned `found=False` — use
  `rs02_resolve_start(probe_q, plant_fb)` (prefers live plant FB once available, falls back to the
  probe reading) rather than defaulting to 0.0, which is a real (and wrong) commandable position on
  this joint.

## Human deep dive

### Why MCP buses need a kick

CH5/CH6 route through an MCP2518 SPI-CAN bridge rather than the STM32's native FDCAN peripheral.
After any idle period the MCP ring can go quiet enough that a probe frame sent cold gets no answer.
`_kick_mcp_buses()` forces `McuState.DIAG_ONLY` and round-trips `SESSION_BEGIN`/`SESSION_END` scan
frames on bus 5 and bus 6 before any real probe — this is a bus-wake, not a functional command, and
its own timeouts/exceptions are swallowed (logged, not fatal) since a kick failing on one bus
shouldn't abort the other.

### The daisy-chain reset ordering bug this avoids

Bus 5 carries two RobStride drives (`0x70`, `0x74`) on the same physical CAN segment. Enabling one
into MIT mode and *then* sending a `PROBE_RESET` to its sibling was observed to also reset the
already-armed motor — likely a broadcast-adjacent side effect of how the MCP ring or the drives
themselves handle a reset frame addressed to a chain-mate. `_probe_base()` works around this by
doing **all resets for a bus first** (tracked via a `seen_reset` set keyed on `(bus, motor_id & 0xFF)`
so a bus is only reset-swept once), and only then walking the same list again to actually
probe/enable each ID. The per-slot enable itself (`hub.debug.probe_robstride`) is reset→enable
scoped to *that* ID only, so once the initial bus-wide reset sweep is done, enabling motor A does
not re-perturb already-enabled motor B.

### Rail-reverse: integrator vs. feedback

`_tick_base_and_dxl()` maintains a per-slot **command integrator** (`base_integ[slot]`) separate
from live feedback. Each tick it advances the integrator by `sign * base_rate * dt_nom`; only if
*that advanced integrator value* would cross `lo`/`hi` does it clamp to the rail and flip `sign`.
Feedback is used solely to cap how far the *desire* (the actually-transmitted position command) is
allowed to lead the integrator (`BASE_LEAD`), and to halve the commanded velocity when near that
lead cap — it never triggers a direction reversal by itself. This is a deliberate fix for an earlier
bug where reversing on "FB isn't keeping up with cmd" mid-travel produced rapid direction flip-flops
under normal bus lag rather than genuine rail contact.

## Verified

**Date:** 2026-07-24, live board on Jetson, `python scripts/_tmp_launch_continuous.py`.

Sibling-safe reset-then-probe sequence, all three RobStride IDs found:
```
  rest bus5 id=0x70
  rest bus5 id=0x74
  rest bus6 id=0x75
OK  probe_id=0x70  kind=12  found=1  comm=2  pos=+0.6128  raw=1
  probe CH5 RS02 q=+0.6128
OK  probe_id=0x74  kind=12  found=1  comm=2  pos=+0.5840  raw=1
  probe CH5 RS01 q=+0.5840
OK  probe_id=0x75  kind=12  found=1  comm=2  pos=+0.6650  raw=1
  probe CH6 RS01 q=+0.6650
base centers: s22=+0.613 s23=+0.584 s24=+0.665 s25=+1.496
```

Continuous spin: s22/s23/s24 track together (same sign, same velocity — bus-5 pair *and* the bus-6
singleton stay synchronized under the shared spin-sign policy), sweeping from ~+0.6 rad down toward
the rail and reversing:
```
s22=-0.92/-0.88@-0.79 s23=-0.95/-0.92@-0.79 s24=-0.87/-0.84@-0.79
s22=-8.50/-8.44@-0.79 s23=-8.53/-8.48@-0.79 s24=-8.45/-8.42@-0.79
s22=-11.52/-11.46@-0.79 s23=-11.55/-11.50@-0.79 s24=-11.47/-11.45@-0.79
s22=-11.36/-11.41@+0.79 s23=-11.39/-11.44@+0.79 s24=-11.47/-11.48@+0.79   # rail reversal, sign flipped +0.79
```
`cmd/fb` stay within ~0.06 rad of each other throughout (well under `BASE_LEAD=0.45`) — no lead-cap
throttling needed at `base_rate=π/4 rad/s`. Full run recorded to
`.deft_session/recordings/record_20260724T214351.ndjson` on the Jetson.

## Known falsehoods retired

- **"CH6's RobStride canonical slot is 23."** Superseded — per `docs/bench-pdb-sdk-contract-2026-07-24.md`,
  CH6 RobStride moved to slot **24** under wire layout v3 (was 23 under v2). This doc and
  `yam_continuous_all.py::BASE_ROWS` both already reflect slot 24; treat any reference to "CH6 RS →
  slot 23" as stale v2-era material.
- **"Base steer/drive live on slots 14–19."** That's `yam_product_rows()`'s idealized product CFG
  (CH4/5/6 steer+drive, motor IDs `0x01`/`0x02`) — it is **not** what's wired or driven on this
  bench today. The bench rig's real RobStride IDs are `0x70`/`0x74`/`0x75` on slots 22–24, per
  `BASE_ROWS` above. Don't cite `yam_product_rows()` as current bench truth.
- **"A reset probe only affects the ID it's addressed to."** Not true for a daisy-chained bus with
  an already-armed sibling — see the reset-ordering section above. Always reset the whole bus before
  enabling any motor on it.
