# Base — Damiao on CH6 (slot 25)

Live-verified operating manual for the single Damiao motor sharing MCP CH6 with the RobStride RS01
at slot 24 (see [base-robstride-mcp.md](base-robstride-mcp.md) for that motor and the shared bus-6
kick/discover mechanics). Source of truth: `scripts/yam_continuous_all.py` `BASE_ROWS` (row for
slot 25) and the `_probe_base()` / `_tick_base_and_dxl()` functions in that same script.

## AI quickstart

- **Bus/slot/ID**: MCP CH6 (bus 6), slot `25`, motor ID `0x06`, master RX ID `0x16` (`= (mid + 0x10)
  & 0xFF`, computed from the discovered ID — not hardcoded like the arm's `_DAMIAO_MASTER` table).
- **Discover ID first, dynamically**: `hub.debug.discover_damiao(bus=6, start=1, end=16,
  listen_ms=80)` — do not assume `0x06` without discovering; the code discovers by ESC sweep and
  only then CFGs slot 25 with whatever ID answered.
- **Discover position is always 0.0 — do not use it as a seed.** Damiao discover/ID-sweep frames
  report `pos=+0.0000` regardless of the motor's real position (that field isn't populated by a
  discover response). The correct seed is **live plant feedback after CFG**, obtained by CFGing the
  slot enabled, then polling `_read_base(session)[25]` for up to 0.6 s until it reports a real value.
  Seeding from the discover response's fake `0.0` would command the motor to (possibly) an
  unreachable or jarring position.
- **`kp=0` never latches enable — floor it.** Unlike the RobStride slots, sending `kp=0.0` while
  seeding means firmware skips the CAN TX for that slot entirely (no enable latch can fire). Use
  `DM_BASE_MIN_KP = 2.0` (with `kd` floored at `DM_BASE_KD * 0.25`) as the minimum non-zero seed
  gain — see `_write_plant()`'s `if proto == PROTO_DAMIAO and kp_out < DM_BASE_MIN_KP:` branch.
- **Full drive gains**: `DM_BASE_KP = 10.0`, `DM_BASE_KD = 0.5` once past the seed/soft-engage phase.
- **Travel window is soft, not a hardware MIT rail**: `DM_BASE_TRAVEL = 2π` rad (±one full turn)
  around the *seeded center*, unlike the RobStride slots' fixed `±12.57` MIT limits. There is no
  margin subtracted (no `RS_MARGIN` equivalent) — the bounce bound is exactly `center ± 2π`.
- **Don't** treat this slot's CFG as static — the master ID is derived from whatever ID
  `discover_damiao` returns, so if the physical ESC's configured ID ever changes, the master ID
  computed here changes with it automatically (no manual table edit needed, unlike the CH1 arm).

## Human deep dive

### Why this motor needs its own seed path

The other base slots (22–24, RobStride) get a real position out of `probe_robstride()` at reset time
— a proper request/response with a position payload. Damiao's `discover_damiao` is a bus-presence
sweep (find which ESC ID answers on the ID range), not a position query, so its response always
carries a placeholder `pos=+0.0000`. `_probe_base()` explicitly does **not** use that value as a
seed (the code comment: "Do NOT invent a fake probe pose — Damiao seed must use plant FB after CFG").
Instead, once `cfg_set_slot(slot=25, ...)` is applied, the script polls live plant feedback for up to
0.6 s, holding RS idle (`base_gain_scale=0.0`) but with slot 25 given the `DM_BASE_MIN_KP` floor so
its own enable can latch, until a real nonzero-or-confirmed FB sample for slot 25 arrives — that
becomes `base_center[25]` / `base_cmd[25]`.

### The `kp=0` enable-latch gotcha

Firmware only transmits a CAN command frame for a Damiao slot if `kp` is nonzero (a `kp=0` desire is
treated as "nothing to send" and skipped entirely, presumably to avoid spamming zero-effort MIT
frames). This is invisible for RobStride, whose enable path doesn't depend on the same TX-skip
logic, but for Damiao it means a naively "safe-looking" `kp=0.0` seed command silently never enables
the motor at all — `_write_plant()` upgrades any Damiao `kp` below `DM_BASE_MIN_KP=2.0` to that
floor (and floors `kd` proportionally) specifically so the seed/discovery phase can still latch
enable while still being a soft, low-authority command.

### Soft window vs. MIT rail

Because Damiao here has no documented hard position limit equivalent to RobStride's `±12.57` MIT
range, the continuous script imposes its own **soft** travel window — `±2π` around wherever the
motor was seeded — purely so the bounce pattern has *something* to reverse against. This is a
bench-cruise convenience, not a hardware or firmware-enforced limit; don't read `DM_BASE_TRAVEL` as
a real mechanical constraint the way `RS_P_MIN`/`RS_P_MAX` are for RobStride.

## Verified

**Date:** 2026-07-24, live board on Jetson, `python scripts/launch_continuous.py`.

Discover (dynamic ID resolution, master computed from the discovered ID):
```
Damiao discover on CH6 (PA4 MCP SPI-CAN)  IDs 1..16
FOUND  probe=0x06  esc_id=0x06  master_rx=0x16  mode=id_sweep  pos=+0.0000  err=0x0
Damiao discover summary: 1 motor(s) — 0x06
  CH6 Damiao discover id=0x06
```

Plant-FB seed (not the fake `0.0000` from discover):
```
  Damiao plant seed s25=+1.4906
base centers: s22=+0.613 s23=+0.584 s24=+0.665 s25=+1.496
```

Continuous bounce inside the `center ± 2π` = `[-4.79, +7.77]` soft window, direction reversal
observed near both bounds:
```
s25=+6.09/+6.04@+0.79
s25=+7.58/+7.54@+0.79    # near +7.77 upper bound
s25=+6.40/+6.44@-0.79    # reversed, sign now -0.79
...
s25=-2.63/-2.55@-0.79
s25=-4.12/-4.05@-0.79    # approaching -4.79 lower bound
s25=-3.88/-3.95@+0.79    # reversed again, sign +0.79
```
Full run recorded to `.deft_session/recordings/record_20260724T214351.ndjson` on the Jetson.

## Known falsehoods retired

- **"Discover gives you a usable starting position."** False for Damiao — `pos=+0.0000` on every
  discover response is a placeholder, not real telemetry. Always warm plant FB after CFG before
  computing a seed.
- **"kp=0 is the safest possible seed command."** False for this motor — it silently prevents
  enable from ever latching (firmware skips the TX). The safe-and-functional floor is
  `DM_BASE_MIN_KP=2.0`, not `0`.
- **"CH6 base motors all share the RobStride MIT rail behavior."** No — slot 24 (RobStride) and slot
  25 (Damiao) are different motor families with different rail semantics: hard `±12.57` MIT limit
  vs. a soft `±2π`-from-seed bounce window with no hardware-enforced equivalent.
