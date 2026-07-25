# vbeta live-prove plan (Claude-Vbeta, four-agent next pass)

**Status (2026-07-24 evening):** Live ladder executed on Jetson — see
[`docs/bench-vbeta-live-prove-2026-07-24.md`](bench-vbeta-live-prove-2026-07-24.md).
Left hold+jog PASS; right/base/neck logged as HW blockers. Claudistic GUI
untouched.

Originally written offline. Executable checklist for a session with CDC free.
Does **not** touch Soft-DFU, FW kill math, or lift CANopen — those stay out of
scope (see bottom).

Ground truth this plan cites: [`docs/vbeta-pcb-adapter.md`](vbeta-pcb-adapter.md)
(contract), [`scripts/deft_controls_sdk/vbeta/`](../scripts/deft_controls_sdk/vbeta/)
(`PcbArmDriver` [`arm.py`](../scripts/deft_controls_sdk/vbeta/arm.py),
`PcbPlatformClient` [`platform.py`](../scripts/deft_controls_sdk/vbeta/platform.py),
`PcbRobotSession` [`session.py`](../scripts/deft_controls_sdk/vbeta/session.py),
`yam_product_rows` [`slots.py`](../scripts/deft_controls_sdk/vbeta/slots.py)),
working tree [`deft_vbeta/`](../deft_vbeta/) +
[`pcb_bridge.py`](../deft_vbeta/src/deft_amr/amr/amr/pcb_bridge.py)
`install_pcb_backend`, read-only reference
[`docs/deft_vbeta_ref/deft_vbeta`](deft_vbeta_ref/deft_vbeta), entrypoints
[`vbeta_product_prove.py`](../scripts/vbeta_product_prove.py),
[`vbeta_smoke.py`](../scripts/vbeta_smoke.py) (`arm` / `base` / `neck`), offline
[`tests/test_deft_controls_sdk_vbeta.py`](../scripts/tests/test_deft_controls_sdk_vbeta.py).

---

## 1. Exclusive COM checklist (before opening the port)

Per `docs/vbeta-pcb-adapter.md`'s COM-ownership table, `PcbRobotSession` is
the sole hub owner while live. Before running anything below:

1. Debug dashboard: click **Disconnect** (or confirm it never Connected this
   session) — it must not be holding COM in `control` mode.
2. `yam_continuous_all.py` / any other bench script on the same port: stop it
   (Ctrl-C, confirm process exit) — do not rely on it timing out.
3. Confirm no other Cursor/Claude session is mid Soft-DFU or a load-matrix run
   on the same port (see `docs/vbeta-pcb-adapter.md`'s ownership table — one
   role owns CDC at a time).
4. Only then run a vbeta entrypoint; it will hold COM exclusively until the
   `with PcbRobotSession.connect(...)` block exits or the script errors.
5. If a run aborts hard (Ctrl-C mid-hold), reopen once and let `close()` run
   its idle-park sequence (blank desires → `DIAG_ONLY` → cornflower LED)
   rather than power-cycling to clear state.

## 2. Offline gate — must be green before any Jetson step

```
python -m pytest scripts/tests/test_deft_controls_sdk_vbeta.py -q
```

Expect the full fake-hub suite green (19 passed as of the last recorded run,
`docs/bench-vbeta-arm-2026-07-24.md`) including `test_platform_neck_cmd_no_double_offset`,
`test_robstride_soft_hold_*`, `test_neck_hold_present_*`, `test_pdb_poll_*`,
`test_rig_components_tick_*`. Do not proceed to hardware if this fails —
fix or revert first. This is a no-board gate; run it from a laptop with no
Jetson access if needed.

## 3. Product CFG path — arms 0–13, base 14–19

`yam_product_rows()` is the only CFG this plan drives on hardware:

| Slots | Name | Bus | Protocol |
|------:|------|-----|----------|
| 0–6 | left arm J1–J7 | CH1 | Damiao |
| 7–13 | right arm J1–J7 | CH2 | Damiao |
| 14–16 | BwC/BwR/BwL (steer) | CH4–6 | RobStride |
| 17–19 | BpC/BpR/BpL (drive) | CH4–6 | RobStride |
| 20 | lift, reserved | CH3 | disabled |
| 21–25 | spare | — | disabled |

**Never** silently fall back to the continuous bench's spare-slot map
(22–25) inside a product-CFG run to paper over a missing device — a
not-found probe is the correct, honest result. This bit the team once
already: the base's physical RobStride/Damiao drives answer bench-spare IDs
(`0x70`/`0x74`/`0x75`/`0x06`), not the product map's `0x01`(steer)/`0x02`(drive)
— see the "Base ID remap gap" in `docs/vbeta-pcb-adapter.md` and
`docs/bench-vbeta-product-cfg-2026-07-24.md`. Treat that as a hardware/CFG
mismatch to log, not something to patch around by pointing `yam_product_rows()`
at bench IDs.

Apply via `PcbRobotSession.connect(..., apply_yam_cfg=True)` (RAM only,
`force_cfg=True` if a stale table is suspected — see `ensure_yam_product_cfg`
in `cfg.py`). Do not `persist=True` outside a deliberate CFG-change session.

## 4. Prove ladder

Run in order; each step gates the next. Stop and log a blocker rather than
skipping ahead.

1. **Left-arm hold** — `python scripts/vbeta_smoke.py arm --port <COM> --side left --apply-cfg --hold-s 2.0`
   (or the combined `vbeta_product_prove.py`, which already sequences this).
   Pass: COM opens exclusively, CFG applies (`CH1×7` slots reported), FB
   `Position_Rad` is non-zero on at least one joint (proves the arm is
   physically live, not just CFG'd — see the `left_live` check in
   `vbeta_product_prove.py`).
2. **Left-arm jog** — clamped `Goal_Position` delta via `plan_jog_q7`
   (`yam_limits.py`), converge check `|q_after - q_target| < 0.05 rad`
   within ~1 s. This is the actual tracking proof, not just hold.
3. **Right arm (optional)** — `discover_damiao_all(bus=2, start=1, end=7)`
   first; only attempt `PcbArmDriver(side="right").connect()` if the sweep
   returns IDs. If CH2 is empty, log "CFG'd but not physically present" and
   move on — do not treat as a failure of the adapter.
4. **Base (optional)** — direct product-ID probe (`hub.debug.probe_robstride`)
   at `0x01`/`0x02` on CH4/5/6 *before* trusting `PcbPlatformClient` FB. If
   none found, this is the known remap gap (§3) — log it, do not block the
   rest of the ladder on it.
5. **Neck (optional, HW blocker if not wired)** — log as blocked if the neck
   servo strip isn't on the bench this session; do not fabricate a pass.

Log every HW blocker (right arm absent, base remap gap, neck absent) in a
dated `docs/bench-vbeta-*.md` entry, same format as the existing
`bench-vbeta-arm-2026-07-24.md` / `bench-vbeta-product-cfg-2026-07-24.md` —
future sessions need "not present this session" distinguished from
"adapter broken."

## 5. `install_pcb_backend` vs direct adapters

Two integration points exist; pick based on what the Jetson env actually has:

- **Direct adapters** (`PcbArmDriver` / `PcbPlatformClient` / `PcbRobotSession`,
  no `YAMAIMobile`) — what `vbeta_product_prove.py` and the smoke scripts use
  today. Works even when the Jetson's Python env is missing `torch`/`mujoco`,
  since nothing imports `YAMAIMobile`. **Default choice for this pass** — it's
  what's actually been proven live (§ live-prove note in
  `docs/vbeta-pcb-adapter.md`).
- **`install_pcb_backend(robot)`** (`deft_vbeta/src/deft_amr/amr/amr/pcb_bridge.py`,
  landed code, not yet exercised live) — only call this **after**
  `YAMAIMobile(config)` has already constructed successfully, i.e. only when
  the Jetson env has both `torch` and `mujoco` importable. If either is
  missing, `YAMAIMobile.__init__` fails before `install_pcb_backend` ever
  runs — do not chase that as a `deft_controls_sdk` bug.
- **Fallback rule:** if the Jetson env can't satisfy `YAMAIMobile`'s imports,
  stay on direct adapters for this pass; document the missing packages rather
  than installing torch/mujoco ad hoc on shared hardware without asking.

## 6. Soft-kill — what actually happens today

`PcbRobotSession` auto-registers `soft_kill_park_if_requested` +
`soft_kill_park_if_bad_vi` as a pre-plant-send hook via
`hub.start_streaming(..., auto_soft_kill=True)` (the default —
`PcbRobotSession.connect` never passes `auto_soft_kill=False`). On top of
that, `session.send_once()` / `service_soft_kill()` re-checks
`soft_kill_park_if_requested` explicitly each tick — belt-and-suspenders, not
redundant-for-no-reason: the plant hook fires around plant TX, the explicit
call is what smoke loops use to detect+log a park mid-window
(`vbeta_product_prove.py`'s `SOFT_KILL_REQ -> parked` line).

**What this does *not* cover:** `soft_kill_park_if_requested` only checks the
PDU-level `kill_state` (`pdb_status().kill_state == KILL_SOFT_REQ`, a wire
flag). It does **not** poll the debug dashboard's follow-mode file flag
(`<session_dir>/soft_kill_request`, written by `debug_dashboard/app.py`'s
Soft-kill Park button when the dashboard doesn't own COM). Only
`yam_continuous_all.py` polls that file today
(`_dashboard_soft_kill_requested` at `scripts/yam_continuous_all.py:489`). A
vbeta session run in parallel with a dashboard in follow mode will **not**
honor a dashboard Soft-kill Park click — the PDU-level hooks are the only
safety net. Practical implications for this plan:

- Don't run the dashboard in follow mode against a vbeta session and expect
  its Soft-kill button to do anything — it won't, silently, unless someone
  ports the same file-poll into `PcbRobotSession.service_soft_kill()`
  (out of scope for this pass; flag it as a Claudistic follow-up instead of
  fixing it here).
- The PDU-level auto-park (bad V/I, hardware `SOFT_KILL_REQ`) still works
  regardless — that's the one this plan's prove ladder actually exercises
  and can verify (§4 hold/jog windows should tick `service_soft_kill()` and
  log `False` every tick as the "nothing tripped" baseline).

## 7. Out of scope for this pass

- Soft-DFU (flash path) — untouched, see `docs/soft-dfu.md`.
- FW kill math / `pdb_link.c` — untouched.
- Lift CANopen bring-up — stub stays a stub; see
  `docs/feathersdk-lift-teardown.md` for the separate de-stub plan.
- Debug dashboard UX/teleop rewrite — Claudistic's lane, not this plan's.
- Live execution of this ladder — this pass is the plan only; run it next
  time CDC is free and one owner has it (§1).
