# Plan: YAM joint command options (user + AI)

**Audience:** Claude / Cursor implementing host-side joint commands for the Damiao YAM arm.  
**Date:** Jul 2026  
**Constraint:** joint-space only (no IK). One process owns COM (USB CDC). Do not expand FreeRTOS / CubeMars workstreams.

---

## 0. Paste prompt

```
Implement YAM joint command options for the controls PCB host per
docs/plan-yam-joint-commands.md.

Constraints:
- Joint-space only. No IK / Cartesian.
- Reuse plant path (send_plant / teleop / hello-world). No new USB PDU unless required.
- Enforce soft limits from control_hub.yam_limits (yam.xml J1–J6 + bench J7).
- Motor encoder frame ≠ MuJoCo qpos until zeros are calibrated — default to
  relative moves; absolute goto requires an explicit --absolute flag + warning.
- Exclusive COM5: document handoff between user teleop and AI hello-world/goto.
- Do not command actuators in the planning pass; add --dry-run everywhere motion exists.
- Keep changes small; update docs/bringup.md with the joint↔slot map when done.
```

---

## 1. Ground truth (do not re-litigate)

| Item | State |
|------|--------|
| Plant slots | `ACTUATOR_COUNT=7`; slot `i` ↔ joint `i+1` (J1=slot0 … J7=slot6) |
| Bus | All seven Damiao on **CH1** daisy; ESC_ID `0x01`…`0x07`, Master `0x11`…`0x17` |
| Limits file | `External_Documentation/yam_arm_damiao/yam.xml` (J1–J6 only) |
| Host helper | `scripts/control_hub/yam_limits.py` + limit-aware `hello_world.py` |
| J7 | **Not in XML.** Provisional motor-frame soft range `[1.10, 2.80]` from Jul 2026 bench: hard-stop fight at ~+1.11 when commanding more negative; MIT +travel OK to ~+2.5; torque spun to ~+6.4 (ignore as limit). EE range is intentionally loose. |
| Hello-world | `--dry-run`, `--joint N`, soft clamp, J7 negative-flip near hard-stop, “limit fight” FAIL hint |

### Limit reasonableness (review)

| Joint | XML / derived range (rad) | Verdict |
|-------|---------------------------|---------|
| J1 | `[-2.618, 3.13]` | Reasonable asymmetric base yaw |
| J2 | `[0, 3.65]` | Reasonable one-sided shoulder; large span (>π) — keep slew gentle |
| J3 | `[0, 3.13]` | Same pattern as J2 |
| J4 | `±π/2` | Standard wrist |
| J5 | `±π/2` | Standard wrist |
| J6 | `±2.094` (±120°) | Standard roll |
| J7 EE | `[1.10, 2.80]` motor-frame provisional | Not in model; OK as soft EE stroke until calibrated |
| τ | XML `actuatorfrcrange ±10` all | Matches host Damiao `TMAX`; keep as soft cap even on 4340 |

**Calibration gap:** Until motor zero ≡ model zero, **absolute** XML clamps on live `fb` are wrong for J1–J6 (fb can sit outside XML while the arm is valid). Default command path must be **relative + span-capped**; absolute mode is opt-in.

---

## 2. Goals

1. **Unified joint addressing:** `--joint 1..7` and `--slot 0..6` everywhere motion is invoked.
2. **Safe single-joint moves for AI:** hello-world / `goto` with `--dry-run`, limit clamp, clear PASS/FAIL (incl. limit-fight).
3. **User interactive path:** existing `--plant-teleop` gains soft limit ceilings (optional phase).
4. **Handoff protocol:** user vs AI never share COM concurrently; document who holds the port.
5. **Persist mapping + limits** in bringup notes once zeros are agreed.

**Non-goals:** IK, trajectory blending across joints, writing Damiao CTRL_MODE from host, expanding XML meshes.

---

## 3. Command surface to implement

### 3.1 Already present (extend, do not rewrite)

| Command | Role |
|---------|------|
| `control_hub.py --hello-world` / `hello-world` | AI smoke jog, limit-aware |
| `control_hub.py teleop --slot N` | User single-joint arrows |
| `control_hub.py --plant-teleop --plant-slots 0,1,2,3,4,5,6` | User multi-joint plant |
| `config show` / `config set` | Slot ↔ ESC_ID map |

### 3.2 Add (priority order)

| Priority | CLI | Behavior | Status |
|----------|-----|----------|--------|
| P0 | `hello-world --joint N --dry-run` | Done in helper; ensure CLI flags wired | **Done** |
| P0 | `hello-world --limits` | Print `yam_limits` table + reasonableness notes | **Done** |
| P1 | `joint goto --joint N --delta D` | Alias of hello-world relative jog (same codepath) | **Done** — `scripts/control_hub/joint_cmd.py::run_joint_goto` |
| P1 | `joint goto --joint N --to RAD --absolute` | Slew to absolute motor target; clamp with `absolute_limits=True`; refuse without `--i-know-zeros` until calibration doc exists | **Done** — gated in `hello_world.run_hello_world` (`to` param) |
| P1 | `joint status [--joint N]` | One-shot fb + soft limit + distance-to-stop (read-only; kp=0 stream briefly or status PDU) | **Done** — `run_joint_status` |
| P2 | Plant teleop soft stop | When `cmd` would cross soft limit, clamp slew / zero kp toward wall; print `LIMIT` once | **Deferred** — needs bench time to tune without motion in the planning pass |
| P2 | `joint home --joints 1-6` | Per-joint relative home toward model mid **or** motor-captured “session home”; do not assume 0.0 is safe for J2/J3 (XML lo=0) | **Deferred** — CLI stub wired (`joint home`, exits 2, points here) |
| P3 | AI batch script | `scripts/control_hub/joint_script.py` YAML/JSON list of `{joint, delta, hold_s}` with dry-run | **Deferred** — not started |

### 3.3 Suggested UX flags (all motion commands)

```
--port COM5
--joint N | --slot S
--delta RAD          # relative (default path)
--to RAD --absolute  # absolute motor frame
--slew RAD_S
--kp / --kd
--dry-run
--no-limit-clamp     # escape hatch; print WARNING
--xml PATH           # override yam.xml
```

---

## 4. Implementation sketch

```
scripts/control_hub/
  yam_limits.py          # exists — source of truth for soft ranges
  hello_world.py         # exists — single-joint jog + clamp + dry-run
  joint_cmd.py           # NEW thin wrapper: status / goto / home → hello_world primitives
scripts/controls_pcb_host/cli/main.py
  wire --joint, --dry-run, --limits, joint subparser
docs/bringup.md
  joint↔slot↔ESC_ID table + “AI uses hello-world; user uses plant-teleop”
```

**Shared rules in code:**

1. Resolve `--joint` / `--slot` via `yam_limits.joint_to_slot` (conflict = error).
2. Always print `J#`, soft `[lo,hi]`, source, planned vs clamped delta before TX.
3. On FAIL with `|τ|≥4` and `moved<0.05` → message “limit fight”, not “bus dead”.
4. J7: if `fb ≲ 1.15` and requested delta `<0`, flip or refuse (already in `clamp_delta`).
5. Default `|delta| ≤ min(0.35, 0.08×span)` unless user overrides (still clamped to soft walls when calibrated).

---

## 5. User ↔ AI handoff

| Who | Holds COM | Command |
|-----|-----------|---------|
| User | yes | `--plant-teleop` or `teleop --slot` |
| AI | yes | `hello-world` / `joint goto` / `joint status` |
| Idle | — | both release; `release_bench_gates` on exit |

Protocol:

1. User stops teleop (`q`) before asking AI to move.
2. AI runs `--dry-run` first when joint/delta is ambiguous.
3. AI prefers **one joint per invocation**; no background threads on COM.
4. After AI motion, leave slot disabled or kp=0; tell user “port free”.

---

## 6. Calibration follow-up (blocks safe `--absolute`)

1. Per joint: record motor `fb` at two known model poses (or hard stops).
2. Store `motor_offset[j] = fb - q_model` in a small JSON under `scripts/control_hub/` (not flash).
3. Only then enable default absolute goto using `q_motor = q_model + offset`.
4. Re-measure J7 stops; replace provisional `[1.10, 2.80]` if needed.

Until step 3 ships, **reject** `--absolute` without `--i-know-zeros`.

---

## 7. Test plan (when bench is free — human or AI with COM)

- [ ] `hello-world --limits` prints J1–J7 table
- [ ] `hello-world --joint 7 --delta -0.3 --dry-run` shows flip/clamp note
- [ ] `hello-world --joint 7 --delta +0.3` PASS (off hard-stop)
- [ ] `hello-world --joint 5 --delta -0.2` PASS inside XML span
- [ ] `joint status --joint 1` read-only
- [ ] Absolute goto without `--i-know-zeros` exits non-zero
- [ ] Plant teleop does not drive past soft clamp (P2)

---

## 8. Done when

- [x] CLI exposes `--joint`, `--dry-run`, `--limits` on hello-world
- [x] `joint` subcommands exist for status/goto (P1) or are clearly deferred with stubs in this doc
  — `joint status` / `joint goto` implemented (`scripts/control_hub/joint_cmd.py`); `joint home`
  (P2) is a stub that exits non-zero and points here
- [x] bringup.md lists joint↔slot↔ESC_ID and points here for command policy — see
  [bringup.md § YAM joint-slot-ESC_ID map and command policy](bringup.md#yam-joint-slot-esc_id-map-and-command-policy-jul-2026)
- [x] No actuator motion required to merge the host/docs portion — verified via
  `--dry-run` on `hello-world` and `joint goto`, plus `scripts/tests/test_joint_cmd.py`
  (pure gating/dry-run logic, no hardware)

**Remaining (P2/P3, deferred to a bench session):** plant-teleop soft stop, `joint home`,
AI batch script (`joint_script.py`). Bench test-plan checklist in §7 above still applies
once hardware is available.
