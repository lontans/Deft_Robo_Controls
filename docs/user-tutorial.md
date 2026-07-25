# Controls PCB — user tutorial (exercises)

Hands-on checklist for a human on the bench. Complete each exercise in order;
check boxes as you go. This file is the **operator workbook**; deep truth stays
in the linked docs (if they disagree, the linked doc wins).

**Dashboard note:** Claudistic is rewriting debug-dashboard teleop / Soft-kill
UX. Treat GUI steps as *optional / may look different*. Prefer the CLI paths
below until the dashboard work lands — they do not depend on that UI.

**Cursonier note:** Use this file as the walkthrough spine; expand with
[`manual/README.md`](manual/README.md) chapters when teaching.

---

## Before you start (once)

| Asset | Typical value |
|-------|----------------|
| Jetson | `192.168.50.48`, user `deft-robotics`, repo `~/controls_pcb` |
| Host Windows repo | this checkout (build ELFs here) |
| CDC | one Controls PCB USB — **one process owns it** |
| Password | env `JETSON_PASS` (bench default often `4565`) |

```bash
# From host or Jetson — who holds USB?
python scripts/soft_dfu_flash.py scan
```

Write your answers here:

- [ ] Board serial from `scan`: `________________`
- [ ] CDC device (e.g. `COM5` or `/dev/ttyACM0`): `________________`
- [ ] Nothing else is holding that port (dashboard Disconnect, no continuous/vbeta)

---

## Exercise 1 — Inventory and ownership

**Goal:** Know who owns CDC and which stack map you will use.

**Do:**

1. Read [manual §1](manual/README.md#1-setup--jetson--serial-ownership) (2 minutes).
2. Run `scan` (above). Note serial + port.
3. Pick **one** stack for this session (do not mix base maps):

| Stack | When to use | Base slots |
|-------|-------------|------------|
| Continuous (bench) | Familiar multi-peripheral cruise | **22–25** spare IDs |
| vbeta / product | Product CFG / adapter proves | **14–19** product IDs `0x01`/`0x02` |

**Pass:**

- [ ] I wrote serial + port above
- [ ] I chose continuous **or** vbeta (circled)
- [ ] I know dashboard Connect / Soft-DFU / smokes must not share CDC

---

## Exercise 2 — Soft-DFU flash (optional this session)

**Goal:** Put a known ELF on the board over USB. Full SOP: [soft-dfu.md](soft-dfu.md).

**Do (USB-only path):**

1. Build or copy `Debug/DeftRoboticsControlsPCB.elf` (or Release).
2. Stop every CDC owner.
3. On the machine that has the board USB (often Jetson):

```bash
python scripts/soft_dfu_flash.py scan
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf \
  --serial <YOUR_SERIAL> --require-usb-dfu
```

4. Expect `flash ok — CDC at …` **without** `(SWD)`.
5. `scan` again — CDC should return with the same serial.

**If soft-enter drops CDC and DF11 never appears** (seen on Jetson with serial
`3167376F3435`): stop calling it Soft-DFU success. Recover with ST-Link SWD
(`nBOOT0=1` + flash), then retry `--require-usb-dfu` only when DF11 is visible.
See [soft-dfu.md](soft-dfu.md) “Jetson caveat”.

**Pass:**

- [ ] `scan` before and after
- [ ] `--require-usb-dfu` succeeded **or** I documented SWD recovery + why
- [ ] Serial pinned (never “whichever board answers”)

---

## Exercise 3 — Offline software gate (no motors)

**Goal:** Prove the SDK/adapters on this machine before touching hardware.

```bash
cd scripts   # or set PYTHONPATH=scripts from repo root
python -m pytest tests/test_deft_controls_sdk_vbeta.py -q
python -m pytest tests/test_deft_controls_sdk_pdb_limits.py -q
```

**Pass:**

- [ ] vbeta fake-hub tests green
- [ ] PDB V/I limits tests green

If either fails: fix/revert before Exercise 4–7. Do not “just try live.”

---

## Exercise 4 — PDU sim (Jetson) so kill strip is not always COMMS_LOSS

**Goal:** Fresh `PDBF` on UART4 so USB kill can be `NORMAL` (or intentional soft-kill).

On Jetson (PDU UART is **not** the Controls CDC):

```bash
# Example — adjust UART device if your bench differs
cd ~/controls_pcb/scripts
python3 -u pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 \
  --force-kill-state 0 --estop-sense 1 \
  --pack-v 4800 4800 0 0 --rail-v 4800 1900 1200 500 \
  --pack-i 180 140 0 0 --rail-i 90 70 40 25 --contactor-state 15 \
  --control-port 8767
```

Use **8767** (or anything ≠ dashboard default **8766**) for the sim control panel.

**Pass:**

- [ ] Sim process stays up (`ps` / log)
- [ ] Later, when something owns CDC, `pdb_status` is not stuck on
      `hard_estop`/`comms_loss` for the whole session (unless you kill the sim)

Deep dive: [peripherals/pdu-uart-soft-kill.md](peripherals/pdu-uart-soft-kill.md).

---

## Exercise 5 — Discover actuators (DEBUG path)

**Goal:** See what answers on each bus **before** teleop.

Use product or continuous CFG only after you know IDs. Prefer a short exclusive
session (stop dashboard Connect).

Examples (API surface — exact CLI wrappers evolve; `hub.debug.*` is the
stable idea):

```text
# Conceptual — from a small Python REPL or existing smoke:
# Damiao on CH1 IDs 1..7, CH2 IDs 1..7
# RobStride on CH4/5/6 — product 0x01/0x02 vs bench 0x70/0x74/...
```

Practical entrypoints:

| Intent | Command |
|--------|---------|
| Product CFG + left arm prove | `python scripts/vbeta_product_prove.py --port <CDC> --hold-s 10` |
| Left arm only smoke | `python scripts/vbeta_smoke.py arm --port <CDC> --side left --apply-cfg --hold-s 2` |
| Continuous ops (bench map) | see [peripherals/continuous-ops.md](peripherals/continuous-ops.md) |

**Pass:**

- [ ] I listed which buses answered (CH1 / CH2 / CH4–6)
- [ ] I noted base ID story: product `0x01`/`0x02` vs bench spares — **do not
      silently remap** inside product CFG
- [ ] Right arm empty is OK if unpowered — log “CFG’d, no motor,” don’t force hold

---

## Exercise 6 — vbeta product prove ladder (CLI)

**Goal:** Run the realistic path from [vbeta-live-prove-plan.md](vbeta-live-prove-plan.md)
using **direct adapters** (not full `YAMAIMobile` / torch).

**Do in order:**

1. Exclusive COM (Exercise 1).
2. Offline gate green (Exercise 3).
3. Left hold + jog:

```bash
python scripts/vbeta_product_prove.py --port <CDC> --hold-s 30 --jog-joint 0
```

(`--jog-joint 0` avoids a soft-limit parking issue seen on joint 1 — see
[bench-vbeta-product-cfg-2026-07-24.md](bench-vbeta-product-cfg-2026-07-24.md).)

4. Optional: right arm discover-only if CH2 empty.
5. Optional: base product-ID probe — expect **known gap** if motors still on
   bench spare IDs; log it, don’t patch CFG silently.
6. Confirm process exited and CDC is free (`scan` / `fuser`).

**Pass:**

- [ ] Left arm live FB + jog converged (or HW blocker written down)
- [ ] Soft-kill service ticked without false trip (script logs)
- [ ] CDC released

**Do not expect:** dashboard Soft-kill Park (follow mode) to park a vbeta
session — only `yam_continuous_all` polls that flag today. PDU-level
`SOFT_KILL_REQ` / host V/I auto-park still apply inside the session.

---

## Exercise 7 — Continuous cruise (bench map) — optional

**Goal:** Multi-peripheral cruise on the **continuous** slot map (base 22–25).

Only if you chose continuous in Exercise 1. Ops manual:
[peripherals/continuous-ops.md](peripherals/continuous-ops.md).

Typical remote launcher:

```bash
# From host, with JETSON_PASS set — see continuous-ops.md AI quickstart
python scripts/launch_continuous.py
```

**Pass:**

- [ ] Arms/base/neck behave as continuous-ops “what good looks like”
- [ ] Cleanup: stop script; blank/DIAG if needed (`stop_can.py`)

---

## Exercise 8 — Dashboard (optional — Claudistic in progress)

**Goal:** Observe health without fighting the CLI owner.

Today’s durable ideas (UI labels may change):

1. Start UI (default HTTP **8766**, not sim panel 8765/8767):

```bash
python -m deft_controls_sdk.debug_dashboard --http-port 8766 --no-browser
# open http://127.0.0.1:8766
```

2. **Without Connect:** follow mode if something else writes `scripts/.deft_session/state.json`.
3. **Connect (observe):** telemetry / DIAG_ONLY — safe look, not teleop.
4. **Enable control:** only when you intend to command (Claudistic teleop landing here).
5. **Soft-kill Park:** when this process owns COM → direct park; when follow-only →
   flag file for continuous. If the button “does nothing,” check which mode you are in.

**Pass:**

- [ ] I can open the UI without stealing CDC from a running prove
- [ ] I understand observe vs control (even if buttons move under Claudistic)

Skip teleop-via-GUI until Claudistic marks dashboard teleop ready; use Exercise 6/7 instead.

---

## Exercise 9 — Soft-kill awareness (no need to trip hardware)

**Goal:** Know the three layers.

| Layer | What parks? |
|-------|-------------|
| PDU wire `SOFT_KILL_REQ` | Host `soft_kill_park_if_requested` while streaming (product default) |
| Host V/I check | `pdb/limits.py` / FW overlay → soft-kill / park |
| Dashboard follow flag | Only if COM owner polls `soft_kill_request` (continuous today) |

Read: [manual §4](manual/README.md#4-pdu--soft-kill), [pdu-uart-soft-kill.md](peripherals/pdu-uart-soft-kill.md).

**Pass:**

- [ ] I can say which layer fires for “bad pack voltage” vs “dashboard button while continuous owns COM”
- [ ] I know stale PDB → `HARD_ESTOP`/`COMMS_LOSS` is fail-safe, not “dashboard broke the board”

---

## Exercise 10 — Cleanup / handoff

**Do:**

1. Stop smokes / continuous / dashboard Connect.
2. `python scripts/soft_dfu_flash.py scan` — CDC idle.
3. Optional: leave a healthy PDU sim running for the next person, or stop it and
   accept COMMS_LOSS LEDs until sim returns.
4. Write one line for the next session: what worked, what’s a HW blocker
   (right arm / base IDs / Soft-DFU serial).

**Pass:**

- [ ] No orphan Python holding `/dev/ttyACM*` / COMx
- [ ] Handoff note written (chat, sticky, or `docs/bench-*.md`)

---

## How this relates to `vbeta-live-prove-plan.md`

That plan is the **detailed** CLI prove for Claude-Vbeta. This tutorial is the
**human** path. Mapping:

| Tutorial | Prove plan |
|----------|------------|
| Ex 1 ownership | §1 Exclusive COM |
| Ex 3 offline gate | §2 pytest |
| Ex 5–6 | §3–4 product CFG + ladder |
| Ex 6 direct adapters | §5 default (not YAMAIMobile) |
| Ex 9 soft-kill | §6 (incl. dashboard flag gap) |

You can finish Exercises 1–6 **without** Claudistic finishing dashboard teleop.
You **do** need CDC free (dashboard Disconnect).

---

## Quick command cheat sheet

```bash
# Inventory
python scripts/soft_dfu_flash.py scan

# Flash (USB-only)
python scripts/soft_dfu_flash.py --image Debug/DeftRoboticsControlsPCB.elf \
  --serial <SN> --require-usb-dfu

# Offline gates
python -m pytest scripts/tests/test_deft_controls_sdk_vbeta.py -q
python -m pytest scripts/tests/test_deft_controls_sdk_pdb_limits.py -q

# Product prove (Jetson example)
python3 scripts/vbeta_product_prove.py --port /dev/ttyACM0 --hold-s 30 --jog-joint 0

# Dashboard (observe-friendly default port)
python -m deft_controls_sdk.debug_dashboard --http-port 8766
```
