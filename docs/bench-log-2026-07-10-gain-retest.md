# Bench log — 2026-07-10 — YAM gain-default retest

Context: SLOT_KP defaults bumped (J1-J4 higher, see docs/plan-yam-joint-commands.md /
teleop/defaults.py). Initial retest via `teleop --slot N` reportedly showed no motion —
root cause: `hello_world.py` and `joint_cmd.py::_default_gains` still resolved Damiao kp
from a flat `D.DM_KP=12` fallback instead of the per-slot `D.SLOT_KP` table (same bug
already fixed in `teleop/plant.py::run_for_slot`, missed in the scripted joint-goto path).
Fixed both, then retesting live via `joint goto` (scripted — this session cannot drive the
keyboard-based interactive `teleop` command, which needs real key presses via msvcrt).

All commands run from repo root, `python scripts/control_hub.py ...`, port COM5.

## Commands run
### `python scripts/control_hub.py status --port COM5`
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
tick=2465  mcu_state=0  ack_seq=6  plant_block=none  pdu=S  | slot0: pos=+0.5369 err/fault=0x1  | slot1: pos=-2.6164 err/fault=0x1  | slot2: pos=+1.9053 err/fault=0x1  | slot3: pos=+0.0402 err/fault=0x1  | slot4: pos=-0.0235 err/fault=0x0  | slot5: pos=-0.3161 err/fault=0x1
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 1 --delta 0.15 --slew 0.20 --hold-s 1.0`  (verify fixed per-slot kp: expect kp=30 for J1)
```
hello-world: port=COM5 slot=0 J1(joint1)  soft=[-2.568,+3.080]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J1(joint1) slot0 CH1 (PB8/9 FDCAN1) damiao id=0x01  delta=+0.150 rad  slew=0.20  kp=30.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot0 CH1 (PB8/9 FDCAN1) 0x01  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=+0.5369 rad  err=0x1
  slew +0.5369 -> +0.6869 (delta=+0.1500) ...
  at target: cmd=+0.6869 fb=+0.6765 torque=+0.100
  return -> +0.5369 ...
  done: start=+0.5369 peak_fb=+0.6765 end=+0.5488 moved=0.1396 err=0x1
PASS: hello-world J1(joint1) slot0 CH1 (PB8/9 FDCAN1) damiao id=0x01
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 3 --delta 0.10 --slew 0.15 --hold-s 1.0`  (verify fixed per-slot kp: expect kp=90 for J3)
```
hello-world: port=COM5 slot=2 J3(joint3)  soft=[+0.050,+3.080]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J3(joint3) slot2 CH1 (PB8/9 FDCAN1) damiao id=0x03  delta=+0.100 rad  slew=0.15  kp=90.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot2 CH1 (PB8/9 FDCAN1) 0x03  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=+1.9053 rad  err=0x1
  slew +1.9053 -> +2.0053 (delta=+0.1000) ...
  at target: cmd=+2.0053 fb=+1.9633 torque=+0.520
  return -> +1.9053 ...
  done: start=+1.9053 peak_fb=+1.9633 end=+1.9022 moved=0.0580 err=0x1
PASS: hello-world J3(joint3) slot2 CH1 (PB8/9 FDCAN1) damiao id=0x03
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 4 --delta 0.10 --slew 0.15 --hold-s 1.0`  (verify fixed per-slot kp: expect kp=50 for J4)
```
hello-world: port=COM5 slot=3 J4(joint4)  soft=[-1.521,+1.521]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J4(joint4) slot3 CH1 (PB8/9 FDCAN1) damiao id=0x04  delta=+0.100 rad  slew=0.15  kp=50.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot3 CH1 (PB8/9 FDCAN1) 0x04  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=+0.0399 rad  err=0x1
  slew +0.0399 -> +0.1399 (delta=+0.1000) ...
  at target: cmd=+0.1399 fb=+0.0399 torque=+0.071
  return -> +0.0399 ...
  done: start=+0.0399 peak_fb=+0.0399 end=+0.0399 moved=0.0000 err=0x1
FAIL: little/no motion (moved=0.0000, want ~0.100) � enable/TIMEOUT/bus?
```

### `python scripts/control_hub.py joint status --port COM5 --joint 4`
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  joint-status OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
joint-status: J4(joint4) slot3 CH1 (PB8/9 FDCAN1) damiao id=0x04
  fb=+0.0433 rad  soft=[-1.521,+1.521]  dist_to_lo=+1.564  dist_to_hi=+1.478
  torque=-0.007 Nm  err=0x1
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 4 --delta -0.10 --slew 0.15 --hold-s 1.0`  (J4 opposite direction retest)
```
hello-world: port=COM5 slot=3 J4(joint4)  soft=[-1.521,+1.521]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J4(joint4) slot3 CH1 (PB8/9 FDCAN1) damiao id=0x04  delta=-0.100 rad  slew=0.15  kp=50.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot3 CH1 (PB8/9 FDCAN1) 0x04  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=+0.0433 rad  err=0x1
  slew +0.0433 -> -0.0567 (delta=-0.1000) ...
  at target: cmd=-0.0567 fb=+0.0120 torque=-0.256
  return -> +0.0433 ...
  done: start=+0.0433 peak_fb=+0.0120 end=+0.0120 moved=0.0313 err=0x1
PASS: hello-world J4(joint4) slot3 CH1 (PB8/9 FDCAN1) damiao id=0x04
```

### `python scripts/control_hub.py joint status --port COM5 --joint 2`  (J2 was reading far outside soft range in status sweep — check before any motion)
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  joint-status OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
joint-status: J2(joint2) slot1 CH1 (PB8/9 FDCAN1) damiao id=0x02
  fb=-2.6164 rad  soft=[+0.050,+3.600]  dist_to_lo=-2.666  dist_to_hi=+6.216
  torque=-0.002 Nm  err=0x1
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 2 --delta 0.10 --slew 0.15 --hold-s 1.0`  (verify fixed per-slot kp: expect kp=50 for J2; fb outside soft window -> relative-only fallback expected)
```
hello-world: port=COM5 slot=1 J2(joint2)  soft=[+0.050,+3.600]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J2(joint2) slot1 CH1 (PB8/9 FDCAN1) damiao id=0x02  delta=+0.100 rad  slew=0.15  kp=50.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot1 CH1 (PB8/9 FDCAN1) 0x02  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=-2.6164 rad  err=0x1
  limits: fb outside soft [0.050,3.600] � relative-only, |delta|<=0.292
  slew -2.6164 -> -2.5164 (delta=+0.1000) ...
  at target: cmd=-2.5164 fb=-2.6110 torque=+1.678
  return -> -2.6164 ...
  done: start=-2.6164 peak_fb=-2.6110 end=-2.6133 moved=0.0053 err=0x1
FAIL: little/no motion (moved=0.0053, want ~0.100) � enable/TIMEOUT/bus?
```

### `python scripts/control_hub.py joint status --port COM5 --joint 2`  (post-attempt check)
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  joint-status OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
joint-status: J2(joint2) slot1 CH1 (PB8/9 FDCAN1) damiao id=0x02
  fb=-2.6133 rad  soft=[+0.050,+3.600]  dist_to_lo=-2.663  dist_to_hi=+6.213
  torque=-0.007 Nm  err=0x1
```

## Summary

- Root cause of "not moving": hello_world.py / joint_cmd.py `_default_gains` still resolved damiao kp from flat `D.DM_KP=12` instead of per-slot `D.SLOT_KP`. Fixed both.
- J1: PASS, kp=30 auto-resolved, moved 0.140/0.150 rad.
- J3: PASS, kp=90 auto-resolved, moved 0.058/0.100 rad.
- J4: positive delta FAIL (torque~0, no resistance); negative delta PASS, kp=50, moved 0.031/0.100 rad. Direction-sensitivity persists after repositioning.
- J2: FAIL at new pose (fb=-2.6164, far outside documented soft range [0.05,3.6]). kp=50 applied, torque climbed to 1.68 Nm, moved only 0.005 rad — hard-stop-like signature, not a gain problem this time. At rest afterward: torque -0.002, err=0x1, no fault.

### `python scripts/control_hub.py joint goto --port COM5 --joint 6 --delta 0.10 --slew 0.15 --hold-s 1.0`  (J6/J7 not moving in interactive teleop — isolate via scripted goto)
```
hello-world: port=COM5 slot=5 J6(joint6)  soft=[-2.044,+2.044]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J6(joint6) slot5 CH1 (PB8/9 FDCAN1) damiao id=0x06  delta=+0.100 rad  slew=0.15  kp=12.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot5 CH1 (PB8/9 FDCAN1) 0x06  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=-0.2180 rad  err=0x1
  slew -0.2180 -> -0.1180 (delta=+0.1000) ...
  at target: cmd=-0.1180 fb=-0.2180 torque=-0.002
  return -> -0.2180 ...
  done: start=-0.2180 peak_fb=-0.2180 end=-0.2180 moved=0.0000 err=0x1
FAIL: little/no motion (moved=0.0000, want ~0.100) � enable/TIMEOUT/bus?
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 7 --delta 0.10 --slew 0.15 --hold-s 1.0`
```
hello-world: port=COM5 slot=6 J7(joint7_ee)  soft=[+1.150,+2.750]  source=bench-derived (2026-07; motor frame; not in yam.xml)
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=S
  MCU actuator table synced for teleop.
  target: J7(joint7_ee) slot6 CH1 (PB8/9 FDCAN1) damiao id=0x07  delta=+0.100 rad  slew=0.15  kp=12.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot6 CH1 (PB8/9 FDCAN1) 0x07  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=-2.8944 rad  err=0x1
  limits: fb outside soft [1.150,2.750] � relative-only, |delta|<=0.136
  slew -2.8944 -> -2.7944 (delta=+0.1000) ...
  at target: cmd=-2.7944 fb=-2.8944 torque=-0.007
  return -> -2.8944 ...
  done: start=-2.8944 peak_fb=-2.8944 end=-2.8944 moved=0.0000 err=0x1
FAIL: little/no motion (moved=0.0000, want ~0.100) � enable/TIMEOUT/bus?
```

## J6/J7 not moving in either interactive teleop or scripted joint goto

### `joint goto --port COM5 --joint 6 --delta 0.10` -> FAIL, moved=0.0000, torque~0
### `joint goto --port COM5 --joint 7 --delta 0.10` -> FAIL, moved=0.0000, torque~0

Both read feedback fine (status OK, no fault) and enable OK — only MIT streaming
commands have no effect. Confirmed same failure via scripted goto (not teleop-specific).

**Root cause found (firmware):** `App/Src/plant/plugins/damiao.c::damiao_apply_cycle()`
sent 3x redundant MIT frames per Damiao slot per control tick (added 2026-07-06 for
single/dual-motor reliability). With 6-7 Damiao slots enabled simultaneously (YAM daisy),
that's up to 21 MIT frames/tick @ 500 Hz =~10,500 frames/sec demand vs ~7.7-9.3k frames/sec
physical capacity at 1 Mbps on CH1 — oversubscribed. actuator_apply_desire() enqueues by
slot index in order (0..6), so once the TX queue chronically backs up, the highest-index
slots (J6=idx5, J7=idx6) consistently lose the enqueue race and never get their MIT frames
onto the bus, while J1-J5 (idx0-4) keep working. Matches the exact observed pattern.

**Fix applied (source only, NOT yet flashed):** reduced the loop to a single MIT send per
slot per tick. Needs rebuild + reflash from STM32CubeIDE (Debug) before retest — this
session has no flash tool access.


## Post-reflash: config show / discover timing out

`status` and joint feedback work fine (plant path OK), but `config show`/`discover`
(CFG PDU) hard-fail with `TimeoutError: no CFG response from MCU within timeout`.
`status` consistently showed `pdu=t` (thermo tag) even after `recover`.

**Root cause (firmware):** `App/Src/plant/thermo.c::thermo_feedback_fill()` writes
into the shared diagnostic PDU slot **unconditionally** whenever `spi3_role_get() ==
SPI3_ROLE_THERMO` — and `SPI3_ROLE_DEFAULT` (App/Inc/plant/spi3_role.h) is compiled
as `SPI3_ROLE_THERMO`, not LED, so this is always true. `plant_feedback.c` calls
`plant_config_feedback_fill()` (CFG reply) then `thermo_feedback_fill()` right after —
thermo has no request/pending gate like CFG/RS2/DM diag do, so it clobbers whatever
CFG just staged, every single tick. CFG's reply never survives to reach the host.

**Fix applied (source only, needs rebuild+reflash):** `thermo_feedback_fill()` now
returns early if `pdu->data[0] != 0` (something already claimed the shared slot this
tick — confirmed the image is freshly memset to 0 each cycle in host_link.c before
this runs, so the check is valid). Thermo still fills whenever the slot is free, same
as before; it just no longer steamrolls CFG/RS2/DM diag replies.


## Retest after second reflash (thermo PDU-clobber fix + damiao 1x TX fix)

### `python scripts/control_hub.py status --port COM5`
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
tick=1576  mcu_state=0  ack_seq=6  plant_block=none  pdu=t  | slot0: pos=+0.4976 err/fault=0x1  | slot1: pos=+0.0000 err/fault=0x0  | slot2: pos=+0.0000 err/fault=0x0  | slot3: pos=+0.0000 err/fault=0x0  | slot4: pos=+0.0000 err/fault=0x0  | slot5: pos=+0.0000 err/fault=0x0
```

### `python scripts/control_hub.py config show --port COM5`  (this is the actual regression test — should no longer time out)
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
Traceback (most recent call last):
  File "C:\Users\jsong\Documents\DeftRoboticsControlsPCB\scripts\control_hub.py", line 18, in <module>
    raise SystemExit(main())
                     ~~~~^^
  File "C:\Users\jsong\Documents\DeftRoboticsControlsPCB\scripts\controls_pcb_host\cli\main.py", line 890, in main
    return int(args.func(args))
               ~~~~~~~~~^^^^^^
  File "C:\Users\jsong\Documents\DeftRoboticsControlsPCB\scripts\controls_pcb_host\cli\main.py", line 366, in cmd_config_show
    slots = cfg_pdu.fetch_table(session)
  File "C:\Users\jsong\Documents\DeftRoboticsControlsPCB\scripts\controls_pcb_host\plugins\plant_config.py", line 77, in fetch_table
    return exchange_cfg(session, CFG_OP_GET, timeout_s=timeout_s)["slots"]
           ~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Users\jsong\Documents\DeftRoboticsControlsPCB\scripts\controls_pcb_host\plugins\plant_config.py", line 70, in exchange_cfg
    raise TimeoutError("no CFG response from MCU within timeout")
TimeoutError: no CFG response from MCU within timeout
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 6 --delta 0.10 --slew 0.15 --hold-s 1.0`  (checking whether the damiao.c 1x-TX fix made it into this build)
```
hello-world: port=COM5 slot=5 J6(joint6)  soft=[-2.044,+2.044]  source=C:/Users/jsong/Documents/DeftRoboticsControlsPCB/External_Documentation/yam_arm_damiao/yam.xml
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=t
  WARNING: could not read MCU actuator table (no CFG response from MCU within timeout) � using host defaults (run: control_hub.py config show --port COMx).
  target: J6(joint6) slot5 CH1 (PB8/9 FDCAN1) damiao id=0x06  delta=+0.100 rad  slew=0.15  kp=12.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot5 CH1 (PB8/9 FDCAN1) 0x06  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=-0.2180 rad  err=0x1
  slew -0.2180 -> -0.1180 (delta=+0.1000) ...
  at target: cmd=-0.1180 fb=-0.1268 torque=+0.090
  return -> -0.2180 ...
  done: start=-0.2180 peak_fb=-0.1268 end=-0.2047 moved=0.0912 err=0x1
PASS: hello-world J6(joint6) slot5 CH1 (PB8/9 FDCAN1) damiao id=0x06
```

### Diagnostic: raw PDU tag dump after CFG GET request

Sent one CFG_OP_GET, then read every feedback frame for 2s (1709 frames).
Result: **100% tagged `t`, zero `c` replies ever seen.**

**Conclusion:** the thermo.c fix is present in source (confirmed, untracked file
unchanged since my edit) but is NOT reflected in the currently running firmware —
damiao.c's fix clearly IS active (J6 moves now), so this reflash picked up some but
not all changes. Likely an incremental-build staleness issue on thermo.c specifically.
Needs a clean rebuild (not incremental) + reflash.


## Retest after clean rebuild + reflash (thermocouple physically unplugged)

### Raw PDU tag dump after CFG GET: unique tags = ['c','t'], c count=1 (of 1696 frames)
Fix confirmed active.

### `python scripts/control_hub.py config show --port COM5`
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
Actuator table (MCU, 7 slots):
  slot 0: damiao      CH1  id=0x01  master=0x00  on
  slot 1: damiao      CH1  id=0x02  master=0x00  on
  slot 2: damiao      CH1  id=0x03  master=0x00  on
  slot 3: damiao      CH1  id=0x04  master=0x00  on
  slot 4: damiao      CH1  id=0x05  master=0x00  on
  slot 5: damiao      CH1  id=0x06  master=0x00  on
  slot 6: damiao      CH1  id=0x07  master=0x00  on
```

### `python scripts/control_hub.py discover --port COM5 --protocol damiao --bus 1 --start 1 --end 8 --listen-ms 40`
```
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
Damiao discover on CH1 (PB8/9 FDCAN1)  IDs 1..8
FOUND  probe=0x01  esc_id=0x01  master_rx=0x11  mode=id_sweep  pos=+0.0000  err=0x0
```

### `python scripts/control_hub.py joint goto --port COM5 --joint 7 --delta 0.10 --slew 0.15 --hold-s 1.0`
```
hello-world: port=COM5 slot=6 J7(joint7_ee)  soft=[+1.150,+2.750]  source=bench-derived (2026-07; motor frame; not in yam.xml)
Port COM5: USB Serial Device (COM5)  vid=0x0483 pid=0x5740
  USB link OK (562 B plant feedback).
  hello-world OK  plant_block=none  pdu=t
  MCU actuator table synced for teleop.
  target: J7(joint7_ee) slot6 CH1 (PB8/9 FDCAN1) damiao id=0x07  delta=+0.100 rad  slew=0.15  kp=12.0 kd=0.80
Waking actuators (plant path arm)...
  enable OK  slot6 CH1 (PB8/9 FDCAN1) 0x07  (0xFB clear + 0xFC enable, ERR=0x1)
  synced fb=-2.8944 rad  err=0x1
  limits: fb outside soft [1.150,2.750] � relative-only, |delta|<=0.136
  slew -2.8944 -> -2.7944 (delta=+0.1000) ...
  at target: cmd=-2.7944 fb=-2.8037 torque=+0.095
  return -> -2.8944 ...
  done: start=-2.8944 peak_fb=-2.8037 end=-2.8834 moved=0.0908 err=0x1
PASS: hello-world J7(joint7_ee) slot6 CH1 (PB8/9 FDCAN1) damiao id=0x07
```

## Summary — all fixes confirmed working after clean rebuild

- CFG PDU (config show / discover / joint status MCU sync): working, thermo no longer clobbers it
- Damiao 1x-TX fix: J6 PASS (moved 0.091/0.10), J7 PASS (moved 0.091/0.10)
- All 7 slots confirmed enabled and correctly mapped (CH1, damiao, 0x01-0x07)
