# In-person teleop input (Cursor track)

## What to run on the laptop (board plugged in here)

**Driver (moves the arm):** `scripts/mouse_arm_teleop.py`  
Uses Claude-2 per-joint `DEFAULT_ARM_KD`, continuous-style progressive latch (`fault==1` = MIT green), **J2 frozen at FB** (hardstop-safe).

**Input-only (no CDC):** `scripts/mouse_teleop.py` — emits `TeleopSample` / RTC 122B blobs only. Does **not** talk to the board.

### Steps

1. Unplug Controls PCB from the Jetson (CDC free there).
2. Plug board USB into this laptop. Expect STM32 VCP (often `COM5`, vid=`0x0483` pid=`0x5740`).
3. Confirm power / current limit OK (UV caused earlier false faults).
4. From repo root:

```powershell
cd C:\Users\jsong\Documents\DeftRoboticsControlsPCB
python -m pip install pynput pyserial
python scripts\mouse_arm_teleop.py
# or: python scripts\mouse_arm_teleop.py --port COM5
```

5. Wait for progressive latch (`faults=[1,1,1,1,1,1,1]`).
6. **Press+hold middle** — enable + stick center.

| M650 | Effect |
|------|--------|
| Middle hold | Enable + stick origin |
| Stick L / R | J1 |
| Stick U / D | J2 |
| Stick U / D + right hold | J3 |
| Stick U / D + thumb1 | J4 |
| Double-right (while enabled) | J7 open/close toggle |
| Scroll | J5 |
| Left hold | 0.5× rates |
| Release middle | Stop |
| Ctrl+C | Blank + exit |

### Sync note

Laptop `scripts/deft_controls_sdk/vbeta/slots.py` already has  
`DEFAULT_ARM_KD = (2.5, 3.75, 5.6, 3.75, 1.5, 1.5, 1.25)`.  
`mouse_arm_teleop.py` imports that — same gains as the Jetson J1 smoke.

## Parked

| Path | Status |
|------|--------|
| Meta Quest Link / OpenXR | Blocked on ThinkPad |
| Deft Quest app UDP | Not used |
| `deft_rtc` shared rooms | Do not join |
