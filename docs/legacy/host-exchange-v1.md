# Host exchange — layout v1

Fixed **562-byte** binary images in both directions. Same layout on USB CDC and UART; only the Linux/Windows device path differs.

**Superseded by [host-exchange-v2.md](host-exchange-v2.md) (672 B, `HOST_LAYOUT_VERSION` 2).** This file is historical; do not use for new firmware or SDK work.

**Former source of truth (v1):** `App/Inc/host/host_exchange_schema.h` at layout version 1.

## Identifiers

| Field | Command | Feedback |
|-------|---------|----------|
| Magic | `0x434D4448` (`"CMDH"`) | `0x46424848` (`"HBHF"`) |
| `layout_version` | `1` | `1` |
| `byte_size` | `562` | `562` |

Bump `HOST_LAYOUT_VERSION` and add `host-exchange-v2.md` when the layout changes — do not silently edit v1.

## Command image layout (562 B)

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 12 | `header` — magic, layout_version, byte_size, seq (u32) |
| 12 | 4 | `system` — e-stop / mcu_state / heartbeat bitfields |
| 16 | 500 | `actuator_commands[25]` — 20 B each |
| 516 | 12 | `servos[2]` — 6 B each |
| 528 | 2 | `leds[1]` — packed uint16 LE; see below |
| 530 | 32 | `pdu` — opaque payload (RS2 bench backdoor when tagged) |

### LED command (offset 528, 2 B)

Single `host_led_command_t` as **uint16 little-endian** at byte 528:

| Bits | Field | Range |
|------|-------|-------|
| 0–4 | `mode` | 0 = test scan, 1 = off (firmware-defined) |
| 5–9 | `master_brightness` | 0–31 (SK9822 global brightness) |
| 10–15 | `led_count` | 0 = use firmware `LED_STRIP_MAX`; else 1–63 |

Python: `word = (mode & 0x1F) | ((brightness & 0x1F) << 5) | ((count & 0x3F) << 10)` → `struct.pack_into("<H", buf, 528, word)`.

Script: `scripts/sk9822_led_test.py`. MCU applies via `led_command_mount` + `led_service()` (~30 Hz in main loop, not 500 Hz TIM6).

### Actuator command (20 B per slot)

| Offset in slot | Type | Field |
|----------------|------|-------|
| 0 | float | position (rad) |
| 4 | float | velocity (rad/s) |
| 8 | float | kp |
| 12 | float | kd |
| 16 | float | torque (Nm) |

Slot 0 starts at **byte offset 16** in the image (`ACTUATOR0_CMD_OFF` in scripts).

**Plant teleop** patches slots **0–3** (four configured actuators). Unused wire slots remain zero.

### RS2 PDU backdoor (`pdu` offset 530)

When `pdu.data[0..2] == 'R','S','2'`, firmware runs `plant_diag` instead of (or in addition to) the normal plant path:

| `pdu` offset | Field |
|-------------|-------|
| 0–2 | Tag `'R','S','2'` |
| 3 | Motor ID |
| 4 | Probe kind (`PLANT_DIAG_PROBE_*`, session 254/255) |
| 5–10 | Probe parameters (pararead index, cal timeout, …) |
| **11** | **Schematic CAN bus: `1` = CH1, `2` = CH2 (PA8/PA15), `3` = CH3 (PB12/PB13)** |

Feedback probe results are mirrored in `pdu` on the feedback image (see `parse_probe_pdu` in Python).

**Plant teleop:** leave `pdu` zero — only `actuator_commands[]` are consumed at 500 Hz.

### DM0 PDU backdoor (`pdu` offset 530) — Damiao bench

When `pdu.data[0..2] == 'D','M','0'` and `system.mcu_state == DIAG_ONLY (2)`, firmware runs `plant_diag_on_dm_command` (CH3 standard CAN probes). Same 562 B image as RS2; host script: `scripts/damiao_scan.py`.

| `pdu` offset | Field |
|-------------|-------|
| 0–2 | Tag `'D','M','0'` |
| 3 | Motor ID (candidate ESC_ID) |
| 4 | Probe kind (`DM_PROBE_REG_SCAN` = 16 preferred for discover) |
| 5 | Master ID filter (`0xFF` = any; ignored for reg scan) |
| 6 | listen_ms |
| 7 | param_rid (e.g. ESC_ID `0x08`) |
| **11** | Schematic CAN bus (`3` = CH3 Damiao) |

Feedback: `pdu.data[0] == 'm'` with `found`, `discovered_id`, `master_id`, `raw_frames_seen`, `tx_frames_sent`. Slot 2 actuator mirror also carries probe results during bench (`fault` marker `0xDA000000`).

**Host note:** `mcu_state` is bits 1–3 of the u32 at offset 12 — use `patch_system_mcu_state()` in Python (same as RS2 scripts).

## Feedback image layout (562 B)

Same structure with feedback types:

| Offset | Size | Field |
|-------:|-----:|-------|
| 0 | 12 | `header` |
| 12 | 4 | `system` — see below |
| 16 | 500 | `actuator_feedback[25]` — 20 B each |
| 516 | 12 | `servos[2]` |
| 528 | 2 | `leds[1]` |
| 530 | 32 | `pdu` |

### Actuator feedback (20 B per slot)

| Offset in slot | Type | Field |
|----------------|------|-------|
| 0 | float | position |
| 4 | float | velocity |
| 8 | float | torque |
| 12 | float | temperature (°C) |
| 16 | uint32 | fault flags |

Slots 0–3 are populated when the corresponding motor is enabled and talking on CAN.

### System feedback word (offset 12, u32 LE)

| Bits | Field |
|------|-------|
| 0–11 | `control_tick_count` (12-bit, TIM6 counter) |
| 12–16 | e-stop / mcu / heartbeat readback |
| 17–24 | `last_command_seq` (8-bit echo of command header seq) |
| 25–31 | reserved |

Python: `tick = sys_word & 0xFFF`, `last_cmd_seq = (sys_word >> 17) & 0xFF`.

**Note:** Feedback `header.seq` is not incremented by firmware yet (always 0).

## Rates and bandwidth

| Transport | Typical host cmd rate | Notes |
|-----------|----------------------|--------|
| USB CDC | ~40 Hz (`--plant-teleop`) | Primary laptop bench |
| USB CDC | ~30 Hz (legacy Jetson script) | `host_teleop.py` default |
| UART 115200 | ~10 Hz | Jetson UART path |

Host sends periodically (**hold-last-command**); plant applies at **500 Hz** with hold-last desires. Stale host commands stop applying after `ACTUATOR_HOST_STALE_MS` (500 ms).

## Validation (MCU)

`host_command_image_valid()` checks magic, layout_version, and byte_size before `host_command_image_dispatch()`.

## Version bump checklist

1. Update `HOST_LAYOUT_VERSION` and structs in `host_exchange_schema.h`
2. Update `_Static_assert` block in `host_exchange_schema.c`
3. Update `scripts/host_teleop_laptop_usb.py` and `scripts/rs02_can_scan.py`
4. Add `docs/host-exchange-vN.md`; keep v1 doc immutable
