# Vendor notes (Controls PCB–relevant)

Local PDFs/EDS live under [`External_Documentation/`](../External_Documentation/) (gitignored bulk). Prefer firmware (`App/Src/plant/plugins/*`) when a PDF sample disagrees.

Host sends MIT-shaped `ActuatorDesire`; firmware packs frames. Summary pass: [vendor PDF subagent](580ae463-80fe-4673-b060-bfc8b8b38c6e).

## Damiao (DM-J4310 / J4340 / J4340P)

- **Bus:** standard 11-bit; default **1 Mbps**. Extended manuals: baud reg `0x23` codes 0–9 (125k…5M); ≤1M = classic, >1M = CAN FD (FD FB invisible to classic masters). Short PDFs claim fixed 1 Mbps.
- **IDs:** cmd on **ESC_ID**; FB on **Master ID** (`MST_ID`, default 0). Mode-offset IDs: PosVel `0x100+ID`, Vel `0x200+ID`, Force-Pos `0x300+ID`. Register R/W via **`0x7FF`**.
- **MIT pack (8 B):**

| D | Content |
|---|--------|
| 0–1 | p_des 16b |
| 2 | v[11:4] |
| 3 | v[3:0] \| kp[11:8] |
| 4 | kp[7:0] |
| 5 | kd[11:4] |
| 6 | kd[3:0] \| t_ff[11:8] |
| 7 | t_ff[7:0] |

- Kp **[0,500]**, Kd **[0,5]**; P/V/T maps via PMAX/VMAX/TMAX (regs `0x15`–`0x17`). Plugin limits in `damiao.h` (DM4310 T±10; DM4340P T±28) — **factory PMAX/VMAX/TMAX numbers not fixed in PDFs**.
- **Opcodes (Extended manuals):** enable `0xFC`, disable `0xFD`, zero `0xFE`, clear fault `0xFB` (intent: `0xFF` fill + opcode in D[7]; short PDFs omit these). FB ERR: 0 disabled, 1 enabled, `8`–`E` faults.
- **Status:** live dual-YAM path.

## RobStride (RS01–RS04)

- **Bus:** CAN 2.0, **1 Mbps**, **extended 29-bit**. ID: type[28:24] \| data2[23:8] \| dest[7:0]. Data **big-endian**.
- **Type `0x1` operation (plant):** torque **16-bit in ID bits 23–8**; payload Angle/Vel/Kp/Kd u16 each.

| Model | Pos | Vel | Torque | Kp / Kd |
|-------|-----|-----|--------|---------|
| RS01 / RS02 | ±4π | ±44 rad/s | ±17 N·m | 0–500 / 0–5 |
| RS03 | ±4π | ±20 rad/s | ±60 N·m | same |
| RS04 | ±4π | ±15 rad/s | ±120 N·m | same |

- **Lifecycle types:** `0x0` get ID, `0x2` FB, `0x3` enable, `0x4` stop, `0x6` zero, `0x7` set CAN_ID, `0x11`/`0x12` param R/W, `0x17` baud (power-cycle).
- **Status:** live on CH4–6 base / channel bringup; confirm product vs bench IDs. RS01 PDF was image-only (OCR recovered).

## CubeMars (AK MIT)

- **MIT:** standard ID = motor ID, 1 Mbps. Servo mode (EXT) compile-gated off (`CUBEMARS_ENABLE_SERVO_MODE=0`).
- **Opcodes:** `{FF×7, 0xFC}` enable / `0xFD` disable / `0xFE` zero — same pattern as Damiao.
- **Pack table matches Damiao nibble layout** (pos/vel/kp/kd/t).
- **PDF bug — do not copy sample `pack_cmd`:**

```c
msg->data[6] = ((kd_int&0xF)<<4)|(kp_int>>8); // WRONG
// correct: (kd_lo4 << 4) | (t_int >> 8)
```

- Model maps vary (plugin default AK80-9). Do not hot-switch servo↔MIT without power cycle.
- **Status:** code present, **not HW-proven**.

## ZeroErr (eRob / eDriver CANopen)

- **EDS:** product eDriver; Vendor `0x5A65726F`; Product `0x26483052`; **BaudRate_1000=1 only** → 1 Mbps.
- **Default PDO map is sparse:** RxPDO1/`0x200+N` = CW only; RxPDO2/`0x300+N` disabled but maps CW+target; TxPDO1/`0x180+N` = SW only. Plugin remaps to DLC6 CW+target / SW+actual — see `zeroerr.c`.
- CiA 402 PP: `0x6060=1`; CW `0x06→0x07→0x0F` (+ `0x1F` new setpoint in examples). Encoder res provisional **524288**.
- **Status:** boot FSM present, **not bench-proven**. Official manuals form-gated.

## MCP2518FD + MCP2562FD (CH4–6)

- **MCP2518FD:** SPI 0,0/1,1 ≤20 MHz; SCK ≤ 0.85×(SYSCLK/2); classic arbitration ≤1 Mbps (we stay classic @ 1M). Message RAM R/W **word-aligned**; many config bits / FRESET only in Configuration mode; INT-gated RX in our driver.
- **Transceiver PDF filename mismatch:** `MCP2562_Documentation.pdf` content is **MCP2561/2FD**. MCP2562FD pin5 = **VIO** (1.8–5.5 V); MCP2561FD pin5 = SPLIT.
- Living behavior: `mcp2518fd.c` / `spi_can_router.c`.

## Dynamixel XL330 (neck)

- Protocol **2.0**, TTL half-duplex 8N1, **3.3 V logic / 5 V tolerant**. Default baud table entry **57600** (addr 8); up to 4 Mbps.
- M288-T: ~288:1, model 1200, stall ~0.52 N·m @ 5 V. M077-T: ~77.5:1, stall ~0.215 N·m (image PDF). PeripheralTask — not MIT plant path.

## SK9822 (LEDs)

- Clocked LED (APA102-class): start 32×0; LED frame `111` + 5-bit global + BGR; end frame longer than datasheet’s 32×1 (see hardware guide / Pololu notes). ≤30 MHz clock; ~50 mA/LED full white.

## Cross-cutting

| Bus | Typical use |
|-----|-------------|
| FDCAN CH1–3 @ 1 Mbps | Damiao STD MIT |
| MCP2518FD CH4–6 | RobStride EXT (torque in ID) |
| Not single-frame MIT | ZeroErr CANopen; DXL TTL; SK9822 SPI |

STM32/FreeRTOS tutorial PDFs under `External_Documentation/STM32/` are not summarized — see [architecture.md](architecture.md).
