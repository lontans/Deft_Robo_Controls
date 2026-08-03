# SPI3: status LEDs vs MAX31855 thermocouple

SK9822 status LEDs and the MAX31855 thermocouple **share SPI3**. Only one role may own the bus at a time.

## Wiring (makes sense — not MOSI-as-MISO)

| Signal | Pin | LED (SK9822) | Thermo (MAX31855) |
|--------|-----|--------------|-------------------|
| SCK | PB3 | clock | clock |
| MISO | PB4 | unused | SO (sensor data out) |
| MOSI | PB5 | LED data in | dummy TX while clocking (must not drive LEDs) |
| CS | PB7 | — | soft CS (active low) |

Thermo readout uses `HAL_SPI_TransmitReceive` with dummy MOSI bytes so SCK clocks SO onto **MISO**. That is full-duplex SPI, not remapping MOSI to MISO. Exclusive `spi3_role` keeps LEDs off during thermo so dummy MOSI does not glitch the strip ([`max31855.c`](../App/Src/plant/plugins/max31855.c)).

## Runtime role

[`spi3_role.h`](../App/Inc/plant/spi3_role.h): `LED` (default) | `THERMO` | `NONE`.

Peripheral task ([`app.c`](../App/Src/app.c)):

- `LED` → `led_service()`
- `THERMO` → `thermo_service()` (~5 Hz)
- Thermo feedback fills the shared PDU slot only when the slot is empty (so it does not clobber CFG/diag replies).

## NVM / CFG (host-selectable)

Periph flags byte (GET/SET_PERIPH + NVM v2 image `flags`):

| Bits | Meaning |
|------|---------|
| 0 | `listen_pdu` |
| 1..2 | `spi3_role` (0=led, 1=thermo, 2=none) |

Old images with only bit0 set keep **LED**. After firmware with this packing is flashed:

```python
periph = hub.debug.cfg_get_periph()
periph["spi3_role"] = "thermo"   # or "led" / "none"
hub.debug.cfg_set_periph(periph, persist=True)  # SAVE to NVM
```

Until that FW is on the board, role stays compile-time `SPI3_ROLE_DEFAULT` (LED) unless changed in RAM by a future host tool talking to new FW.

## Bring-up checklist

1. Flash FW that includes SPI3 role in periph flags.
2. `cfg_set_periph(..., spi3_role="thermo", persist=True)`.
3. Confirm PDU/debug feedback carries thermo tags (`ok`, °C, CJ, faults).
4. Switch back to `led` before relying on status lights.
